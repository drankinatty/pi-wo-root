/**
 *  Inversense MPU API
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */

#include <stdio.h>  /* temp for mag calibrate */

#ifdef RPIPICO

#include "pico/stdlib.h"

#else

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>

#endif

/* TODO
 *   create mpu-testdata.h to hold saved accel, gyro and mag bias and scale values
 *   in array form so they can be indexed and incorporated by arr[mpu->id]. These
 *   can be set before auto-calibrate is run and for scale, soft and hard-iron
 *   biases prevent having to dynamically rotate mpu through all axis in figure-8
 *   on startup each time to compute values.
 * NOTE
 *   arr[] must be 1-based for actual data, with arr[0] holding zeros as default
 *   data for new/unseen mpu chip-IDs.
 */

#include "mpu-constants.h"
#include "mpu-calibrate.h"
#include "mpu-caldata.h"

#define ACCELSTEP     0.00098f
#define GYROSTEP      0.0305f
#define CALSAMPLES    300
#define CALROUNDS       2

static uint8_t calcount = 0;
volatile bool autocalibrated;

/* temp extern variables */

uint8_t   fifo_overflow,
          packet_count;

uint16_t  fifo_count_rpt;

int32_t   accel_sum[NAXIS] = {0},
          gyro_sum[NAXIS] = {0};

/*************************/

/* self-test values for mpu boards 1, 2 & 3
 * used as fingerprint to ID board.
 *
 * TODO - move to mpu-caldata with ptr for extern in header and no. of elements
 */
// static const uint8_t mpu_st_list[][6] = { {  93,  88, 115, 191, 198, 225 },
//                                           { 104,  94, 122, 199, 207, 226 },
//                                           {  94,  95, 120, 198, 215, 219 } };

/* temperature compensation bit-0 of accel_offset */
static uint8_t temp_comp_bit[3] = {0};

/**
 *  read factory selftest values from registers 0x00 - 0x02 and
 *  0x0d - 0x0f.
 */
bool get_factory_st (mpu_t *mpu)
{
  uint8_t buffer[RWBUFSZ];

  if (i2c_read_reg (ADDR, MPU_SELF_TEST_X_GYRO, buffer, 3) != 3) {
    return false;
  }

  mpu->gyro_st.x = buffer[0];
  mpu->gyro_st.y = buffer[1];
  mpu->gyro_st.z = buffer[2];

  if (i2c_read_reg (ADDR, MPU_SELF_TEST_X_ACCEL, buffer, 3) != 3) {
    return false;
  }

  mpu->accel_st.x = buffer[0];
  mpu->accel_st.y = buffer[1];
  mpu->accel_st.z = buffer[2];

  return true;
}

/**
 *  compare mpu->self-test for current mpu to the self-test results
 *  in mpu_st_list. if matched, the self-test can serve as a
 *  fingerprint to set the mpu->id.
 *
 *  ID must be 1-based to allow zero-initialized struct to set default.
 */
uint8_t get_id_from_factory_st (mpu_t *mpu)
{
  // int nids  = sizeof mpu_st_list / sizeof *mpu_st_list,
  //     nelem = sizeof *mpu_st_list / sizeof **mpu_st_list;
  int nids  = get_mpu_st_list_sz(),
      nelem = NAXIS * 2;
  const uint8_t mpuid[6] =  { mpu->accel_st.x, mpu->accel_st.y, mpu->accel_st.z,
                              mpu->gyro_st.x, mpu->gyro_st.y, mpu->gyro_st.z };
  int i = 0;

  for (; i < nids; i++) {                     /* loop over each known id */
    bool found = true;                        /* initialize flag true */
    for (int j = 0; j < nelem; j++) {         /* loop over each st value */
      if (mpuid[j] != mpu->st_data[i][j]) {   /* if any fails to match */
        found = false;                        /* set found flag false */
        break;
      }
    }
    if (found) {                    /* if self-test data matched */
      return (mpu->id = i + 1);     /* return one-based index for mpu */
    }
  }

  return 0;
}

min_max_sum calaccel = { .min = {{0}} },
            calgyro = { .min = {{0}} };
uint32_t nsamples = 0;
volatile bool ag_cal_rdy;
vect_float_t  accelaxis = {{ 0, 0, 0 }},
              gyroaxis =  {{ 0, 0, 0 }};

// void ag_min_max_init (min_max_sum *accel, min_max_sum *gyro, mpu_t *mpu)
void ag_min_max_init (mpu_t *mpu)
{
  /* capture accelerometer min, max, sum */
  calaccel.min.x = calaccel.max.x = mpu->accel.x;
  calaccel.min.y = calaccel.max.y = mpu->accel.y;
  calaccel.min.z = calaccel.max.z = mpu->accel.z;
  calaccel.sum.x = calaccel.sum.y = calaccel.sum.z = 0;

  /* capture gyroscope rate min, max, sum */
  calgyro.min.x = calgyro.max.x = mpu->gyro.x;
  calgyro.min.y = calgyro.max.y = mpu->gyro.y;
  calgyro.min.z = calgyro.max.z = mpu->gyro.z;
  calgyro.sum.x = calgyro.sum.y = calgyro.sum.z = 0;

  nsamples = 0;
  ag_cal_rdy = false;
}

// void ag_min_max_updt (min_max_sum *accel, min_max_sum *gyro, mpu_t *mpu)
void ag_min_max_updt_samples (mpu_t *mpu, uint32_t samples)
{
  if (!ag_cal_rdy) {
    /* increment accel and gyro sums (subtract gravity from Z-accel) */
    calaccel.sum.x += mpu->accel.x;
    calaccel.sum.y += mpu->accel.y;
    calaccel.sum.z += mpu->accel.z - 1;

    calgyro.sum.x += mpu->gyro.x;
    calgyro.sum.y += mpu->gyro.y;
    calgyro.sum.z += mpu->gyro.z;

    /* capture acceleration min / max */
    if (mpu->accel.x < calaccel.min.x) { calaccel.min.x = mpu->accel.x; }
    if (mpu->accel.x > calaccel.max.x) { calaccel.max.x = mpu->accel.x; }
    if (mpu->accel.y < calaccel.min.y) { calaccel.min.y = mpu->accel.y; }
    if (mpu->accel.y > calaccel.max.y) { calaccel.max.y = mpu->accel.y; }
    if (mpu->accel.z < calaccel.min.z) { calaccel.min.z = mpu->accel.z; }
    if (mpu->accel.z > calaccel.max.z) { calaccel.max.z = mpu->accel.z; }

    /* capture gyroscope rate min / max */
    if (mpu->gyro.x < calgyro.min.x) { calgyro.min.x = mpu->gyro.x; }
    if (mpu->gyro.x > calgyro.max.x) { calgyro.max.x = mpu->gyro.x; }
    if (mpu->gyro.y < calgyro.min.y) { calgyro.min.y = mpu->gyro.y; }
    if (mpu->gyro.y > calgyro.max.y) { calgyro.max.y = mpu->gyro.y; }
    if (mpu->gyro.z < calgyro.min.z) { calgyro.min.z = mpu->gyro.z; }
    if (mpu->gyro.z > calgyro.max.z) { calgyro.max.z = mpu->gyro.z; }

    nsamples += 1;    /* increment nsamples */

    if (nsamples == samples) {
      /* calculate accel axis adjustments */
      accelaxis.x = calaccel.sum.x / nsamples;
      accelaxis.y = calaccel.sum.y / nsamples;
      accelaxis.z = calaccel.sum.z / nsamples;

      /* calculate gyro axis adjustments */
      gyroaxis.x = calgyro.sum.x / nsamples;
      gyroaxis.y = calgyro.sum.y / nsamples;
      gyroaxis.z = calgyro.sum.z / nsamples;

      /* set calibration ready flag */
      ag_cal_rdy = true;
    }
  }
}

void ag_min_max_updt (mpu_t *mpu)
{
  ag_min_max_updt_samples (mpu, CALSAMPLES);
}

/**
 *  get current gyro offsets
 */
bool get_gyro_offset (uint8_t *buf, mpu_t *mpu)
{
  /* read values from hardware registers */
  if (i2c_read_reg (ADDR, MPU_XG_OFFSET_H, buf, 6) != 6) {
    return false;
  }

  /* update struct trim values */
  mpu->gyro_offset.x = (buf[0] << 8) | buf[1];
  mpu->gyro_offset.y = (buf[2] << 8) | buf[3];
  mpu->gyro_offset.z = (buf[4] << 8) | buf[5];

  return true;
}

void compute_gyro_offset (mpu_t *mpu, vect_float_t *axis)
{
  int16_t xadj = (int16_t)(axis->x / GYROSTEP),  /* compute LSB offset steps */
          yadj = (int16_t)(axis->y / GYROSTEP),
          zadj = (int16_t)(axis->z / GYROSTEP);

  mpu->gyro_offset.x -= xadj;
  mpu->gyro_offset.y -= yadj;
  mpu->gyro_offset.z -= zadj;
}

/**
 *  set gyro offset with values in buf (reset: all 0x0)
 */
bool set_gyro_offset (uint8_t *buf, mpu_t *mpu)
{
  /* write new values to hardware registers */
  if (i2c_write_reg (mpu->fd, MPU_XG_OFFSET_H, buf, 6) != 6) {
    return false;
  }

  /* update struct values with new values set */
  mpu->gyro_offset.x = (buf[0] << 8) | buf[1];
  mpu->gyro_offset.y = (buf[2] << 8) | buf[3];
  mpu->gyro_offset.z = (buf[4] << 8) | buf[5];

  return true;
}

/**
 *  get current trim values, compute new values from axis, set new trim values.
 */
bool set_gyro_trim (mpu_t *mpu, vect_float_t *axis)
{
  uint8_t buf[RWBUFSZ];

  if (!get_gyro_offset (buf, mpu)) {
    return false;
  }

  compute_gyro_offset (mpu, axis);

  buf[0] = (mpu->gyro_offset.x >> 8) & 0xff;
  buf[1] = mpu->gyro_offset.x & 0xff;
  buf[2] = (mpu->gyro_offset.y >> 8) & 0xff;
  buf[3] = mpu->gyro_offset.y & 0xff;
  buf[4] = (mpu->gyro_offset.z >> 8) & 0xff;
  buf[5] = mpu->gyro_offset.z & 0xff;

  if (!set_gyro_offset (buf, mpu)) {
    return false;
  }

  return true;
}


/**
 *  get current accel offsets
 */
bool get_accel_offset (uint8_t *buf, mpu_t *mpu)
{
  uint8_t mask0 = 0xfe;   /* mask bit-0 of low byte */

  /* read values from hardware registers */
  if (i2c_read_reg (ADDR, MPU_XA_OFFSET_H, buf, 2) != 2) {
    return false;
  }

  temp_comp_bit[0] = buf[1] & 0x1;    /* save temperature compensation bit */
  buf[1] &= mask0;                    /* mask bit-0 */

  if (i2c_read_reg (ADDR, MPU_YA_OFFSET_H, buf + 2, 2) != 2) {
    return false;
  }

  temp_comp_bit[1] = buf[3] & 0x1;    /* save temperature compensation bit */
  buf[3] &= mask0;                    /* mask bit-0 */

  if (i2c_read_reg (ADDR, MPU_ZA_OFFSET_H, buf + 4, 2) != 2) {
    return false;
  }

  temp_comp_bit[2] = buf[5] & 0x1;    /* save temperature compensation bit */
  buf[5] &= mask0;                    /* mask bit-0 */

  mpu->accel_offset.x = ((int16_t)buf[0] << 8) | buf[1];
  mpu->accel_offset.y = ((int16_t)buf[2] << 8) | buf[3];
  mpu->accel_offset.z = ((int16_t)buf[4] << 8) | buf[5];

  return true;
}

/**
 *  compute new acceleration offsets given average axis offsets
 */
void compute_accel_offset (uint8_t *buf, mpu_t *mpu, vect_float_t *axis)
{
  int16_t xadj = (int16_t)(axis->x / ACCELSTEP),  /* compute LSB offset steps */
          yadj = (int16_t)(axis->y / ACCELSTEP),
          zadj = (int16_t)(axis->z / ACCELSTEP),
          xtrim = mpu->accel_offset.x >> 1,              /* shift raw trim values */
          ytrim = mpu->accel_offset.y >> 1,
          ztrim = mpu->accel_offset.z >> 1;

#ifdef DEBUG
  printf ("\nincoming trim and adjustments:\n"
          "  accel_offset.x : %hd\n"
          "  accel_offset.y : %hd\n"
          "  accel_offset.z : %hd\n"
          "  xadj    : %hd\n"
          "  yadj    : %hd\n"
          "  zadj    : %hd\n"
          "  xtrim   : %hd\n"
          "  ytrim   : %hd\n"
          "  ztrim   : %hd\n",
          mpu->accel_offset.x, mpu->accel_offset.y, mpu->accel_offset.z,
          xadj, yadj, zadj, xtrim, ytrim, ztrim);
#endif

  xtrim -= xadj;    /* apply adjustments to shifted 15-bit values */
  ytrim -= yadj;
  ztrim -= zadj;

  /* shift to raw register format and zero temperature compensation bit */
  mpu->accel_offset.x = xtrim << 1;
  mpu->accel_offset.y = ytrim << 1;
  mpu->accel_offset.z = ztrim << 1;

  /*
  mpu->accel_offset.x = (xtrim << 1) & 0xfe;
  mpu->accel_offset.y = (ytrim << 1) & 0xfe;
  mpu->accel_offset.z = (ztrim << 1) & 0xfe;
  */
  buf[0] = (mpu->accel_offset.x >> 8) & 0xff;
  buf[1] = mpu->accel_offset.x & 0xff;
  buf[2] = (mpu->accel_offset.y >> 8) & 0xff;
  buf[3] = mpu->accel_offset.y & 0xff;
  buf[4] = (mpu->accel_offset.z >> 8) & 0xff;
  buf[5] = mpu->accel_offset.z & 0xff;

#ifdef DEBUG
  printf ("\noutgoing trim and buffer:\n"
          "  accel_offset.x : %hd\n"
          "  accel_offset.y : %hd\n"
          "  accel_offset.z : %hd\n"
          "  buf[0]  : %hd\n"
          "  buf[1]  : %hhu\n"
          "  buf[2]  : %hhu\n"
          "  buf[3]  : %hhu\n"
          "  buf[4]  : %hhu\n"
          "  buf[5]  : %hhu\n",
          mpu->accel_offset.x, mpu->accel_offset.y, mpu->accel_offset.z,
          buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
#endif

}

/**
 *  set accel offset with values in buf as change to factory trim value
 *  (NOTE: must restore temperature compensation bit [bit-0] of each value
 *         removed and saved in get_accel_offset)
 */
bool set_accel_offset (uint8_t *buf, mpu_t *mpu)
{
  uint8_t mask0 = 0xfe;   /* mask bit-0 of low byte */

  /* restore temperature compensation bit-0 */
  buf[1] = (buf[1] & mask0) | temp_comp_bit[0];

  /* write accel offset x value to hardware register */
  if (i2c_write_reg (mpu->fd, MPU_XA_OFFSET_H, buf, 2) != 2) {
    return false;
  }

  /* restore temperature compensation bit-0 */
  buf[3] = (buf[3] & mask0) | temp_comp_bit[1];

  /* write accel offset y value to hardware register */
  if (i2c_write_reg (mpu->fd, MPU_YA_OFFSET_H, buf + 2, 2) != 2) {
    return false;
  }

  /* restore temperature compensation bit-0 */
  buf[5] = (buf[5] & mask0) | temp_comp_bit[2];

  /* write accel offset z value to hardware register */
  if (i2c_write_reg (mpu->fd, MPU_ZA_OFFSET_H, buf + 4, 2) != 2) {
    return false;
  }

  /* update struct values with new raw values that contain temperature
   * compensation bit. This is different than stored trim values.
   * TODO make same as trim if an issue.
   * right-shift only in compute function.
   *
   * the assignment here duplicates assignment in compute function, but
   * allows set function to be called independent of compute.
   */
  mpu->accel_offset.x = (((int16_t)buf[0] << 8) | buf[1]); // >> 1;
  mpu->accel_offset.y = (((int16_t)buf[2] << 8) | buf[3]); // >> 1;
  mpu->accel_offset.z = (((int16_t)buf[4] << 8) | buf[5]); // >> 1;

  return true;
}

/**
 *  get current trim values, compute new values from axis, set new trim values.
 */
bool set_accel_trim (mpu_t *mpu, vect_float_t *axis)
{
  uint8_t buf[RWBUFSZ];

  if (!get_accel_offset (buf, mpu)) {
    return false;
  }

  compute_accel_offset (buf, mpu, axis);

  if (!set_accel_offset (buf, mpu)) {
    return false;
  }

  return true;
}

/**
 *  accel and gyro auto-calibration
 */
bool ag_bias_autocal (mpu_t *mpu)
{
  if (calcount < CALROUNDS) {
    /* set gyro trim / offsets */
    if (!set_gyro_trim (mpu, &gyroaxis)) {
      return false;
    }

    /* set accel trim / offsets */
    if (!set_accel_trim (mpu, &accelaxis)) {
      return false;
    }

    ag_min_max_init (mpu);    /* reinitialize calbration vars */

    if ((calcount += 1) == CALROUNDS) {
      autocalibrated = true;
    }
  }

  return true;
}

/* autocalibrate function to be called from mpu read signal handler
 * automatically deactivates when autocalibration complete
 */
void autocalibrate (mpu_t *mpu)
{
  ag_min_max_updt (mpu);       /* update sum for average each mpu read */

  if (ag_cal_rdy) {             /* if autocal ready -- do it */
    ag_bias_autocal (mpu);
  }
}

uint8_t get_calcount (void)
{
  return calcount;
}

void prn_min_max_values (void)
{
  printf ("\nmin / max:\n"
          "  acc:  (% 6.2f/% 6.2f % 6.2f)    "
                  "(% 6.2f/% 6.2f % 6.2f)    "
                  "(% 6.2f/% 6.2f % 6.2f)\n"
          "  gyr:  (% 6.2f/% 6.2f % 6.2f)    "
                  "(% 6.2f/% 6.2f % 6.2f)    "
                  "(% 6.2f/% 6.2f % 6.2f)\n",
          calaccel.min.x, calaccel.max.x, accelaxis.x,
          calaccel.min.y, calaccel.max.y, accelaxis.y,
          calaccel.min.z, calaccel.max.z, accelaxis.z,
          calgyro.min.x, calgyro.max.x, gyroaxis.x,
          calgyro.min.y, calgyro.max.y, gyroaxis.y,
          calgyro.min.z, calgyro.max.z, gyroaxis.z);
}

void prn_autocal_min_max_avg (void)
{
  printf ("\navg accel axis error (g):\n"
          "  x : % .4f\n"
          "  y : % .4f\n"
          "  z : % .4f\n"
          "avg gyro axis error (deg/s):\n"
          "  x : % .4f\n"
          "  y : % .4f\n"
          "  z : % .4f\n",
          accelaxis.x, accelaxis.y, accelaxis.z,
          gyroaxis.x, gyroaxis.y, gyroaxis.z);
}

/******************************************************/
uint8_t gbidx, packet_count;

uint16_t fifo_count;
/******************************************************/

/**
 *  get gyro and accel bias works, but the problem is the sample is taken
 *  over 40ms which gives only a snapshot of the range of noise present on
 *  the accel and gyro values. to work this would need to be applied 4 times
 *  at 1/4 second intervals over an entire second. raw values without
 *  gyro_sens and accel_sens divided out provide best base to apply from.
 *  don't forget to add code to clear gyro and accel offset values before
 *  each test.
 */
bool get_ga_bias (mpu_t *mpu)
{
  uint8_t accel_sens = 1,
          data[FIFO_PKT_LEN] = {0},
          fifo[FIFOSZ] = {0},
          i = 0;

  uint16_t fifoidx = 0,
           gyro_sens = 1;

//   uint16_t fifo_count,
//            fifoidx;

  data[0] = 0x01;
  data[1] = 0;

  /* clock select - Auto select best available */
  if (i2c_write_reg (ADDR, MPU_PWR_MGMT_1, data, 2) != 2 /* + 1 */) {
    return false;
  }
#ifdef RPIPICO
  sleep_ms (200);
#else
  usleep (200000);
#endif
  gbidx++;  // 1

  data[0] = 0;  /* reset for all except Reg. 107 PWR_MGMT_1 and Reg. 117 WHO_AM_I */

  /* interrupt enable - reset */
  if (i2c_write_reg (ADDR, MPU_INT_ENABLE, data, 1) != 1 /* + 1 */) {
    return false;
  }
  gbidx++;  // 2
  /* FIFO enable - reset */
  if (i2c_write_reg (ADDR, MPU_FIFO_EN, data, 1) != 1 /* + 1 */) {
    return false;
  }
  gbidx++;  // 3
  /* power management 1 - clock select 20MHz oscillator */
  if (i2c_write_reg (ADDR, MPU_PWR_MGMT_1, data, 1) != 1 /* + 1 */) {
    return false;
  }
  gbidx++;  // 4
  /* I2C master control - reset */
  if (i2c_write_reg (ADDR, MPU_I2C_MST_CTRL, data, 1) != 1 /* + 1 */) {
    return false;
  }
  gbidx++;  // 5
  /* user control - reset */
  if (i2c_write_reg (ADDR, MPU_USER_CTRL, data, 1) != 1 /* + 1 */) {
    return false;
  }
  gbidx++;  // 6

  /* FIFO - reset */
  data[0] = MPU_USR_FIFO_RESET_BIT | MPU_USR_DMP_RESET_BIT;
  if (i2c_write_reg (ADDR, MPU_USER_CTRL, data, 1) != 1 /* + 1 */) {
    return false;
  }
#ifdef RPIPICO
  sleep_ms (15);
#else
  usleep (15000);
#endif
  gbidx++;  // 7

  /* configure bandwidth, clock divider and FS select per self-test,
   * bandwidth 92Hz, clock divider 0, FS_sel gyro 250 dps, accel 2g.
   * self-test enable bits NOT set.
   */
  data[0] = MPU_DLPF_BW_98;
  if (i2c_write_reg (ADDR, MPU_CONFIG, data, 1) != 1 /* + 1 */) {
    return false;
  }
  gbidx++;  // 8
  data[0] = 0x00;
  if (i2c_write_reg (ADDR, MPU_SMPLRT_DIV, data, 1) != 1 /* + 1 */) {
    return false;
  }
  gbidx++;  // 9
  data[0] = MPU_GYRO_FS_250;
  if (i2c_write_reg (ADDR, MPU_GYRO_CONFIG, data, 1) != 1 /* + 1 */) {
    return false;
  }
  gbidx++;  // 10
  data[0] = MPU_ACCEL_FS_2;
  if (i2c_write_reg (ADDR, MPU_ACCEL_CONFIG, data, 1) != 1 /* + 1 */) {
    return false;
  }
  gbidx++;  // 11

  /* Fill FIFO for test.wait_ms milliseconds. */
  /* enable FIFO */
  data[0] = 1u << MPU_USR_FIFO_EN_BIT;
  if (i2c_write_reg (ADDR, MPU_USER_CTRL, data, 1) != 1 /* + 1 */) {
    return false;
  }
  gbidx++;  // 12
  /* enable write of X,Y,Z H/L for gyro & acces to FIFO */
  data[0] = MPU_XYZG_FIFO_EN | MPU_ACCEL_FIFO_EN;
  if (i2c_write_reg (ADDR, MPU_FIFO_EN, data, 1) != 1 /* + 1 */) {
    return false;
  }

#ifdef RPIPICO
  sleep_ms (40);      /* 40 milliseconds. wait to fill FIFO (adjust as needed)
                       * currently fill 480 bytes in FIFO / 40 packets
                       */
#else
  // usleep (500000);
  usleep (40000);
  // usleep (38000);
#endif
  gbidx++;  // 13

  /* disable FIFO */
  data[0] = 0;
  if (i2c_write_reg (ADDR, MPU_FIFO_EN, data, 1) != 1 /* + 1 */) {
    return false;
  }
  gbidx++;  // 14

  /* read FIFO overflow register */
  if (i2c_read_reg (ADDR, MPU_INT_STATUS, data, 1) != 1) {
    return false;
  }
  gbidx++;  // 15
  /* check if FIFO overflow occurred */
  if (data[0] & (1u << MPU_INT_FIFO_OFLOW_BIT)) {
    mpu->fifo_overflow = *data & (1u << MPU_INT_FIFO_OFLOW_BIT);
    /* not an error if fifo_overflow occurs. it simply means earlier
     * values are discarded and replaced by newer values.
     */
    // return false;
  }
  gbidx++;  // 16

  /* read FIFO count registers FIFO_COUNTH 114 (5-bits), and
   * FIFO_COUNTl 115 (8-bits).
   */
  if (i2c_read_reg (ADDR, MPU_FIFO_COUNTH, data, 2) != 2) {
    return false;
  }
  gbidx++;  // 17
  fifo_count = ((uint16_t)(data[0] & 0x1f) << 8) | data[1];

  fifo_count_rpt = fifo_count;  /* temp global */

  if (!fifo_count) {  /* validate bytes in FIFO */
    return false;
  }
  gbidx++;  // 18

  /* one packet is one set of values written to FIFO. values are written
   * in the to FIFO in register-order, lowest to highest. for gyro and
   * accel the order is AccelX_H, AccelX_L, ... GyroZ_L. (12-bytes)
   */
  packet_count = fifo_count / FIFO_PKT_LEN;

  /* loop reading each set of values */
  for (i = 0; i < packet_count; i++) {
    uint8_t j = 0;

    /* read packet (accel & gyro X,Y,Z H/L values from FIFO */
    if (i2c_read_reg (ADDR, MPU_FIFO_R_W, data, FIFO_PKT_LEN) !=
        FIFO_PKT_LEN) {
      gbidx++;  // 19
      return false;
    }

    /* fill temporary extern array for output FIXME - remove fifo[] */
    for (j = 0; j < FIFO_PKT_LEN; j++) {
      fifo[fifoidx++] = data[j];
    }
    (void)fifo;

    /* compute acceleration from register values in FIFO, add to sum.
     * cast to signed from unsigned is implementation defined
     * behavior in both TI CCS and GCC
     */
    accel_sum[0] += (((int16_t)(data[0] << 8) | data[1]) - 16384);
    // accel_sum[0] += (int16_t)(data[0] << 8) | data[1];
    accel_sum[1] += (int16_t)(data[2] << 8) | data[3];
    /* remove gravity from a_z axis value */
    // accel_sum[2] += (((int16_t)(data[4] << 8) | data[5]) - 16384);
    accel_sum[2] += (int16_t)(data[4] << 8) | data[5];

    /* compute gyro rate from register values in FIFO, add to sum.
     * (same note)
     */
    gyro_sum[0] += (int16_t)(data[6] << 8) | data[7];
    gyro_sum[1] += (int16_t)(data[8] << 8) | data[9];
    gyro_sum[2] += (int16_t)(data[10] << 8) | data[11];

  }

  /* compute average acceleration and gyro rate values
   * and add offsets to current values. Otherwise zero gyro offsets
   * but do not zero factory trim values for accel axis. In that case
   * you can add to (subtract from) current accel trim values and
   * replace zeroed gyro offsets.
   *
   * NOTE: values are unreliable over short FIFO fill period of 40ms.
   *       instead use manual calcuations over serveral seconds for
   *       most accurate results.
   */
  mpu->accel_offset.x -= accel_sum[0] / (accel_sens * packet_count);
  mpu->accel_offset.y -= accel_sum[1] / (accel_sens * packet_count);
  mpu->accel_offset.z -= accel_sum[2] / (accel_sens * packet_count);

  mpu->gyro_offset.x -= gyro_sum[0] / (gyro_sens * packet_count);
  mpu->gyro_offset.y -= gyro_sum[1] / (gyro_sens * packet_count);
  mpu->gyro_offset.z -= gyro_sum[2] / (gyro_sens * packet_count);

  return true;
}

/** determine hard-iron and soft-iron offsets
 *  get average and scale of each axis and  apply to raw values.
 */
// #define MAG_CAL_SMP 512
#define MAG_CAL_SMP 4096
/*********** temp variables ****************/
bool mag_calibrated;

uint16_t  mag_count;

int16_t   mag_bias_raw[3] = {0},
          mag_scale_raw[3] = {0};
/*******************************************/

void get_mag_bias_scale (mpu_t *mpu)
{
  int16_t data[3] = {0},
          mag_max[3],
          mag_min[3]/*,
          mag_bias_raw[3] = {0},
          mag_scale_raw[3] = {0} */
          ;
  float mag_sens = MAG_SENS / 32768.;   /* convert to raw to uT */

  char c = '.';

  get_raw_mag (mpu, data);              /* read raw data */

  mag_max[0] = mag_min[0] = data[0];    /* initialize max/min */
  mag_max[1] = mag_min[1] = data[1];
  mag_max[2] = mag_min[2] = data[2];

  /* TODO FIXME - move all stdio.h functions to separate source */

  /* loop MAG_CAL_SMP times getting samples.
   *
   * NOTE: you must be moving the sensor in figure-8 patterns
   *       in order to get full sensor range
   */
  for (mag_count = 0; mag_count < MAG_CAL_SMP; mag_count++) {
    get_raw_mag (mpu, data);            /* read raw data */

    if (mag_count && mag_count % 64 == 0) {
      putchar ('\r');
      c = c == '.' ? ' ' : '.';
    }

    if (data[0] > mag_max[0]) { mag_max[0] = data[0]; }
    if (data[0] < mag_min[0]) { mag_min[0] = data[0]; }

    if (data[1] > mag_max[1]) { mag_max[1] = data[1]; }
    if (data[1] < mag_min[1]) { mag_min[1] = data[1]; }

    if (data[2] > mag_max[2]) { mag_max[2] = data[2]; }
    if (data[2] < mag_min[2]) { mag_min[2] = data[2]; }

    putchar (c);
    fflush (stdout);
#ifdef RPIPICO
    sleep_ms (10);
#else
    usleep (10000);
#endif
  }
  fputs ("\r\033[0K\033[1A", stdout);
  fflush (stdout);

  /* get average bias for each axis
   * (remove mag_bias_raw after testing
   *  combine all in mpu->mag_bias.x)
   */
  mag_bias_raw[0] = (mag_max[0] + mag_min[0]) / 2;
  mag_bias_raw[1] = (mag_max[1] + mag_min[1]) / 2;
  mag_bias_raw[2] = (mag_max[2] + mag_min[2]) / 2;

  /* convert mag bias from raw values to uT */
  mpu->mag_bias.x = mag_bias_raw[0] * mag_sens;
  mpu->mag_bias.y = mag_bias_raw[1] * mag_sens;
  mpu->mag_bias.z = mag_bias_raw[2] * mag_sens;

  /* get average scale for each axis */
  mag_scale_raw[0] = (mag_max[0] - mag_min[0]) / 2;
  mag_scale_raw[1] = (mag_max[1] - mag_min[1]) / 2;
  mag_scale_raw[2] = (mag_max[2] - mag_min[2]) / 2;

  /* compute average scale adjustment */
  float scale_avg = (mag_scale_raw[0] +
                     mag_scale_raw[1] +
                     mag_scale_raw[2]) / 3.;

  /* normalize scale reading to fit in standard range */
  mpu->mag_scale.x = scale_avg / mag_scale_raw[0];
  mpu->mag_scale.y = scale_avg / mag_scale_raw[1];
  mpu->mag_scale.z = scale_avg / mag_scale_raw[2];
//   mpu->mag_scale.x = mag_scale_raw[0] / scale_avg;
//   mpu->mag_scale.y = mag_scale_raw[1] / scale_avg;
//   mpu->mag_scale.z = mag_scale_raw[2] / scale_avg;

  mag_calibrated = true;
}
