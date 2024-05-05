/**
 *  Inversense MPU API Example
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <stdlib.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <unistd.h>

#include <sys/select.h>
#include <time.h>

#include "mpu-constants.h"
#include "mpu.h"
#include "mpu-console.h"
#include "mpu-calibrate.h"
#include "mpu-caldata.h"

#include "itimer.h"
#include "i2c-smbus.h"

#define SAMPLES   500

/* globals for real-time signal numbers
 * NOTE: can be removed with it0, it1 declared global now
 */
static volatile uint8_t sigrt_mpu_sample = 0;
static volatile uint8_t sigrt_mpu_output = 0;

uint8_t tmpbuf[RWBUFSZ] = {0};          /* buffer for register values */

static volatile bool mpu_data_rdy;      /* data and output ready flags */
static volatile bool mpu_output_rdy;

/* extern variables from mpu_calibrate.c */
extern uint32_t nsamples;               /* current number of calibration samples */

extern volatile bool autocalibrated;    /* auto-calibrate successfully completed */

itimer it0, it1;        /* timer struct instances for read and output timers */

/* typedef for signal handler functions */
typedef void (*sighdlr)(int, siginfo_t*, void*);

/* id set from factory self-test fingerprint */
mpu_t mpu = { .addr = MPU_ADDRESS_AD0_LOW };

/* accommodate both milkv-duo and raspberry pi on default I2C pins */
#ifdef MILKV
const char *i2cdev = "/dev/i2c-0";
#else
const char *i2cdev = "/dev/i2c-1";
#endif

/**
 *  signal handler for mpu read itimer signal (currently 0.01s)
 */
static void sighdlr_mpu_sample (int sig, siginfo_t *si, void *uc)
{
  if (sig == sigrt_mpu_sample && SIGRTMIN <= sig && sig <= SIGRTMAX) {
    /* if accel, gyro and temp data read */
    if (get_acc_gyro_tempc (&mpu)) {
      mpu_data_rdy = true;              /* set data read flag true */
      if (!autocalibrated) {            /* if autocal not complete */
        autocalibrate (&mpu);           /* call autocalibrate func */
      }
    }
  }

  (void)si;     /* suppress -Wunused warnings */
  (void)uc;
}

/**
 *  signal handler for data output itimer signal (currently 0.2s)
 */
static void sighdlr_mpu_output (int sig, siginfo_t *si, void *uc)
{
  if (sig == sigrt_mpu_output && SIGRTMIN <= sig && sig <= SIGRTMAX) {
    /* if data values are ready */
    if (mpu_data_rdy) {
      mpu_output_rdy = true;            /* set data output flag true */
      mpu_data_rdy = false;             /* set data ready flag false */
    }
  }

  (void)si;     /* suppress -Wunused warnings */
  (void)uc;
}

/**
 *  pselect with second, nanosecond timout controlling
 *  the update of values printed to screen and indicates
 *  whether input is available on stdin ('q' quit, etc..)
 */
int pselect_timer (unsigned sec, unsigned nsec)
{
  fd_set set;
  int res;
  struct timespec ts = {  .tv_sec = sec,
                          .tv_nsec = nsec  };

  /* Initialize the file descriptor set. */
  FD_ZERO (&set);
  FD_SET (0, &set);

  /* watch STDIN_FILENO with timeout_sec timeout */
  // do {
    res = pselect (1, &set, NULL, NULL, &ts, NULL);
  // } while (res == -1 && errno == EINTR);

  return res;
}

/* simple empty-stdin function, c must be initialized */
void empty_stdin (int c)
{
  while (c != '\n' || c == EOF) {
    c = getchar();
  }
}

/* initializes and configures MPU accel and gyro functions */
static int mpu_start (mpu_t *_mpu, const char *_i2cdev, uint8_t mpu_i2c_addr,
                      uint16_t gyrofs, uint8_t accelfs, uint8_t dlpf)
{
  int fdi2c;

  /* initialize required members of C-struct mpu since using
   * implicit constructor to create temporary zeros all and the
   * normal brace initialization generates broken warnings for
   * missing initializer (-Wmissing-field-initializers)
   */
  _mpu->addr =  mpu_i2c_addr;
  _mpu->id = 0;

  /* initialize i2c device obtaining i2c file descriptor */
  if ((fdi2c = i2c_init_device (_i2cdev, mpu_i2c_addr)) < 0) {
    fprintf (stderr, "error: IOCTL I2C Setup failed for device '%s' (return: %d)\n",
              _i2cdev, fdi2c);
    return 1;
  }
  _mpu->fd = fdi2c;

  printf ("I2C Setup succeeded for device '%s', fd: %d\n", _i2cdev, _mpu->fd);

  /* get factory self-test and use as fingerprint for mpu-chip */
  /* MOVED into mpu.c mpu_init() */
  // if (get_factory_st (_mpu)) {
  //   get_id_from_factory_st (_mpu);
  // }
  // else {
  //   puts ("get_factory_st(&mpu) - failed");
  // }

  /* if gyro, accel and DLPF init values provided */
  if (250 <= gyrofs && gyrofs <= 2000 &&
      2 <= accelfs && accelfs <= 16 &&
      dlpf <= 7) {
    if (!mpu_init (_mpu, gyrofs, accelfs, dlpf)) {
      fputs ("error: mpu_init - failed.\n", stderr);
      return 1;
    }
  }
  else {  /* otherwise default initialize */
    if (!mpu_init_default (_mpu)) {
      fputs ("error: mpu_init_default - failed\n", stderr);
      return 1;
    }
  }
  usleep (50000);

  puts ("mpu_init_default - succeeded - mpu9250 identified and initialized");

#ifdef MPU9250
  if (!ak8963_initialize (_mpu, _i2cdev)) {
    puts ("ak8963_initialize - failed");
    printf ("I2C Setup failed for ak8963 device '%s', fd: %d\n",
            _i2cdev, _mpu->magfd);
    return 1;
  }
  puts ("ak8963_initialize - succeeded - ak8963 identified and initialized");
  printf ("I2C Setup succeeded for ak8963 device '%s', fd: %d\n\n",
          _i2cdev, _mpu->magfd);

#ifdef MAGBIAS
  /* setting mag bias and scale causes them to be automaically applied
   * in get_mag() when mag_bias.x is non-zero.
   *
   * NOTE: will need to move MAGBIAS and Accel/Gyro axis scale into mpu_calibrate
   *       based on MPU chip ID and apply from saved values as all require
   *       the mpu be moved through the full range of all axis to compute scale
   *       along with soft/hard iron biases -- which is impractical to do on
   *       every mpu start. Values can be checked periodically.
   */
  // mpu.mag_bias.x = -2.92;    /* hard iron bias */
  // mpu.mag_bias.y = 19.94;
  // mpu.mag_bias.z = -7.12;

  // mpu.mag_scale.x = 1.;      /* soft iron bias ellipsoid correction */
  // mpu.mag_scale.y = 1.09;
  // mpu.mag_scale.z = 0.93;

  set_usr_mag_bias_scale (&mpu);

#endif

#endif

  return 0;
}

/* TODO add mpu_t parameter and make it0, it1 part of mpu_t
 * using tmp globals
 */
// itimer it0, it1;

/* configures and starts timers for mpu read and output after delay of ns_to_start */
static int timers_start (uint64_t ns_to_start, uint64_t ns_mpu_read, uint64_t ns_mpu_output,
                        sighdlr mpu_samplefn, sighdlr mpu_outputfn)
{
  /* initialize interval timers, get real-time signal no.
   *  it0 - mpu_read:   100 times per-second
   *  it1 - oled_write:   5 times per-second
   */
  it0 = itimer_create_timer (ns_to_start, ns_mpu_read, mpu_samplefn);
  it1 = itimer_create_timer (ns_to_start, ns_mpu_output, mpu_outputfn);

  /* validate realtime signal no. returned */
  if (it0.signo < SIGRTMIN || SIGRTMAX < it0.signo) {
    fputs ("error: itimer_create_timer - mpu_read\n", stderr);
    return 1;
  }

  /* validate realtime signal no. returned */
  if (it1.signo < SIGRTMIN || SIGRTMAX < it1.signo) {
    fputs ("error: itimer_create_timer - oled-write\n", stderr);
    return 1;
  }

  sigrt_mpu_sample = it0.signo;     /* update global values */
  sigrt_mpu_output = it1.signo;

  if (itimer_start_timer (&it0) == 0) {   /* start reading mpu */
    fputs ("error: itimer_start_timer it0 - mpu read\n", stderr);
    return 1;
  }

  if (itimer_start_timer (&it1) == 0) {   /* start mpu output */
    fputs ("error: itimer_start_timer it1 - oled_write\n", stderr);
    return 1;
  }

  return 0;
}

/* output and information function, no other critical use */
void get_st_gyro_accel (void)
{
  // if (get_factory_st (&mpu)) {
    printf ("Factory Self-Text Values:\n"
            "  acc:  %3hhu  %3hhu  %3hhu\n"
            "  gyr:  %3hhu  %3hhu  %3hhu\n",
            mpu.accel_st.x, mpu.accel_st.y, mpu.accel_st.z,
            mpu.gyro_st.x, mpu.gyro_st.y, mpu.gyro_st.z);
  // }
  // else {
  //   puts ("get_factory_st(&mpu) - failed");
  // }

  // uint8_t tmpid;
  // if (mpu.id == (tmpid = get_id_from_factory_st (&mpu))) {
    printf ("\nmpu.id (%hhu) matches self-test fingerprint\n\n", mpu.id);
  // }
  // else if (tmpid) {
  //   printf ("warning: mismatch between mpu.id (%hhu) "
  //           "and self-test fingerprint (%hhu) or id not set\n", mpu.id, tmpid);
  // }

  if (get_gyro_offset (tmpbuf, &mpu)) {
    printf ("Current gyro offset values:\n"
            "  xg (h/l):  0x%02hhx / 0x%02hhx\n"
            "  yg (h/l):  0x%02hhx / 0x%02hhx\n"
            "  zg (h/l):  0x%02hhx / 0x%02hhx\n",
            tmpbuf[0], tmpbuf[1], tmpbuf[2], tmpbuf[3], tmpbuf[4], tmpbuf[5]);
  }
  else {
    puts ("get_gyro_offset (tmpbuf, &mpu) - failed");
  }

  if (get_accel_offset (tmpbuf, &mpu)) {
    printf ("\nCurrent accel offset values:\n"
            "  xa (h/l):  0x%02hhx / 0x%02hhx\n"
            "  ya (h/l):  0x%02hhx / 0x%02hhx\n"
            "  za (h/l):  0x%02hhx / 0x%02hhx\n",
            tmpbuf[0], tmpbuf[1], tmpbuf[2], tmpbuf[3], tmpbuf[4], tmpbuf[5]);

    /* update struct values with new values set */
    mpu.accel_offset.x = ((int16_t)tmpbuf[0] << 8) | tmpbuf[1];
    mpu.accel_offset.y = ((int16_t)tmpbuf[2] << 8) | tmpbuf[3];
    mpu.accel_offset.z = ((int16_t)tmpbuf[4] << 8) | tmpbuf[5];
  }
  else {
    puts ("get_accel_offset (tmpbuf, &mpu) - failed");
  }
}

/* function to change message showing when autocalibration complete */
void show_calibrated (void)
{
  printf ("\033[2A%s Output:  (\033[1;32mAUTO-Calibrated\033[0m) "
          "press Enter to quit.\033[0K\n\n", mpu.typenm);
}


int main (int argc, char **argv) {

  int samples = SAMPLES, tmp = 0;
  bool shown = false;

  /* take samples on the command line (default 500) */
  if (argc > 1 && sscanf (argv[1], "%d", &tmp) == 1 && tmp > 0) {
    samples = tmp;
  }

  /* start/initialize MPU
   *   0 - accel full-scale config (2g)
   *   0 - gyro full-scale config (250 deg/sec)
   *   0 - DLPF config value (digital low-pass filter) - see datasheet
   */
  if (mpu_start (&mpu, i2cdev, MPU_ADDRESS_AD0_LOW, 0, 0, 0) == 1) {
    return 1;
  }

  /* get_st_gyro_accel values */
  get_st_gyro_accel();

  if (tmp) {  /* if samples given as argument report no. of samples */
    printf ("\nReading %d samples from mpu.id %hhu\n", samples, mpu.id);
  }
  else {  /* otherwise reading until Enter pressed (or "q..." Enter) */
    puts ("\nReading MPU continually, Press Enter to Quit");
  }

  /* initialize interval timers
   *   .02 sec delay before timers are started
   *   .01 sec interval timer for mpu data sample rate
   *   .2  sec interval timer for display output rate
   * with signal handler functions for data sampling and display update
   */
  if (timers_start (20000000, 10000000, 200000000,
                    sighdlr_mpu_sample, sighdlr_mpu_output) == 1) {
    return 1;
  }

  /* show warning not to move until auto-calibration done, hide cursor */
  printf ("\n%s Output:  (\033[1;31mDO NOT MOVE UNTIL "
          "CALIBRATED\033[0m)\033[?25l\n\n", mpu.typenm);

  /* program loop with increment controlling infinite or no. of loops */
  for (int i = 0; i < samples; i += tmp ? 1 : 0) {
    /* data ready block (adjust usleep below if used) */
    if (mpu_data_rdy) {
    }
    /* output ready block */
    if (mpu_output_rdy) {
      show_mpu_output (&mpu);
      mpu_output_rdy = false;
      if (autocalibrated && !shown) {
        show_calibrated();
        shown = true;
      }
    }
    /* check if user input provided */
    if (pselect_timer (0, 2.e8) == 1) {
    // if (pselect_timer (0, 0) == 1) {
      int c = getchar();
      /* quit on 'q', empty-string or manual EOF (ctrl + d) (z on windows) */
      if (c == 'q' || c == '\n' || c == EOF) {
        break;
      }
      empty_stdin (c);    /* empty stdin of any remaining chars */
    }
    usleep (100000);      /* slow program loop to .1 sec */
  }
  /* skip over data values, restore cursor */
  printf ("\r\033[4B\033[?25h\n");

  puts ("\nMove MPU though ALL axis and figure-eights for mag calibration\n");

  get_mag_bias_scale (&mpu);

  prn_autocal_min_max_avg();
  prn_mpu_struct (&mpu);

  itimer_delete_timer (&it0);
  itimer_delete_timer (&it1);

  // printf ("\r\033[4B\033[?25h\n");
}
