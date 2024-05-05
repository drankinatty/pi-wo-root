/**
 *  Inversense MPU API
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */

#ifndef MPU_H
#define MPU_H  1

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "itimer.h"
#include "i2c-smbus.h"

/* I2C Address for Inversense MPU6xxx / MPU925x
 *   MPU_ADDRESS_AD0_LOW        0x68
 *   MPU_ADDRESS_AD0_HIGH       0x69
 *
 * AK8963 Magnetometer on MPU925x
 *   AK8963_DEFAULT_ADDRESS     0x0C
 */

#define NAXIS                 3     /* No. axis */
#define FIFO_PKT_LEN         12     /* gyro & accel X,Y,Z H/L values */
#define RWBUFSZ              16     /* for general 16-byte buffer size */
#define FIFOSZ              512     /* MPU FIFO size (bytes) */

/* denoise limits stop bounce about zero on each axis */
#define DENOISE_LIM_ACCEL   0.1f
#define DENOISE_LIM_GYRO    0.2f

#ifdef RPIPICO
#define ADDR mpu->addr
#else
#define ADDR mpu->fd
#endif

/* vector typdefs for common types */
typedef union {
  uint8_t arr[NAXIS];
  struct { uint8_t x, y, z; };
} vect_uint8_t;

typedef union {
  uint16_t arr[NAXIS];
  struct { uint16_t x, y, z; };
} vect_uint16_t;

typedef union {
  float arr[NAXIS];
  struct { float x, y, z; };
} vect_float_t;

/**
 *  typedef for struct holding configuration and sensor data
 */
typedef struct mpu_t {

  uint8_t addr, id;               /* mpu I2C address and individual chip-ID */
#ifndef RPIPICO
  uint8_t fd, magfd;              /* file-descriptor for mpu and magnetometer */
#endif

  uint8_t type;                           /* enum of type, MPU6050, MPU9250 */
  // int8_t  sigrt_sample,                   /* timer sig no. for mpu sample */
  //         sigrt_output;                   /* timer sig no. for mpu output */
  itimer  *itimer_sample,                 /* pointers to itimer instances in */
          *itimer_output;                 /* mpu.c */

  const char *typenm;                     /* string corresponding to type */

  vect_float_t  gyro, accel, mag,         /* sensor data values */
                accel_usr_bias,           /* saved accel offsets per chip-ID */
                mag_bias, mag_scale,      /* magnetometer axis bias/scale */
                mag_asa;                  /* mag factory axis scale adjustment */

  float room_temp_offset,                 /* room temp calibration value */
        tempc;                            /* sensor temperature value */

  const uint8_t (*st_data)[NAXIS*2];      /* pointer to array of stored st values */

  vect_uint16_t accel_offset,             /* accel H/L register offsets */
                gyro_offset;              /* gyro H/L register offsets */

  int16_t gyro_sens;                      /* gyro sensitivity */

  vect_uint8_t  accel_st, gyro_st,        /* factory self-test values */
                gyro_usr_bias;            /* saved gyro offsets per chip-ID */

  uint8_t accel_sens,                     /* accel sensitivity  */
          dlpf_cfg,                       /* CFG & FS_SEL are bit */
          accel_fs_sel,                   /* full-scale sel, 0, 1, 2, ... */
          gyro_fs_sel,                    /* same for gyro */
          fifo_overflow;                  /* fifo overflow bit */

} mpu_t;


#ifdef RPIPICO
/** i2c helper functions for read and write with default values */
int i2c_read (uint8_t addr, uint8_t *dst, size_t len);
int i2c_write (uint8_t addr,  uint8_t *src, size_t len);
int i2c_read_reg (uint8_t addr, const uint8_t reg_addr, uint8_t *dst,
                  size_t len);
int i2c_write_reg (uint8_t addr, const uint8_t reg_addr, uint8_t *src,
                   size_t len);
#else
/**
 * I2C read length (len) bytes from I2C device configured with open
 * file-descriptor (fd) from device register (reg) address into
 * the storage pointed to by data.
 */
int32_t i2c_read_reg (int fd, uint8_t cmd, uint8_t *vals, uint8_t len);

/**
 * I2C write length (len) bytes from I2C device configured with open
 * file-descriptor (fd) from device register (reg) address into
 * the storage pointed to by data.
 */
int32_t i2c_write_reg (int fd, uint8_t cmd, const uint8_t *values,
                              uint8_t length);

#endif

/** reset accel, gyro and temperature sensor signal paths */
bool mpu_reset (const uint8_t mpu_addr);

/** Check whether MPU is available on the I2C BUS */
bool mpu_i2c_check (mpu_t *mpu);

/** Initialize mpu gyro with FS_SEL = 250 and accel FS_SEL = 2, 92Hz */
bool mpu_init_default (mpu_t *mpu);

/**
 *  Initialize mpu gyro and accel with values given by gyro_fs_sel and
 *  accel_fs_sel, if parameter values do not match valid FS-SEL values the
 *  corresponding gyro or accel will be initialized to the default FS_SEL
 *  values of 250 deg/sec and 2 g.
 */
bool mpu_init (mpu_t *mpu, uint16_t gyro_fs_sel_dps, uint8_t accel_fs_sel_g,
               uint8_t dlpfval);

/** Get raw acceleration values for linear and angular rate.
 *  called automatically by get_acc_gyro(), but can be used to
 *  get raw values if desired. acc and gyro are int16_t[3].
 */
bool get_raw_acc_gyro (const uint8_t addr, int16_t *acc, int16_t *gyro);

/** Convert raw linear accel and gyro values into g and deg/sec. */
bool get_acc_gyro (mpu_t *mpu);

/** Get raw acceleration values for linear and angular rate with temperature. */
bool get_raw_acc_gyro_tempc (const uint8_t addr, int16_t *acc, int16_t *gyro,
                             int16_t *tempc);

/**
 *  Convert raw linear acceleration, angular rate and temperature values into
 *  g, deg/sec and degrees C.
 */
bool get_acc_gyro_tempc (mpu_t *mpu);

/** apply limit around zero to reduce noise in data around zero */
void denoise_values (float *dest, const float *data, const float limit);

#ifdef MPU9250
/** Code specific to the AK8953 magnetometer on the MPU9250
 *
 *  NOTE: only one MPU9250 can be used at a time with magnetometer
 *        readings due to mag address being fixed at 0x0C. Unlike
 *        the mpu address that can be changed with AD0 high, there
 *        is no way to differentiate between magnetometers.
 */

/** Check whether AK8953 is available on the I2C BUS */
bool ak8963_i2c_check (void);

/** Set AK8953 magnetometer axis bias and scale to predetermined values */
void ak8963_set_bias_scale (mpu_t *mpu, const float *bias, const float *scale);

/**
 *  Initialize the AK8963 magnetometer CNTL1 by placing the magnetometer in
 *  FUSE mode to read the Axis Sensitivity Adjustment (ASA) values from
 *  FUSE_ROM, power-down the chip between mode changes and then place the
 *  magnetometer in 16-bit adc and continual measurement 2 for normal
 *  operations. Save the computed adjustments factors in asax, asay, asaz.
 *  (Page 51 & 52 of Register_Map)
 */
#ifndef RPIPICO
bool ak8963_initialize (mpu_t *mpu, const char *i2cdev);
#else
bool ak8963_initialize (mpu_t *mpu);
#endif

/**
 *  Get raw magnetometer orientation values stored in two's-compliment
 *  little-endian format.
 */
bool get_raw_mag (mpu_t *mpu, int16_t *data);

/** Convert raw magnetometer values into mx, my, mz in uT */
bool get_mag (mpu_t *mpu);

#endif

/**
 *  mpu start, timers and pselect for user input in program loop and shutdown.
 */
/* pselect in program loop to capture user input, sec should be set 0 */
int pselect_timer (unsigned sec, unsigned nsec);

/* initializes and configures MPU accel and gyro functions */
int mpu_start (mpu_t *mpu, const char *_i2cdev, uint8_t mpu_i2c_addr,
                uint16_t gyrofs, uint8_t accelfs, uint8_t dlpf);

/* configures and starts timers for mpu read and output after delay of ns_to_start */
int timers_start (uint64_t ns_to_start, uint64_t ns_mpu_read, uint64_t ns_mpu_output);

/*  mpu_stop shut down timers and close file descriptors */
int mpu_stop (mpu_t *mpu);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif
