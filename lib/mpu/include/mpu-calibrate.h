/**
 *  Inversense MPU API
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */

#ifndef MPU_CALIBRATE_H
#define MPU_CALIBRATE_H  1

#ifdef __cplusplus
extern "C" {
#endif

#ifdef RPIPICO
#include "pico/stdlib.h"
#else
#include <stdbool.h>
#endif

#include "mpu.h"

typedef struct {
  vect_float_t min, max, sum;
} min_max_sum;

/** MPU SelfTest Funcitons */
bool get_factory_st (mpu_t *mpu);
uint8_t get_id_from_factory_st (mpu_t *mpu);

/** accel & gyro min, max, sum init and update */
void ag_min_max_init (mpu_t *mpu);
void ag_min_max_updt (mpu_t *mpu);
void ag_min_max_updt_samples (mpu_t *mpu, uint32_t samples);

/** get gyro and accel offsets */
bool get_gyro_offset (uint8_t *buf, mpu_t *mpu);
bool get_accel_offset (uint8_t *buf, mpu_t *mpu);

/** set gyro and accel offsets */
bool set_gyro_offset (uint8_t *buf, mpu_t *mpu);
bool set_accel_offset (uint8_t *buf, mpu_t *mpu);

/**
 *  compute new gyro and accel offsets given average axis offsets
 */
void compute_gyro_offset (mpu_t *mpu, vect_float_t *axis);
void compute_accel_offset (uint8_t *buf, mpu_t *mpu, vect_float_t *axis);


/** functions to place MPU in test mode and write
 *  accelerometer and gyroscope values to FIFO
 *  to average offsets/biases over 40-41 sequential values.
 */
bool get_ga_bias (mpu_t *mpu);
void get_mag_bias_scale (mpu_t *mpu);

/**
 *  set current trim values, get, compute and set new trim values.
 */
bool set_gyro_trim (mpu_t *mpu, vect_float_t *axis);
bool set_accel_trim (mpu_t *mpu, vect_float_t *axis);

/**
 *  auto-calibrate gyro and accel offsets called from mpu read
 *  signal handler.
 */
bool ag_bias_autocal (mpu_t *mpu);

/* autocalibrate wrapper function to be called from mpu read signal handler
 * automatically deactivates when autocalibration complete
 */
void autocalibrate (mpu_t *mpu);

void prn_min_max_values (void);
void prn_autocal_min_max_avg (void);

uint8_t get_calcount (void);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif
