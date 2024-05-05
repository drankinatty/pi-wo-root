/**
 *  Inversense MPU API
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */

#ifndef MPU_CONSOLE_H
#define MPU_CONSOLE_H  1

#ifdef __cplusplus
extern "C" {
#endif

#ifdef RPIPICO
#include "pico/stdlib.h"
#include "pico_mpu.h"
#else
#include "mpu.h"
#include "mpu-calibrate.h"
#endif

/** console output functions */
void show_self_test (mpu_t *mpu);

void show_ga_sens (mpu_t *mpu);
void show_ga_bias (mpu_t *mpu);

void show_raw_acc_gyro_output (int16_t *acc, int16_t *gyro);
void show_raw_mpu_output (int16_t *acc, int16_t *gyro, int16_t *mag);

void show_mpu_output (mpu_t *mpu);
void show_mpu_output_w_fusion (mpu_t *mpu,
                               const float *euler, const float *earth);

void show_mag_bias (mpu_t *mpu);
void show_mag_bias_raw (mpu_t *mpu);

void prn_mpu_struct (mpu_t *mpu);

void prn_ag_min_max_values (min_max_sum *accel, min_max_sum *gyro,
                            vect_float_t *accelaxis, vect_float_t *gyroaxis);
void prn_ag_autocal_min_max_avg (vect_float_t *accelaxis, vect_float_t *gyroaxis);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif
