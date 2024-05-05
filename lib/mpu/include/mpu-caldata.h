/**
 *  Inversense MPU API
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */

/**
 *  mpu-caldata.[ch]  hold calibration data for specific MPU chips. Each chip
 *  will have specific error due to manufacturing differences that must be
 *  calibrated to obtain accurate sensor readins. The accel and gyro scale and the
 *  magnetometer bias and scale require the mpu to be moved through the full range
 *  of each axis to compute. The mag bias and scale require continual movement
 *  through all axis (figure-eights) for several seconds.
 *
 *  This data is saved and applied manually as it is impractical to perform these
 *  calibrations on every program start. The accel and gyro bias can be
 *  auto-calibrated with the sensor left at rest for the first few seconds.
 */
#ifndef MPU_CALDATA_H
#define MPU_CALDATA_H  1

#ifdef __cplusplus
extern "C" {
#endif

#include "mpu.h"

extern const uint8_t (*mpu_st_data)[6];

int get_mpu_st_list_sz (void);

/**
 *  room temperature offset in degrees C.
 *  temperator reported is chip temperature, not room temperature, to get
 *  get close approximation, set room_temp_offset. (2.8 C * 333.87)
 */
void set_room_temp_offset (mpu_t *mpu);

/** set magnetometor bias and scale based on mpu->id from saved data */
void set_usr_mag_bias_scale (mpu_t *mpu);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif
