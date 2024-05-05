/**
 *  Inversense MPU API
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */

#include "mpu-caldata.h"

/**
 *  saved factory self-test values for each unique MPU chip.
 *  used as a fingerprint to ID which chip is attached allowing
 *  saved accel/gyro scale and magnetometer bias and scale values
 *  to be automatically applied from saved data avoiding repeated
 *  movement of MPU through the full range of all 3 axis on every
 *  program start.
 */
const uint8_t mpu_st_list[][6] = { {  93,  88, 115, 191, 198, 225 },
                                   { 104,  94, 122, 199, 207, 226 },
                                   {  94,  95, 120, 198, 215, 219 } };

/**
 *  pointer for extern of self-test data in header
 *  NOTE: must be initialize to data, as above,
 *  or set NULL if no saved data exists so a check of
 *  if (mpu_st_data) validates the presence or absence
 *  of saved self-test value of
 */
const uint8_t (*mpu_st_data)[6] = mpu_st_list;

/* getter to provide size of mpu_st_list tp other parts of code */
int get_mpu_st_list_sz (void)
{
  return sizeof mpu_st_list / sizeof *mpu_st_list;
}


/**
 *  room temperature offset in degrees C.
 *  temperator reported is chip temperature, not room temperature, to get
 *  get close approximation, set room_temp_offset. (2.8 C * 333.87)
 */
void set_room_temp_offset (mpu_t *mpu)
{
  switch (mpu->id) {
    case 1:
      mpu->room_temp_offset = 4.58;
    break;
    case 2:
      mpu->room_temp_offset = 4.2;
    break;
    case 3:
      mpu->room_temp_offset = 6.455;
    break;
    default:
      mpu->room_temp_offset = 0.;
    break;
  }
}

/** set magnetometor bias and scale based on mpu->id from saved data */
void set_usr_mag_bias_scale (mpu_t *mpu)
{
  switch (mpu->id) {
    case 1:
      mpu->mag_bias.x =  48.72;   /* hard iron bias */
      mpu->mag_bias.y =  40.92;
      mpu->mag_bias.z = -17.99;

      mpu->mag_scale.x =  0.98;   /* soft iron bias ellipsoid correction */
      mpu->mag_scale.y =  1.06;
      mpu->mag_scale.z =  0.96;
      break;
    case 2:
      mpu->mag_bias.x =  -2.92;   /* hard iron bias */
      mpu->mag_bias.y =  19.94;
      mpu->mag_bias.z =  -7.12;

      mpu->mag_scale.x =  1.;     /* soft iron bias ellipsoid correction */
      mpu->mag_scale.y =  1.09;
      mpu->mag_scale.z =  0.93;
      break;
    case 3:
      mpu->mag_bias.x = -25.78;   /* hard iron bias */
      mpu->mag_bias.y =  73.30;
      mpu->mag_bias.z =  49.17;

      mpu->mag_scale.x =  0.97;   /* soft iron bias ellipsoid correction */
      mpu->mag_scale.y =  1.09;
      mpu->mag_scale.z =  0.95;
      break;
    default:
      break;
  }
}

