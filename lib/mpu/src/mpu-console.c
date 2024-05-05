/**
 *  Inversense MPU API
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */

#include <stdio.h>

#include "mpu-console.h"

void show_self_test (mpu_t *mpu)
{
  printf ("Factory Self-Test Values:\n"
          "  Accel ST      : % 6d  % 6d  % 6d\n"
          "  Gyro ST       : % 6d  % 6d  % 6d\n\n",
          mpu->accel_st.x, mpu->accel_st.y, mpu->accel_st.z,
          mpu->gyro_st.x, mpu->gyro_st.y, mpu->gyro_st.z);
}

void show_ga_sens (mpu_t *mpu)
{
  printf ("Accel & Gyro Full-Scale Selection:\n"
          "  Accel Sens    : %6hhu\n"
          "  Gyro Sens     : %6hu\n\n",
          mpu->accel_sens, mpu->gyro_sens);
}

/**********  temporary extern varaibles  **********/

extern uint8_t  gbidx, packet_count;
extern uint16_t fifo_count;

extern int32_t  accel_sum[NAXIS], gyro_sum[NAXIS];

/**************************************************/

void show_ga_bias (mpu_t *mpu)
{
//   uint16_t i = 0;

  printf ("Gyro and Accelerometer Calibration:\n  FIFO Overflow : %s\n",
          mpu->fifo_overflow ? "OVERFLOW" : "None");
  printf ("  FIFO Count    : %hu\n"
          "  packet Size   : %hu\n"
          "  packet Count  : %hu\n",
          fifo_count, FIFO_PKT_LEN, packet_count);
  if (gbidx != 18) {
    printf ("  gbidx Count   : %hhu  (error: 18 expected)\n\n", gbidx);
  }
  else {
    putchar ('\n');
  }

  puts ("Accel & Gyro Sums:");
  printf ("  Accel Sum     : % 6d  % 6d  % 6d\n"
          "  Gyro Sum      : % 6d  % 6d  % 6d\n\n",
          accel_sum[0], accel_sum[1], accel_sum[2],
          gyro_sum[0], gyro_sum[1], gyro_sum[2]);
#ifdef SHOWSENSE
  printf ("  Accel Sense   : %4hhu\n"
          "  Gyro Sense    : %4hu\n\n", mpu->accel_sens, mpu->gyro_sens);
#endif
  puts ("Avg. Accel & Gyro Offset Values (sum/pkt_count):");

  printf ("%6hd  %6hd  %6hd  %6hd  %6hd  %6hd\n\n",
          mpu->accel_offset.x, mpu->accel_offset.y, mpu->accel_offset.z,
          mpu->gyro_offset.x, mpu->gyro_offset.y, mpu->gyro_offset.z);

// #ifdef SHOWFIFO
//   puts ("FIFO Values (XA_H ... ZG_L):\n\n"
//         "AXH  AXL  AYH  AYL  AZH  AZL  GXH  GXL  GYH  GYL  GZH  GZL");
//   for (i = 0; i < fifo_count; i+=12) {
//     printf ("%3hhu  %3hhu  %3hhu  %3hhu  %3hhu  %3hhu  "
//             "%3hhu  %3hhu  %3hhu  %3hhu  %3hhu  %3hhu\n",
//             fifo[  i], fifo[i+1], fifo[i+2], fifo[i+3], fifo[ i+4], fifo[ i+5],
//             fifo[i+6], fifo[i+7], fifo[i+8], fifo[i+9], fifo[i+10], fifo[i+11]);
//   }
//   putchar ('\n');
// #endif
}

void show_raw_acc_gyro_output (int16_t *acc, int16_t *gyro)
{
  printf ("  Acc.    x: % 6hd,   Y: % 6hd,   z: % 6hd\n"
          "  Gyro.   x: % 6hd,   y: % 6hd,   z: % 6hd\r\033[1A",
          acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
}

void show_raw_mpu_output (int16_t *acc, int16_t *gyro, int16_t *mag)
{
  printf ("  Acc.    x: % 6hd,   Y: % 6hd,   z: % 6hd\n"
          "  Gyro.   x: % 6hd,   y: % 6hd,   z: % 6hd\n"
          "  Mag.    x: % 6hd,   y: % 6hd,   z: % 6hd\r\033[2A",
          acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2],
          mag[0], mag[1], mag[2]);
}

void show_mpu_output (mpu_t *mpu)
{
  float a_tmp[NAXIS] = {0},
        g_tmp[NAXIS] = {0},
        tempf = mpu->tempc * 9. / 5. + 32.;

#ifdef DENOISE
  denoise_values (a_tmp, mpu->accel.arr, DENOISE_LIM_ACCEL);
  denoise_values (g_tmp, mpu->gyro.arr,  DENOISE_LIM_GYRO);
#else
  a_tmp[0] = mpu->accel.arr[0];
  a_tmp[1] = mpu->accel.arr[1];
  a_tmp[2] = mpu->accel.arr[2];

  g_tmp[0] = mpu->gyro.arr[0];
  g_tmp[1] = mpu->gyro.arr[1];
  g_tmp[2] = mpu->gyro.arr[2];
#endif

  printf ("  Acc.    x: % 6.2f,   Y: % 6.2f,   z: % 6.2f\n"
          "  Gyro.   x: % 6.1f,   y: % 6.1f,   z: % 6.1f\n",
          a_tmp[0], a_tmp[1], a_tmp[2],
          g_tmp[0], g_tmp[1], g_tmp[2]);
#if defined (MPU9250) || defined (MPU9255)
  /* read magnetometer sensor values and convert to uT */
  // if (get_mag (mpu)) {
    printf ("  Mag.    x: % 6.1f,   y: % 6.1f,   z: % 6.1f\n\n"
            "  Temp.   %.1f C   %.1f F\r\033[4A",
            mpu->mag.x, mpu->mag.y, mpu->mag.z, mpu->tempc, tempf);
  // }
#else
  /* output temp only for MPU6050 */
  printf ("\n  Temp.   %.1f C   %.1f F\r\033[3A", mpu->tempc, tempf);

#endif
}


void show_mpu_output_w_fusion (mpu_t *mpu,
                               const float *euler, const float *earth)
{
  float a_tmp[NAXIS] = {0},
        g_tmp[NAXIS] = {0},
        earth_d[NAXIS] = {0},
        tempf = mpu->tempc * 9. / 5. + 32.;

#ifdef DENOISE
  denoise_values (a_tmp, mpu->accel.arr, DENOISE_LIM_ACCEL);
  denoise_values (g_tmp, mpu->gyro.arr,  DENOISE_LIM_GYRO);
  denoise_values (earth_d, earth, 0.1);
#else
  a_tmp[0] = mpu->accel.arr[0];
  a_tmp[1] = mpu->accel.arr[1];
  a_tmp[2] = mpu->accel.arr[2];

  g_tmp[0] = mpu->gyro.arr[0];
  g_tmp[1] = mpu->gyro.arr[1];
  g_tmp[2] = mpu->gyro.arr[2];
  for (int i = 0; i < NAXIS; i++) {
    earth_d[i] = earth[i];
  }
#endif

  printf ("  Acc.    x: % 6.1f,   Y: % 6.1f,   z: % 6.1f\n"
          "  Gyro.   x: % 6.1f,   y: % 6.1f,   z: % 6.1f\n",
          a_tmp[0], a_tmp[1], a_tmp[2],
          g_tmp[0], g_tmp[1], g_tmp[2]);
#ifdef MPU9250
  /* read magnetometer sensor values and convert to uT */
  if (get_mag (mpu)) {
    printf ("  Mag.    x: % 6.1f,   y: % 6.1f,   z: % 6.1f\n\n"
            "  Temp.   %.1f C   %.1f F\n\n"
            "  Att.    r: % 6.1f,   p: % 6.1f,   y: % 6.1f\n"
            "  pos.    x: % 6.1f,   y: % 6.1f,   z: % 6.1f\n\033[8A",
            mpu->mag.x, mpu->mag.y, mpu->mag.z, mpu->tempc, tempf,
            euler[0], euler[1], euler[2],
            earth_d[0], earth_d[1], earth_d[2]);
  }
#else
  /* output temp only for MPU6050 */
  printf ("\n  Temp.   %.1f C   %.1f F\n\n"
          "  Att.    r: % 6.1f,   p: % 6.1f,   y: % 6.1f\n"
          "  pos.    x: % 6.1f,   y: % 6.1f,   z: % 6.1f\n\033[7A",
          mpu->tempc, tempf,
          euler[0], euler[1], euler[2],
          earth_d[0], earth_d[1], earth_d[2]);
#endif
}


void show_mag_bias (mpu_t *mpu)
{
  puts ("Magnetometer Hard-Iron Bias & Soft-Iron Scale:");
  printf ("  Mag Bias      : % 7.3f  % 7.3f  % 7.3f\n"
          "  Mag Scale     : % 7.3f  % 7.3f  % 7.3f\n\n",
          mpu->mag_bias.x, mpu->mag_bias.y, mpu->mag_bias.z,
          mpu->mag_scale.x, mpu->mag_scale.y, mpu->mag_scale.z);
}


extern uint16_t mag_bias_raw[3], mag_scale_raw[3];

void show_mag_bias_raw (mpu_t *mpu)
{
  puts ("Magnetometer Hard-Iron Bias & Soft-Iron Scale:");
  printf ("  Mag Bias Raw  : % 7hd  % 7hd  % 7hd\n"
          "  Mag Scale Raw : % 7hd  % 7hd  % 7hd\n"
          "  Mag Bias      : % 7.3f  % 7.3f  % 7.3f\n"
          "  Mag Scale     : % 7.3f  % 7.3f  % 7.3f\n\n",
          mag_bias_raw[0], mag_bias_raw[1], mag_bias_raw[2],
          mag_scale_raw[0], mag_scale_raw[1], mag_scale_raw[2],
          mpu->mag_bias.x, mpu->mag_bias.y, mpu->mag_bias.z,
          mpu->mag_scale.x, mpu->mag_scale.y, mpu->mag_scale.z);
}

void prn_mpu_struct (mpu_t *mpu)
{
  printf ("\nMPU struct detail:\n\n"
          "  addr:  0x%02hhx    id:  %hhu\n"
          "  type:  0x%02hhx    name: %s\n\n"
          "  self-test values:\n"
          "    acc : %3hhu, %3hhu, %3hhu\n"
          "    gyro: %3hhu, %3hhu, %3hhu\n\n"
          "  user bias:\n"
          "    acc :  % 7.2f, % 7.2f, % 7.2f\n"
          "    gyro:  %7hhu, %7hhu, %7hhu\n"
          "    temp:  % 7.1f\n\n"
          "  offset:\n"
          "    acc :  % 6hd, % 6hd, % 6hd\n"
          "    gyro:  % 6hd, % 6hd, % 6hd\n\n"
          "  sensitivity:\n"
          "    acc:  %hhu    gyro:  %hu\n\n"
          "  DLPF: %hhu\n\n"
          "  FS_sel:\n"
          "    acc:  %hhu    gyro:  %hhu\n\n"
          "  Magnetometer:\n\n"
          "  bias:\n"
          "    bias :  % 7.2f, % 7.2f, % 7.2f\n"
          "    scale:  % 7.2f, % 7.2f, % 7.2f\n",
          mpu->addr, mpu->id,
          mpu->type, mpu->typenm,
          mpu->accel_st.x, mpu->accel_st.y, mpu->accel_st.z,
          mpu->gyro_st.x, mpu->gyro_st.y, mpu->gyro_st.z,
          mpu->accel_usr_bias.x, mpu->accel_usr_bias.y, mpu->accel_usr_bias.z,
          mpu->gyro_usr_bias.x, mpu->gyro_usr_bias.y, mpu->gyro_usr_bias.z,
          mpu->room_temp_offset,
          mpu->accel_offset.x, mpu->accel_offset.y, mpu->accel_offset.z,
          mpu->gyro_offset.x, mpu->gyro_offset.y, mpu->gyro_offset.z,
          mpu->accel_sens, mpu->gyro_sens,
          mpu->dlpf_cfg,
          mpu->accel_fs_sel, mpu->gyro_fs_sel,
          mpu->mag_bias.x, mpu->mag_bias.y, mpu->mag_bias.z,
          mpu->mag_scale.x, mpu->mag_scale.y, mpu->mag_scale.z);
 }

void prn_ag_min_max_values (min_max_sum *accel, min_max_sum *gyro,
                            vect_float_t *accelaxis, vect_float_t *gyroaxis)
{
  printf ("\nmin / max:\n"
          "  acc:  (% 6.2f/% 6.2f % 6.2f)    "
                  "(% 6.2f/% 6.2f % 6.2f)    "
                  "(% 6.2f/% 6.2f % 6.2f)\n"
          "  gyr:  (% 6.2f/% 6.2f % 6.2f)    "
                  "(% 6.2f/% 6.2f % 6.2f)    "
                  "(% 6.2f/% 6.2f % 6.2f)\n",
          accel->min.x, accel->max.x, accelaxis->x,
          accel->min.y, accel->max.y, accelaxis->y,
          accel->min.z, accel->max.z, accelaxis->z,
          gyro->min.x, gyro->max.x, gyroaxis->x,
          gyro->min.y, gyro->max.y, gyroaxis->y,
          gyro->min.z, gyro->max.z, gyroaxis->z);
}

void prn_ag_autocal_min_max_avg (vect_float_t *accelaxis, vect_float_t *gyroaxis)
{
  printf ("\navg accel axis error (g):\n"
          "  x : % .4f\n"
          "  y : % .4f\n"
          "  z : % .4f\n"
          "avg gyro axis error (deg/s):\n"
          "  x : % .4f\n"
          "  y : % .4f\n"
          "  z : % .4f\n",
          accelaxis->x, accelaxis->y, accelaxis->z,
          gyroaxis->x, gyroaxis->y, gyroaxis->z);
}
