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


/* sample timer, data and output ready flags */
extern volatile sig_atomic_t sample_timer_rdy;
extern volatile sig_atomic_t mpu_data_rdy;
extern volatile sig_atomic_t mpu_output_rdy;

/* extern variables from mpu_calibrate.c */
extern uint32_t nsamples;               /* current number of calibration samples */

extern volatile bool autocalibrated;    /* auto-calibrate successfully completed */

/* id set from factory self-test fingerprint */
mpu_t mpu = { .addr = MPU_ADDRESS_AD0_LOW };

/* accommodate both milkv-duo and raspberry pi on default I2C pins */
#ifdef MILKV
const char *i2cdev = "/dev/i2c-0";
#else
const char *i2cdev = "/dev/i2c-1";
#endif


/* simple empty-stdin function, c must be initialized */
void empty_stdin (int c)
{
  while (c != '\n' || c == EOF) {
    c = getchar();
  }
}


/* output and information function, no other critical use */
void get_st_gyro_accel (void)
{
  uint8_t tmpbuf[RWBUFSZ] = {0};          /* buffer for register values */

  printf ("Factory Self-Text Values:\n"
          "  acc:  %3hhu  %3hhu  %3hhu\n"
          "  gyr:  %3hhu  %3hhu  %3hhu\n",
          mpu.accel_st.x, mpu.accel_st.y, mpu.accel_st.z,
          mpu.gyro_st.x, mpu.gyro_st.y, mpu.gyro_st.z);

  printf ("\nmpu.id (%hhu) matches self-test fingerprint\n\n", mpu.id);

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

#ifdef MAGHOFL
extern volatile uint_fast8_t drdymax;
extern volatile uint32_t hoflset;
#endif

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

  /* initialize interval timers (values in NANOSECONDS)
   *   .02 sec delay before timers are started
   *   .01 sec interval timer for mpu data sample rate
   *   .2  sec interval timer for display output rate
   * with signal handler functions for data sampling and display update
   */
  if (timers_start (20000000, 10000000, 200000000) == 1) {
    return 1;
  }

  /* show warning not to move until auto-calibration done, hide cursor */
  printf ("\n%s Output:  (\033[1;31mDO NOT MOVE UNTIL "
          "CALIBRATED\033[0m)\033[?25l\n\n", mpu.typenm);

  /* program loop with increment controlling infinite or no. of loops */
  for (int i = 0; i < samples;) {
    /* data ready block (adjust usleep below if used) */
    if (sample_timer_rdy && get_acc_gyro_tempc (&mpu)
#if defined (MPU9250) || defined (MPU9255)
        && get_mag (&mpu)
#endif
    ) {
      mpu_data_rdy = true;
      sample_timer_rdy = false;
      if (!autocalibrated) {            /* if autocal not complete */
        autocalibrate (&mpu);           /* call autocalibrate func */
      }
    }
    if (mpu_data_rdy) {
    }
    /* output ready block */
    if (mpu_output_rdy) {
      show_mpu_output (&mpu);
      mpu_output_rdy = false;
      if (!shown && autocalibrated) {
        show_calibrated();
        shown = true;
      }
      i += tmp ? 1 : 0;
    }
    /* check if user input provided */
    if (pselect_timer (0, 2.e8) == 1) {
      int c = getchar();
      /* quit on 'q', empty-string or manual EOF (ctrl + d) (z on windows) */
      if (c == 'q' || c == '\n' || c == EOF) {
        break;
      }
      empty_stdin (c);    /* empty stdin of any remaining chars */
    }
  }
  /* skip over data values, restore cursor */
  printf ("\r\033[4B\033[?25h\n");

  printf ("\nReal-Time signal numbers for timers:\n"
          "  mpu.sigrt_sample: %hhu\n"
          "  mpu.sigrt_output: %hhu\n",
          mpu.itimer_sample->signo, mpu.itimer_output->signo);

  printf ("\nnsamples: %u\n", nsamples);

  prn_autocal_min_max_avg();
  prn_mpu_struct (&mpu);

  /* stop/shutdown mpu, close file descriptors, stop/delete mpu timers */
  if (mpu_stop (&mpu) != 0) {
    fputs ("error on mpu_stop\n", stderr);
  }

#ifdef MAGHOFL
  printf ("data ready max fails (drdymax)  : %hhu\n"
          "magnetic overflow cnt (hoflset) : %u\n", drdymax, hoflset);
#endif
}
