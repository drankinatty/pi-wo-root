/**
 *  Hardware PWM utilizing Linux sysfs for Raspberry Pi
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#include <pwm.h>


int main (int argc, char **argv) {

  /* if argument given use pwm1, otherwise pwm0 by default*/
  pwm_t pwm = { .pwmfs = argc > 1 ? PWM1FS : PWM0FS };   /* pwm struct instance */

  if (argc > 1) {       /* set channel if not using default pwm0 */
    pwm.channel = 1;
  }

  /* validate pwmX sysfs file exists */
  if (dir_exists (pwm.pwmfs) == -1) {
    /* otherwise export the PWM channel */
    if (pwm_export (&pwm) == -1) {
      return 1;
    }
    usleep (50000);     /* give time for sysfs to propogate export qchange */
  }

  /* set period, duty_cycle and enable pwm */
  if (pwm_set_period (&pwm, 400000) == -1) {   /* set pwm period */
    return 1;
  }

  if (pwm_enable_pwm (&pwm, 1) == -1) {        /* enable pwm on channel */
    return 1;
  }
  printf ("current state of pwm:\n\n"
          "  pwm.sysfs      : %s\n"
          "  pwm.period     : %u\n"
          "  pwm.frequency  : %.2f Hz\n"
          "  pwm.duty_cycle : %u\n"
          "  pwm.enabled    : %hhu\n",
          pwm.pwmfs, pwm.period, pwm.frequency,
          pwm.duty_cycle, pwm.enabled);

  /* open duty_cycle file for repeated writes */
  if (sysfs_open_duty_cycle (&pwm) == -1) {
    return -1;
  }

  /* cycle duty cycle from 1000 to period - 1000, 50 msec delay */
  for (uint32_t dc = 0; dc < 100; dc += 1) {
    if (pwm_write_duty_cycle_pct (&pwm, dc) == -1) {
      pwm_enable_pwm (&pwm, 0);   /* on failure, disable PWM, exit */
      return -1;
    }
#ifdef DEBUG
    printf ("percent: %3u,  duty_cycle: %u\n", dc, pwm.duty_cycle);
#endif
    usleep (100000);
  }
  /* cycle duty_cycle from period - 1000 to zero, 50 msec delay */
  for (int32_t dc = 100; dc >= 0; dc -= 1) {
    if (pwm_write_duty_cycle_pct (&pwm, dc) == -1) {
      pwm_enable_pwm (&pwm, 0);   /* on failure, disable PWM, exit */
      return -1;
    }
#ifdef DEBUG
    printf ("percent: %3u,  duty_cycle: %u\n", dc, pwm.duty_cycle);
#endif
    usleep (100000);
  }

  /* close channel duty_cycle file, set file descriptor zero */
  if (sysfs_close_duty_cycle (&pwm) == -1) {
    return -1;
  }

  pwm_enable_pwm (&pwm, 0);      /* disable pwm */

  puts ("\nsuccess");

  (void)argv;
}
