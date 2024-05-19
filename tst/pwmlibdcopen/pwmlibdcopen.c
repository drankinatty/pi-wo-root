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

  char buf[PWMPATHMAX];

  /* if argument given use pwm1, otherwise pwm0 by default*/
  pwm_t pwm = { .frequency = 0 };   /* pwm struct instance */

  if (argc > 1) {
    __u8 tmp = 0;
    if (sscanf (argv[1], "%hhu", &tmp) != 1 && tmp >= PWMCHANNELS) {
      fprintf (stderr, "error: argument %hhu exceeds PWMCHANNELS %hhu.\n",
              tmp, PWMCHANNELS);
      return 1;
    }
    pwm.channel = tmp;
  }
  sprintf (buf, "%s%hhu/pwm%hhu", PWMCHIP, pwm.channel - pwm.channel % 4,
          pwm.channel % 4);

  if (pwm_set_channel (&pwm, pwm.channel) == -1) {
    return 1;
  }

  /* validate pwmX sysfs file exists */
  if (dir_exists (buf) == -1) {
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
          buf, pwm.period, pwm.frequency,
          pwm.duty_cycle, pwm.enabled);

  /* open duty_cycle file for repeated writes */
  if (pwm_open_duty_cycle (&pwm) == -1) {
    return -1;
  }

  /* cycle duty cycle from 1000 to period - 1000, 50 msec delay */
  for (uint32_t dc = 0; dc < 100; dc += 1) {
    if (pwm_write_duty_cycle_pct (&pwm, dc) == -1) {
      pwm_enable_pwm (&pwm, 0);   /* on failure, disable PWM, exit */
      return -1;
    }
#ifndef NOANSI
    printf ("\x1b[2A\x1b[19C\x1b[0K%u\n\n", (pwm.duty_cycle * 100) / pwm.period);
#endif
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
#ifndef NOANSI
    printf ("\x1b[2A\x1b[19C\x1b[0K%u\n\n", (pwm.duty_cycle * 100) / pwm.period);
#endif
#ifdef DEBUG
    printf ("percent: %3u,  duty_cycle: %u\n", dc, pwm.duty_cycle);
#endif
    usleep (100000);
  }

  /* close channel duty_cycle file, set file descriptor zero */
  if (pwm_close_duty_cycle (&pwm) == -1) {
    return -1;
  }

  pwm_enable_pwm (&pwm, 0);           /* disable pwm */
#ifndef NOANSI
  printf ("\x1b[1A\x1b[19C\x1b[0K%hhu\n", pwm.enabled);
#endif

  if (pwm_unexport (&pwm) == -1) {    /* unexport the pwm channel */
    return 1;
  }

  puts ("\nsuccess");
}
