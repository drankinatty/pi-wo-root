/**
 *  PWM utilizing Linux sysfs for Raspberry Pi
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#include <math.h>

#include <pwm.h>


float deg2rad (float deg)
{
  return deg * M_PI / 180.f;
}

float rad2deg (float rad)
{
  return rad * 180.f / M_PI;
}

int main (int argc, char **argv) {

  pwm_t pwm = { .frequency = 0 };   /* pwm struct instance */
  char buf[PWMPATHMAX];
  unsigned  delay = 30000,
            revs = 24;


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

  if (argc > 2) {   /* delay argument */
    unsigned tmp;
    if (sscanf (argv[2], "%u", &tmp) != 1) {
      fprintf (stderr, "error: invalid unsigned delay value for argv[1] (%s)\n",
              argv[2]);
      return 1;
    }
    delay = tmp;
  }

  if (argc > 3) {   /* revolution argument */
    unsigned tmp;
    if (sscanf (argv[3], "%u", &tmp) != 1) {
      fprintf (stderr, "error: invalid unsigned revs value for argv[1] (%s)\n",
              argv[3]);
      return 1;
    }
    revs = tmp;
  }

  /* set period, duty_cycle and enable pwm */
  if (pwm_set_frequency (&pwm, 2500) == -1) {   /* set pwm frequency */
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
          "  pwm.enabled    : %hhu\n"
          "  delay          : %u\n"
#ifdef UNICODEPI
          "  \u03C0 iterations   : %u     (%u revolutions)\n",
#else
          "  Pi iterations  : %u     (%u revolutions)\n",
#endif
          buf, pwm.period, pwm.frequency,
          pwm.duty_cycle, pwm.enabled, delay, revs, revs / 2);

  /* open duty_cycle file for repeated writes */
  if (pwm_open_duty_cycle (&pwm) == -1) {
    return -1;
  }

  /* cycle duty cycle from 1000 to period - 1000, 50 msec delay */
  for (float dc = 0; dc < revs * M_PI; dc += 0.1) {
    float sindc = 0.5f * sinf (dc - M_PI_2) + 0.5f,
          // pct = fabsf (sindc * 100.f);
          pct = sindc * 100.f;

 #ifdef SHOWCALC
   printf ("dc : %6.2f,  sindc : %.2f,  pct : %6.2f\n", dc, sindc, pct);
#endif

    if (pwm_write_duty_cycle_pct (&pwm, pct) == -1) {
      pwm_enable_pwm (&pwm, 0);   /* on failure, disable PWM, exit */
      return -1;
    }
#ifdef DEBUG
    printf ("percent: %3u,  duty_cycle: %u\n", dc, pwm.duty_cycle);
#endif
    usleep (delay);
  }

  /* close channel duty_cycle file, set file descriptor zero */
  if (pwm_close_duty_cycle (&pwm) == -1) {
    return -1;
  }

  pwm_enable_pwm (&pwm, 0);      /* disable pwm */

  puts ("\nsuccess");
}
