/**
 *  Hardware PWM utilizing Linux sysfs for Raspberry Pi
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <errno.h>
#include <unistd.h>

#define PWM_MAXPATH     128

#define PWM_CLOCK       100000000

#define EXPORTDEV       "/sys/class/pwm/pwmchip0/export"
#define UNEXPORTDEV     "/sys/class/pwm/pwmchip0/unexport"

#define PWM0FS          "/sys/class/pwm/pwmchip0/pwm0"
#define PWM1FS          "/sys/class/pwm/pwmchip0/pwm1"

#define PWMPERIOD       "period"
#define PWMDUTYCYCLE    "duty_cycle"
#define PWMENABLE       "enable"


/**
 * @brief struct containing PWM channel settings.
 */
typedef struct {
  const char  *pwmfs;       /* pointer to PWM sysfs file */

  float       frequency;    /* PWM frequency (Hz) (periods per-sec) */
  uint32_t    duty_cycle,   /* PWM duty cycle (0 <= duty_cycle <= period) */
              period;       /* PWM period (clock ticks between rollover) */

  uint8_t     channel,      /* PWM channel (0 - pwm0, 1 - pwm1) */
              enabled;      /* PWM channel enabled (1 - enabled, 0 - not) */
} pwm_t;


/**
 * @brief atomic test that directory exists.
 * @param dir character string holding directory path to test.
 * @note all sysfs values must be written as character strings.
 * @return returns 0 on success, -1 otherwise.
 */
int dir_exists (const char *dir)
{
  int flags = O_DIRECTORY | O_RDONLY,
      mode = S_IRUSR | S_IWUSR,
      fd = open (dir, flags, mode);

  if (fd < 0) {       /* directory does not exist */
    perror ("dir_exists");
    return -1;
  }
  else if (fd) {      /* directory exists, rtn fd */
    close (fd);
  }

  return 0;
}


/**
 * @brief export or unexport PWM channel based on value of export.
 * @param pwm pointer to pwm struct to export/unexport channel.
 * @param export 1 - export the PWM channel, 0 - unexport the channel.
 * @return returns 0 on success, -1 otherwise.
 */
static int pwm_export_ctrl (pwm_t *pwm, uint8_t export)
{
  char buf[16];
  int fd;
  ssize_t nbytes = 0;

  /* open pwm export or unexport sysfs file */
  if ((fd = open (export ? EXPORTDEV : UNEXPORTDEV, O_WRONLY)) == -1) {
    perror (export ? "open-pwm-export" : "open-pwm-unexport");
    return -1;
  }

  buf[0] = pwm->channel | '0';      /* set channel to export */

  if ((nbytes = write (fd, buf, 1)) == -1) {  /* write channel to sysfs */
    perror (export ? "write-pwm-export" : "write-pwm-unexport");
    close (fd);
    return -1;
  }

  if (nbytes == 0) {  /* validate byte written */
    fprintf (stderr, "error: pwm-%s failed to write byte.\n",
              export ? "export" : "unexport");
    close (fd);
    return -1;
  }

  if (close (fd) == -1) { /* validate close-after-write */
    perror ("pwm_export_ctrl-close-after-write");
    return -1;
  }

  return 0;     /* return success */
}


/**
 * @brief export the pwm channel set in the pwm struct.
 * @param pwm pointer to pwm struct to export channel.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_export (pwm_t *pwm)
{
  return pwm_export_ctrl (pwm, 1);
}


/**
 * @brief unexport the pwm channel set in the pwm struct.
 * @param pwm pointer to pwm struct to unexport channel.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_unexport (pwm_t *pwm)
{
  return pwm_export_ctrl (pwm, 0);
}


/**
 * @brief set the PWM period for channel in pwm struct, update period in struct.
 * @param pwm pointer to pwm struct for channel to set period for.
 * @param period pwm period in samples within the 1E9 MHz PWM_CLOCK.
 * @note all sysfs values must be written as character strings.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_set_period (pwm_t *pwm, uint32_t period)
{
  char buf[PWM_MAXPATH];
  int fd, len;
  ssize_t nbytes = 0;

  // pwm->period = period;           /* set period in pwm struct */
  // /* set PWM signal frequency 100MHz clock / period (Hz) */
  // pwm->frequency = (float)PWM_CLOCK / period;
  pwm->period = period;   /* set period in pwm struct (nanoseconds) */
  /* set PWM signal frequency 1. / period * 1e9 (Hz) */
  pwm->frequency = (float)1.f / period * 1e9f;

  /* create path to pwm period */
  if (sprintf (buf, "%s/%s", pwm->pwmfs, PWMPERIOD) !=
      (int)(strlen (pwm->pwmfs) + 1 + strlen (PWMPERIOD))) {
    fputs ("pwm_set_period - sprintf-period.\n", stderr);
    return -1;
  }

  /* open pwm channel period sysfs file */
  if ((fd = open (buf, O_RDWR)) == -1) {
    perror ("open-pwm->pwmfs/period");
    return -1;
  }

  len = sprintf (buf, "%u", period);  /* convert period to string in buf */

  /* write period to sysfs file */
  if ((nbytes = write (fd, buf, len)) == -1) {
    perror ("pwm_set_period-write-period");
    close (fd);
    return -1;
  }

  if (nbytes != (ssize_t)len) {       /* validate no. of bytes written */
    fputs ("error: pwm_set_period failed to write period bytes.\n",
            stderr);
    close (fd);
    return -1;
  }

  if (close (fd) == -1) {             /* validate close-after-write */
    perror ("pwm_set_period-close");
    return -1;
  }

  return 0;                           /* return success */
}


/**
 * @brief set the PWM duty_cycle for channel in pwm struct, update period in struct.
 * @param pwm pointer to pwm struct for channel to set period for.
 * @param duty_cycle pwm duty_cycle in nanoseconds as fraction of period.
 * @note all sysfs values must be written as character strings.
 * @return
 */
int pwm_set_duty_cycle (pwm_t *pwm, uint32_t duty_cycle)
{
  char buf[PWM_MAXPATH];
  int fd, len;
  ssize_t nbytes = 0;

  if (duty_cycle > pwm->period) {   /* check duty_cycle < period */
    fputs ("error: requested duty_cycle exceeds PWM persiod.\n", stderr);
    duty_cycle = pwm->period;
  }

  pwm->duty_cycle = duty_cycle;     /* set duty_cycle in pwm struct */

  /* create path to pwm duty_cycle */
  if (sprintf (buf, "%s/%s", pwm->pwmfs, PWMDUTYCYCLE) !=
      (int)(strlen (pwm->pwmfs) + 1 + strlen (PWMDUTYCYCLE))) {
    fputs ("pwm_set_duty_cycle - sprintf-duty_cycle.\n", stderr);
    return -1;
  }

  /* open pwm channel duty_cycle sysfs file */
  if ((fd = open (buf, O_RDWR)) == -1) {
    perror ("open-pwm->pwmfs/duty_cycle");
    return -1;
  }

  /* convert duty_cycle to string in buf */
  len = sprintf (buf, "%u", duty_cycle);

  /* write duty_cycle to sysfs file */
  if ((nbytes = write (fd, buf, len)) == -1) {
    perror ("pwm_set_duty_cycle-write-duty_cycle");
    return -1;
  }

  if (nbytes != len) {      /* validate no. of bytes written */
    fputs ("error: pwm_set_duty_cycle failed to write duty_cycle bytes.\n",
            stderr);
    return -1;
  }

  if (close (fd) == -1) {   /* validate close-after-write */
    perror ("pwm_set_duty_cycle-close");
    return -1;
  }

  return 0;                 /* return success */
}


/**
 * @brief set PWM duty cycle as a percentage of PWM period.
 * @param pwm pointer to pwm struct for channel to set duty cycle.
 * @param pct percent duty cycle, (valid range 0 - 100).
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_set_duty_cycle_pct (pwm_t *pwm, float pct)
{
  if (pct > 100) {        /* validate percent in range 0 - 100 */
    pct = 100.;
  }
  if (pct < 0) {
    pct = 0.;
  }

  /* calculate duty_cycle based on percentage of period */
  return pwm_set_duty_cycle (pwm, (uint32_t)(pct / 100. * pwm->period));
}


/**
 * @brief enable or disable PWM for channel specified in pwm struct.
 * @param pwm pointer to pwm struct for channel to enable/disable PWM.
 * @param enabled 1 - enable PWM, 0 - disable PWM.
 * @note all sysfs values must be written as character strings.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_enable_pwm (pwm_t *pwm, uint8_t enabled)
{
  char buf[PWM_MAXPATH];
  int fd;
  ssize_t nbytes = 0;

  /* create path to pwm enabled */
  if (sprintf (buf, "%s/%s", pwm->pwmfs, PWMENABLE) !=
      (int)(strlen (pwm->pwmfs) + 1 + strlen (PWMENABLE))) {
    fputs ("pwm_set_enabled - sprintf-enabled.\n", stderr);
    return -1;
  }

  /* open pwm channel enable sysfs file */
  if ((fd = open (buf, O_RDWR)) == -1) {
    perror ("open-pwm->pwmfs/enable");
    return -1;
  }

  buf[0] = enabled ? '1' : '0';
  buf[1] = 0;

  /* write enabled to sysfs file */
  if ((nbytes = write (fd, buf, 1)) == -1) {
    perror ("pwm_set_enabled-write-enabled");
    return -1;
  }

  if (nbytes != 1) {     /* validate no. of bytes written */
    fputs ("error: pwm_set_enabled failed to write enabled byte.\n",
            stderr);
    return -1;
  }

  if (close (fd) == -1) {             /* validate close-after-write */
    perror ("pwm_set_enabled-close");
    return -1;
  }

  pwm->enabled = (uint8_t)enabled;    /* set enabled in pwm struct */

  return 0;                           /* return success */
}


int main (int argc, char **argv) {

  /* if argument given use pwm1, otherwise pwm0 by default*/
  pwm_t pwm = { .pwmfs = argc > 1 ? PWM1FS: PWM0FS };   /* pwm struct instance */

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
  if (pwm_set_duty_cycle (&pwm, 0) == -1) {    /* set pwm duty_cycle */
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

  /* NOTE: for excessive changes to duty_cycle it is better to simply
   *       open the duty_cycle file and change the values rather than
   *       continually open/close the file to write one change each time.
   *
   *   (fpr this example simply calling the set function is fine)
   *
   * See the shared object library pwm.c for implimentation of the method for
   * opening duty_cycle once and writing many times to avoid this overhead.
   */
  /* cycle duty cycle from 1000 to period - 1000, 50 msec delay */
  for (uint32_t dc = 1000; dc < pwm.period; dc += 1000) {
    if (pwm_set_duty_cycle (&pwm, dc) == -1) {
      pwm_enable_pwm (&pwm, 0);   /* on failure, disable PWM, exit */
      return -1;
    }
    usleep (50000);
  }
  /* cycle duty_cycle from period - 1000 to zero, 50 msec delay */
  for (int32_t dc = pwm.period - 1000; dc >= 0; dc -= 1000) {
    if (pwm_set_duty_cycle (&pwm, dc) == -1) {
      pwm_enable_pwm (&pwm, 0);   /* on failure, disable PWM, exit */
      return -1;
    }
    usleep (50000);
  }

  pwm_enable_pwm (&pwm, 0);      /* disable pwm */

  puts ("\nsuccess");

  (void)argv;
}
