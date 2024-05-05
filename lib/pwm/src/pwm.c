/**
 *  PWM utilizing Linux sysfs for Raspberry Pi
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

#include "pwm.h"


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
 * @brief set the PWM signal frequency (and period) from given frequency.
 * @param pwm pointer to pwm struct for channel to set frequency for.
 * @param frequency no. of PWM signals (Hz), 25 MHz sane limit for 3b+/zero.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_set_frequency (pwm_t *pwm, uint32_t frequency)
{
  uint32_t period;

  /* impose reasonable limit on chosen frequency to provide at least four
   * PWM levels per-period. In reality a frequency greater than 25MHz for
   * the 3B+/zero 100MHz clock derived from osc/plld/plld_per/pwm cannot
   * provide a decent square wave signal.
   */
  if (frequency > PWM_CLOCK / 4) {
    fprintf (stderr, "error: frequency provided exceeds reasonable %u MHz limit.\n",
              PWM_CLOCK / 4);
    return -1;
  }

  /* compute period (in nanoseconds for sysfs) from requested frequency */
  period = 1.f / frequency * 1e9f;

  /* set period and frequency returing result */
  return pwm_set_period (pwm, period);
}


/**
 * @brief open duty_cycle sysfs file for repeated duty cycle writes.
 * @param pwm pointer to pwm struct for channel to open duty_cycle.
 * @return returns 0 on success, -1 otherwise.
 */
int sysfs_open_duty_cycle (pwm_t *pwm)
{
  char buf[PWM_MAXPATH];

  /* create path to pwm duty_cycle */
  if (sprintf (buf, "%s/%s", pwm->pwmfs, PWMDUTYCYCLE) !=
      (int)(strlen (pwm->pwmfs) + 1 + strlen (PWMDUTYCYCLE))) {
    fputs ("pwm_set_duty_cycle - sprintf-duty_cycle.\n", stderr);
    return -1;
  }

  /* open pwm channel duty_cycle sysfs file */
  if ((pwm->fddc = open (buf, O_RDWR)) == -1) {
    perror ("open-pwm->pwmfs/duty_cycle");
    return -1;
  }

  return 0;
}


/**
 * @brief close the duty_cycle file descriptor and reset fd zero.
 * @param pwm pointer to pwm struct for channel to fd for.
 * @return returns 0 on success, -1 otherwise.
 */
int sysfs_close_duty_cycle (pwm_t *pwm)
{
  /* validate close-after-write of duty_cycle file descriptor */
  if (close (pwm->fddc) == -1) {
    perror ("sysfs_close_duty_cycle");
    return -1;
  }

  pwm->fddc = 0;    /* reset duty_cycle file descriptor zero */

  return 0;
}


/**
 * @brief write duty cycle for pwm channel to sysfs duty_cycle file.
 * @param pwm pointer to pwm struct for channel to write duty_cycle for.
 * @param duty_cycle fraction of period in nanoseconds for signal high.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_write_duty_cycle (pwm_t *pwm, uint32_t duty_cycle)
{
  char buf[PWM_TMPSIZE];
  int len;
  ssize_t nbytes = 0;

  /* validate duty_cycle file open and valid descriptor in struct */
  if (pwm->fddc <= STDERR_FILENO) {
    fprintf (stderr, "error: duty_cycle file descriptor (%d) invalid.\n",
              pwm->fddc);
    return -1;
  }

  /* convert duty_cycle to string in buf */
  len = sprintf (buf, "%u", duty_cycle);

  /* write duty_cycle to sysfs file */
  if ((nbytes = write (pwm->fddc, buf, len)) == -1) {
    perror ("pwm_write_duty_cycle-write");
    return -1;
  }

  if (nbytes != len) {      /* validate no. of bytes written */
    fputs ("error: pwm_write_duty_cycle failed to write duty_cycle bytes.\n",
            stderr);
    return -1;
  }

  pwm->duty_cycle = duty_cycle;     /* set duty_cycle in pwm struct */

  return 0;
}


/**
 * @brief set PWM duty cycle as a percentage of PWM period for open
 * sysfs duty_cycle file.
 * @param pwm pointer to pwm struct for channel to set duty cycle.
 * @param pct percent duty cycle, (valid range 0 - 100).
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_write_duty_cycle_pct (pwm_t *pwm, float pct)
{
  if (pct > 100) {        /* validate percent in range 0 - 100 */
    pct = 100.;
  }
  if (pct < 0) {
    pct = 0.;
  }

  /* calculate duty_cycle based on percentage of period */
  return pwm_write_duty_cycle (pwm, (uint32_t)(pct / 100. * pwm->period));
}


/**
 * @brief set the PWM duty_cycle for channel in pwm struct, update period
 * and frequency in struct, duty_cycle sysfs file is opened and closed.
 * @param pwm pointer to pwm struct for channel to set period for.
 * @param duty_cycle pwm duty_cycle in nanoseconds as fraction of period.
 * @note all sysfs values must be written as character strings.
 * @return
 */
int pwm_set_duty_cycle (pwm_t *pwm, uint32_t duty_cycle)
{
  if (duty_cycle > pwm->period) {   /* check duty_cycle < period */
    fputs ("error: requested duty_cycle exceeds PWM persiod.\n", stderr);
    duty_cycle = pwm->period;
  }

  /* open duty_cycle sysfs file for channel */
  if (sysfs_open_duty_cycle (pwm) == -1) {
    return -1;
  }

  /* write duty cycle to sysfs channel duty_cycle */
  if (pwm_write_duty_cycle (pwm, duty_cycle) == -1) {
    return -1;
  }

  /* close channel duty_cycle file, set file descriptor zero */
  if (sysfs_close_duty_cycle (pwm) == -1) {
    return -1;
  }

  return 0;                 /* return success */
}


/**
 * @brief set PWM duty cycle as a percentage of PWM period, sysfs
 * duty_cycle file is opened and closed.
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
 * @param enabled 1 - enable PWM on channel, 0 - disable PWM on channel.
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
