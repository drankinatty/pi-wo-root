/**
 *  Hardware PWM utilizing Linux sysfs for Raspberry Pi
 *  (and other boards using Linux sysfs PWM interface)
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <linux/types.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <errno.h>
#include <unistd.h>

#define PWM_CLOCK       100000000

#ifdef MILKVFS
#define PWMCHANNELS     16
#else
#define PWMCHANNELS     2
#endif

#define PWMCHIP         "/sys/class/pwm/pwmchip"
#define PWMPATHMAX      64

#define TMPBUFSZ        16


/**
 * @struct pwm_t
 *
 * @brief struct containing PWM channel settings.
 */
typedef struct {
  float       frequency;    /**< PWM frequency (Hz) (periods per-sec) */
  __u32       duty_cycle,   /**< PWM duty cycle (0 <= duty_cycle <= period) */
              period;       /**< PWM period (nanoseconds between rollover) */

  int         fddc;         /**< duty_cycle file descriptor, manually opened */

  __u8        channel,      /**< PWM channel  ( 0-15 board dependent ) */
              enabled;      /**< PWM channel enabled (1 - enabled, 0 - not) */
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
#ifdef DEBUG
    perror ("dir_exists");
#endif
    return -1;
  }
  else if (fd) {      /* directory exists, rtn fd */
    close (fd);
  }

  return 0;
}


/**
 * @brief pwm_set_channel constructs string-literal from PWMPATH assigning
 * retuls to pwmfs member of struct, and sets chip and channel members.
 * @param pwm pointer to pwm_t struct to hold values.
 * @param chan pwm channel to construct devfs path to open.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_set_channel (pwm_t *pwm, __u8 chan)
{
  if (chan >= PWMCHANNELS) {
    fprintf (stderr, "error: setpwmchan() chan %hhu exceeds MAX %hhu.\n",
              chan, PWMCHANNELS);
    return -1;
  }

  pwm->channel = chan;

  return 0;
}


/**
 * @brief export or unexport PWM channel based on value of export.
 * @param pwm pointer to pwm struct to export/unexport channel.
 * @param export 1 - export the PWM channel, 0 - unexport the channel.
 * @return returns 0 on success, -1 otherwise.
 */
static int pwm_export_ctrl (__u8 chan, __u8 export)
{
  char  buf[PWMPATHMAX];
  int fd;
  __u8 chip = chan - chan % 4;

  if (export) {
    sprintf (buf, "%s%hhu/export", PWMCHIP, chip);
  }
  else {
    sprintf (buf, "%s%hhu/unexport", PWMCHIP, chip);
  }

  /* open pwm export or unexport sysfs file */
  if ((fd = open (buf, O_WRONLY)) == -1) {
    perror ("open-pwm_export_ctrl");
    return -1;
  }

  buf[0] = (chan % 4) | '0';      /* set channel to export */

  if (write (fd, buf, 1) < 1) {   /* write channel to sysfs */
    perror ("write-pwm-export_ctrl");
    close (fd);
    return -1;
  }

  if (close (fd) == -1) { /* validate close-after-write */
    perror ("pwm_export_ctrl-close-after-write");
    return -1;
  }

  /* check delay required if pwm dir not created fast enaough */
  sprintf (buf, "%s%hhu/pwm%hhu", PWMCHIP, chip, chan % 4);

  /* return validation on export, zero on unexport */
  return export ? dir_exists (buf) : 0;
}


/**
 * @brief export the pwm channel set in the pwm struct.
 * @param pwm pointer to pwm struct to export channel.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_export (pwm_t *pwm)
{
  return pwm_export_ctrl (pwm->channel, 1);
}


/**
 * @brief unexport the pwm channel set in the pwm struct.
 * @param pwm pointer to pwm struct to unexport channel.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_unexport (pwm_t *pwm)
{
  return pwm_export_ctrl (pwm->channel, 0);
}


/**
 * @brief set the PWM period for channel in pwm struct, update period in struct.
 * @param pwm pointer to pwm struct for channel to set period for.
 * @param period pwm period in samples within the 1E9 MHz PWM_CLOCK.
 * @note all sysfs values must be written as character strings.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_set_period (pwm_t *pwm, __u32 period)
{
  char buf[PWMPATHMAX];
  int fd, len;
  __u8 chip = pwm->channel - pwm->channel % 4;

  pwm->period = period;   /* set period in pwm struct (nanoseconds) */
  /* set PWM signal frequency 1. / period * 1e9 (Hz) */
  pwm->frequency = (float)1.f / period * 1e9f;

  /* create path to pwm period */
  sprintf (buf, "%s%hhu/pwm%hhu/period", PWMCHIP, chip, pwm->channel % 4);

  /* open pwm channel period sysfs file */
  if ((fd = open (buf, O_RDWR)) == -1) {
    perror ("open-pwm_set_period");
    return -1;
  }

  len = sprintf (buf, "%u", period);  /* convert period to string in buf */

  /* write period to sysfs file */
  if (write (fd, buf, len) < (ssize_t)len) {
    perror ("pwm_set_period-write-period");
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
int pwm_set_frequency (pwm_t *pwm, __u32 frequency)
{
  __u32 period;

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
int pwm_open_duty_cycle (pwm_t *pwm)
{
  char buf[PWMPATHMAX];
  __u8 chip = pwm->channel - pwm->channel % 4;

  /* create path to pwm duty_cycle */
  sprintf (buf, "%s%hhu/pwm%hhu/duty_cycle", PWMCHIP, chip, pwm->channel % 4);

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
int pwm_close_duty_cycle (pwm_t *pwm)
{
  /* validate close-after-write of duty_cycle file descriptor */
  if (close (pwm->fddc) == -1) {
    perror ("pwm_close_duty_cycle");
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
int pwm_write_duty_cycle (pwm_t *pwm, __u32 duty_cycle)
{
  char buf[TMPBUFSZ];
  int len;

  /* validate duty_cycle file open and valid descriptor in struct */
  if (pwm->fddc <= STDERR_FILENO) {
    fprintf (stderr, "error: duty_cycle file descriptor (%d) invalid.\n",
              pwm->fddc);
    return -1;
  }

  /* convert duty_cycle to string in buf */
  len = sprintf (buf, "%u", duty_cycle);

  /* write duty_cycle to sysfs file */
  if (write (pwm->fddc, buf, len) < (ssize_t)len) {
    perror ("pwm_write_duty_cycle-write");
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
  return pwm_write_duty_cycle (pwm, (__u32)(pct / 100. * pwm->period));
}


/**
 * @brief set the PWM duty_cycle for channel in pwm struct, update period
 * and frequency in struct, duty_cycle sysfs file is opened and closed.
 * @param pwm pointer to pwm struct for channel to set period for.
 * @param duty_cycle pwm duty_cycle in nanoseconds as fraction of period.
 * @note all sysfs values must be written as character strings.
 * @return
 */
int pwm_set_duty_cycle (pwm_t *pwm, __u32 duty_cycle)
{
  if (duty_cycle > pwm->period) {   /* check duty_cycle < period */
    fputs ("error: requested duty_cycle exceeds PWM persiod.\n", stderr);
    duty_cycle = pwm->period;
  }

  /* open duty_cycle sysfs file for channel */
  if (pwm_open_duty_cycle (pwm) == -1) {
    return -1;
  }

  /* write duty cycle to sysfs channel duty_cycle */
  if (pwm_write_duty_cycle (pwm, duty_cycle) == -1) {
    return -1;
  }

  /* close channel duty_cycle file, set file descriptor zero */
  if (pwm_close_duty_cycle (pwm) == -1) {
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
  return pwm_set_duty_cycle (pwm, (__u32)(pct / 100. * pwm->period));
}


/**
 * @brief enable or disable PWM for channel specified in pwm struct.
 * @param pwm pointer to pwm struct for channel to enable/disable PWM.
 * @param enabled 1 - enable PWM on channel, 0 - disable PWM on channel.
 * @note all sysfs values must be written as character strings.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_enable_pwm (pwm_t *pwm, __u8 enabled)
{
  char buf[PWMPATHMAX];
  int fd;
  __u8 chip = pwm->channel - pwm->channel % 4;

  /* create path to pwm enabled */
  sprintf (buf, "%s%hhu/pwm%hhu/enable", PWMCHIP, chip, pwm->channel % 4);

  /* open pwm channel enable sysfs file */
  if ((fd = open (buf, O_RDWR)) == -1) {
    perror ("open-pwm->pwmfs/enable");
    return -1;
  }

  buf[0] = enabled ? '1' : '0';
  buf[1] = 0;

  /* write enabled to sysfs file */
  if (write (fd, buf, 1) < 1) {
    perror ("pwm_set_enabled-write-enabled");
    return -1;
  }

  if (close (fd) == -1) {             /* validate close-after-write */
    perror ("pwm_set_enabled-close");
    return -1;
  }

  pwm->enabled = (__u8)enabled;       /* set enabled in pwm struct */

  return 0;                           /* return success */
}


void prn_pwm_state (pwm_t *pwm)
{
  char pwmdev[PWMPATHMAX],
       exportdev[PWMPATHMAX],
       unexportdev[PWMPATHMAX],
       period[PWMPATHMAX],
       duty_cycle[PWMPATHMAX],
       enable[PWMPATHMAX];
  __u8 chip = pwm->channel - pwm->channel % 4;

  sprintf (pwmdev, "%s%hhu/pwm%hhu", PWMCHIP, chip, pwm->channel % 4);
  sprintf (exportdev, "%s%hhu/export", PWMCHIP, chip);
  sprintf (unexportdev, "%s%hhu/unexport", PWMCHIP, chip);
  sprintf (period, "%s%hhu/pwm%hhu/period", PWMCHIP, chip, pwm->channel % 4);
  sprintf (duty_cycle, "%s%hhu/pwm%hhu/duty_cycle", PWMCHIP, chip, pwm->channel % 4);
  sprintf (enable, "%s%hhu/pwm%hhu/enable", PWMCHIP, chip, pwm->channel % 4);

  printf ("current state of pwm:\n\n"
          "  pwm->sysfs         : %s\n"
          "  pwm->exportfs      : %s\n"
          "  pwm->unexportfs    : %s\n"
          "  pwm->periodfs      : %s\n"
          "  pwm->duty_cyclefs  : %s\n"
          "  pwm->enablefs      : %s\n"
          "  pwm->period        : %u\n"
          "  pwm->frequency     : %.2f Hz\n"
          "  pwm->duty_cycle    : %u\n"
          "  pwm->enable        : %hhu\n",
          pwmdev, exportdev, unexportdev, period, duty_cycle, enable,
          pwm->period, pwm->frequency, pwm->duty_cycle, pwm->enabled);
}

int main (int argc, char **argv) {

  pwm_t pwm = { .frequency = 0 };   /* pwm struct instance */
  char buf[PWMPATHMAX];

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

  prn_pwm_state (&pwm);                        /* dump pwm state */

  /* open duty_cycle file for repeated writes */
  if (pwm_open_duty_cycle (&pwm) == -1) {
    return 1;
  }

  /* cycle duty cycle from 1000 to period - 1000, 50 msec delay */
  for (__u32 dc = 0; dc < 100; dc += 1) {
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
  if (pwm_close_duty_cycle (&pwm) == -1) {
    return -1;
  }

  pwm_enable_pwm (&pwm, 0);      /* disable pwm */

  pwm_unexport (&pwm);           /* unexport the bus/channel */

  puts ("\nsuccess");

  (void)argv;
}
