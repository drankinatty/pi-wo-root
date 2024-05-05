/**
 *  PWM utilizing Linux sysfs for Raspberry Pi
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */
#ifndef PWM_H
#define PWM_H  1

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PWM_TMPSIZE     16
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
              period;       /* PWM period (nanoseconds between rollover) */

  int         fddc;         /* duty_cycle file descriptor if manually opened */

  uint8_t     channel,      /* PWM channel (0 - pwm0, 1 - pwm1) */
              enabled;      /* PWM channel enabled (1 - enabled, 0 - not) */
} pwm_t;


/**
 * @brief atomic test that directory exists.
 * @param dir character string holding directory path to test.
 * @note all sysfs values must be written as character strings.
 * @return returns 0 on success, -1 otherwise.
 */
int dir_exists (const char *dir);


/**
 * @brief export the pwm channel set in the pwm struct.
 * @param pwm pointer to pwm struct to export channel.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_export (pwm_t *pwm);


/**
 * @brief unexport the pwm channel set in the pwm struct.
 * @param pwm pointer to pwm struct to unexport channel.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_unexport (pwm_t *pwm);


/**
 * @brief set the PWM period for channel in pwm struct, update period in struct.
 * @param pwm pointer to pwm struct for channel to set period for.
 * @param period pwm period in nanoseconds for Linux sysfs file.
 * @note all sysfs values must be written as character strings.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_set_period (pwm_t *pwm, uint32_t period);


/**
 * @brief set the PWM signal frequency (and period) from given frequency.
 * @param pwm pointer to pwm struct for channel to set frequency for.
 * @param frequency no. of PWM signals (Hz), 25 MHz sane limit for 3b+/zero.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_set_frequency (pwm_t *pwm, uint32_t frequency);


/**
 * @brief open duty_cycle sysfs file for repeated duty cycle writes.
 * @param pwm pointer to pwm struct for channel to open duty_cycle.
 * @return returns 0 on success, -1 otherwise.
 */
int sysfs_open_duty_cycle (pwm_t *pwm);


/**
 * @brief close the duty_cycle file descriptor and reset fd zero.
 * @param pwm pointer to pwm struct for channel to fd for.
 * @return returns 0 on success, -1 otherwise.
 */
int sysfs_close_duty_cycle (pwm_t *pwm);


/**
 * @brief write duty cycle for pwm channel to open sysfs duty_cycle file.
 * @param pwm pointer to pwm struct for channel to write duty_cycle for.
 * @param duty_cycle fraction of period in nanoseconds for signal high.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_write_duty_cycle (pwm_t *pwm, uint32_t duty_cycle);


/**
 * @brief set PWM duty cycle as a percentage of PWM period for open
 * sysfs duty_cycle file.
 * @param pwm pointer to pwm struct for channel to set duty cycle.
 * @param pct percent duty cycle, (valid range 0 - 100).
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_write_duty_cycle_pct (pwm_t *pwm, float pct);


/**
 * @brief set the PWM duty_cycle for channel in pwm struct, update period
 * and frequency in struct, duty_cycle sysfs file is opened and closed.
 * @param pwm pointer to pwm struct for channel to set period for.
 * @param duty_cycle pwm duty_cycle in nanoseconds as fraction of period.
 * @note all sysfs values must be written as character strings.
 * @return
 */
int pwm_set_duty_cycle (pwm_t *pwm, uint32_t duty_cycle);


/**
 * @brief set PWM duty cycle as a percentage of PWM period, sysfs
 * duty_cycle file is opened and closed.
 * @param pwm pointer to pwm struct for channel to set duty cycle.
 * @param pct percent duty cycle, (valid range 0 - 100).
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_set_duty_cycle_pct (pwm_t *pwm, float pct);


/**
 * @brief enable or disable PWM for channel specified in pwm struct.
 * @param pwm pointer to pwm struct for channel to enable/disable PWM.
 * @param enabled 1 - enable PWM on channel, 0 - disable PWM on channel.
 * @note all sysfs values must be written as character strings.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_enable_pwm (pwm_t *pwm, uint8_t enabled);


#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif
