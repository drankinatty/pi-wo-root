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
#include <linux/types.h>


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
int dir_exists (const char *dir);


/**
 * @brief pwm_set_channel constructs string-literal from PWMPATH assigning
 * retuls to pwmfs member of struct, and sets chip and channel members.
 * @param pwm pointer to pwm_t struct to hold values.
 * @param chan pwm channel to construct devfs path to open.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_set_channel (pwm_t *pwm, __u8 chan);


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
 * @param period pwm period in samples within the 1E9 MHz PWM_CLOCK.
 * @note all sysfs values must be written as character strings.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_set_period (pwm_t *pwm, __u32 period);


/**
 * @brief set the PWM signal frequency (and period) from given frequency.
 * @param pwm pointer to pwm struct for channel to set frequency for.
 * @param frequency no. of PWM signals (Hz), 25 MHz sane limit for 3b+/zero.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_set_frequency (pwm_t *pwm, __u32 frequency);


/**
 * @brief open duty_cycle sysfs file for repeated duty cycle writes.
 * @param pwm pointer to pwm struct for channel to open duty_cycle.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_open_duty_cycle (pwm_t *pwm);


/**
 * @brief close the duty_cycle file descriptor and reset fd zero.
 * @param pwm pointer to pwm struct for channel to fd for.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_close_duty_cycle (pwm_t *pwm);


/**
 * @brief write duty cycle for pwm channel to sysfs duty_cycle file.
 * @param pwm pointer to pwm struct for channel to write duty_cycle for.
 * @param duty_cycle fraction of period in nanoseconds for signal high.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_write_duty_cycle (pwm_t *pwm, __u32 duty_cycle);


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
int pwm_set_duty_cycle (pwm_t *pwm, __u32 duty_cycle);


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
int pwm_enable_pwm (pwm_t *pwm, __u8 enabled);


#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif
