#ifndef pwmsoft_h_
#define pwmsoft_h_  1

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/types.h>
#include <signal.h>

#include <itimer.h>

/* gpiochipX (0-4) depending on Pi model */
#define GPIOCHIP        "/dev/gpiochip0"

/* 64-bit and 32-bit limits differ by 100%, 32-bit OS much more capable
 * of producing PWM signal with twice the frequecy before saturating single-
 * core of processor (physical limit to produce stable PWM).
 */
#if defined(__LP64__) || defined(_LP64)
#define MAXTICKS            25000
#else
#define MAXTICKS            50000
#endif
#define NGPIOPINS              54

typedef struct {
  __u32 period,           /* clock period in nanoseconds (1e9 / Hz) */
        frequency,        /* no. of PWM cycles / sec (Hz) */
        range;            /* no. of divisions per PWM cycle */
  itimer clock;           /* instance of interval timer clock */
  __u8  configured,       /* flag - struct configured */
        enabled;          /* flag - clock enabled */
} pwmclk_t;

typedef struct {
  __u8  gpio,             /* GPIO pin no. */
        dutycycle;        /* (on_cnt / range) * 100 (%) */
  __u16 on_cnt,           /* no. of divisions pin set HI */
        off_cnt;          /* no. of divisions pin set LO */
} softpwm_t;

extern softpwm_t *pwmpins;    /* pointer to pins array */

/**
 * @brief calculate and set the period in nanoseconds for software PWM clock.
 * @param pwmclk pointer to valid memory holding instance of pwmclk_t struct.
 * @param frequency PWM frequency in Hz (no. of full PWM cycles per-period).
 * @param range no. of timer clock-ticks per PWM cycle (no. of increments in
 * PWM dutycycle).
 * @return returns 0 on success, -1 otherwise.
 */
int pwmclk_set_period (pwmclk_t *pwmclk, __u32 frequency, __u32 range);

/**
 * @brief create interval timer for instance of pwmclk_t and populate struct
 * values for pwmclk.
 * @param pwmclk pointer to struct pwmclk_t associated with interval timer
 * created to drive software PWM.
 * @note this creates the interval timer, but the timer clock is not started.
 * @param frequency PWM frequency in Hz (no. of full PWM cycles per-period).
 * @param range no. of timer clock-ticks per PWM cycle (no. of increments in
 * PWM dutycycle).
 * @return returns 0 on success, -1 otherwise.
 */
int pwmclk_create (pwmclk_t *pwmclk, __u32 frequency, __u32 range);

/**
 * @brief start the interval timer associated with instance of pwmclk_t.
 * @param pwmclk pointer to struct pwmclk_t associated with interval timer.
 * @return returns 0 on success, -1 otherwise.
 */
int pwmclk_start (pwmclk_t *pwmclk);

/**
 * @brief disable the interval timer associated with pwmclk_t instance.
 * @param pwmclk pointer to struct pwmclk_t associated with interval timer.
 * @return returns 0 on success, nonzero otherwise.
 */
int pwmclk_stop (pwmclk_t *pwmclk);

/**
 * @brief disable and delete interval timer associated with pwmclk_t instance.
 * @param pwmclk pointer to struct pwmclk_t associated with interval timer.
 * @return returns 0 on success, -1 otherwise.
 */
int pwmclk_delete (pwmclk_t *pwmclk);

/**
 * @brief add gpiopin to array of pininst to configure to use soft PWM.
 * @param pininst pointer to array of softpwm_t.
 * @param gpiopin pin number to add to pins controlled by soft PWM.
 * @return returns index within pininst on success, NGPIIOPINS otherwise.
 */
int pwm_pin_add (softpwm_t *pininst, __u8 gpiopin);

/**
 * @brief remove gpiopin from array of pininst shifting all pins in array
 * down by one index that have index in array greater than gpiopin.
 * @param pininst pointer to array of softpwm_t.
 * @param gpiopin gpio pin to remove from array of pins using soft PWM.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_pin_remove (softpwm_t *pininst, __u8 gpiopin);

/**
 * @brief configure the PWM dutycycle and off count for gpiopin given the on
 * count (number of clock ticks signal is HI in PWM range).
 * @param pwmclk pointer to struct pwmclk_t associated with interval timer.
 * @param pininst pointer to array of softpwm_t holding config for gpio pins
 * using soft PWM.
 * @param gpiopin gpio pin to configure dutycycle, off count and on count for.
 * @param on_count the no. of clock ticks PWM signal is HI for.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_pin_cfg_count (pwmclk_t *pwmclk, softpwm_t *pininst,
                        __u8 gpiopin, __u16 on_count);

/**
 * @brief set PWM dutycycle and compute off count, on count for gpiopin given
 * the dutycycle as a percentage of time signal is HI in PWM cycle.
 * @param pwmclk pointer to struct pwmclk_t associated with interval timer.
 * @param pininst pointer to array of softpwm_t holding config for gpio pins
 * using soft PWM.
 * @param gpiopin gpio pin to configure dutycycle, off count and on count for.
 * @param percent dutycycle as percent signal is HI in PWM cycle (0 - 100).
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_pin_cfg_dutycycle (pwmclk_t *pwmclk, softpwm_t *pininst,
                           __u8 gpiopin, __u8 percent);


/**
 * @brief set PWM dutycycle and off count for gpio pin at pinndx given the on
 * count (number of clock ticks signal is HI in PWM range).
 * @param pwmclk pointer to struct pwmclk_t associated with interval timer.
 * @param pininst pointer to array of softpwm_t holding config for gpio pins
 * using soft PWM.
 * @param pinndx index in both pwmpins array and linreq.offsets array.
 * @param on_count the no. of clock ticks PWM signal is HI for.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_set_on_count (pwmclk_t *pwmclk, softpwm_t *pininst,
                      __u8 pinndx, __u16 on_count);

/**
 * @brief set PWM dutycycle and compute off count, on count for gpio pin index
 * given the dutycycle as a percentage of time signal is HI in PWM cycle.
 * @param pwmclk pointer to struct pwmclk_t associated with interval timer.
 * @param pininst pointer to array of softpwm_t holding config for gpio pins
 * using soft PWM.
 * @param pinndx index in both pwmpins array and linreq.offsets array.
 * @param percent dutycycle as percent signal is HI in PWM cycle (0 - 100).
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_set_dutycycle (pwmclk_t *pwmclk, softpwm_t *pininst,
                        __u8 pinndx, __u8 percent);

#ifdef __cplusplus
}
#endif

#endif
