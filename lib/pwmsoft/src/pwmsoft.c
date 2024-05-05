#include <stdio.h>
#include <stdlib.h>

#include "pwmsoft.h"

volatile sig_atomic_t clkcnt = 0;
static volatile sig_atomic_t clkmax = 0;

volatile sig_atomic_t nextval = 0;

/* array of softpwm_t to hold gpio pin configs */
static softpwm_t pwmpin[NGPIOPINS] = {{ .gpio = 0 }};
__u8 npins = 0;        /* number of configured pins */

softpwm_t *pwmpins = pwmpin;  /* pointer to provide extern in header */

/**
 * @brief calculate and set the period in nanoseconds for software PWM clock.
 * @param pwmclk pointer to valid memory holding instance of pwmclk_t struct.
 * @param frequency PWM frequency in Hz (no. of full PWM cycles per-period).
 * @param range no. of timer clock-ticks per PWM cycle (no. of increments in
 * PWM dutycycle).
 * @return returns 0 on success, -1 otherwise.
 */
int pwmclk_set_period (pwmclk_t *pwmclk, __u32 frequency, __u32 range)
{
  __u32 ticks = frequency * range;  /* compute required total clock frequency */

  if (range != ticks / frequency) { /* check for overflow */
    fputs ("error: pwmclk_set_period() frequency * range overflow 32-bit.\n",
            stderr);
    return -1;
  }

  if (ticks > MAXTICKS) { /* sanity check frequency within MAXTICKS Hz */
    fprintf (stderr, "error: pwmclk_set_period() frequency * range = %u Hz\n"
                     "       exceeds allowable max of %u Hz\n",
                     ticks, MAXTICKS);
    return -1;
  }

  /* set pwm clock struct values */
  pwmclk->frequency = frequency;
  pwmclk->range = range;
  pwmclk->period = 1e9 / ticks;

  return 0;
}


/**
 * @brief signal handler for softpwm timer maintains the PWM clock count.
 * @param sig real-time signal number (unused).
 * @param si pointer to siginfo struct associated with signal (unused).
 * @param uc pointer to ucontext_t struct containing signal context information
 * placed on the user-stack by the kernel (unused and rarely used in any
 * circumstance).
 */
static void sighdlr_pwmclk (int sig, siginfo_t *si, void *uc)
{
  if (clkcnt < clkmax) {    /* if less than max, increment */
    clkcnt += 1;
  }
  else {                    /* otherwise reset 0 and set nextval flag */
    clkcnt = 0;
    nextval = 1;
  }

  (void)sig;                /* suppress -Wunused warnings */
  (void)si;
  (void)uc;
}


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
int pwmclk_create (pwmclk_t *pwmclk, __u32 frequency, __u32 range)
{
  /* validate calculation and setting of PWM period in nanoseconds */
  if (pwmclk_set_period (pwmclk, frequency, range) == -1) {
    return -1;
  }

  clkmax = range;     /* set global clkmax value for signal handler */

  /* create the interval timer to drive PWM, save timer instance */
  pwmclk->clock = itimer_create_timer (10, pwmclk->period, sighdlr_pwmclk);

  /* validate timer creation */
  if (pwmclk->clock.signo < 0) {
    errexit ("itimer_create_timer-pwmclk_create()");
  }

  pwmclk->configured = 1;   /* set configured flag true */

  return 0;
}


/**
 * @brief start the interval timer associated with instance of pwmclk_t.
 * @param pwmclk pointer to struct pwmclk_t associated with interval timer.
 * @return returns 0 on success, -1 otherwise.
 */
int pwmclk_start (pwmclk_t *pwmclk)
{
  if (pwmclk->enabled) {    /* if clock already started, return success */
    return 0;
  }

  /* start timer for PWM and validate */
  if (itimer_start_timer (&pwmclk->clock) == 0) {
    fputs ("error: pwmclk_start() failed to start itimer.\n", stderr);
    return -1;
  }

  pwmclk->enabled = pwmclk->clock.enabled;  /* set clock enabled flag true */

  return 0;
}


/**
 * @brief disable the interval timer associated with pwmclk_t instance.
 * @param pwmclk pointer to struct pwmclk_t associated with interval timer.
 * @return returns 0 on success, nonzero otherwise.
 */
int pwmclk_stop (pwmclk_t *pwmclk)
{
  if (pwmclk->enabled) {
    itimer_stop_timer (&pwmclk->clock);
  }

  return pwmclk->enabled = pwmclk->clock.enabled;
}


/**
 * @brief disable and delete interval timer associated with pwmclk_t instance.
 * @param pwmclk pointer to struct pwmclk_t associated with interval timer.
 * @return returns 0 on success, -1 otherwise.
 */
int pwmclk_delete (pwmclk_t *pwmclk)
{
  pwmclk_stop (pwmclk);

  if (itimer_delete_timer (&pwmclk->clock) == 0) {
    return -1;
  }

  pwmclk->configured = 0;

  return 0;
}


/**
 * @brief static helper - gets index in pwmpin array for gpiopin.
 * @param pininst pointer to array of struct softpwm_t holding PWM
 * configurations for one or more gpio pins.
 * @param gpiopin gpio pin to obtain index in pininst for.
 * @return returns index for gpiopin in pininst on success (0 to npins - 1),
 * returns npins otherwise for gpiopin not found or out-of-range.
 */
static __u8 pwm_pin_get_index (softpwm_t *pininst, __u8 gpiopin)
{
  __u8 ndx = 0;     /* index in pininst array */

  if (gpiopin >= NGPIOPINS) {   /* validate gpiopin in valid range */
    fprintf (stderr, "error: gpiopin parameter (%hhu) "
              "out of range of gpiopins: (0 - %hhu)\n",
              gpiopin, NGPIOPINS);
    return npins;
  }

  /* loop to find gpiopin in pininst array */
  for (; ndx < npins; ndx++) {
    if (pininst[ndx].gpio == gpiopin) {
      break;
    }
  }

  return ndx;     /* return indix if found or npins otherwise */
}


/**
 * @brief add gpiopin to array of pininst to configure to use soft PWM.
 * @param pininst pointer to array of softpwm_t.
 * @param gpiopin pin number to add to pins controlled by soft PWM.
 * @return returns index within pininst on success, NGPIIOPINS otherwise.
 */
int pwm_pin_add (softpwm_t *pininst, __u8 gpiopin)
{
  __u8 ndx;     /* index in pininst array */

  if (npins == NGPIOPINS) {   /* validate storage available for new pininst */
    fputs ("warning: pwmpin array full.\n", stderr);
    return -1;
  }

  /* if pin already exists in pininst, return index */
  if ((ndx = pwm_pin_get_index (pininst, gpiopin)) < npins) {
#ifdef DEBUGPINS
    printf ("note: pin %hhu exists in pin array at index %hhu.\n",
            gpiopin, ndx);
#endif
    return ndx;
  }

  /* increment global npins adding element to pininst array */
  pininst[npins++].gpio = gpiopin;

  return ndx;     /* return index for gpiopin */
}


/**
 * @brief remove gpiopin from array of pininst shifting all pins in array
 * down by one index that have index in array greater than gpiopin.
 * @param pininst pointer to array of softpwm_t.
 * @param gpiopin gpio pin to remove from array of pins using soft PWM.
 * @return returns 0 on success, -1 otherwise.
 */
int pwm_pin_remove (softpwm_t *pininst, __u8 gpiopin)
{
  /* get index of pin to remove from soft PWM control */
  __u8 ndx = pwm_pin_get_index (pininst, gpiopin);

  if (ndx == npins) { /* handle error for not found or out-of-range */
    fprintf (stderr, "error: pwm_pin_remove(), pin not found: %hhu.\n",
              gpiopin);
    return -1;
  }

  /* shift all remaining pins with index greater than gpiopin down by one */
  for (; ndx < npins; ndx++) {
    pininst[ndx] = pininst[ndx + 1];
  }

  npins -= 1;   /* decrement npins count */

  return 0;
}


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
                        __u8 gpiopin, __u16 on_count)
{
  __u8 pinndx = npins;
  int  tmp = 0;

  /* if on count exceeds PWM range, set to range */
  if (on_count > pwmclk->range) {
    on_count = pwmclk->range;
  }

  /* add pin if not already in array of pins, bail on error */
  if ((tmp = pwm_pin_add (pininst, gpiopin)) == -1) {
    return -1;
  }
  pinndx = (__u8)tmp;   /* use index if pin already in array */

  /* compute dutycycle and off_cnt from on_cnt, assign on_cnt */
  pininst[pinndx].dutycycle = (on_count * 100) / pwmclk->range;
  pininst[pinndx].on_cnt    = on_count;
  pininst[pinndx].off_cnt   = pwmclk->range - on_count;

  return 0;
}


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
                           __u8 gpiopin, __u8 percent)
{
  __u8 pinndx = npins;
  int  tmp = 0;

  if (percent > 100) {    /* if dutycycle exceeds 100, set to 100 */
    percent = 100;
  }

  /* add pin if not already in array of pins, bail on error */
  if ((tmp = pwm_pin_add (pininst, gpiopin)) == -1) {
    return -1;
  }
  pinndx = (__u8)tmp;   /* use index if pin already in array */

  /* compute on_cnt and off_cnt from dutycycle */
  pininst[pinndx].dutycycle = percent;
  pininst[pinndx].on_cnt    = (percent * pwmclk->range) / 100;
  pininst[pinndx].off_cnt   = pwmclk->range - pininst[pinndx].on_cnt;

  return 0;
}


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
                      __u8 pinndx, __u16 on_count)
{
  if (pinndx >= npins) {
    fprintf (stderr, "error: pwm_on_count_set pinndx param (%hhu) exceeds "
                      " last index (%hhu)\n", pinndx, npins ? npins - 1 : 0);
    return -1;
  }

  /* if on count exceeds PWM range, set to range */
  if (on_count > pwmclk->range) {
    on_count = pwmclk->range;
  }

  /* compute dutycycle and off_cnt from on_cnt, assign on_cnt */
  pininst[pinndx].dutycycle = (on_count * 100) / pwmclk->range;
  pininst[pinndx].on_cnt    = on_count;
  pininst[pinndx].off_cnt   = pwmclk->range - on_count;

  return 0;
}


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
                        __u8 pinndx, __u8 percent)
{
  if (pinndx >= npins) {
    fprintf (stderr, "error: pwm_on_count_set pinndx param (%hhu) exceeds "
                      " last index (%hhu)\n", pinndx, npins ? npins - 1 : 0);
    return -1;
  }

  if (percent > 100) {    /* if dutycycle exceeds 100, set to 100 */
    percent = 100;
  }

  /* compute on_cnt and off_cnt from dutycycle */
  pininst[pinndx].dutycycle = percent;
  pininst[pinndx].on_cnt    = (percent * pwmclk->range) / 100;
  pininst[pinndx].off_cnt   = pwmclk->range - pininst[pinndx].on_cnt;

  return 0;
}


