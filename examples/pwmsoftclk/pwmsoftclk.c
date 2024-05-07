/**
 *  Software PWM utilizing interval timers and gpio_v2 ioctl for Raspberry Pi
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <time.h>
#include <unistd.h>

#include <itimer.h>

/* gpiochipX (0-4) depending on Pi model */
#define GPIOCHIP  "/dev/gpiochip0"

#define MAXTICKS            20500
#define NGPIOPINS              54

static volatile sig_atomic_t clkcnt = 0;
static volatile sig_atomic_t clkmax = 0;

static volatile sig_atomic_t nextval = 0;

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

/* array of softpwm_t to hold gpio pin configs */
static softpwm_t pwmpin[NGPIOPINS] = {{ .gpio = 0 }};
static __u8 npins = 0;     /* number of configured pins */


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

  if (ticks > MAXTICKS) { /* sanity check frequency within 20.5 KHz max */
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
  if (itimer_start_timer (&pwmclk->clock) == -1) {
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

  if (itimer_delete_timer (&pwmclk->clock) == -1) {
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
 * @brief open gpiochipX device for control of gpio pins via ioctl().
 * @param gpiodev path to gpiochipX in filesystem, e.g. "/dev/gpiochip0".
 * @return returns file descriptor for device on success, -1 otherwise.
 */
int gpio_dev_open (const char *gpiodev)
{
  int gpiofd;

  /* open gpiochipX */
  if ((gpiofd = open (gpiodev, O_RDONLY)) < 0) {
    perror ("open-GPIOCHIP");
  }

  return gpiofd;
}


/**
 * @brief configure ioctl for gpio control using ABI V2 access.
 * @param gpiofd file descriptor returned by prior call to gpio_dev_open().
 * @param linecfg v2 ioctl config struct for gpio pins (lines).
 * @param linereq v2 ioctl request struct member of linecfg containing
 * separate file-descriptor used for read/write access of configured
 * gpio pins (lines).
 * @return returns 0 on success, -1 otherwise.
 */
int gpio_line_cfg_ioctl (int gpiofd,
                          struct gpio_v2_line_config *linecfg,
                          struct gpio_v2_line_request *linereq)
{
  /* get ioctl values for line request */
  if (ioctl (gpiofd, GPIO_V2_GET_LINE_IOCTL, linereq) < 0) {
    perror ("ioctl-GPIO_V2_GET_LINE_IOCTL");
    return -1;
  }

  /* set the line config for the retured linereq file descriptor */
  if (ioctl (linereq->fd, GPIO_V2_LINE_SET_CONFIG_IOCTL, linecfg) < 0) {
    perror ("ioctl-GPIO_V2_LINE_SET_CONFIG_IOCTL");
    return -1;
  }

  return 0;
}


/**
 * @brief write gpio pin (line) values (HI/LO) using ioct line request.
 * @param linereq gpio v2 line request struct holding linereq file descriptor
 * set by prior call to gpio_line_cfg_ioctl() used to write linevals to
 * gpio pins.
 * @param linevals gpio value to write (0 - LO, 1 - HI) configured in .bits
 * member of linevals struct.
 * @return returns 0 on success, -1 otherwise.
 */
int gpio_line_set_values (struct gpio_v2_line_request *linereq,
                          struct gpio_v2_line_values *linevals)
{
  /* set GPIO pin value to lineval->bits (0 or 1) */
  if (ioctl (linereq->fd, GPIO_V2_LINE_SET_VALUES_IOCTL, linevals) < 0) {
    perror ("ioctl-GPIO_V2_LINE_SET_VALUES_IOCTL-1");
    return -1;
  }

  return 0;
}


/**
 * @brief closes the open line request file descriptor for v2 linereq.
 * @param linereq gpio v2 line request struct holding linereq file descriptor
 * set by prior call to gpio_line_cfg_ioctl().
 * @return returns 0 on success, -1 otherwise.
 */
int gpio_line_close_fd (struct gpio_v2_line_request *linereq)
{
  if (close (linereq->fd) < 0) {    /* close linereq fd */
    perror ("close-linereq.fd");
    return -1;
  }

  return 0;
}


/**
 * @brief closes gpio file descriptor associated with "/dev/gpiochipX"
 * @param gpiofd file descriptor retured by previos call to gpio_dev_open().
 * @return returns 0 on success, -1 otherwise.
 */
int gpio_dev_close (int gpiofd)
{
  if (close (gpiofd) < 0) {         /* close gpiochipX fd */
    perror ("close-gpiofd");
    return -1;
  }

  return 0;
}


/* dump pwmclk_t values */
void prn_pwmclk (pwmclk_t *pwmclk)
{
  printf ("\npwmclk:\n"
          "  clock freq : %6u  (Hz)\n"
          "  period     : %6u  (ns)\n"
          "  PWM freq   : %6u  (Hz)\n"
          "  range      : %6u\n"
          "  clkcnt     : %6u\n"
          "  RT signo   : %6hhu\n"
          "  configured : %6s\n"
          "  enabled    : %6s\n",
          pwmclk->frequency * pwmclk->range, pwmclk->period,
          pwmclk->frequency, pwmclk->range,
          clkcnt, pwmclk->clock.signo,
          pwmclk->configured ? "true" : "false",
          pwmclk->enabled ? "true" : "false");
}


/* dump softpwm_t values */
void prn_pwmpin (softpwm_t *pin)
{
  for (__u8 i = 0; i < npins; i++) {
    printf ("\npwmpin:\n"
            "  GPIO       : %6hhu\n"
            "  dutycycle  : %6hhu  (%%)\n"
            "  off_cnt    : %6hu\n"
            "  on_cnt     : %6hu\n",
            pin[i].gpio, pin[i].dutycycle, pin[i].off_cnt, pin[i].on_cnt);
  }
}


/* poll for input within sec/nsec timeout period */
int pselect_timer (unsigned sec, unsigned nsec)
{
  fd_set set;
  int res;
  struct timespec ts = {  .tv_sec = sec,
                          .tv_nsec = nsec  };

  /* Initialize the file descriptor set. */
  FD_ZERO (&set);
  FD_SET (0, &set);

  /* watch STDIN_FILENO with timeout_sec timeout */
  res = pselect (1, &set, NULL, NULL, &ts, NULL);

  return res;
}


/* simple empty-stdin function, c must be initialized */
void empty_stdin (int c)
{
  while (c != '\n' && c != EOF) {
    c = getchar();
  }
}


int main (int argc, char **argv) {

  __u32 frequency = 64,
        range = 80,
        period = 0;
  __u16 dutycycle = 0,
        on_cnt = 0;
  __u8  pin = 0,
        dir = 1,
        dc = 0
#ifndef DEBUG
        ;
#else
        ,

        dcmin = 100,
        dcmax = 0;
#endif

  int gpiofd;

  struct gpio_v2_line_config linecfg = { .flags = GPIO_V2_LINE_FLAG_OUTPUT |
                                         GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN };
  struct gpio_v2_line_request linereq = { .offsets[0] = pwmpin[0].gpio,
                                          .consumer = "pwmsoftclock",
                                          .config = linecfg,
                                          .num_lines = 1 };

  /* gpio_v2_line_values with bit initially enabled for GPIO_WR_PIN */
  struct gpio_v2_line_values linevals = { .bits = 1, .mask = 1 };

  pwmclk_t pwmclk = { .period = 0 };

  if (argc > 1) {
    __u32 tmp;
    if (sscanf (argv[1], "%u" , &tmp) == 1) {
      frequency = tmp;
    }
  }
  if (argc > 2) {
    __u32 tmp;
    if (sscanf (argv[2], "%u" , &tmp) == 1) {
      range = tmp;
    }
  }
  if (argc > 3) {
    __u8 tmp;
    if (sscanf (argv[3], "%hhu" , &tmp) == 1) {
      pin = tmp;
    }
  }
  if (argc > 4) {
    __u16 tmp;
    if (sscanf (argv[4], "%hu" , &tmp) == 1) {
      if (argc == 5) {
        dutycycle = tmp;
      }
      else {
        on_cnt = tmp;
      }
    }
  }

  period = 1e9 / (frequency * range);

  puts ("\ncreating pwm clock from itimer interval timer, adding pins.\n");
  if (pwmclk_create (&pwmclk, frequency, range) == -1) {
    return 1;
  }

  if (dutycycle) {
    if (pwm_pin_cfg_dutycycle (&pwmclk, pwmpin, pin, dutycycle) == -1) {
      return 1;
    }
  }
  else {
    if (pwm_pin_cfg_count (&pwmclk, pwmpin, pin, on_cnt) == -1) {
      return 1;
    }
  }

  prn_pwmclk (&pwmclk);
  prn_pwmpin (pwmpin);

  puts ("\nconfiguring gpio");

  if ((gpiofd = gpio_dev_open (GPIOCHIP)) == -1) {
    return 1;
  }

  linereq.offsets[0] = pwmpin[0].gpio;

  if (gpio_line_cfg_ioctl (gpiofd, &linecfg, &linereq) == -1) {
    return 1;
  }

  printf ("\ngpio config:\n"
          "  gpiofd     : %d\n"
          "  linereqfd  : %d\n"
          "  lr_off[0]  : %hhu\n",
          gpiofd, linereq.fd, linereq.offsets[0]);

  puts ("\nstarting PWM clock:  (press Enter to Quit)");

  if (pwm_pin_cfg_dutycycle (&pwmclk, pwmpin, pin, dc) == -1) {
    fprintf (stderr, "error: setting dutycycle at %hhu\n", dc);
    return 1;
  }

  if (pwmclk_start (&pwmclk) == -1) {
    return 1;
  }
  // nextval = 1;    /* ensure dc is incremented on loop entry */

  prn_pwmclk (&pwmclk);
  putchar ('\n');
#ifdef DEBUG
  int chkclk = 0;
#endif
  do {

    if (nextval == 1) {
      nextval = 0;

      if (dc == 0) {
        dir = 1;
      }
      else if (dc == 100) {
        dir = 0;
      }

      dc += dir ? 1 : -1;

      if (pwm_pin_cfg_dutycycle (&pwmclk, pwmpin, pin, dc) == -1) {
        fprintf (stderr, "error: setting dutycycle at %hhu\n", dc);
        break;
      }
    }

#ifdef DEBUG
    if (dc > dcmax) {
      dcmax = dc;
    }
    if (dc < dcmin) {
      dcmin = dc;
    }
    if (clkcnt % 10 == 0) {
      if (dc < 98) {
        printf ("chk: %6d,  clkcnt: %3d,  clkmax: %d,  dc: %3hhu (%3hhu)\x1b[1A\n",
                chkclk, clkcnt, clkmax, dc, pwmpin[0].dutycycle);
      }
      else {
        printf ("chk: %6d,  clkcnt: %3d,  clkmax: %d,  dc: %3hhu (%3hhu)\n",
                chkclk, clkcnt, clkmax, dc, pwmpin[0].dutycycle);
      }
      chkclk++;
    }
#endif

    /* set gpio state on cnt == 0 (LO) / off_cnt == cnt (HI) */
    if (clkcnt == 0 && pwmpin[0].off_cnt) {
      linevals.bits = 0;
      if (gpio_line_set_values (&linereq, &linevals) == -1) {
        return 1;
      }
    }
    else if (clkcnt == pwmpin[0].off_cnt) {
      linevals.bits = 1;
      if (gpio_line_set_values (&linereq, &linevals) == -1) {
        return 1;
      }
    }

    if (pselect_timer (0, 1e8) == 1) {
      int c = getchar();
      /* check for quit (or empty input or EOF) */
      if (c == 'q' || c == '\n' || c == EOF) {
        break;
      }
      empty_stdin (c);
      (void)period;
    }
  } while (/*dc != 0*/ 1);

linevals.bits = 0;
if (gpio_line_set_values (&linereq, &linevals) == -1) {
  return 1;
}

#ifdef DEBUG
  printf ("\ndcmin : %hhu\ndcmax : %hhu\n", dcmin, dcmax);
#endif

  if (pwmclk_stop (&pwmclk) == -1) {
    return 1;
  }

  putchar ('\n');

  pwmclk_delete (&pwmclk);

  /* close gpio file descriptors */
  if (gpio_line_close_fd (&linereq) == -1) {
    return 1;
  }

  if (gpio_dev_close (gpiofd) == -1) {
    return 1;
  }
}

