/**
 *  Software PWM utilizing gpio_v2 ioctl and lib pthread for Raspberry Pi
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
#include <pthread.h>
#include <errno.h>

#include <itimer.h>     /* itimer library */
#include <pwmsoft.h>    /* pwmsoft library */

/* extern variables from pwmsoft library */
extern volatile sig_atomic_t clkcnt;        /* pwm clock tick count */
extern volatile sig_atomic_t nextval;       /* pwm clock rollowver */
extern __u8 npins;

/* single use struct to combine needed values for sending to timer thread */
typedef struct {
  struct gpio_v2_line_config *linecfg;
  struct gpio_v2_line_request *linereq;
  struct gpio_v2_line_values *linevals;
  pwmclk_t *pwmclk;
  int gpiofd;
  __u8 *pwmrun;
} pwmgpio_t;

/**
 * pthread validation macros
 */
#define handle_error_en(en, msg) \
  do { if (en) { errno = en; perror(msg); exit(EXIT_FAILURE); }} while (0)

#define handle_error(msg) \
  do { perror(msg); exit(EXIT_FAILURE); } while (0)

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


/**
 * @brief pselect wrapper to poll on stdin with sec, nsec timeout.
 * @param sec no. of whole seconds in timeout.
 * @param nsec no. or nanoseconds for any fractional part of timeout.
 * @return no. of file descriptors ready to be read on success, -1 otherwise.
 */
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


/**
 * @brief simple helper function to clear characters in stdin.
 * @param c last character read from stdin (must be initialized).
 */
void empty_stdin (int c)
{
  while (c != '\n' && c != EOF) {
    c = getchar();
  }
}


/**
 * @brief pthread version - blocks timer signal for itimer->signo.
 * @param it itimer struct returned by itimer_create_timer.
 * @return returns 0 on success, -1 otherwise.
 */
int itimer_pthread_block_timer (itimer *it)
{
  sigset_t mask;

  sigemptyset (&mask);
  sigaddset (&mask, it->signo);
  if (pthread_sigmask (SIG_BLOCK, &mask, NULL) == -1) {
    perror ("sigprocmask SIG_UNBLOCK");
    return -1;
  }

  return 0;
}


/**
 * @brief pthread version - unblocks timer signal previously blocked by
 * call to itimer_block_timer.
 * @param it itimer struct returned by itimer_create_timer.
 * @return returns 0 on success, -1 otherwise.
 */
int itimer_pthread_unblock_timer (itimer *it)
{
  sigset_t mask;

  sigemptyset (&mask);
  sigaddset (&mask, it->signo);
  if (pthread_sigmask (SIG_UNBLOCK, &mask, NULL) == -1) {
    perror ("sigprocmask SIG_UNBLOCK");
    return -1;
  }

  return 0;
}


/**
 * @brief pthread thread function to accept interval timer signal to drive
 * PWM and set dutycycle for pin providing PWM signal. Currently it simply
 * increments dutycycle by 1 from 0 - 100 and then decrements dutycycle by 1
 * from 100 - 0 through full range of PWM dutycycles.
 * @param data pointer to pwmgpio_t struct holding timer and ioctl struct and
 * in the future the gpio pin and PWM flags and dutycycle variables.
 * @return pointer to data struct (currently unmodified).
 */
void *signal_thread (void *data)
{
  /* cast and assignment of variables past through data */
  pwmgpio_t *pwmgpio = (pwmgpio_t*)data;
  // struct gpio_v2_line_config *linecfg = pwmgpio->linecfg;
  struct gpio_v2_line_request *linereq = pwmgpio->linereq;
  struct gpio_v2_line_values *linevals = pwmgpio->linevals;
  pwmclk_t *pwmclk = pwmgpio->pwmclk;
  itimer *itimer = &pwmclk->clock;

  __u8  dc = 0,
        dir = 1,
        pinndx = 0,
        lastndx = 0;

  /* unblock the timer signal for PWM */
  if (itimer_pthread_unblock_timer (itimer) == -1) {
    pwmgpio->pwmrun = 0;
  }

  /* interval timer processing loop to set PWM state */
  while (pwmgpio->pwmrun) {
    if (nextval == 1) {         /* if timer rollover occurred */
      nextval = 0;              /* reset flag zero */

      if (dc == 0) {            /* if dutycycle zero, set dir to increment */
        if (dir == 0) {         /* if starting new color */
          lastndx = pinndx;     /* save current index to disable */
          pinndx += 1;          /* increment pin index */
          pinndx %= npins;      /* mod to rollowver on max index */
        }
        dir = 1;
      }
      else if (dc == 100) {     /* if at 100 percent, set dir to decrement */
        dir = 0;
      }

      dc += dir ? 1 : -1;       /* increment or decrement dutycycle */

      /* update pwmclk struct, set PWM duty cycle for pin */
      if (pwm_set_dutycycle (pwmclk, pwmpins, pinndx, dc) == -1) {
        fprintf (stderr, "error: signal_thread() setting dutycycle at %hhu\n",
                  dc);
        break;
      }
    }

    /*
     * set gpio PWM state, on cnt < off_cnt (LO) / off_cnt <= cnt (HI)
     */
    /* set pin state low on rollowver if dutycycle not 100% (0 off_cnt) */
    if (clkcnt == 0) {
      linevals->bits = 0;     /* gpio_v2 ioctl pin values bitmap all 0 */
      /* gpio_v2 ioctl request to set linevals on pin(s)
       * set last pin LO on rollowever
       */
      if (pinndx != lastndx) {
        linevals->mask = (1u << lastndx);
        if (gpio_line_set_values (linereq, linevals) == -1) {
          fputs ("error: thread gpio_line_set_values - lastndx LO\n", stderr);
          pwmgpio->pwmrun = 0;
          break;
        }
      }
      /* set current pin LO if dutycycle not 100% (i.e. off_cnt != 0) */
      if (pwmpins[pinndx].off_cnt) {
        linevals->mask = (1u << pinndx);    /* set bit on for pin index */
        if (gpio_line_set_values (linereq, linevals) == -1) {
          fputs ("error: thread gpio_line_set_values - pinndx LO\n", stderr);
          pwmgpio->pwmrun = 0;
          break;
        }
      }
    } /* otherwise set pin high for on_cnt (from off_cnt <= cnt < clkmax) */
    else if (clkcnt == pwmpins[pinndx].off_cnt) {
      /* set linevals bit for both .bits (HI) and .mask to pin index */
      linevals->bits = linevals->mask = (1u << pinndx);
      if (gpio_line_set_values (linereq, linevals) == -1) {
        fputs ("error: thread gpio_line_set_values - pinndx HI\n", stderr);
        pwmgpio->pwmrun = 0;
        break;
      }
    }

    sleep (1);    /* sleep until interrupted by timer signal */
  }

  itimer_pthread_block_timer (itimer);    /* (optional) block timer on exit */

  return data;    /* returned unchanged */
}


/* dump pwmclk_t values */
void prn_pwmclk (pwmclk_t *pwmclk)
{
  printf ("\npwmclk config\n"
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
    printf ("\npwmpin[%hhu]\n"
            "  GPIO       : %6hhu\n"
            "  dutycycle  : %6hhu  (%%)\n"
            "  off_cnt    : %6hu\n"
            "  on_cnt     : %6hu\n",
            i, pin[i].gpio, pin[i].dutycycle,
            pin[i].off_cnt, pin[i].on_cnt);
  }
}


/* dump gpio_v2 gpiofd and linereq state */
void prn_gpio_v2_cfg (int gpiofd, struct gpio_v2_line_request *linereq)
{
  printf ("\nlinereq config\n"
          "  gpiofd     : %6d\n"
          "  .fd        : %6d\n"
          "  .num_lines : %6u\n"
          "  .consumer  :      %s\n\n"
          "  pins\n", gpiofd,
          linereq->fd, linereq->num_lines, linereq->consumer);

  for (__u8 l = 0; l < linereq->num_lines; l++) {
    printf ("  .offset[%hhu] : %6hhu\n", l, linereq->offsets[l]);
  }
}


/* accepts 3 GPIO pin numbers to light RGB LED (default pins: 5, 6, 16) */
int main (int argc, char **argv) {

  __u32 frequency = 64,     /* PWM frequency (PWM cycles per-second) */
        range     = 100;    /* no. of ticks (levels) per PWM cycle */
  __u16 dutycycle = 0;      /* dutycycle (on count) / range */
  __u8  pwmrun    = 1,      /* flag controlled loop in thread function */
        pinred    = 5,      /* GPIO pin for red diode in RGB LED */
        pingreen  = 6,      /* GPIO pin for green diode in RGB LED */
        pinblue   = 16;     /* GPIO pin for blue diode in RGB LED */

  int gpiofd;

  /* gpio_v2 ioctl line (pin) config and line (pin) request structs */
  struct gpio_v2_line_config linecfg = { .flags = GPIO_V2_LINE_FLAG_OUTPUT |
                                          GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN };
  struct gpio_v2_line_request linereq = { .offsets = {pinred,
                                                      pingreen,
                                                      pinblue},
                                          .consumer = "pwmsoftclk",
                                          .config = linecfg,
                                          .num_lines = 3 };

  /* gpio_v2_line_values with bit initially disabled for GPIO pins.
   * both .bits and .mask are bitmaps where each bit corresponds to an
   * line (pin) index in the .offsets array. if the bit is on in .bits
   * the pin value is HI. if the bit is on in .mask, the linereq will
   * set the corresponding pin value in .bits for the pin at that bit
   * index in .offsets. linevals below has all valuse (.bits) LO and
   * the LO will be set on indexes 0, 1, 2 for the pins in .offsets.
   * (in .masks 7 == 0b00000111)
   */
  struct gpio_v2_line_values linevals = { .bits = 0, .mask = 7 };

  pwmclk_t pwmclk = { .period = 0 };    /* PWM clock struct */

  /* struct to pass GPIO and clock as data parameter to thread function */
  pwmgpio_t pwmgpio = { .linecfg = &linecfg, .linereq = &linereq,
                        .linevals = &linevals, .pwmclk = &pwmclk,
                        .pwmrun = &pwmrun };

  pthread_t id;                       /* pwm signal thread id */
  pthread_attr_t attr;                /* pwm thread attributes */
  void *res;                          /* results pointer */

  /* process command line arguments (if provided) */
  if (argc > 1) {   /* GPIO pin - red diode */
    __u8 tmp;
    if (sscanf (argv[1], "%hhu" , &tmp) == 1) {
      pinred = tmp;
    }
  }
  if (argc > 2) {   /* GPIO pin - green diode */
    __u8 tmp;
    if (sscanf (argv[2], "%hhu" , &tmp) == 1) {
      pingreen = tmp;
    }
  }
  if (argc > 3) {   /* GPIO pin - blue diode */
    __u8 tmp;
    if (sscanf (argv[3], "%hhu" , &tmp) == 1) {
      pinblue = tmp;
    }
  }

  puts ("\ncreating pwm clock from itimer interval timer, adding pins.\n");
  /* create pwm clock from interval timer */
  if (pwmclk_create (&pwmclk, frequency, range) == -1) {
    return 1;
  }

  puts ("adding GPIO pins to pwmpins array and set dutycycle zero\n");
  /* add gpio pins, configure with default dutycycle (0) */
  if (pwm_pin_cfg_dutycycle (&pwmclk, pwmpins, pinred, dutycycle) == -1) {
    return 1;
  }
  if (pwm_pin_cfg_dutycycle (&pwmclk, pwmpins, pingreen, dutycycle) == -1) {
    return 1;
  }
  if (pwm_pin_cfg_dutycycle (&pwmclk, pwmpins, pinblue, dutycycle) == -1) {
    return 1;
  }

  puts ("blocking interval timer signal globally.");
  /* block interval timer signal from being handled */
  if (itimer_pthread_block_timer (&pwmclk.clock) == -1) {
    return 1;
  }

  /* dump current clock and pin config to stdout (optional) */
  prn_pwmclk (&pwmclk);
  prn_pwmpin (pwmpins);

  printf ("\nGPIO_V2 ABI config using '%s'\n", GPIOCHIP);

  /* open gpiochipX device for pin control with ioctl calls */
  if ((gpiofd = gpio_dev_open (GPIOCHIP)) == -1) {
    return 1;
  }
  pwmgpio.gpiofd = gpiofd;

  /* set gpio_v2 linereq offsets for all three pins from pwmpins array */
  for (__u8 p = 0; p < linereq.num_lines; p++) {
    linereq.offsets[p] = pwmpins[p].gpio;
  }

  /* configure GPIO linecnf with linereq and open linereq file descriptor */
  if (gpio_line_cfg_ioctl (gpiofd, &linecfg, &linereq) == -1) {
    return 1;
  }

  /* sell all 3 GPIO pins LO initially (with values and mask in linevals) */
  if (gpio_line_set_values (&linereq, &linevals) == -1) {
    fputs ("error: gpio_line_set_values (linereq, linevals) - all\n",
            stderr);
    return 1;
  }

  /* dump of gpio configured values to stdout (optional) */
  prn_gpio_v2_cfg (gpiofd, &linereq);

  puts ("\ncreating separate thread to handle interval timer signal");

  /* initialize thread attributes (using defaults) and validate */
  handle_error_en (pthread_attr_init (&attr), "pthread_attr_init");

  /* create thread to handle PWM interval timer signal / validate */
  handle_error_en (pthread_create (&id, &attr, signal_thread, &pwmgpio),
                   "pthread_create");

  puts ("\nenabling PWM soft-clock interval timer signal:");
  /* start interval timer to drive PWM */
  if (pwmclk_start (&pwmclk) == -1) {
    return 1;
  }

  /* dump PWM clock config before main program loop (optional) */
  prn_pwmclk (&pwmclk);

  puts ("\n[ ==> press Enter to Quit ]");
#ifdef DEBUG
  __u64 loopcnt = 0;
#endif

  /* main program loop - just loop waiting for input to quit */
  do {
    if (pselect_timer (1, 0) == 1) {  /* loop with 1 sec timeout */
      int c = getchar();
      /* check for quit (or empty input or EOF) */
      if (c == 'q' || c == '\n' || c == EOF) {
        pwmgpio.pwmrun = 0;                     /* stop thread loop */
        break;
      }
      empty_stdin (c);
    }
#ifdef DEBUG
    loopcnt++;
#endif
  } while (1);

  /* set all gpio pin values LO on program exit */
  linevals.bits = 0;
  linevals.mask = 7;
  if (gpio_line_set_values (&linereq, &linevals) == -1) {
    return 1;
  }

  usleep (100000);    /* give time for thread to exit */

  /* wait for thread to exit */
  handle_error_en (pthread_join (id, &res), "pthread_join");

  /* stop interval timer */
  if (pwmclk_stop (&pwmclk) == -1) {
    return 1;
  }

  /* delete interval timer */
  pwmclk_delete (&pwmclk);

  /* close gpio file descriptors for linereq */
  if (gpio_line_close_fd (&linereq) == -1) {
    return 1;
  }

  /* close file descriptor for /dev/gpiochipX */
  if (gpio_dev_close (gpiofd) == -1) {
    return 1;
  }

#ifdef DEBUG
  printf ("main program loop count : %llu\n", loopcnt);
#endif
}

