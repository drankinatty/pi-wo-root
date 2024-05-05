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
  __u8 *pwmrun;
} pwmgpio_t;

/* TODO: add to pwmgpio_t and remove from global scope, declare in main */
// static volatile sig_atomic_t  dir = 1,      /* flag - count up/down */
//                               dc = 0;       /* pwm dutycycle */
//
// static volatile __u8 pwmrun = 1;            /* flag - thread loop control */

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
  struct gpio_v2_line_request *linereq = pwmgpio->linereq;
  struct gpio_v2_line_values *linevals = pwmgpio->linevals;
  pwmclk_t *pwmclk = pwmgpio->pwmclk;
  itimer *itimer = &pwmclk->clock;

  __u8  dc = 0,
        dir = 1;

  /* unblock the timer signal for PWM */
  if (itimer_pthread_unblock_timer (itimer) == -1) {
    pwmgpio->pwmrun = 0;
  }

  /* interval timer processing loop to set PWM state */
  while (pwmgpio->pwmrun) {
    if (nextval == 1) {         /* if timer rollover occurred */
      nextval = 0;              /* reset flag zero */

      if (dc == 0) {            /* if dutycycle zero, set to increment */
        dir = 1;
      }
      else if (dc == 100) {     /* if at 100 percent, set to decrement */
        dir = 0;
      }

      dc += dir ? 1 : -1;       /* increment or decrement dutycycle */

      /* update pwmclk struct, set PWM duty cycle for pin */
      if (pwm_pin_cfg_dutycycle (pwmclk, pwmpins, pwmpins[0].gpio, dc) == -1) {
        fprintf (stderr, "error: setting dutycycle at %hhu\n", dc);
        break;
      }
    }

    /*
     * set gpio PWM state, on cnt < off_cnt (LO) / off_cnt <= cnt (HI)
     */
    /* set pin state low on rollowver if dutycycle not 100% (0 off_cnt) */
    if (clkcnt == 0 && pwmpins[0].off_cnt) {
      linevals->bits = 0;     /* gpio_v2 ioctl pin state setting */
      /* gpio_v2 ioctl request to set value on pin */
      if (gpio_line_set_values (linereq, linevals) == -1) {
        fputs ("error: thread gpio_line_set_values (linereq, linevals)\n",
                stderr);
        pwmgpio->pwmrun = 0;
        break;
      }
    } /* otherwise set pin high for on_cnt (from off_cnt <= cnt < clkmax) */
    else if (clkcnt == pwmpins[0].off_cnt) {
      linevals->bits = 1;
      if (gpio_line_set_values (linereq, linevals) == -1) {
        fputs ("error: thread gpio_line_set_values (linereq, linevals)\n",
                stderr);
        pwmgpio->pwmrun = 0;
        break;
      }
    }

    sleep (1);    /* sleep until interrupted by timer signal */
  }

  itimer_pthread_block_timer (itimer);    /* (optional) block timer on exit */

  return data;
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
void prn_pwmpin (softpwm_t *_pin)
{
  for (__u8 i = 0; i < npins; i++) {
    printf ("\npwmpin:\n"
            "  GPIO       : %6hhu\n"
            "  dutycycle  : %6hhu  (%%)\n"
            "  off_cnt    : %6hu\n"
            "  on_cnt     : %6hu\n",
            _pin[i].gpio, _pin[i].dutycycle,
            _pin[i].off_cnt, _pin[i].on_cnt);
  }
}


int main (int argc, char **argv) {

  __u32 frequency = 64,
        range = 80;
  __u16 dutycycle = 0,
        on_cnt = 0;
  __u8  pin = 0,
        pwmrun = 1;

  int gpiofd;

  /* gpio_v2 ioctl line (pin) config and request structs */
  struct gpio_v2_line_config linecfg = { .flags = GPIO_V2_LINE_FLAG_OUTPUT |
                                          GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN };
  struct gpio_v2_line_request linereq = { .offsets[0] = pwmpins[0].gpio,
                                          .consumer = "pwmsoftclock",
                                          .config = linecfg,
                                          .num_lines = 1 };

  /* gpio_v2_line_values with bit initially enabled for GPIO PIN */
  struct gpio_v2_line_values linevals = { .bits = 1, .mask = 1 };

  pwmclk_t pwmclk = { .period = 0 };

  pwmgpio_t pwmgpio = { .linecfg = &linecfg, .linereq = &linereq,
                        .linevals = &linevals, .pwmclk = &pwmclk,
                        .pwmrun = &pwmrun };

  pthread_t id;                       /* pwm signal thread id */
  pthread_attr_t attr;                /* pwm thread attributes */
  void *res;                          /* results pointer */

  /* process command line arguments (if provided) */
  if (argc > 1) {   /* frequency */
    __u32 tmp;
    if (sscanf (argv[1], "%u" , &tmp) == 1) {
      frequency = tmp;
    }
  }
  if (argc > 2) {   /* range */
    __u32 tmp;
    if (sscanf (argv[2], "%u" , &tmp) == 1) {
      range = tmp;
    }
  }
  if (argc > 3) {   /* gpio pin for pwm */
    __u8 tmp;
    if (sscanf (argv[3], "%hhu" , &tmp) == 1) {
      pin = tmp;
    }
  }
  if (argc > 4) {   /* dutycycle or on_cnt based on argv[5] */
    __u16 tmp;
    if (sscanf (argv[4], "%hu" , &tmp) == 1) {
      if (argc == 5) {    /* if present, the tmp is dutycycle */
        dutycycle = tmp;
      }
      else {              /* otherwise it's on_cnt */
        on_cnt = tmp;
      }
    }
  }

  puts ("\ncreating pwm clock from itimer interval timer, adding pins.\n");
  /* create pwm clock from interval timer */
  if (pwmclk_create (&pwmclk, frequency, range) == -1) {
    return 1;
  }

  if (dutycycle) {  /* if dutycycle provided, set PWM from dutycycle */
    if (pwm_pin_cfg_dutycycle (&pwmclk, pwmpins, pin, dutycycle) == -1) {
      return 1;
    }
  }
  else {  /* otherwise set PWM from on_cnt */
    if (pwm_pin_cfg_count (&pwmclk, pwmpins, pin, on_cnt) == -1) {
      return 1;
    }
  }

  puts ("blocking timer signal globally.");
  /* block interval timer signal from being handled */
  if (itimer_pthread_block_timer (&pwmclk.clock) == -1) {
    return 1;
  }

  /* dump current clock and pin config to stdout (optional) */
  prn_pwmclk (&pwmclk);
  prn_pwmpin (pwmpins);

  puts ("\nconfiguring gpio");

  /* open gpiochipX device for pin control with ioctl calls */
  if ((gpiofd = gpio_dev_open (GPIOCHIP)) == -1) {
    return 1;
  }

  /* set gpio_v2 line request to handle pin */
  linereq.offsets[0] = pwmpins[0].gpio;
  if (gpio_line_cfg_ioctl (gpiofd, &linecfg, &linereq) == -1) {
    return 1;
  }

  /* dump of gpio configured values to stdout (optional) */
  printf ("\ngpio config:\n"
          "  gpiofd     : %6d\n"
          "  linereqfd  : %6d\n"
          "  lr_off[0]  : %6hhu\n",
          gpiofd, linereq.fd, linereq.offsets[0]);

  /* set initial dutycycle for pin (regardless of user input) */
  if (pwm_pin_cfg_dutycycle (&pwmclk, pwmpins, pin, 0) == -1) {
    fprintf (stderr, "error: setting dutycycle at %hhu\n", 0);
    return 1;
  }

  /* initialize thread attributes (using defaults) and validate */
  handle_error_en (pthread_attr_init (&attr), "pthread_attr_init");

  /* create thread to handle PWM interval timer signal / validate */
  handle_error_en (pthread_create (&id, &attr, signal_thread, &pwmgpio),
                   "pthread_create");

  puts ("\nstarting PWM clock:");
  /* start interval timer to drive PWM */
  if (pwmclk_start (&pwmclk) == -1) {
    return 1;
  }

  /* dump PWM clock config before main program loop (optional) */
  prn_pwmclk (&pwmclk);
  // putchar ('\n');

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

  /* set gpio pin value LO on program exit */
  linevals.bits = 0;
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

  /* close gpio file descriptors - for linereq and /dev/gpiochipX */
  if (gpio_line_close_fd (&linereq) == -1) {
    return 1;
  }
  if (gpio_dev_close (gpiofd) == -1) {
    return 1;
  }

#ifdef DEBUG
  printf ("main program loop count : %llu\n", loopcnt);
#endif
}

