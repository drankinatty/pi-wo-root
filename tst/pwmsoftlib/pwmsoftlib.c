/**
 *  Software PWM utilizing gpio_v2 ioctl for Raspberry Pi
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
#include <pwmsoft.h>

extern volatile sig_atomic_t clkcnt;
extern volatile sig_atomic_t nextval;
extern __u8 npins;

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
  struct gpio_v2_line_request linereq = { .offsets[0] = pwmpins[0].gpio,
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
    if (pwm_pin_cfg_dutycycle (&pwmclk, pwmpins, pin, dutycycle) == -1) {
      return 1;
    }
  }
  else {
    if (pwm_pin_cfg_count (&pwmclk, pwmpins, pin, on_cnt) == -1) {
      return 1;
    }
  }

  prn_pwmclk (&pwmclk);
  prn_pwmpin (pwmpins);

  puts ("\nconfiguring gpio");

  if ((gpiofd = gpio_dev_open (GPIOCHIP)) == -1) {
    return 1;
  }

  linereq.offsets[0] = pwmpins[0].gpio;

  if (gpio_line_cfg_ioctl (gpiofd, &linecfg, &linereq) == -1) {
    return 1;
  }

  printf ("\ngpio config:\n"
          "  gpiofd     : %d\n"
          "  linereqfd  : %d\n"
          "  lr_off[0]  : %hhu\n",
          gpiofd, linereq.fd, linereq.offsets[0]);

  puts ("\nstarting PWM clock:  (press Enter to Quit)");

  if (pwm_pin_cfg_dutycycle (&pwmclk, pwmpins, pin, dc) == -1) {
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

      if (pwm_pin_cfg_dutycycle (&pwmclk, pwmpins, pin, dc) == -1) {
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
                chkclk, clkcnt, clkmax, dc, pwmpins[0].dutycycle);
      }
      else {
        printf ("chk: %6d,  clkcnt: %3d,  clkmax: %d,  dc: %3hhu (%3hhu)\n",
                chkclk, clkcnt, clkmax, dc, pwmpins[0].dutycycle);
      }
      chkclk++;
    }
#endif

    /* set gpio state on cnt == 0 (LO) / off_cnt == cnt (HI) */
    if (clkcnt == 0 && pwmpins[0].off_cnt) {
      linevals.bits = 0;
      if (gpio_line_set_values (&linereq, &linevals) == -1) {
        return 1;
      }
    }
    else if (clkcnt == pwmpins[0].off_cnt) {
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

