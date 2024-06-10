/**
 *  GPIO access using kernel GPIO_V2 ABI for ioctl for Raspberry Pi
 *
 *  This code reads from a GPIO pin pulsed continually by IR sensor
 *  measuring rotational speed in RPM using the GPIO_V2 uABI line_event
 *  .timestamp_ns member on each logical falling edge to measure the time
 *  taken for 1 revolution in nanoseconds and converting that value to RPM.
 *
 *  The circuit diagram and many suggestions for improvement can be found in
 *  the related Electronics Stack Exchange hardware question:
 *  https://electronics.stackexchange.com/q/715923/275752
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */
#include <stdio.h>
#include <stdlib.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <unistd.h>

/* pthread validation macros */
#define handle_error_en(en, msg) \
  do { if (en) { errno = en; perror(msg); exit(EXIT_FAILURE); }} while (0)

#define handle_error(msg) \
  do { perror(msg); exit(EXIT_FAILURE); } while (0)

/* gpiochipX (0-4) depending on Pi model */
#define GPIOCHIP "/dev/gpiochip0"

/* define max number of GPIO lines (note: bcm2710A has 58) */
#define GPIOMAX           54

/* default button GPIO pin (Raspberry Pi GPIO26)
 * can be set with 1st program argument (argv[1])
 * debounce us can be adjust with the 2nd (argv[2])
 */
#define GPIO_BTN_PIN      26
// #define PIN_DEBOUNCE    5000
#define PIN_DEBOUNCE       0

#define INPUTTIMEOUT     100          /* poll input timeout (ms) */

#define NANOSECRPM      6e10          /* nanoseconds per RPM */

/* global thread loop control flag */
static volatile __u8 monitoring = 1;

/* global flag for button pressed */
static volatile __u8 pressed = 0;

/* global for rpm */
static volatile __u32 rpm = 0;

/* mutex for rpm update */
pthread_mutex_t lock;

/* simple 3 value / stuct union for outlier evaluation */
typedef union {
  __u32 v[3];
  struct { __u32 v0, v1, v2; };
} triu32_t;

/* global for max RPM evaluation */
static triu32_t maxrpm = { .v = {0} };

/* thread data struct of values needed by reader thread */
typedef struct {
  struct gpio_v2_line_config *linecfg;
  struct gpio_v2_line_request *linereq;
  int fd;
} gpio_v2_t;

/**
 * @brief outputs usage for command line arguments and if err nonzero exits.
 * @param argv program argument vector with executable name as 1st element.
 * @param err if non-zero, exit program with return of EXIT_FAILURE.
 */
void usage (char * const *argv, __u8 err)
{
  printf ("\nCommand line argument usage (with defaults shown)\n\n"
          "  %s [ gpio_pin (26) debounce_period (5000 us) ]\n\n", argv[0]);
  if (err) {
    exit (EXIT_FAILURE);
  }
}


/**
 * @brief wrapper for usage outputting provided error message before usage.
 * @param argv program argument vector with executable name as 1st element.
 * @param errmsg text to write on stderr following "error: ".
 */
void usage_err (char * const *argv, const char *errmsg)
{
  fprintf (stderr, "error: %s\n", errmsg);
  usage (argv, 1);
}


/**
 * @brief simple function to add val to t as max in decreasing order if val
 * is greater than any of the current values.
 * @param t pointer to initialize triu32_t struct.
 * @param val value to add to t in-order if greater than current value.
 */
void triu32_add_max (triu32_t *t, __u32 val)
{
  for (int i = 0; i < 3; i++) {
    if (val > t->v[i]) {
      for (int j = i + 1; j < 3; j++) {
        t->v[j] = t->v[i];
      }
      t->v[i] = val;
      break;
    }
  }
}


/**
 * @brief set the pins for ioctl linereq from the npins specified in pinarr.
 * @param gpio pointer to struct containing pointers to gpio_v2 data
 * structures use by ioctl.
 * @param pinarr array (pointer to first element) of Broadcom GPIO pin
 * numbers to control through gpio->linereq.
 * @param npins no. of pins contained in pinarr.
 * @return returns 0 on success, -1 otherwise.
 */
int gpio_set_pins (gpio_v2_t *gpio, __u8 *pinarr, __u8 npins)
{
  /* validate npins in range */
  if (npins > GPIOMAX) {
    fprintf (stderr, "error: gpio_set_pins, npins '%hhu' excees maximum "
                     "number of GPIOs (%d).\n", npins, GPIOMAX);
    return -1;
  }

  /* set gpio_v2 linereq offsets for npins in pinarr array */
  for (__u8 p = 0; p < npins; p++) {
    if (pinarr[p] >= GPIOMAX) {
      fprintf (stderr, "error: gpio_set_pins, invalid GPIO '%hhu' requested "
                       "exceeds max no. of GPIO (%d).\n", pinarr[p], GPIOMAX);
      return -1;
    }
    gpio->linereq->offsets[p] = pinarr[p];
  }

  /* set number of lines (pins) for gpio_v2 linereq */
  gpio->linereq->num_lines = npins;

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
 * @param gpio gpio v2_struct holding pointers to linecfg and linereq
 * structs and open file descriptor returned by prior call to
 * gpio_dev_open()..
 * linecfg - v2 ioctl config struct for gpio pins (lines).
 * linereq - v2 ioctl request struct member of linecfg containing
 * separate file-descriptor used for read/write access of configured
 * gpio pins (lines).
 * @return returns 0 on success, -1 otherwise.
 */
int gpio_line_cfg_ioctl (gpio_v2_t *gpio)
{
  /* get ioctl values for line request */
  if (ioctl (gpio->fd, GPIO_V2_GET_LINE_IOCTL, gpio->linereq) < 0) {
    perror ("ioctl-GPIO_V2_GET_LINE_IOCTL");
    return -1;
  }

  /* set the line config for the retured linereq file descriptor */
  if (ioctl (gpio->linereq->fd, GPIO_V2_LINE_SET_CONFIG_IOCTL,
              gpio->linecfg) < 0) {
    perror ("ioctl-GPIO_V2_LINE_SET_CONFIG_IOCTL");
    return -1;
  }

  return 0;
}


/**
 * @brief write gpio pin (line) values (HI/LO) from bits set or clearned
 * in gpio->linevals->bits for pins with bit set high in gpio->linevals->mask
 * from mask using gpio_v2 ioct line request.
 * @param linereq pointer to gpio_v2_line_request struct holding linereq with
 * open linereq file descriptor set by prior call to gpio_line_cfg_ioctl()
 * used to write linevals to gpio pin index(s) in linereq->offsets specified
 * by bits HI in mask.
 * @param bits gpio value to write (0 - LO, 1 - HI) to set bit in
 * linereq->bits for bits specified in mask.
 * @param mask bitmap with bits 1 (HI) that correspond to index in
 * gpio->linereq->offsets pin array that will be set.
 * @return returns 0 on success, -1 otherwise.
 */
int gpio_line_set_values (struct gpio_v2_line_request *linereq,
                          __u64 bits,  __u64 mask)
{
  struct gpio_v2_line_values linevals = { .bits = bits, .mask = mask };

  /* set or clear linereq .bits pin values (bitmap of pin values set to
   * bits that correspond to pin bits (bitmap indexes) set HI in mask.
   */
  if ((bits & mask) > 0) {
    linevals.bits |= mask;
  }
  else {
    linevals.bits &= (0xffffffffffffffff & ~mask);
  }

  /* set GPIO pin value to bit in lineval->bits (0 or 1) for pins with
   * bit == 1 in mask.
   */
  if (ioctl (linereq->fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &linevals) < 0) {
    perror ("gpio_line_set_values()-GPIO_V2_LINE_SET_VALUES_IOCTL");
    return -1;
  }

  return 0;
}


/* TODO - add get value that takes line event and uses offset to set mask.
 * like gpio_event_get_values()
 */

/**
 * @brief read gpio pins (lines) values (HI/LO) for lines with bit set in mask
 * into linevals->bits using gpio_v2 ioct line request.
 * @param linereq pointer to gpio_v2_line_request struct holding linereq with
 * open linereq file descriptor set by prior call to gpio_line_cfg_ioctl()
 * used to read linevals for gpio pin index(s) in linereq->offsets specified
 * by bits HI in mask.
 * @param bits gpio values (0 - LO, 1 - HI) read for bits specified in mask.
 * @param mask bitmap with bits 1 (HI) that correspond to index in
 * linereq->offsets pin array that will be read into linevals->bits.
 * @return returns 0 on success, -1 otherwise.
 */
int gpio_line_get_values (struct gpio_v2_line_request *linereq,
                          struct gpio_v2_line_values *linevals,
                          __u64 mask)
{
  /* set linevals mask */
  linevals->mask = mask;

  /* get GPIO pin value(s) for pins with bit == 1 in mask. */
  if (ioctl (linereq->fd, GPIO_V2_LINE_GET_VALUES_IOCTL, linevals) < 0) {
    perror ("gpio_line_get_values()-GPIO_V2_LINE_GET_VALUES_IOCTL");
    return -1;
  }

  return 0;
}


/**
 * @brief closes the open line request file descriptor for v2 linereq.
 * @param gpio gpio v2 data struct holding line request struct with open
 * linereq file descriptor opened by prior call to gpio_line_cfg_ioctl().
 * @return returns 0 on success, -1 otherwise.
 */
int gpio_line_close_fd (gpio_v2_t *gpio)
{
  if (close (gpio->linereq->fd) < 0) {    /* close linereq fd */
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
 * @brief separate thread to read rising edge when IR receiver active and
 * gpio line pulled low, and falling edge on loss of IR signal.
 * @param data thread function parameter passing pointer to gpio_v2_t struct
 * with configured gpio_v2_line_request.
 * @return returns pointer to data (same gpio_v2_line_request - unchanged).
 * @todo add debug output writing all rpm values to file (or store in array)
 * so actual data can be used to cautch outlier data and discard.
 */
void *threadfn_read_ir (void *data)
{
  /* cast/get linereq struct from data parameter */
  gpio_v2_t *gpio = data;
  struct gpio_v2_line_request *linereq = gpio->linereq;

  /* poll file descriptor for reading pin */
  struct pollfd readfds = { .fd = linereq->fd, .events = POLLIN };

  /* gpio_v2 line event struct as storage for data read from pin */
  struct gpio_v2_line_event lineevent = { .timestamp_ns = 0 };

  __u64 event_ts_start = 0,   /* timespec start/end times */
        event_ts_end = 0,
        count_rising = 0,     /* count of rising edges */
        count_falling = 0,    /* count of falling edges */
        count_other = 0;      /* count for other - SNAFU */

  /* loop while writing loop-control flag true */
  while (monitoring == 1) {
    /* poll pin event with 1/10th second timeout (should not timeout) */
    int nfds = poll (&readfds, 1, INPUTTIMEOUT),
        nbytes = 0;

    /* validate poll return */
    if (nfds < 0) {   /* error */
      perror ("poll-readfds");
      continue;
    }

    /* for button press, long timeout is fine */
    if (nfds == 0) {  /* timeout occurred */
      continue;
    }

    if (readfds.revents & POLLIN) {   /* process event */
      /* read and validate return */
      if ((nbytes = read (readfds.fd, &lineevent, sizeof lineevent)) == -1) {
        perror ("read - lineevent");
        continue;
      }
      /* handle rising and falling or other event IDs */
      if (lineevent.id == GPIO_V2_LINE_EVENT_RISING_EDGE) {
        event_ts_start = event_ts_end;
        count_rising += 1;
      }
      else if (lineevent.id == GPIO_V2_LINE_EVENT_FALLING_EDGE) {
        __u32 revns = 0;
        event_ts_end = lineevent.timestamp_ns;
        count_falling += 1;

        revns = event_ts_end - event_ts_start;
        if (revns) {
          pthread_mutex_lock (&lock);       /* lock mutex */
          /* compute current RPM from ns / rev */
          rpm = (__u32)(NANOSECRPM / revns);
          pthread_mutex_unlock (&lock);     /* unlock mutex */
          triu32_add_max (&maxrpm, rpm);
        }
      }
      else {
        count_other += 1;
      }
    }
  }

  /* output thread read results on thread exit */
  printf ("Received %llu rising and %llu falling edges (%llu other).\n"
          "in %llu ns\n",
          count_rising, count_falling, count_other,
          event_ts_end - event_ts_start);

  return data;    /* return pointer to data parameter (unchanged) */
}


/**
 * @brief simple wrapper for pselect used in main to exit program.
 * @param sec timeout period seconds.
 * @param nsec timeout period nanoseconds.
 * @return returns the number of descriptors ready for reading.
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
 * @brief empty any characters remaining in stdin after input.
 * @param c last character read from stdin (must be initialized)
 */
void empty_stdin (int c)
{
  while (c != '\n' && c != EOF) {
    c = getchar();
  }
}


/**
 * @brief configure GPIO pin for button press detection using kernel
 * userspace gpio_v2 API. GPIO pin can be set by first program argument
 * (GPIO26 on Raspberry Pi by default - Broadcom pin no.)
 * @param argc program argument count.
 * @param argv program argument vector.
 *   argv[1] - GPIO pin with button attached (pull-up)
 *   argv[2] - debounce period in microseconds (us)
 * @return returns EXIT_SUCCESS on success, EXIT_FAILURE otherwise.
 */
int main (int argc, char * const *argv) {

  int gpiofd;                         /* returned gpiochipX file descriptor */
  __u8  gpio_btn_pin = GPIO_BTN_PIN;  /* initial set of gpio pin to default */
  __u32 pin_debounce = PIN_DEBOUNCE;  /* button debounce period (us)*/

  /* gpio_v2 line config, line request and line values, read defaults set */
  struct gpio_v2_line_config linecfg = {
                              .flags =  GPIO_V2_LINE_FLAG_ACTIVE_LOW      |
                                        GPIO_V2_LINE_FLAG_INPUT           |
                                        GPIO_V2_LINE_FLAG_EDGE_RISING     |
                                        GPIO_V2_LINE_FLAG_EDGE_FALLING    |
                                        GPIO_V2_LINE_FLAG_BIAS_PULL_UP,
                              .num_attrs = 1 };
  struct gpio_v2_line_request linereq = { .offsets[0] = gpio_btn_pin,
                                          .num_lines = 1 };

  /* pins is a convenience struct of gpio_v2 config structs to which
   * a pointer can be provided as the parameter fo the thread function.
   */
  gpio_v2_t pins =  { .linecfg = &linecfg,
                      .linereq = &linereq };
  /* gpio_v2 attribute and attribute config setting button debounce to 5 ms.
   * for button gpio pin (offsets[0]) - adjust as needed)
   */
  struct gpio_v2_line_attribute rd_attr = { .id =
                                              GPIO_V2_LINE_ATTR_ID_DEBOUNCE,
                                            .debounce_period_us =
                                              pin_debounce };
  struct gpio_v2_line_config_attribute rd_cfg_attr = { .attr = rd_attr,
                                                       .mask = 0x01 };

  pthread_t id;                           /* pwm signal thread id */
  pthread_attr_t attr;                    /* pwm thread attributes */
  void *res;                              /* results pointer */
  __u32 lastrpm = 0;                      /* last rpm update value */

  /* process command line arguments */
  if (argc > 1) {   /* gpio button pin */
    __u8 tmp = 0;
    if (sscanf (argv[1], "%hhu", &tmp) != 1) {
      usage_err (argv, "invalid unsigned byte value provided for write pin");
    }
    gpio_btn_pin = tmp;                   /* assign new pin number */
    linereq.offsets[0] = gpio_btn_pin;    /* update linreq offsets[0] */
  }
  if (argc > 2) {   /* debounce period for gpio pin */
    __u32 tmp = 0;
    if (sscanf (argv[2], "%u", &tmp) != 1) {
      usage_err (argv, "invalid unsigned byte value provided for write pin");
    }
    pin_debounce = tmp;                   /* assign new pin debounce */
    rd_attr.debounce_period_us = pin_debounce;  /* update attribute value */
    rd_cfg_attr.attr = rd_attr;           /* update line config attribute */
  }

  /* open gpiochipX device - validate */
  if ((gpiofd = gpio_dev_open (GPIOCHIP)) == -1) {
    return 1;
  }
  pins.fd = gpiofd;

  /* assign additional debounce line attribute to 1st element of
   * linecfg attrs array.
   */
  linecfg.attrs[0] = rd_cfg_attr;

  /* set line (pin) configuration using convenience struct pins */
  if (gpio_line_cfg_ioctl (&pins) == -1) {
    return 1;
  }

  usleep (INPUTTIMEOUT * 100);      /* delay input timeout microsecs */

  /* initialize thread mutex */
  handle_error_en (pthread_mutex_init (&lock, NULL), "pthread_mutex_init");

  /* initialize thread attributes (using defaults) and validate */
  handle_error_en (pthread_attr_init (&attr), "pthread_attr_init");

  /* create thread to handle PWM interval timer signal / validate */
  handle_error_en (pthread_create (&id, &attr, threadfn_read_ir, &pins),
                   "pthread_create");

  printf ("\nwaiting on button pressesses (press Enter to exit)\n\n"
          "  button GPIO pin : %hhu\n"
          "  debounce period : %u (us)\033[?25l\n\n"
          "  RPM : ", gpio_btn_pin, pin_debounce);

  for (;;) {  /* loop continually until input received for exit */
    __u32 rpmupdate = 0;
    if (pselect_timer (0, 2.e8) == 1) {
      int c = getchar();
      /* check for 'q' quit (or empty input or EOF) */
      if (c == 'q' || c == '\n' || c == EOF) {
        empty_stdin (c);    /* empty stdin before loop exit */
        monitoring = 0;     /* set thread loop flag 0, ending thread */
        break;
      }
      empty_stdin (c);      /* empty stdin after each input */
    }
    pthread_mutex_lock (&lock);       /* lock mutex */
    rpmupdate = rpm;                  /* assign global rpm */
    pthread_mutex_unlock (&lock);     /* unlock mutex */
    if (rpmupdate != lastrpm) {
      printf ("%u\033[0K\n\033[1A\033[8C", rpmupdate);
      lastrpm = rpmupdate;
    }
  }

  puts ("\033[?25h");               /* restore cursor and newline */

  printf ("  MAXRPM : %u\n\n", maxrpm.v0);

  usleep (INPUTTIMEOUT * 100);      /* delay input timeout microsecs */

  /* wait for thread to exit / destroy mutex */
  handle_error_en (pthread_join (id, &res), "pthread_join");
  handle_error_en (pthread_mutex_destroy (&lock), "pthread_mutex_destroy");

  /* close gpiochipX file descriptor */
  gpio_dev_close (gpiofd);
}
