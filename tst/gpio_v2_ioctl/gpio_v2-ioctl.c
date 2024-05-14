/**
 *  GPIO access using kernel GPIO_V2 ABI for ioctl for Raspberry Pi
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

/* chip info output format constant(s) */
#define INFOCOLS          3u

/* pthread validation macros */
#define handle_error_en(en, msg) \
  do { if (en) { errno = en; perror(msg); exit(EXIT_FAILURE); }} while (0)

#define handle_error(msg) \
  do { perror(msg); exit(EXIT_FAILURE); } while (0)

/* gpiochipX (0-4) depending on Pi model */
#define GPIOCHIP "/dev/gpiochip0"

/* define max number of GPIO lines (note: bcm2710A has 58) */
#define GPIOMAX           54

/* default write and read GPIO pins */
#define GPIO_WR_PIN       23
#define GPIO_RD_PIN       24

#define INPUTTIMEOUT     100          /* poll input timeout (ms) */

static volatile __u8 writing = 1;     /* flag - read thread loop control */

/* thread data struct of values needed by reader thread */
typedef struct {
  struct gpio_v2_line_config *linecfg;
  struct gpio_v2_line_request *linereq;
  struct gpio_v2_line_values *linevals;
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
          "  %s [ write_pin (23) read_pin (24) "
          "cycles (1000) delayns (5000) ]\n\n", argv[0]);

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
 * @brief set the pins for ioctl linereq from the npins specified in pinarr.
 * @param gpio pointer to struct containing pointers to gpio_vs data
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
 * @param gpio gpio v2_t struct holding linereq with open linereq file
 * descriptor set by prior call to gpio_line_cfg_ioctl() used to write
 * linevals to gpio pin index(s) in linereq->offsets specified by bits HI
 * in mask.
 * @param mask bitmap with bits 1 (HI) that correspond to index in
 * gpio->linereq->offsets pin array that will be set.
 * @param bits gpio value to write (0 - LO, 1 - HI) to set bit in
 * linereq->bits for bits specified in mask.
 * @return returns 0 on success, -1 otherwise.
 */
int gpio_line_set_values (gpio_v2_t *gpio, __u64 bits,  __u64 mask)
{
  /* set or clear linereq .bits pin values (bitmap of pin values set to
   * bits that correspond to pin bits (bitmap indexes) set HI in mask.
   */
  if ((bits & mask) > 0) {
    gpio->linevals->bits |= mask;
  }
  else {
    gpio->linevals->bits &= (0xffffffffffffffff & ~mask);
  }

  /* set linevals mask to mask */
  gpio->linevals->mask = mask;

  /* set GPIO pin value to bit in lineval->bits (0 or 1) for pins with
   * bit == 1 in mask.
   */
  if (ioctl (gpio->linereq->fd, GPIO_V2_LINE_SET_VALUES_IOCTL,
              gpio->linevals) < 0) {
    perror ("ioctl-GPIO_V2_LINE_SET_VALUES_IOCTL-1");
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



void *threadfn_reader (void *data)
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
        count_rising = 0,     /* count of rising, falling edges */
        count_falling = 0,
        count_other = 0;      /* count for other - SNAFU */

  /* loop while writing loop-control flag true */
  while (writing == 1) {
    /* poll pin event with 1/10th second timeout (should not timeout) */
    int nfds = poll (&readfds, 1, INPUTTIMEOUT),
        nbytes = 0;

    /* validate poll return */
    if (nfds < 0) {   /* error */
      perror ("poll-readfds");
      continue;
    }

    if (nfds == 0) {  /* timeout occurred */
#ifdef DEBUG
      fputs ("poll - timeout", stderr);
#endif
      writing = 0;
      break;
    }

    if (readfds.revents & POLLIN) {   /* process event */
      /* read and validate return */
      if ((nbytes = read (readfds.fd, &lineevent, sizeof lineevent)) == -1) {
        perror ("read - lineevent");
        continue;
      }
      /* hand rising, falling or other event IDs */
      if (lineevent.id == GPIO_V2_LINE_EVENT_RISING_EDGE) {
        count_rising += 1;
      }
      else if (lineevent.id == GPIO_V2_LINE_EVENT_FALLING_EDGE) {
        count_falling += 1;
      }
      else {
        count_other += 1;
      }

      /* use timeout to set edge frequency */
      if (event_ts_start == 0) {
        event_ts_start = lineevent.timestamp_ns;
      }
      else {
        event_ts_end = lineevent.timestamp_ns;
      }
    }
  }

  /* output thread read results on thread exit */
  printf ("Received %llu rising and %llu falling edges (%llu other).\n",
          count_rising, count_falling, count_other);

  __u64 duration = event_ts_end - event_ts_start;
  double seconds = duration / 1e9;

  printf ("Total duration %llu ns (%f s).\n", duration, seconds);

  /* guard agaist floating point exception if no edges caught */
  if (count_rising || count_falling) {
    __u64 nanos_per_edge = duration / (count_rising + count_falling);

    printf ("Average %llu ns (%llu microseconds) per edge.\n",
            nanos_per_edge, (nanos_per_edge / 1000));
  }

  __u64 per_second = count_rising / seconds;

  printf ("Rising edge frequency %llu Hz.\n", per_second);

  close (readfds.fd);   /* close poll file descriptor */

  return data;    /* return pointer to data parameter (unchanged) */
}



/**
 * @brief print the GPIO number and function for each GPIO line (pin) in
 * a 3-column format.
 * @param gpiofd open file descriptor for gpiochipX returned from prior call
 * to gpio_dev_open().
 * @return returns 0 on success, -1 otherwise.
 */
int prn_gpio_v2_ghip_info (int gpiofd)
{
  __u8  rows = 0,
        remstart = 0;
  struct gpiochip_info chip_info = { .name = "" };

  /* gpio_v2 ioctl call to get chip information */
  if (ioctl (gpiofd, GPIO_GET_CHIPINFO_IOCTL, &chip_info) == -1) {
    perror ("ioctl-GPIO_GET_CHIPINFO_IOCTL");
    return -1;
  }

  /* validate lines returned (integer division intentional) */
  if ((rows = chip_info.lines / INFOCOLS) == 0) {
    fputs ("error: GPIO_GET_CHIPINFO_IOCTL no GPIO lines.\n", stderr);
    return -1;
  }
  remstart = rows * INFOCOLS;   /* compute no. of rows to print at end */

  /* output chip information */
  printf ("\nGPIO chip information\n\n"
          "  name  : %s\n"
          "  label : %s\n"
          "  lines : %u\n\n",
          chip_info.name, chip_info.label, chip_info.lines);

  /* loop producing output of gpios in 3-column format */
  for (__u8 r = 0; r < rows; r++) {
    for (__u32 c = 0; c < INFOCOLS; c++) {
      __u32 chip = r * INFOCOLS + c;
      struct gpio_v2_line_info line_info = { .name = "",
                                             .consumer = "",
                                             .offset = chip };

      if (ioctl (gpiofd, GPIO_V2_GET_LINEINFO_IOCTL, &line_info) == -1) {
        perror ("ioctl-GPIO_GET_LINEINFO_IOCTL");
        fprintf (stderr, "Failed getting line %u info.\n", chip);
        continue;
      }

      printf ("  %2hhu :  %-18s", chip, line_info.name);
    }
    putchar ('\n');
  }

  /* output any remaining lines (e.g. r * c <= line < chip_info.lines) */
  for (__u32 c = remstart; c < chip_info.lines; c++) {
    struct gpio_v2_line_info line_info = { .name = "",
                                           .consumer = "",
                                           .offset = c };

    if (ioctl (gpiofd, GPIO_V2_GET_LINEINFO_IOCTL, &line_info) == -1) {
      perror ("ioctl-GPIO_GET_LINEINFO_IOCTL");
      fprintf (stderr, "Failed getting line %u info.\n", c);
      continue;
    }

    printf ("  %2hhu :  %-18s", c, line_info.name);
  }
  puts ("\n");    /* tidy up with an additional (2) newlines */

  return 0;
}


int main (int argc, char * const *argv) {

  int gpiofd;
  unsigned  cycles = 1000,
            delayns = 5000;
  __u8  gpio_wr_pin = GPIO_WR_PIN,
        gpio_rd_pin = GPIO_RD_PIN;

  /* gpio_v2 line config, line request and line values, */
  struct gpio_v2_line_config linecfg = {
                              .flags =  GPIO_V2_LINE_FLAG_OUTPUT          |
                                        GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN };
  struct gpio_v2_line_request linereq = { .offsets = {0} };
  struct gpio_v2_line_values linevals = { .bits = 0x01, .mask = 0x01 };

  /* pins is a convenience struct of gpio_v2 config structs to which
   * a pointer can be provided as the parameter fo the thread function.
   */
  gpio_v2_t pins =  { .linecfg = &linecfg,
                      .linereq = &linereq,
                      .linevals = &linevals };
  /* gpio_v2 attribute and attribute config for flags for read pin
   * (write pin, e.g. offsets[0] will use default linecfg flags)
   */
  struct gpio_v2_line_attribute rd_attr = { .id = 1 };
  struct gpio_v2_line_config_attribute rd_cfg_attr = { .attr = {0} };

  pthread_t id;                           /* pwm signal thread id */
  pthread_attr_t attr;                    /* pwm thread attributes */
  void *res;                              /* results pointer */

  struct timespec ts = { .tv_sec = 0 },   /* delay timespecs */
                  tr = { .tv_sec = 0 };

  /* process command line arguments */
  if (argc > 1) {   /* write pin gpio */
    __u8 tmp = 0;
    if (sscanf (argv[1], "%hhu", &tmp) != 1) {
      usage_err (argv, "invalid unsigned byte value provided for write pin");
    }
    gpio_wr_pin = tmp;
  }

  if (argc > 2) {   /* read pin gpio */
    __u8 tmp = 0;
    if (sscanf (argv[2], "%hhu", &tmp) != 1) {
      usage_err (argv, "invalid unsigned byte value provided for read pin");
    }
    gpio_rd_pin = tmp;
  }

  if (argc > 3) {   /* number or rising and falling edges to count */
    unsigned tmp = 0;
    if (sscanf (argv[3], "%u", &tmp) != 1) {
      usage_err (argv, "invalid unsigned value provided for no. of cycles");
    }
    cycles = tmp;
  }

  if (argc > 4) {   /* delay between reads */
    unsigned tmp = 0;
    if (sscanf (argv[4], "%u", &tmp) != 1) {
      usage_err (argv, "invalid unsigned value provided for delay (ns)");
    }
    delayns = tmp;
  }

  /**
   *  gpiochipX - open and get info
   */

  /* open gpiochipX device - validate */
  if ((gpiofd = gpio_dev_open (GPIOCHIP)) == -1) {
    return 1;
  }
  pins.fd = gpiofd;

  /* output gpiochip and each current gpio line (pin) function */
  if (prn_gpio_v2_ghip_info (gpiofd) == -1) {
    return 1;
  }

  /**
   *  configure gpio line request and line configuration
   */

  /* gpio_v2 line request for lines (pins) */
  pins.linereq->offsets[0] = gpio_wr_pin;
  pins.linereq->offsets[1] = gpio_rd_pin;
  pins.linereq->num_lines = 2;

  /* dump chip file descriptor and configured pins */
  printf ("Chip file descriptor and GPIO pins\n\n"
          "  pins->fd   : %d\n"
          "  offsets[0] : %u\n"
          "  offsets[1] : %u\n\n",
          pins.fd, pins.linereq->offsets[0], pins.linereq->offsets[1]);

  /* provide separate attributes for read pin to catch both edges */
  rd_attr.id = GPIO_V2_LINE_ATTR_ID_FLAGS;
  rd_attr.flags = GPIO_V2_LINE_FLAG_INPUT           |
                  GPIO_V2_LINE_FLAG_EDGE_RISING     |
                  GPIO_V2_LINE_FLAG_EDGE_FALLING    |
                  GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;

  /* gpio_v2 attribute config for read pin */
  rd_cfg_attr.attr = rd_attr;
  rd_cfg_attr.mask = 0x02;                /* 0b00000010 - bit indexes read pin */

  /* gpio_v2 line lines (pins) configuration */
  pins.linecfg->num_attrs = 1;            /* for read pin attributes */
  pins.linecfg->attrs[0] = rd_cfg_attr;   /* assign read attr to linecfg array */

  /* set line (pin) configuration */
  if (gpio_line_cfg_ioctl (&pins) == -1) {
    return 1;
  }

  /**
   *  create reader thread
   */

  /* initialize thread attributes (using defaults) and validate */
  handle_error_en (pthread_attr_init (&attr), "pthread_attr_init");

  /* create thread to handle PWM interval timer signal / validate */
  handle_error_en (pthread_create (&id, &attr, threadfn_reader, &pins),
                   "pthread_create");

  /**
   *  write to write pin providing edges for reader
   */

  printf ("-------------- now writing and reading -------------------\n\n"
          "  cycles     : %u\n"
          "  delay (ns) : %u\n\n", cycles, delayns);

  ts.tv_nsec = delayns;     /* set pin write time between edges delay */

  /* write loop for generating rising/falling edges to be read */
  for (__u32 i = 0; writing && i < cycles; i++) {

    /* write pin is bit-index 0 in linevals.mask to set corresponding
     * bit value in linvals.bits (bit 0 is 1 - HI)
     */
    if (gpio_line_set_values (&pins, 0x01, 0x01) == -1) {
      writing = 0;
      break;
    }
    nanosleep (&ts, &tr);

    /* now set/clear bit 0 to set LO */
    if (gpio_line_set_values (&pins, 0x00, 0x01) == -1) {
      writing = 0;
      break;
    }
    nanosleep (&ts, &tr);
  }

  writing = 0;                      /* terminate thread loop */
  usleep (INPUTTIMEOUT * 1000);     /* delay input timeout microsecs */

  /* wait for thread to exit */
  handle_error_en (pthread_join (id, &res), "pthread_join");

  /* close gpiochipX file descriptor */
  gpio_dev_close (gpiofd);
}
