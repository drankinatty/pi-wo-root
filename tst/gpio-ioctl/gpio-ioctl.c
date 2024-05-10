/*
 * NOTE: this example uses the GPIO V1 ABI for gpio access via ioctl(),
 *       New code should be written to use the GPIO V2 ABI. See the header
 *       linux/gpio.h for details and see the examples pwmsoftpth.c and
 *       pwmsoftrgb.c for implementation of the V2 ABI strucs and macros.
 */
#include <stdio.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <unistd.h>

/* gpiochipX (0-4) depending on Pi model */
#define GPIOCHIP "/dev/gpiochip0"

#define GPIO_WR_PIN       23
#define GPIO_RD_PIN       24

// Termination flag.
// This is not an example of good coding style.

volatile int TERM = 0;

// Data structures from Linux GPIO libs.
// These here are global because a thread needs them.
// This is not an example of good coding style.

struct gpioevent_request event_request;


void *reader (void *arg)
{
  struct pollfd pollreadfd;

  /* Elevate reader thread priority and use a FIFO scheduler. */
  pthread_t thread = pthread_self();
  const struct sched_param sparam = { .sched_priority = 99, };
  int res = pthread_setschedparam (thread, SCHED_FIFO, &sparam);

  if (res != 0) {
    printf ("Failed to elevate reader thread priority!\n");
  }

  (void)arg;

  struct gpioevent_data event_data;

  pollreadfd.fd = event_request.fd;
  pollreadfd.events = POLLIN;

  // Some variables for evaluating throughput

  __u64 event_ts_start = 0;
  __u64 event_ts_end = 0;

  __u64 count_rising = 0;
  __u64 count_falling = 0;
  __u64 count_other = 0;

  // Receive loop

  while (TERM == 0) {

    int poll_result = poll (&pollreadfd, 1, 1); // time out after 1 milliseconds

    if (poll_result == 0) {
      // printf("Poll timeout.\n");
      continue;
    }

    if (poll_result < 0) {
      // printf("Poll error.\n");
      continue;
    }

    if (poll_result > 1) {
      // printf("Multiple events per poll.\n");
    }

    // The "revents" field counts returned events.
    // The "POLLIN" constant seems to be a bitmask.

    if (pollreadfd.revents & POLLIN) {

      int read_result = read (pollreadfd.fd, &event_data, sizeof event_data);

      if (read_result == -1) {
        // printf("Read error.\n");
        continue;
      }

      if (event_data.id == GPIOEVENT_EVENT_RISING_EDGE) {
        count_rising++;
        // printf("Rising edge at %llu.\n", event_data.timestamp);
      }
      else if (event_data.id == GPIOEVENT_EVENT_FALLING_EDGE) {
        count_falling++;
        // printf("Falling edge at %llu.\n",event_data.timestamp);
      }
      else {
        // printf("Some other event?\n");
        count_other += 1;
      }

      if (event_ts_start == 0) {
        event_ts_start = event_data.timestamp;
      }
      else {
        event_ts_end = event_data.timestamp;
      }
    }
  }

  printf ("Received %llu rising and %llu falling edges (%llu other).\n",
          count_rising, count_falling, count_other);

  __u64 duration = event_ts_end - event_ts_start;
  double seconds = ((double) duration / (double) 1000000000);

  printf ("Total duration %llu ns (%f s).\n", duration, seconds);

  __u64 nanos_per_edge = duration / (count_rising + count_falling);

  printf ("Average %llu ns (%llu microseconds) per edge.\n",
          nanos_per_edge, (nanos_per_edge / 1000));

  __u64 per_second = count_rising / seconds;

  printf ("Rising edge frequency %llu Hz.\n", per_second);

  close( pollreadfd.fd);

  return 0;
}


int main (int argc, char * const *argv) {

  int res, // various call results
      gpiofd;
  unsigned  cycles = 1000,
            delayns = 5000;
  __u8  gpio_wr_pin = GPIO_WR_PIN,
        gpio_rd_pin = GPIO_RD_PIN;

  // Elevate main thread priority and use a FIFO scheduler.

  const struct sched_param sparam = { .sched_priority = 99, };
  res = sched_setscheduler(0, SCHED_FIFO, &sparam);
  if (res != 0) printf("Failed to elevate main thread priority!\n");

  // Data structures from Linux GPIO libs.
  struct gpiochip_info chip_info;
  struct gpiohandle_request handle_request;

  if (argc > 1) {   /* write pin gpio */
    __u8 tmp = 0;
    if (sscanf (argv[1], "%hhu", &tmp) == 1) {
      gpio_wr_pin = tmp;
    }
  }

  if (argc > 2) {   /* read pin gpio */
    __u8 tmp = 0;
    if (sscanf (argv[2], "%hhu", &tmp) == 1) {
      gpio_rd_pin = tmp;
    }
  }

  if (argc > 3) {   /* delay between reads */
    unsigned tmp = 0;
    if (sscanf (argv[3], "%u", &tmp) == 1) {
      delayns = tmp;
    }
  }

  if (argc > 4) {   /* number or rising and falling edges to count */
    unsigned tmp = 0;
    if (sscanf (argv[4], "%u", &tmp) == 1) {
      cycles = tmp;
    }
  }

  gpiofd = open (GPIOCHIP, O_RDONLY);

  if (gpiofd < 0) {
    printf("Failed opening GPIO chip.\n");
    return 1;
  }

  res = ioctl (gpiofd, GPIO_GET_CHIPINFO_IOCTL, &chip_info);

  if (res < 0) {
    perror ("ioctl-GPIO_GET_CHIPINFO_IOCTL");
    printf ("Failed getting chip information.\n");
    close (gpiofd);
    return 1;
  }

  printf ("GPIO chip information:\n"
          "name: %s\n"
          "label: %s\n"
          "lines: %i\n", chip_info.name, chip_info.label, chip_info.lines);

  for (unsigned i = 0; i < chip_info.lines; i++) {

    struct gpioline_info line_info;

    line_info.line_offset = i;

    if (ioctl (gpiofd, GPIO_GET_LINEINFO_IOCTL, &line_info) == -1) {
      perror ("ioctl-GPIO_GET_LINEINFO_IOCTL");
      fprintf (stderr, "Failed getting line %i info.\n", i);
    }
    else {
      printf("%d %s\n",i,line_info.name);
    }
  }

  /* Request events on the reading line. */
  event_request.lineoffset = gpio_rd_pin;
  event_request.eventflags = GPIOEVENT_REQUEST_BOTH_EDGES;
  event_request.handleflags = GPIOHANDLE_REQUEST_INPUT |
                              GPIOHANDLE_REQUEST_BIAS_PULL_DOWN;

  /* get gpio line event handle - for reading */
  res = ioctl (gpiofd, GPIO_GET_LINEEVENT_IOCTL, &event_request);

  if (res < 0) {
    printf ("Failed requesting events.\n");
    close (gpiofd);
    return 1;
  }

  /* get gpio line even handle - for writing.
   * many can be requested instead of one.
   */
  handle_request.lineoffsets[0] = gpio_wr_pin;
  handle_request.flags =  GPIOHANDLE_REQUEST_OUTPUT |
                          GPIOHANDLE_REQUEST_BIAS_PULL_DOWN;
  handle_request.lines = 1;

  res = ioctl (gpiofd, GPIO_GET_LINEHANDLE_IOCTL, &handle_request);

  if (res < 0) {
    perror ("ioctl-GPIO_GET_LINEHANDLE_IOCTL");
    printf ("Failed requesting write handle.\n");
    close (gpiofd);
    return 1;
  }

  // Start a reader thread
  pthread_t reader_thread;
  pthread_create (&reader_thread, NULL, &reader, NULL);


  // Data handle for writing
  struct gpiohandle_data hdata;

  // Time variables for sleeping a short interval.
  struct timespec ts, tr;
  ts.tv_sec = 0;

  printf ("-------------- now writing and reading -------------------\n"
          "delay (ns) : %u\n\n", delayns);

  // Write something out ad hope the reader reads it
  for (unsigned i = 0; i < cycles; i++) {

    hdata.values[0] = 1;
    res = ioctl (handle_request.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &hdata);
    if (res == -1) {
      perror ("error-GPIOHANDLE_SET_LINE_VALUES_IOCTL");
      fputs ("Failed setting line value.\n", stderr);
    }

    // Try to sleep for 5000 nanoseconds.
    // In reality, due to call latencies, you end up sleeping longer.

    ts.tv_nsec = delayns;
    nanosleep (&ts, &tr);

    hdata.values[0] = 0;
    res = ioctl (handle_request.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &hdata);
    if (res == -1) {
      perror ("error-GPIOHANDLE_SET_LINE_VALUES_IOCTL");
      printf ("Failed setting line value.\n");
    }

    ts.tv_nsec = delayns;
    nanosleep (&ts, &tr);
  }

  close (handle_request.fd);

  // Tell the thread to finish
  TERM = 1;

  // Give it time
  usleep (50000);

  // Rejoin main thread with finished thread
  pthread_join (reader_thread,NULL);

  // Close resources
  close (gpiofd);
}
