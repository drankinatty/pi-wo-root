/**
 *  74HC165N access over SPI utilizing Linux ioctl for Raspberry Pi
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <tinygpio.h>     /* required to set SPI_CE1 pin to output */

typedef struct {          /* SPI device struct for mcp3008 */
  struct spi_ioc_transfer spi_xfer;
  uint8_t buf[2];
  int fd;
  uint8_t mode,
          cepin;
} spidev;

/* SPI and mcp3008 constants
 *   SPIDEVFS  "/dev/spidev<bus>.<channel>"
 */
#define SPIDEV         "/dev/spidev0."
#define SPIDEVFS(_c)     (SPIDEV # _c)

#define BITS          8
#define CLOCK         1350000
#define DELAY         5

#define SPI_CE1       7
#define SPI_CE0       8
#define SPI_MISO      9
#define SPI_MOSI     10
#define SPI_CLK      11

#define NGPIOPINS     5

/**
 * @brief initialize the SPI system (e.g. open "/dev/spidev0.0") and configure
 * the device struct for reading from the mcp3008.
 * @param spidevfs filesystem device node for SPI.
 * @param dev pointer to struct holding mcp3008 configuration.
 * @param delay microsecond delay for mcp3008 conversion (sample switching).
 * @param clock SPI bus speed in Hz.
 * @param bits bits per-bytes (e.g. CHAR_BITS).
 * @param mode SPI_MODE_x (single-ended or pseudo-differential pairs).
 * @param cepin CS/CE GPIO pin number.
 * @return returns 0 on success, -1 otherwise.
 */
int spi_device_init (const char *spidevfs, spidev *dev, uint16_t delay,
                      uint32_t clock, uint8_t bits, uint8_t mode, uint8_t cepin)
{
  /* initialize struct members */
  dev->spi_xfer.tx_buf = (unsigned long)dev->buf;
  dev->spi_xfer.rx_buf = (unsigned long)(dev->buf + 1);
  dev->spi_xfer.len = 1;
  dev->spi_xfer.delay_usecs = delay;
  dev->spi_xfer.speed_hz = clock;
  dev->spi_xfer.bits_per_word = bits;

  dev->mode = mode;
  dev->cepin = cepin;

  /* open spi device in Linux sysfs */
  if ((dev->fd = open (spidevfs, O_RDWR)) == -1) {
    perror ("open spidevfs");
    return -1;
  }
  /* set device mode */
  if (ioctl (dev->fd, SPI_IOC_WR_MODE, &dev->mode) == -1) {
    perror ("error init SPI_IOC_WR_MODE");
    return -1;
  }
  /* set number of bits per word (byte) */
  if (ioctl (dev->fd, SPI_IOC_WR_BITS_PER_WORD, &dev->spi_xfer.bits_per_word) == -1) {
    perror ("error init SPI_IOC_WR_BITS_PER_WORD");
    return -1;
  }
  /* send requested SPI bus speed */
  if (ioctl (dev->fd, SPI_IOC_WR_MAX_SPEED_HZ, &dev->spi_xfer.speed_hz) == -1) {
    perror ("error init SPI_IOC_WR_MAX_SPEED_HZ");
    return -1;
  }
  /* read configured SPI bus speed */
  if (ioctl (dev->fd, SPI_IOC_RD_MAX_SPEED_HZ, &dev->spi_xfer.speed_hz) == -1) {
    perror ("error init SPI_IOC_RD_MAX_SPEED_HZ");
    return -1;
  }

  return 0;
}


/**
 * @brief read sample from 74hc165 for channel.
 * @param dev pointer to 74hc165 device struct.
 * @param channel analog input channel to read.
 * @return returns raw 10-bit value for sample.
 */
int spi_read_adc (spidev *dev)
{
  dev->buf[0] = 0;

  if (ioctl (dev->fd, SPI_IOC_MESSAGE(1), &dev->spi_xfer) == -1) {
    perror ("ioctl SPI_IOC_MESSAGE()");
    abort();
  }

  return (dev->buf[1] & 0xff);
}


/* output values for all 8 74hc165 inputs */
void prn_all_channels (spidev *dev)
{
  char fmt[32] = "";
  int n = 8;

  for (int i = 0; i < n; i++) {
    printf ("  chan[%d] : %hhu\n", i, (dev->buf[1] >> i) & 0x01);
  }

  sprintf (fmt, "\033[%dA", n);
  printf (fmt);
}

/**
 * @brief save/set gpio pin modes for SPI use with kernel spidev.
 * @param modes array to hold pins moves for each saved gpio pin.
 * @note
 *   SPI_CE1 must be output and pulled high
 *   SPI MISO must be ALT0 - SPI0_MISO and low
 *   SPI_MOSI must be ALT0 - SPI0_MOSI and low
 *   SPI CLK must be ALT0 - SPI0_SCLK and low
 *
 * @return returns 0 on success, -1 otherwise.
 */
int gpio_set_spi_pinmode (uint8_t *modes)
{
  int rtn;

  modes[0] = gpioGetMode (SPI_CE0);     /* save modes of all hdw spi pins */
  modes[1] = gpioGetMode (SPI_CE1);
  modes[2] = gpioGetMode (SPI_MISO);
  modes[3] = gpioGetMode (SPI_MOSI);
  modes[4] = gpioGetMode (SPI_CLK);

  gpioSetMode (SPI_CE0, PI_OUTPUT);     /* sat all hdw spi pins as required */
  gpioSetMode (SPI_CE1, PI_OUTPUT);
  gpioSetMode (SPI_MISO, PI_ALT0);
  gpioSetMode (SPI_MOSI, PI_ALT0);
  gpioSetMode (SPI_CLK, PI_ALT0);

  /* validate that each pin is set correctly */
  if ((rtn = gpioGetMode (SPI_CE1)) != PI_OUTPUT) {
    fprintf (stderr, "error: failed to change CE0 pin from mode %u\n", rtn);
    return -1;
  }

  if ((rtn = gpioGetMode (SPI_CE0)) != PI_OUTPUT) {
    fprintf (stderr, "error: failed to change CE1 pin from mode %u\n", rtn);
    return -1;
  }

  if ((rtn = gpioGetMode (SPI_MISO)) != PI_ALT0) {
    fprintf (stderr, "error: failed to change MISO pin from mode %u\n", rtn);
    return -1;
  }

  if ((rtn = gpioGetMode (SPI_MOSI)) != PI_ALT0) {
    fprintf (stderr, "error: failed to change MOSI pin from mode %u\n", rtn);
    return -1;
  }

  if ((rtn = gpioGetMode (SPI_CLK)) != PI_ALT0) {
    fprintf (stderr, "error: failed to change CLK pin from mode %u\n", rtn);
    return -1;
  }

  return 0;
}


/**
 * @brief restore SPI gpio pins to original pre-program mode.
 * @param modes array of modes to restore (input, output alt0-5).
 * @param nmodes number of modes in modes array to restore.
 * @return returns 0 on success, -1 otherwise.
 */
int gpio_restore_pinmode (uint8_t *modes, int nmodes)
{
  for (int i = 0; i < nmodes; i++) {
    int pin = i + SPI_CE1,
        rtn;

    gpioSetMode (pin, modes[i]);      /* restore pin to saved mode */

    /* validate mode correctly set */
    if ((rtn = gpioGetMode (pin)) != modes[i]) {
      fprintf (stderr, "error: failed to change pin %d to mode %u\n",
                pin, rtn);
      return -1;
    }
  }

  return 0;
}


/**
 * @brief read no. of samples given on command line (default 400) from
 * 74HC165N over SPI.
 * @note both channels spidev0.0 and spidev0.1 are required. The 74HC165N
 * is not a true SPI device and does not provide tri-state output required
 * to read over a single channel. The SPI CE1 pin is used to trigger SH/LD to
 * shift and load parallel values into the internal register. The 74HC165N
 * requires the SH/LD pin return high before data is shifted. SPI holds the
 * CE pin low during the entire write/read of data so data in the 74HC165N
 * is not shifted until after the single-channel SPI transaction completes.
 * To handle this difference, a read is made on SPI channel 1 triggering the
 * SH/LD pin low and returning it high, then data is read on SPI channel 0.
 * @param argc argument count.
 * @param argv argument vector (optional argument no. of samples)
 * @return returns 0 on success, 1 otherwise
 */
int main (int argc, char **argv) {

  unsigned  nsamples = 400;
  spidev    dev0 = { .spi_xfer = {0} },   /* spi device - chan 0 */
            dev1 = { .spi_xfer = {0} };   /* spi device - chan 1 for SH/LD */
  uint8_t   pinmode[NGPIOPINS] = {0},     /* array to save gpio pin modes */
            last = dev0.buf[1];           /* save last for print on change */

  if (argc > 1) {   /* set the number of samples, (default 300) */
    unsigned tmp;
    if (sscanf (argv[1], "%u", &tmp) != 1) {
      fprintf (stderr, "error: invalid unsigned count for argv[2] (%s)\n",
              argv[1]);
      return 1;
    }
    nsamples = tmp;
  }

  /* initialize spidev0 channel 0 to read 74hc165 values */
  if (spi_device_init (SPIDEVFS(0), &dev0, DELAY, CLOCK, BITS,
                        SPI_MODE_0, SPI_CE0) == -1) {
    return 1;
  }

  /* initialize spidev0 channel 1 to drive SH/LD pin with CE signal */
  if (spi_device_init (SPIDEVFS(1), &dev1, DELAY, CLOCK, BITS,
                        SPI_MODE_0, SPI_CE1) == -1) {
    return 1;
  }

  /* output SPI config */
  printf ("opened SPI device: %s on file descriptor: %d\n"
          "opened SPI device: %s on file descriptor: %d\n"
          "  mode    : %d\n"
          "  bits    : %hhu\n"
          "  clock   : %u\n"
          "  delay   : %hu\n"
          "  samples : %u  (~%u seconds)\n\n",
          SPIDEVFS(0), dev0.fd, SPIDEVFS(1), dev1.fd, dev0.mode,
          dev0.spi_xfer.bits_per_word, dev0.spi_xfer.speed_hz,
          dev0.spi_xfer.delay_usecs, nsamples, nsamples / 20);

  if (gpioInitialise() < 0) {   /* initialize gpio library */
    fputs ("error: gpioInitialize() failed\n", stderr);
    close (dev0.fd);
    return 1;
  }

  /* save gpio pin modes and configure for SPI use */
  if (gpio_set_spi_pinmode (pinmode) == -1) {
    close (dev0.fd);
    return 1;
  }

  puts ("output:\n");

  /* do read of all channels at 5Hz */
  for (unsigned i = 0; i < nsamples; i++) {
    spi_read_adc (&dev1);           /* trigger SH/LD pin with spi CE signal */
    spi_read_adc (&dev0);           /* read 74hc165 values on channel 0 */
    if (dev0.buf[1] != last) {      /* if register changed - update output */
      prn_all_channels (&dev0);
    }
    last = dev0.buf[1];             /* save last value from register */
    usleep (50000);                 /* sleep 50ms (loop ~20 times per sec) */
  }
  fputs ("\033[8B\n", stdout);      /* skip down 8 lines, output newline */

  /* restore original gpio pin modes */
  gpio_restore_pinmode (pinmode, NGPIOPINS);

  close (dev0.fd);                  /* close spidev0 channel 0 */
  close (dev1.fd);                  /* close spidev0 channel 1 */

  return 0;
}
