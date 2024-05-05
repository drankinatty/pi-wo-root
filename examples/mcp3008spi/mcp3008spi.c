/**
 *  MCP3008 access over SPI utilizing Linux ioctl for Raspberry Pi
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

#include <tinygpio.h>     /* required to set SPI_CE pin to output */

typedef struct {          /* SPI device struct for mcp3008 */
  struct spi_ioc_transfer spi_xfer;
  uint8_t buf[6];
  int fd;
  uint8_t mode,
          cepin;
} spidev;

/* SPI and mcp3008 constants  */
#define SPIDEVFS      "/dev/spidev0.0"
#define BITS          8
#define CLOCK         1350000
#define DELAY         5

#define SPI_CE        8
#define SPI_MISO      9
#define SPI_MOSI     10
#define SPI_CLK      11


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
  dev->spi_xfer.rx_buf = (unsigned long)(dev->buf + 3);
  dev->spi_xfer.len = 3;
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
 * @brief set channel to psuedo-differential compare mode.
 * @param channel channel to set control bits on (SGL/DIF = 0, D2=D1=D0=0).
 * @return returns control bits set for channel.
 */
uint8_t channel_cfg_differential (uint8_t channel)
{
  return (channel & 7) << 4;
}


/**
 * @brief set channel to single-ended input mode.
 * @param channel channel to set control bits on (SGL/DIF = 1, D2=D1=D0=0).
 * @return returns control bits set for channel.
 */
uint8_t channel_cfg_single (uint8_t channel)
{
  return (0x8 | channel) << 4;
}


/**
 * @brief read sample from mcp3008 for channel.
 * @param dev pointer to mcp3008 device struct.
 * @param channel analog input channel to read.
 * @return returns raw 10-bit value for sample.
 */
int spi_read_adc (spidev *dev, uint8_t channel)
{
  dev->buf[0] = 1;
  dev->buf[1] = channel_cfg_single (channel);
  dev->buf[2] = 0;

  if (ioctl (dev->fd, SPI_IOC_MESSAGE(1), &dev->spi_xfer) == -1) {
    perror ("ioctl SPI_IOC_MESSAGE()");
    abort();
  }

  return ((dev->buf[4] & 0x03) << 8) |
          (dev->buf[5] & 0xff);
}


void prn_all_channels (float *f, int n)
{
  char fmt[32] = "";

  for (int i = 0; i < n; i++) {
    printf ("  chan[%d] : % 5.2f\n", i, f[i]);
  }

  sprintf (fmt, "\033[%dA", n);
  printf (fmt);
}

/**
 * set gpio pin modes for SPI use with kernel spidev
 *
 *   SPI_CE must be output and pulled high
 *   SPI MISO must be ALT0 - SPI0_MISO and low
 *   SPI_MOSI must be ALT0 - SPI0_MOSI and low
 *   SPI CLK must be ALT0 - SPI0_SCLK and low
 *
 * save pins to modes array to be restored on interrupt or exit.
 */
int gpio_set_spi_pinmode (uint8_t *modes)
{
  int rtn;

  modes[0] = gpioGetMode (SPI_CE);
  modes[1] = gpioGetMode (SPI_MISO);
  modes[2] = gpioGetMode (SPI_MOSI);
  modes[3] = gpioGetMode (SPI_CLK);

  gpioSetMode (SPI_CE, PI_OUTPUT);
  gpioSetMode (SPI_MISO, PI_ALT0);
  gpioSetMode (SPI_MOSI, PI_ALT0);
  gpioSetMode (SPI_CLK, PI_ALT0);

  if ((rtn = gpioGetMode (SPI_CE)) != PI_OUTPUT) {
    fprintf (stderr, "error: failed to change CE pin from mode %u\n", rtn);
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
    int pin = i + 8,
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


int main (int argc, char **argv) {

  uint8_t   channel = 0,
            pinmode[8];
  unsigned  nsamples = 300;
  spidev    dev = { .spi_xfer = {0}};

  if (argc > 1) {   /* set the number of samples, (default 300) */
    unsigned tmp;
    if (sscanf (argv[1], "%u", &tmp) != 1) {
      fprintf (stderr, "error: invalid unsigned count for argv[2] (%s)\n",
              argv[1]);
      return 1;
    }
    nsamples = tmp;
  }

  if (argc > 2) {   /* set single channel to read, (default all channels) */
    uint8_t tmp;
    if (sscanf (argv[2], "%hhu", &tmp) != 1 || tmp > 7) {
      fprintf (stderr, "error: invalid channel given on argv[1] (%s)\n",
              argv[2]);
      return 1;
    }
    channel = tmp;
  }

  /* initialize spidev interface and validate */
  if (spi_device_init (SPIDEVFS, &dev, DELAY, CLOCK, BITS,
                        SPI_MODE_0, SPI_CE) == -1) {
    return 1;
  }

  printf ("opened SPI device: %s on file descriptor: %d\n"
          "  mode    : %d\n"
          "  bits    : %hhu\n"
          "  clock   : %u\n"
          "  delay   : %hu\n"
          "  samples : %u  (~%u seconds)\n\n",
          SPIDEVFS, dev.fd, dev.mode, dev.spi_xfer.bits_per_word,
          dev.spi_xfer.speed_hz, dev.spi_xfer.delay_usecs,
          nsamples, nsamples / 5);

  if (gpioInitialise() < 0) {   /* initialize gpio library */
    fputs ("error: gpioInitialize() failed\n", stderr);
    close (dev.fd);
    return 1;
  }

  /* set gpio pins for SPI use */
  if (gpio_set_spi_pinmode (pinmode) == -1) {
    close (dev.fd);
    return 1;
  }

  if (argc > 2) { /* if channel argument provides display single-channel */
    puts ("single channel output:\n");

    /* do single-channel ADC read at 5Hz */
    for (unsigned i = 0; i < nsamples; i++) {
      int val = spi_read_adc (&dev, channel);
      float res = (float)val / 1023.f * 3.3f;
      printf ("  chan[%d] : % 5.2f\n\033[1A", channel, res);
      usleep (200000);
    }
  }
  else {  /* otherwise display sample values for all channels */
    puts ("all channel output:\n");

    /* do read of all channels at 5Hz */
    for (unsigned i = 0; i < nsamples; i++) {
      float fvals[8] = {0};
      for (uint8_t chan = 0; chan < 8; chan++) {
        int val = spi_read_adc (&dev, chan);
        fvals[chan] = (float)val / 1023.f * 3.3f;
     }
      prn_all_channels (fvals, 8);
      usleep (200000);
    }
    fputs ("\033[8B\n", stdout);
  }

  gpio_restore_pinmode (pinmode, 4);  /* restore original gpio pin modes */

  close (dev.fd);

  return 0;
}
