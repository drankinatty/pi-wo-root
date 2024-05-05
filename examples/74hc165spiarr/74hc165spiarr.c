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

#define ARR_ROWS(_a)  (sizeof _a / sizeof *_a)
#define MSB_LSB(_b)   (((_b & 0xf0) >> 4) | ((_b & 0x0f) << 4))

#define STR(X)        #X
#define STRIFY(X)     STR(X)

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

#define BITS          8         /* spi_ioc_transfer bits_per_word */
#define CLOCK         1350000   /* spi_ioc_transfer speed_hz */
#define DELAY         5         /* spi_ioc_transfer delay_usecs */

#define SPI_CE1       7         /* hardware spi0 gpio pins */
#define SPI_CE0       8
#define SPI_MISO      9
#define SPI_MOSI     10
#define SPI_CLK      11

/* lookup table for hardware SPI pins and required mode */
const uint8_t pins[][2] = { {SPI_CE1,  PI_OUTPUT},
                            {SPI_CE0,  PI_OUTPUT},
                            {SPI_MISO, PI_ALT0},
                            {SPI_MOSI, PI_ALT0},
                            {SPI_CLK,  PI_ALT0} };

#define NGPIOPINS     ARR_ROWS(pins)    /* constant - no. of gpio pins */

#ifndef CHAR_BIT
#define CHAR_BIT 8
#endif

#define WDTH sizeof (uint64_t) * CHAR_BIT


/** returns pointer to binary representation of 'v' zero padded to 'sz'.
 *  returns pointer to string contianing binary representation of
 *  unsigned 64-bit (or less ) value zero padded to 'sz' digits.
 */
char *binpad (const uint64_t v, const size_t sz)
{
    static char s[WDTH + 1] = {0};      /* static for file scope */
    char *p = s + WDTH;                 /* ptr to end of s */

    for (size_t i = 0; i < sz; i++)
        *--p = (v>>i & 1) ? '1' : '0';  /* set binary representation */

    return p;   /* pointer to beginning of binary representation in s */
}


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
  /* requested SPI write bus speed */
  if (ioctl (dev->fd, SPI_IOC_WR_MAX_SPEED_HZ, &dev->spi_xfer.speed_hz) == -1) {
    perror ("error init SPI_IOC_WR_MAX_SPEED_HZ");
    return -1;
  }
  /* requested SPI read bus speed */
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
  char *fmt = "\033[" STRIFY(BITS) "A";

  for (int i = 0; i < BITS; i++) {
    printf ("  chan[%d] : %hhu\n", i, (dev->buf[1] >> i) & 0x01);
  }

  fputs (fmt, stdout);
}


/**
 * @brief save/set gpio pin modes for SPI use with kernel spidev.
 * @param pinmodes array to hold pins moves for each saved gpio pin.
 * @note
 *   SPI_CE1 must be output and pulled high
 *   SPI_CE0 must be output and pulled high
 *   SPI MISO must be ALT0 - SPI0_MISO and low
 *   SPI_MOSI must be ALT0 - SPI0_MOSI and low
 *   SPI CLK must be ALT0 - SPI0_SCLK and low
 *
 * @return returns 0 on success, -1 otherwise.
 */
int gpio_set_pinmode (uint8_t *pinmodes)
{
  int rtn;

  /* loop over all pins */
  for (uint8_t i = 0; i < NGPIOPINS; i++) {
    pinmodes[i] = gpioGetMode (pins[i][0]);       /* save current pin mode */
    gpioSetMode (pins[i][0], pins[i][1]);         /* set SPI pin mode */
    /* validate that each pin is set correctly */
    if ((rtn = gpioGetMode (pins[i][0])) != pins[i][1]) {
      fprintf (stderr, "error: failed to change pin %hhu from mode %u\n",
                pins[i][0], rtn);
      return -1;
    }
  }

  return 0;
}


/**
 * @brief restore SPI gpio pins to original pre-program mode.
 * @param pinmodes array of modes to restore (input, output alt0-5).
 * @return returns 0 on success, -1 otherwise.
 */
int gpio_restore_pinmode (uint8_t *pinmodes)
{
  /* loop over all pins */
  for (uint8_t i = 0; i < NGPIOPINS; i++) {
    int rtn;
    /* restore saved pin mode */
    gpioSetMode (pins[i][0], pinmodes[i]);
    /* validate pin mode restored */
    if ((rtn = gpioGetMode (pins[i][0])) != pinmodes[i]) {
      fprintf (stderr, "error: failed to change pin %d to mode %u\n",
                pins[i][0], pinmodes[i]);
      return -1;
    }
  }

  return 0;
}


/**
 * @brief reverse the bit order in a byte.
 * @param byte value to reverse bits.
 * @note channels 0-7 are in proper order, but if you wish to store
 * the serial value on a little endian machine (like the Pi), you need
 * to change the endianess with LSB first in memory.
 * @return returns byte with the bit order reversed.
 */
uint8_t bitorderswap (uint8_t byte)
{
  byte = ((byte & 0xf0) >> 4) | ((byte & 0x0f) << 4);
  byte = ((byte & 0xcc) >> 2) | ((byte & 0x33) << 2);

  return ((byte & 0xaa) >> 1) | ((byte & 0x55) << 1);
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

  unsigned  nsamples = 400;                 /* no. of sample to read */
  spidev    dev[2] = {{ .spi_xfer = {0} }}; /* spidev0 - chan 0 & 1 (SH/LD) */
  uint8_t   pinmode[NGPIOPINS] = {0},       /* array to save gpio pin modes */
            last = dev[0].buf[1];           /* save last, print on change */

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
  if (spi_device_init (SPIDEVFS(0), &dev[0], DELAY, CLOCK, BITS,
                        SPI_MODE_0, SPI_CE0) == -1) {
    return 1;
  }

  /* initialize spidev0 channel 1 to drive SH/LD pin with CE signal */
  if (spi_device_init (SPIDEVFS(1), &dev[1], DELAY, CLOCK, BITS,
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
          SPIDEVFS(0), dev[0].fd, SPIDEVFS(1), dev[1].fd, dev[0].mode,
          dev[0].spi_xfer.bits_per_word, dev[0].spi_xfer.speed_hz,
          dev[0].spi_xfer.delay_usecs, nsamples, nsamples / 20);

  if (gpioInitialise() < 0) {   /* initialize gpio library */
    fputs ("error: gpioInitialize() failed\n", stderr);
    close (dev[0].fd);
    close (dev[1].fd);
    return 1;
  }

  /* save gpio pin modes and configure for SPI use */
  if (gpio_set_pinmode (pinmode) == -1) {
    close (dev[0].fd);
    close (dev[1].fd);
    return 1;
  }

  puts ("output:\n");

  /* do read of all channels at 20Hz */
  for (unsigned i = 0; i < nsamples; i++) {
    spi_read_adc (&dev[1]);         /* trigger SH/LD pin with spi CE1 signal */
    spi_read_adc (&dev[0]);         /* read 74hc165 values on channel 0 */
    if (dev[0].buf[1] != last) {    /* if register changed - update output */
      prn_all_channels (&dev[0]);
    }
    last = dev[0].buf[1];           /* save last value from register */
    usleep (50000);                 /* sleep 50ms (loop ~20 times per sec) */
  }
  fputs ("\033[8B\n", stdout);      /* skip down 8 lines, output newline */

  last = bitorderswap (last);       /* convert value to little-endian */

  /* output last value in proper bit-order for storage */
  printf ("last value (little endian): 0x%02hhx  (binary: %s)\n",
          last, binpad (last, BITS));

  /* restore original gpio pin modes */
  gpio_restore_pinmode (pinmode);

  close (dev[0].fd);                /* close spidev0 channel 0 */
  close (dev[1].fd);                /* close spidev0 channel 1 */

  return 0;
}
