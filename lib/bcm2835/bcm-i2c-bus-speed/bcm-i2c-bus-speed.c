/**
 *  BCM2835 set I2C Bus Speed Utility for Raspberry Pi
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */

/* from bcm2835.h
 *
 */
/*! \brief bcm2835I2CClockDivider
  Specifies the divider used to generate the I2C clock from the system clock.
  Clock divided is based on nominal base clock rate of 250MHz
*/

// typedef enum
// {
//     BCM2835_I2C_CLOCK_DIVIDER_2500   = 2500,      /*!< 2500 = 10us = 100 kHz */
//     BCM2835_I2C_CLOCK_DIVIDER_626    = 626,       /*!< 622 = 2.504us = 399.3610 kHz */
//     BCM2835_I2C_CLOCK_DIVIDER_150    = 150,       /*!< 150 = 60ns = 1.666 MHz (default at reset) */
//     BCM2835_I2C_CLOCK_DIVIDER_148    = 148        /*!< 148 = 59ns = 1.689 MHz */
// } bcm2835I2CClockDivider;


#include <stdio.h>

#include <bcm2835.h>

int main (int argc, char **argv) {

  /* TODO: get UID of users and check if root, otherwise throw */

  /* bcm2835I2CClockDivider enum ,see readme. */
  const char *bcm2835_clkdiv = "BCM2835_I2C_CLOCK_DIVIDER_626";
  int rtn, tmp;
  uint16_t i2c_clkdiv = BCM2835_I2C_CLOCK_DIVIDER_626;  /* initialize default 400KHz */
  uint32_t bcmver;

  /* convert user input of clock divider value to int, and
   * check value, set divider based on bracketed range
   */
  if (argc > 1 && sscanf (argv[1], "%d", &tmp) == 1) {
    if (150 <= tmp && tmp <= 250) {
      i2c_clkdiv = BCM2835_I2C_CLOCK_DIVIDER_150;
      bcm2835_clkdiv = "BCM2835_I2C_CLOCK_DIVIDER_150";
    }
    else if (tmp < 150) {
      i2c_clkdiv = BCM2835_I2C_CLOCK_DIVIDER_148;
      bcm2835_clkdiv = "BCM2835_I2C_CLOCK_DIVIDER_148";
    }
  }

  /* initialize the bcm2835 interface */
  if (!(rtn = bcm2835_init())) {
    fprintf (stderr, "error: bcm2835 library failed to initialize with: %d.\n",
              rtn);
    return 1;
  }
  bcm2835_delay (250);    /* delay 250 ms */

  bcm2835_i2c_setClockDivider (i2c_clkdiv);     /* set i2c bug clock divider */
  // bcm2835_i2c_set_baudrate (i2c_baudrate);   /* use if set_clock_divider fails */

  bcmver = bcm2835_version();                   /* optional, get bcm library ver */

  if (!bcm2835_close()) {                       /* close libary instance */
    fputs ("error: bcm2835_close() failed.\n", stderr);
  }

  /* output library version and clock divider string */
  printf ("\nbcm2835 library ver : %hu\n"
          "I2C Clock Div set   : %s\n", bcmver, bcm2835_clkdiv);
}

