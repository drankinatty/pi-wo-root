/**
 *  TI ADS1115 ADC for Raspberry Pi
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */

#include <stdio.h>

#include "ads1115.h"


/* ADS1115 ADC config */
#define RETRIES               4


/* i2c devfs devide nodes for each i2c bus */
const char *i2cdev[2] = { "/dev/i2c-0", "/dev/i2c-1" };
/* delay for each datarate ( 1000 + 1000 * 1000 / samples-per-sec) */
const int delay[8] = { 126000, 63500, 32250, 16625, 8812, 5000, 31005, 2162 };


/**
 * @brief open i2c devfs device file assigning the file descriptor and
 * i2c address to the device struct.
 * @param dev pointer to device struct.
 * @param i2cdevfs i2c device filesystem note (e.g. "/dev/i2c-0").
 * @param i2caddr i2c address for device on i2c bus (0x48 - 0x4b).
 * @return returns 0 on success, -1 on error.
 */
int ads_i2c_init (ads_t *dev, const char *i2cdevfs, uint8_t i2caddr)
{
  if ((dev->fd = open (i2cdevfs, O_RDWR)) == -1) {
    perror ("open-i2cdevfs");
    return -1;
  }

  dev->i2c_address = i2caddr;

  return 0;
}


/**
 * @brief configure the device config register values for given analog input
 * (chan) and save to channel_cfg array for device.
 * @param dev pointer to device struct.
 * @param chan ads1115 analog input channel (0-3).
 * @param mux cfg register multiplexer value (differential or single input).
 * @param pga cfg register programmable gain aplifier value (voltage range).
 * @param mode cfg register mode (continuous or single-shot).
 * @param sps cfg register data rate (samples per-second).
 */
void ads_channel_cfg (ads_t *dev, uint8_t chan, uint16_t mux,
                      uint16_t pga, uint16_t mode, uint16_t sps)
{
  dev->channel_cfg[chan] =  mux | pga | mode | sps | CFG_QUE_DISABLE;
}


/**
 * @brief configure the device config register values for ALL aanalog input
 * channels with the values given.
 * @param dev pointer to device struct.
 * @param mux cfg register multiplexer value (differential or single input).
 * @param pga cfg register programmable gain aplifier value (voltage range).
 * @param mode cfg register mode (continuous or single-shot).
 * @param sps cfg register data rate (samples per-second).
 */
void ads_channel_cfg_all (ads_t *dev, uint16_t mux, uint16_t pga,
                          uint16_t mode, uint16_t sps)
{
  for (uint16_t chan = 0; chan < CHANNELS; chan++) {
    dev->channel_cfg[chan] =  mux | CFG_SET_CHAN(chan) | pga |
                              mode | sps | CFG_QUE_DISABLE;
  }
}


/**
 * @brief read analong input (chan) for adc device.
 * @param ads pointer to device struct.
 * @param chan ads1115 analog input channel (0-3).
 * @return returns 0 on success, -1 on error.
 */
int ads_read_channel (ads_t *dev, uint8_t chan)
{
  uint16_t  cnv_config = dev->channel_cfg[chan] | CFG_CNV_START,
            result = 0;
  uint8_t command[] = { APR_CFG_REG,
                        (cnv_config >> 8),
                        cnv_config & 0xff },
          buf[2] = {0},
          n = 0,
          apr_cnv_reg = APR_CNV_REG;
  ssize_t nbytes = 0;

  /* select ads device as slave on i2c bus
   * TODO: if reading all inputs, this only needs to be called once,
   *       move to helper and write a ads_read_all_channels()
   */
  if (ioctl (dev->fd, I2C_SLAVE, dev->i2c_address) < 0) {
    fprintf (stderr, "error: ioctl writing to i2c = 0x%02hhx\n", dev->i2c_address);
    return -1;
  }

  /* write config register to address pointer register and channel
   * config to config register
   */
  if (write (dev->fd, command, sizeof(command)) != sizeof(command)) {
    fprintf (stderr, "error: write channel config - channel %hhu\n"
              "       command: 0x%02hhx 0x%02hhx 0x%02hhx\n",
              chan, command[0], command[1], command[2]);
    return -1;
  }

  /* read config register a max of RETRIES times or until a good read
   * of all bytes and the OS bit (15) indicates conversion ready.
   */
  do {
    /* add appropriate delay for conversion to complete */
    usleep (delay[CFG_GET_DR(dev->channel_cfg[chan])]);
    /* read config register and check result for conversion ready */
    if ((nbytes = read (dev->fd, &buf, sizeof buf)) == -1) {
      perror ("read-config-ready");
      return -1;
    }
    result = (buf[0] << 8) | buf[1];
  } while (n++ < RETRIES && (nbytes != sizeof buf || (result >> 15) == 0));

  if (n == RETRIES) {   /* if retries exhausted, return failure */
    return -1;
  }

  /* write conversion register to address pointer register */
  if (write (dev->fd, &apr_cnv_reg, sizeof apr_cnv_reg) == -1) {
      perror ("write-apr_cnv_reg");
      return -1;
  }

  /* read sample result into buf */
  if (read (dev->fd, &buf, sizeof buf) != sizeof buf) {
    fputs ("read-conversion short result.\n", stderr);
    return -1;
  }

  /* save sample result to device struct */
  dev->sample[chan] = (buf[0] << 8) | buf[1];

  return 0;
}

