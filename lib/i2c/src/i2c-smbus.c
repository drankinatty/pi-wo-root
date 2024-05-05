#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "i2c-smbus.h"

#define RETRIES         4
#define RETRYuSEC   10000

/**
 * i2c_init_device opens the i2c sysfs device and verifies it
 * can communicate with the i2c address for the device.
 *
 * \return returns open i2c file descriptor for device on success
 * -1 otherwise.
 */
int i2c_init_device (const char *i2cdev, uint16_t addr)
{
  int fd;

  if ((fd = open (i2cdev, O_RDWR)) < 0) {
    return -1;
  }

  if (ioctl (fd, I2C_SLAVE, addr) < 0) {
    return -1;
  }

  return fd;
}

/**
 * i2c_close_device closes the i2c device opened with i2c_init_device
 * given the open file descriptor fd.
 */
int i2c_close_device (int fd)
{
  return close (fd);
}

/**
 * i2c_smbus_xxx function taken from busybox 1.36.1 miscutils/i2c_tools.c.
 * https://elixir.bootlin.com/busybox/latest/source/miscutils/i2c_tools.c
 */

/**
 *  I2C helper function providing ioctl call for both
 *  i2c_smbus_read_i2c_block_data and i2c_smbus_write_i2c_block_data.
 *
 */
static int32_t i2c_smbus_access (int fd, char read_write, uint8_t cmd,
				 int size, union i2c_smbus_data *data)
{
  struct i2c_smbus_ioctl_data args;

  args.read_write = read_write;
  args.command = cmd;
  args.size = size;
  args.data = data;

  return ioctl (fd, I2C_SMBUS, &args);
}


/**
 * I2C read length (len) bytes from I2C device configured with open
 * file-descriptor (fd) from device register (reg) address into
 * the storage pointed to by data.
 */
int32_t i2c_read_reg (int fd, uint8_t cmd, uint8_t *vals, uint8_t len)
{
  union i2c_smbus_data data;
  int i, err;

  if (len > I2C_SMBUS_BLOCK_MAX) {
    len = I2C_SMBUS_BLOCK_MAX;
  }
  data.block[0] = len;

  err = i2c_smbus_access (fd, I2C_SMBUS_READ, cmd,
                          len == 32 ? I2C_SMBUS_I2C_BLOCK_BROKEN :
                                      I2C_SMBUS_I2C_BLOCK_DATA, &data);
  if (err < 0) {
    return err;
  }

  for (i = 1; i <= data.block[0]; i++) {
    *vals++ = data.block[i];
  }
  return data.block[0];
}


/**
 * I2C write length (len) bytes from I2C device configured with open
 * file-descriptor (fd) from device register (reg) address into
 * the storage pointed to by data.
 */
int32_t i2c_write_reg (int fd, uint8_t cmd, const uint8_t *vals, uint8_t len)
{
  int err;
  union i2c_smbus_data data;

  if (len > I2C_SMBUS_BLOCK_MAX) {
    len = I2C_SMBUS_BLOCK_MAX;
  }

  for (int i = 0; i < len; i++) {
    data.block[i+1] = vals[i];
  }
  data.block[0] = len;

  err = i2c_smbus_access (fd, I2C_SMBUS_WRITE, cmd,
                          I2C_SMBUS_I2C_BLOCK_BROKEN, &data);

  if (err < 0) {
    return err;
  }

  return data.block[0];
}

/**
 * I2C read byte at cmd reg
 *
 */
int32_t i2c_read_reg_byte (int fd, uint8_t cmd)
{
  union i2c_smbus_data data;
  int err;

  err = i2c_smbus_access (fd, I2C_SMBUS_READ, cmd, I2C_SMBUS_BYTE_DATA, &data);

  if (err < 0) {
    return err;
  }

  return data.byte & 0xff;
}

/**
 * I2C write byte at cmd reg with val
 *
 */
int32_t i2c_write_reg_byte (int fd, uint8_t cmd, const uint8_t val)
{
  union i2c_smbus_data data;
  int err;

  data.byte = val;

  err = i2c_smbus_access (fd, I2C_SMBUS_WRITE, cmd, I2C_SMBUS_BYTE_DATA, &data);

  if (err < 0) {
    return err;
  }

  return data.byte;
}


/**
 * I2C write byte at cmd reg with val making RETRIES attempts
 * at intervals of RETRYuSEC microseconds between write attempts.
 *
 */
int32_t i2c_write_reg_byte_delay (int fd, uint8_t cmd, const uint8_t val)
{
  int n = RETRIES,
      rtn = 0;

  while (n--) {
    rtn = i2c_write_reg_byte (fd, cmd, val);
    if (rtn >= 0) {
      return rtn;
    }
    usleep (RETRYuSEC);
  }

  return rtn;
}


/**
 * I2C read word at cmd reg
 *
 */
int32_t i2c_read_reg_word (int fd, uint8_t cmd)
{
  union i2c_smbus_data data;
  int err;

  err = i2c_smbus_access (fd, I2C_SMBUS_READ, cmd, I2C_SMBUS_WORD_DATA, &data);

  if (err < 0) {
    return err;
  }

  return data.word & 0xffff;
}

/**
 * I2C write word at cmd reg with val
 *
 */
int32_t i2c_write_reg_word (int fd, uint8_t cmd, const uint16_t val)
{
  union i2c_smbus_data data;
  int err;

  data.word = val;

  err = i2c_smbus_access (fd, I2C_SMBUS_WRITE, cmd, I2C_SMBUS_WORD_DATA, &data);

  if (err < 0) {
    return err;
  }

  return data.word;
}

