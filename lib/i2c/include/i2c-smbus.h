#ifndef i2c_smbus_h
#define i2c_smbus_h  1

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * i2c_smbus_xxx function taken from busybox 1.36.1 miscutils/i2c_tools.c.
 * https://elixir.bootlin.com/busybox/latest/source/miscutils/i2c_tools.c
 */


/**
 * i2c_init_device opens the i2c sysfs device and verifies it
 * can communicate with the i2c address for the device.
 *
 * \return returns open i2c file descriptor for device on success
 * -1 otherwise.
 */
int i2c_init_device (const char *i2cdev, uint16_t addr);


/**
 * i2c_close_device closes the i2c device opened with i2c_init_device
 * given the open file descriptor fd.
 */
int i2c_close_device (int fd);


/**
 * I2C read length (len) bytes from I2C device configured with open
 * file-descriptor (fd) from device register (reg) address into
 * the storage pointed to by data.
 */
int32_t i2c_read_reg (int fd, uint8_t cmd, uint8_t *vals, uint8_t len);


/**
 * I2C write length (len) bytes from I2C device configured with open
 * file-descriptor (fd) from device register (reg) address into
 * the storage pointed to by data.
 */
int32_t i2c_write_reg (int fd, uint8_t cmd, const uint8_t *vals, uint8_t len);


/**
 * I2C read byte at cmd reg
 *
 */
int32_t i2c_read_reg_byte (int fd, uint8_t cmd);


/**
 * I2C write byte at cmd reg with val
 *
 */
int32_t i2c_write_reg_byte (int fd, uint8_t cmd, const uint8_t val);


/**
 * I2C write byte at cmd reg with val making RETRIES attempts
 * at intervals of RETRYuSEC microseconds between write attempts.
 *
 */
int32_t i2c_write_reg_byte_delay (int fd, uint8_t cmd, const uint8_t val);


/**
 * I2C read word at cmd reg
 *
 */
int32_t i2c_read_reg_word (int fd, uint8_t cmd);


/**
 * I2C write word at cmd reg with val
 *
 */
int32_t i2c_write_reg_word (int fd, uint8_t cmd, const uint16_t val);


#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

#endif
