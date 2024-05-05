/**
 *  Inversense MPU API
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */

/* FIXME - move denoise to only apply to console output functions */

/* accomodates MPU6050 - MPU9250 */
// #include <math.h>
#ifdef MILKVWX
#include <wiringx.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif

// #include <stdio.h>

#include <stdlib.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unistd.h>

#include "mpu.h"
#include "mpu-constants.h"
#include "mpu-calibrate.h"
#include "mpu-caldata.h"

#define GENERIC_ERROR  -1

/* sample timer, data and output ready flags for signal handlers */
volatile sig_atomic_t sample_timer_rdy;
volatile sig_atomic_t mpu_data_rdy;
volatile sig_atomic_t mpu_output_rdy;

/* extern variables from mpu_calibrate.c */
extern uint32_t nsamples;               /* current number of calibration samples */
// extern volatile bool autocalibrated;    /* auto-calibrate successfully completed */

/* static varibles used in this source */
static itimer it0, it1;     /* timer struct instances for read and output timers */
static mpu_t *mpuptr;       /* global mpu_t for signal handlers set in mpu_start */

static int mag_addr = 0;

/* enum defining Inversense mpu types */
enum { UNKNOWN, MPU_6050, MPU_9250, MPU_9255 };
const char *mputypes[] = { "UNKNOWN", "MPU_6050", "MPU_9250", "MPU_9255" };

/* typedef for signal handler functions */
typedef void (*sighdlr)(int, siginfo_t*, void*);


#ifdef RPIPICO
/** i2c helper functions for read and write with default values */
int i2c_read (uint8_t addr, uint8_t *dst, size_t len)
{
  /* i2c_default, addr, and nostop = false (master releases bus control) */
  return i2c_read_blocking (i2c_default, addr, dst, len, false);
}

int i2c_write (uint8_t addr,  uint8_t *src, size_t len)
{
  /* i2c_default, addr, and nostop = true (master retains bus control) */
  return i2c_write_blocking (i2c_default, addr, src, len, true);
}

/* to read after writing register address */
int i2c_read_reg (uint8_t addr, const uint8_t reg_addr, uint8_t *dst,
                  size_t len)
{
  /* no STOP bit should be required with write of addr before read */
  int rtn = i2c_write (addr, (uint8_t*)&reg_addr, 1);

  if (rtn == PICO_ERROR_GENERIC) {
    return rtn;
  }

  /* i2c_default, addr, and nostop = false (master releases bus control) */
  return i2c_read_blocking (i2c_default, addr, dst, len, false);
}

/* to write data to a specific register address before an independent read */
int i2c_write_reg (uint8_t addr, const uint8_t reg_addr, uint8_t *src,
                   size_t len)
{
  uint8_t buffer[RWBUFSZ] = {0};

  buffer[0] = reg_addr;

  for (size_t i = 0; i < len; i++) {
    buffer[i + 1] = src[i];
  }

  /* no STOP bit should be required with write of addr before read */
  return i2c_write_blocking (i2c_default, addr, buffer, len + 1, false);
}
#else
/**
 * \note I2C function below duplicate functions available in the
 *       companion libi2c-smbus.so library. They can be removed
 *       and replaced in the future just by including <i2csmbus.h>
 *       and linking against -li2csmbus.
 *
 */

/** i2c_init_device initializes i2c device at addr on given _i2cdev fs
 *
 * \description i2c_init_device opens the i2c sysfs device and verifies it
 * can communicate with the i2c address for the device.
 * \parameters _i2cdev is "/dev/i2c-X" fs devices for i2c bus.
               addr is address of device on i2c bus.
 *
 * \return returns open i2c file descriptor for device on success
 * -1 otherwise.
 */
int i2c_init_device (const char *_i2cdev, uint16_t addr)
{
  int fd;

  if ((fd = open (_i2cdev, O_RDWR)) < 0) {
    return -1;
  }

  if (ioctl (fd, I2C_SLAVE, addr) < 0) {
    return -1;
  }

  return fd;
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

#endif


/** reset accel, gyro and temperature sensor signal paths */
bool mpu_reset (const uint8_t mpu_addr)
{
  uint8_t data = 0x00;

  return i2c_write_reg (mpu_addr, MPU_SIGNAL_PATH_RESET, &data, 1) == (1 /* + 1 */);
}


/**
 *  Check whether MPU is available on the I2C BUS
 */
bool mpu_i2c_check (mpu_t *mpu)
{
  uint8_t data;

  if (i2c_read_reg (mpu->fd, MPU_WHO_AM_I, &data, 1) != 1) {
    return false;
  }

  switch (data) {
    case MPU_WHO_AM_I_DATA:
      mpu->addr   = MPU_DEFAULT_ADDRESS;
      mpu->type   = MPU_9250;
      mpu->typenm = mputypes[MPU_9250];
      mag_addr    = AK8963_DEFAULT_ADDRESS;
      break;
    case MPU9255_WHO_AM_I_DATA:
      mpu->addr   = MPU_DEFAULT_ADDRESS;
      mpu->type   = MPU_9255;
      mpu->typenm = mputypes[MPU_9255];
      mag_addr    = AK8963_DEFAULT_ADDRESS;
      break;
    case MPU6050_WHO_AM_I_DATA:
      mpu->addr   = MPU_DEFAULT_ADDRESS;
      mpu->type   = MPU_6050;
      mpu->typenm = mputypes[MPU_6050];
      break;
    default:
      return false;
      break;
  }

  return true;
}


/* (defined but not used) */
/** lookup accel sensitivity in G from config value */
// static uint8_t lookup_accel_sens_in_g (mpu_t *mpu)
// {
//   uint8_t accel_sens_sel[] = { 2, 4, 8, 16 };
//
//   return accel_sens_sel[mpu->accel_fs_sel];
// }


/** lookup gyro sensitivity in deg/s from config value */
// static uint16_t lookup_gyro_sens_in_dps (mpu_t *mpu)
// {
//   uint16_t gyro_sens_sel[] = { 250, 500, 1000, 2000 };
//
//   return gyro_sens_sel[mpu->gyro_fs_sel];
// }


/** get accel sensitivity config value from G */
static uint8_t get_accel_fs_sel_from_g (uint8_t accel_sens)
{
  uint8_t accel_sens_sel[] = { 2, 4, 8, 16 };

  for (uint8_t i = 0; i < sizeof accel_sens_sel; i++) {
    if (accel_sens <= accel_sens_sel[i]) {
      return i;   /* config bit value */
    }
  }

  return 0xff;    /* error */
}


/** get gyro sensitivity config value from deg/s */
static uint8_t get_gyro_fs_sel_from_dps (uint16_t gyro_sens)
{
  uint16_t gyro_sens_sel[] = { 250, 500, 1000, 2000 };

  for (uint8_t i = 0; i < sizeof gyro_sens_sel; i++) {
    if (gyro_sens <= gyro_sens_sel[i]) {
      return i;   /* config bit value */
    }
  }

  return 0xff;    /* error */
}

/**
 * NOTE: need to read and incorporate MPU self-test offsets from get_factory_st()
 */

/**
 *  Initialize mpu gyro with FS_SEL = 250 and accel FS_SEL = 2
 *
 *  (note: idiotic +1 on write return removed from i2c_write_reg()
 *   for non-pico API. TODO fix pico API by subtracting 1 from return)
 *
 */
bool mpu_init_default (mpu_t *mpu)
{
  uint8_t data = 0x00;

  mpu->gyro_sens = 250;
  mpu->accel_sens = 2;

  /* confirm mpu found on I2C bus or return error */
  if (!mpu_i2c_check (mpu)) {
    return false;
  }

  /* reset sample rate divisor zero  */
  if (i2c_write_reg (mpu->fd, MPU_SMPLRT_DIV, &data, sizeof data) !=
      sizeof data /* + 1 */) {
    return false;
  }

  /* reset all sensors - write 0 to MPU_PWR_MGMT_2 */
  if (i2c_write_reg (mpu->fd, MPU_PWR_MGMT_2, &data, sizeof data) !=
      sizeof data /* + 1 */) {
    return false;
  }

  /* PWR1 set - auto select best available clock */
  data = MPU_CLOCK_PLL_XGYRO;          /* (0x01) */
  if (i2c_write_reg (mpu->fd, MPU_PWR_MGMT_1, &data, sizeof data) !=
      sizeof data /* + 1 */) {
    return false;
  }

  /* set DLPF to bandwidth 92Hz for gyro and 98Hz temperature sensor */
  data = mpu->dlpf_cfg = 0x02;

  if (i2c_write_reg (mpu->fd, MPU_CONFIG, &data, sizeof data) !=
      sizeof data /* + 1 */) {
    return false;
  }

  /* set DLPF to bandwidth 99Hz for accel */
  if (i2c_write_reg (mpu->fd, MPU_ACCEL_CONFIG_2, &data, sizeof data) !=
      sizeof data /* + 1 */) {
    return false;
  }

  /* set gyro FS_SEL (250 dps) */
  mpu->gyro_fs_sel = data = 0x00;
  // mpu->gyro_fs_sel = data = 0x01 << 3; // ( 500 dps range)
  // mpu->gyro_sens = 500.;
  // mpu->gyro_fs_sel = data = 0x02 << 3; // (1000 dps range)
  // mpu->gyro_sens = 1000.;

  if (i2c_write_reg (mpu->fd, MPU_GYRO_CONFIG, &data, sizeof data) !=
      sizeof data /* + 1 */) {
    return false;
  }

  /* set accel FS_SEL (2 g) */
  mpu->accel_fs_sel = data = 0x00;
  if (i2c_write_reg (mpu->fd, MPU_ACCEL_CONFIG, &data, sizeof data) !=
      sizeof data /* + 1 */) {
    return false;
  }

  /* if saved factory self-test data, use as fingerprint to set mpu->id */
  if (mpu_st_data != NULL) {
    mpu->st_data = mpu_st_data;
    if (get_factory_st (mpu)) {
      get_id_from_factory_st (mpu);
    }
  }

  /* set accel and gyro bias */
  // set_gyro_usr_offsets (mpu);
  // set_accel_bias (mpu);
  set_room_temp_offset (mpu);

  return true;
}


/**
 *  Initialize mpu gyro and accel with values given by gyro_fs_sel and
 *  accel_fs_sel, if parameter values do not match valid FS-SEL values the
 *  corresponding gyro or accel will be initialized to the default FS_SEL
 *  values of 250 deg/sec and 2 g.
 */
bool mpu_init (mpu_t *mpu, uint16_t gyro_fs_sel_dps, uint8_t accel_fs_sel_g,
               uint8_t dlpfval)
{
  uint8_t data = 0x00;

  /* set mpu->gyro_sens and get gyro config values from fs_sel deg/s */
  mpu->gyro_sens = gyro_fs_sel_dps;
  mpu->gyro_fs_sel = get_gyro_fs_sel_from_dps (gyro_fs_sel_dps);
  if (mpu->gyro_fs_sel == 0xff) {
    return false;
  }

  /* set mpu->accel_sens and get accel config values from fs_sel in g */
  mpu->accel_sens = accel_fs_sel_g;
  mpu->accel_fs_sel = get_accel_fs_sel_from_g (accel_fs_sel_g);
  if (mpu->accel_fs_sel == 0xff) {
    return false;
  }

  /* confirm mpu found on I2C bus or return error */
  if (!mpu_i2c_check (mpu)) {
    return false;
  }

  /* reset sample rate divisor zero  */
  if (i2c_write_reg (mpu->fd, MPU_SMPLRT_DIV, &data, sizeof data) !=
      sizeof data /* + 1 */) {
    return false;
  }

  /* reset all sensors - write 0 to MPU_PWR_MGMT_2 */
  if (i2c_write_reg (mpu->fd, MPU_PWR_MGMT_2, &data, sizeof data) !=
      sizeof data /* + 1 */) {
    return false;
  }

  /* PWR1 set - auto select best available clock */
  data = MPU_CLOCK_PLL_XGYRO;          /* (0x01) */
  if (i2c_write_reg (mpu->fd, MPU_PWR_MGMT_1, &data, sizeof data) !=
      sizeof data /* + 1 */) {
    return false;
  }

  /* set DLPF bandwidth for gyro and temperature sensor */
  mpu->dlpf_cfg = data = dlpfval;   /* (single rate for both accel & gyro) */

  if (i2c_write_reg (mpu->fd, MPU_CONFIG, &data, sizeof data) !=
      sizeof data /* + 1 */) {
    return false;
  }

  /* set DLPF bandwidth for accel */
  if (i2c_write_reg (mpu->fd, MPU_ACCEL_CONFIG_2, &data, sizeof data) !=
      sizeof data /* + 1 */) {
    return false;
  }

  /* set gyro sensitivity */
  data = mpu->gyro_fs_sel << 3;
  if (i2c_write_reg (mpu->fd, MPU_GYRO_CONFIG, &data, sizeof data) !=
      sizeof data /* + 1 */) {
    return false;
  }

  /* set accel FS_SEL and sensitivity */
  data = mpu->accel_fs_sel << 3;
  if (i2c_write_reg (mpu->fd, MPU_ACCEL_CONFIG, &data, sizeof data) !=
      sizeof data /* + 1 */) {
    return false;
  }

  /* if saved factory self-test data, use as fingerprint to set mpu->id */
  if (mpu_st_data != NULL) {
    mpu->st_data = mpu_st_data;
    if (get_factory_st (mpu)) {
      get_id_from_factory_st (mpu);
    }
  }

  /* set accel and gyro bias */
  // set_gyro_usr_offsets (mpu);
  // set_accel_bias (mpu);
  set_room_temp_offset (mpu);

  return true;
}


/* apply limit around zero to reduce noise in values */
void denoise_values (float *dest, const float *data, const float limit)
{
  /* initialize x, y, z to positive values */
  float   x = data[0] < 0. ? -data[0] : data[0],
          y = data[1] < 0. ? -data[1] : data[1],
          z = data[2] < 0. ? -data[2] : data[2],
          lim = limit;

  if (limit < 0.) {           /* ensure limit is positive */
    lim = -limit;
  }

  /* denoise x, y & z values */
  dest[0] = x < lim ? 0. : data[0];
  dest[1] = y < lim ? 0. : data[1];
  dest[2] = z < lim ? 0. : data[2];
}


/**
 *  Get raw acceleration values for linear and angular rate.
 */
bool get_raw_acc_gyro (const uint8_t addr, int16_t *acc, int16_t *gyro)
{
  uint8_t buffer[RWBUFSZ] = {0};

  if (i2c_read_reg (addr, MPU_ACCEL_XOUT_H, buffer, 14) != 14) {
    return false;
  }

  acc[0] = (((uint16_t)buffer[0]) << 8) | buffer[1];
  acc[1] = (((uint16_t)buffer[2]) << 8) | buffer[3];
  acc[2] = (((uint16_t)buffer[4]) << 8) | buffer[5];

  gyro[0] = (((uint16_t)buffer[8]) << 8) | buffer[9];
  gyro[1] = (((uint16_t)buffer[10]) << 8) | buffer[11];
  gyro[2] = (((uint16_t)buffer[12]) << 8) | buffer[13];

  if ((uint16_t)acc[0] > 32767) { acc[0] -= 65536; }
  if ((uint16_t)acc[1] > 32767) { acc[1] -= 65536; }
  if ((uint16_t)acc[2] > 32767) { acc[2] -= 65536; }

  if ((uint16_t)gyro[0] > 32767) { gyro[0] -= 65536; }
  if ((uint16_t)gyro[1] > 32767) { gyro[1] -= 65536; }
  if ((uint16_t)gyro[2] > 32767) { gyro[2] -= 65536; }

  return true;
}


/**
 *  Convert raw linear acceleration and angular rate values into g and deg/sec.
 */
bool get_acc_gyro (mpu_t *mpu)
{
  int16_t raw_acc[3] = {0}, raw_gyro[3] = {0};

  if (!get_raw_acc_gyro (mpu->fd, raw_acc, raw_gyro)) {
    return false;
  }

  mpu->accel.x = (raw_acc[0] / 32768.0) * mpu->accel_sens;
  mpu->accel.y = (raw_acc[1] / 32768.0) * mpu->accel_sens;
  mpu->accel.z = (raw_acc[2] / 32768.0) * mpu->accel_sens;

  mpu->gyro.x = (raw_gyro[0] / 32768.0) * mpu->gyro_sens;
  mpu->gyro.y = (raw_gyro[1] / 32768.0) * mpu->gyro_sens;
  mpu->gyro.z = (raw_gyro[2] / 32768.0) * mpu->gyro_sens;

#ifdef SETACCBIAS
  add_accel_bias (mpu);
#endif

  return true;
}


/**
 *  Get raw acceleration values for linear and angular rate with temperature.
 */
bool get_raw_acc_gyro_tempc (const uint8_t addr, int16_t *acc, int16_t *gyro,
                             int16_t *tempc)
{
  uint8_t buffer[RWBUFSZ] = {0};

  if (i2c_read_reg (addr, MPU_ACCEL_XOUT_H, buffer, 14) != 14) {
    return false;
  }

  acc[0] = (((uint16_t)buffer[0]) << 8) | buffer[1];
  acc[1] = (((uint16_t)buffer[2]) << 8) | buffer[3];
  acc[2] = (((uint16_t)buffer[4]) << 8) | buffer[5];

  *tempc = (((uint16_t)buffer[6]) << 8) | buffer[7];

  gyro[0] = (((uint16_t)buffer[8]) << 8) | buffer[9];
  gyro[1] = (((uint16_t)buffer[10]) << 8) | buffer[11];
  gyro[2] = (((uint16_t)buffer[12]) << 8) | buffer[13];

  if ((uint16_t)acc[0] > 32767) { acc[0] -= 65536; }
  if ((uint16_t)acc[1] > 32767) { acc[1] -= 65536; }
  if ((uint16_t)acc[2] > 32767) { acc[2] -= 65536; }

  if ((uint16_t)*tempc > 32767) { *tempc -= 65536; }

  if ((uint16_t)gyro[0] > 32767) { gyro[0] -= 65536; }
  if ((uint16_t)gyro[1] > 32767) { gyro[1] -= 65536; }
  if ((uint16_t)gyro[2] > 32767) { gyro[2] -= 65536; }

  return true;
}


/**
 *  Convert raw linear acceleration, angular rate and temperature values into
 *  g, deg/sec and degrees C.
 */
bool get_acc_gyro_tempc (mpu_t *mpu)
{
  int16_t raw_acc[3] = {0}, raw_gyro[3] = {0}, raw_tempc = 0;

  if (!get_raw_acc_gyro_tempc (mpu->fd, raw_acc, raw_gyro, &raw_tempc)) {
    return false;
  }

  mpu->accel.x = (raw_acc[0] / 32768.0) * mpu->accel_sens;
  mpu->accel.y = (raw_acc[1] / 32768.0) * mpu->accel_sens;
  mpu->accel.z = (raw_acc[2] / 32768.0) * mpu->accel_sens;

  mpu->tempc = raw_tempc / 333.87 - mpu->room_temp_offset + 21.;

  mpu->gyro.x = (raw_gyro[0] / 32768.0) * mpu->gyro_sens;
  mpu->gyro.y = (raw_gyro[1] / 32768.0) * mpu->gyro_sens;
  mpu->gyro.z = (raw_gyro[2] / 32768.0) * mpu->gyro_sens;

#ifdef SETACCBIAS
  add_accel_bias (mpu);
#endif

  return true;
}


#ifdef MPU9250
/** Code specific to the AK8953 magnetometer on the MPU9250
 *
 *  NOTE: only one MPU9250 can be used at a time with magnetometer
 *        readings due to mag address being fixed at 0x0C. Unlike
 *        the mpu address that can be changed with AD0 high, there
 *        is no way to differentiate between magnetometers.
 */

/**
 *  Obtain i2c device file descriptor for magnetometer i2c bus address.
 *  (only used on Linux implementations, using ioctl through /dev/i2c-X
 *  filesystem, not on bare-metal devices like Pico)
 */
static int ak8963_get_fd (const char *i2cdev)
{
#ifdef MILKVWX
  if ((mag_addr = wiringXI2CSetup (i2cdev, AK8963_DEFAULT_ADDRESS)) < 0) {
    return -1;
  }

  return mag_addr;
#else
  return mag_addr = i2c_init_device (i2cdev, AK8963_DEFAULT_ADDRESS);
#endif
}

/**
 *  Check whether AK8953 is available on the I2C BUS
 */
bool ak8963_i2c_check (void)
{
  uint8_t data = i2c_read_reg_byte (mag_addr, AK8963_WAI);
#ifdef MAGMULTII2C
  int fail = 0;

  /* add retry for x86_64 Raspberry Pi (FIXME re-solder ground pins) */
  while (data != AK8963_WAI_DATA && fail++ < 5) {
    usleep (200000);
    data = i2c_read_reg_byte (mag_addr, AK8963_WAI);
  }

  if (data != AK8963_WAI_DATA) {
    return false;
  }
#else
  // i2c_read_reg (mag_addr, AK8963_WAI, &data, 1);
  data = i2c_read_reg_byte (mag_addr, AK8963_WAI);

  if (data != AK8963_WAI_DATA) {
    return false;
  }
#endif

  return true;
}


/** setter function for magnetometer hard and soft-iron bias */
void ak8963_set_bias_scale (mpu_t *mpu, const float *bias, const float *scale)
{
  /* set magnetometer hard-iron bias in uT
   * from previously determined values
   * to recenter axis origin.
   */
  mpu->mag_bias.x = bias[0];
  mpu->mag_bias.y = bias[1];
  mpu->mag_bias.z = bias[2];

  /* set magnetometer soft-iron axis-scale
   * from previously determined dimensionless values
   * to rescale axis to normalized range.
   */
  mpu->mag_scale.x = scale[0];
  mpu->mag_scale.y = scale[1];
  mpu->mag_scale.z = scale[2];
}


/**
 *  Initialize the AK8963 magnetometer CNTL1 by placing the magnetometer in
 *  FUSE mode to read the Axis Sensitivity Adjustment (ASA) values from
 *  FUSE_ROM, power-down the chip between mode changes and then place the
 *  magnetometer in 16-bit adc and continual measurement 2 for normal
 *  operations. Save the computed adjustments factorsin mag_asa.x, mag_asa.y, mag_asa.z.
 *  (Page 51 & 52 of Register_Map)
 */
#ifndef RPIPICO
bool ak8963_initialize (mpu_t *mpu, const char *i2cdev)
#else
bool ak8963_initialize (mpu_t *mpu)
#endif
{
  uint8_t cntl1_mode = AK8963_CNTL1_OUT_BITS | AK8963_CNTL1_CONT_MSRMT2,
          fuse_mode = AK8963_CNTL1_OUT_BITS | AK8963_CNTL1_FUSE_ROM,
          reset = 0x00,
          buffer[RWBUFSZ / 5] = {0};

#ifndef RPIPICO
  /* get AK8963 i2c file descriptor:
   *   magnetometer uses same /dev/i2c-X device as accel/gyro but must use a
   *   separate file descriptor because magnetometer appears at separate
   *   i2c bus address of AK8963_DEFAULT_ADDRESS (0x0C).
   *
   *   i2cfd is saved as mag_addr allowing duo/Pi and Pico implementations
   *   to share same code.
   */
  if (ak8963_get_fd (i2cdev) == -1) {
    return false;
  }
  mpu->magfd = mag_addr;
#endif

  /* confirm magnetometer found on I2C bus or return error */

  if (!ak8963_i2c_check()) {
    return false;
  }

  // puts ("init - ak8963_i2c_check() - SKIPPED");
  /* set AK8963 in FUSE mode:
   *   write fuse_mode to AK8963_CNTL1
   *   read axis sensitivity adjustments into buffer from AK8963_mag_asa.x
   */
  if (i2c_write_reg (mag_addr, AK8963_CNTL1, &fuse_mode, sizeof fuse_mode) !=
      sizeof fuse_mode /* + 1 */) {
    return false;
  }
#ifdef RPIPICO
  sleep_ms (1);
#else
  usleep (1000);
#endif

  if (i2c_read_reg (mag_addr, AK8963_ASAX, buffer, 3) != 3) {
    return false;
  }
#ifdef RPIPICO
  sleep_ms (1);
#else
  usleep (1000);
#endif

  /* power down ak8963 before mode change */
  if (i2c_write_reg (mag_addr, AK8963_CNTL1, &reset, sizeof reset) !=
      sizeof reset /* + 1 */) {
    return false;
  }
#ifdef RPIPICO
  sleep_ms (10);
#else
  usleep (10000);
#endif

  /* set 16-bit output and continuous measurement mode 2 (8Hz samples) */
  if (i2c_write_reg (mag_addr, AK8963_CNTL1, &cntl1_mode, sizeof cntl1_mode) !=
      sizeof cntl1_mode /* + 1 */) {
    return false;
  }
#ifdef RPIPICO
  sleep_ms (1);
#else
  usleep (1000);
#endif

  /* save sensitivity adjustment factors from the per-axis FUSE_ROM ASA as:
   *
   *  mag_asa.x = (ASA - 128) * 0.5 / 128. + 1
   *
   *  apply to measurement values H as
   *
   *    Hadj = H * mag_asa.x
   */
  mpu->mag_asa.x = (buffer[0] - 128) * 0.5 / 128. + 1;
  mpu->mag_asa.y = (buffer[1] - 128) * 0.5 / 128. + 1;
  mpu->mag_asa.z = (buffer[2] - 128) * 0.5 / 128. + 1;

  return true;
}

volatile uint_fast8_t drdymax = 0;
volatile uint32_t hoflset = 0;

/**
 *  Get raw magnetometer orientation values stored in two's-compliment
 *  little-endian format.
 */
bool get_raw_mag (mpu_t *mpu, int16_t *data)
{
  uint8_t buffer[RWBUFSZ] = {0};
  uint_fast8_t n = 0;

  do {  /* read AK8963_STI (status 1) until DRDY 1 (data ready) */
    i2c_read_reg (mag_addr, AK8963_STI, buffer, 1);
    n += 1;
  } while (!buffer[0] && n < DRDY_LIMIT);

  // do {  /* read AK8963_STI (status 1) until DRDY 1 (data ready) */
  //   i2c_read_reg (mag_addr, AK8963_STI, buffer, 1);
  //   n += 1;
  //   if (buffer[0] == 1) {
  //     break;
  //   }
  //   usleep (1000);
  // } while (n < DRDY_LIMIT);

  if (n > drdymax) {
    drdymax = n;
  }

  /* read magnetometer values and AK8963_ST2 that follows */
  if (i2c_read_reg (mag_addr, AK8963_HXL, buffer, 7) != 7) {
    return false;
  }

  /* check status 2 register HOFL Magnetic Overflow bit.
   * if set, magnetic data is incorrect, discard and keep
   * current value;
   */
  if (buffer[6] & 0x08) {
    hoflset += 1;
    return true;
  }

  data[0] = buffer[0] | ((int16_t)buffer[1] << 8);
  data[1] = buffer[2] | ((int16_t)buffer[3] << 8);
  data[2] = buffer[4] | ((int16_t)buffer[5] << 8);

  /* ASA is part of raw value */
  data[0] *= mpu->mag_asa.x;
  data[1] *= mpu->mag_asa.y;
  data[2] *= mpu->mag_asa.z;

#ifdef USE_MAG_BIAS
  data[0] += mag_offset[0];
  data[1] += mag_offset[1];
  data[2] += mag_offset[2];
#endif

  return true;
}


/**
 *  Convert raw magnetometer values into mx, my, mz in uT
 */
bool get_mag (mpu_t *mpu)
{
  int16_t raw[3] = {0};
  float mag_sens = MAG_SENS;          /* magnetometer sensitivity: 4800 uT */

  if (!get_raw_mag (mpu, raw)) {
    return false;
  }

  /* convert to uT */
  mpu->mag.x = (raw[0] / 32760.0) * mag_sens;
  mpu->mag.y = (raw[1] / 32760.0) * mag_sens;
  mpu->mag.z = (raw[2] / 32760.0) * mag_sens;

  /* mag_bias.x used as a flag to indicate whether bias & scale set */
  if (mpu->mag_bias.x) {
    mpu->mag.x -= mpu->mag_bias.x;
    mpu->mag.y -= mpu->mag_bias.y;
    mpu->mag.z -= mpu->mag_bias.z;

    mpu->mag.x *= mpu->mag_scale.x;
    mpu->mag.y *= mpu->mag_scale.y;
    mpu->mag.z *= mpu->mag_scale.z;
  }

  return true;
}

#endif


/**
 *  signal handler for mpu read itimer signal (currently 0.01s)
 */
static void sighdlr_mpu_sample (int sig, siginfo_t *si, void *uc)
{
  if (sig == mpuptr->itimer_sample->signo) {
    sample_timer_rdy = true;
  }

  (void)si;     /* suppress -Wunused warnings */
  (void)uc;
}


/**
 *  signal handler for data output itimer signal (currently 0.2s)
 */
static void sighdlr_mpu_output (int sig, siginfo_t *si, void *uc)
{
  if (sig == mpuptr->itimer_output->signo) {
    /* if data values are ready */
    if (mpu_data_rdy) {
      mpu_output_rdy = true;            /* set data output flag true */
      mpu_data_rdy = false;             /* set data ready flag false */
    }
  }

  (void)si;     /* suppress -Wunused warnings */
  (void)uc;
}


/**
 *  pselect with second, nanosecond timout controlling program loop
 *  frequency and the update of values printed to screen and indicates
 *  whether input is available on stdin ('q' quit, etc..)
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
 *  initialize and configure MPU accel, gyro and magnetometer features.
 */
int mpu_start (mpu_t *mpu, const char *i2cdev, uint8_t mpu_i2c_addr,
                uint16_t gyrofs, uint8_t accelfs, uint8_t dlpf)
{
#ifndef RPIPICO
  int fdi2c;

  /* set global instance of mpuptr */
  mpuptr = mpu;

  /* initialize required members of C-struct mpu since using
   * implicit constructor to create temporary zeros all and the
   * normal brace initialization generates broken warnings for
   * missing initializer (-Wmissing-field-initializers)
   */
  mpu->addr =  mpu_i2c_addr;
  mpu->id = 0;

  /* initialize i2c device obtaining i2c file descriptor */
  if ((fdi2c = i2c_init_device (i2cdev, mpu_i2c_addr)) < 0) {
    fprintf (stderr, "error: IOCTL I2C Setup failed for device '%s' (return: %d)\n",
              i2cdev, fdi2c);
    return 1;
  }
  mpu->fd = fdi2c;

  printf ("I2C Setup succeeded for device '%s', fd: %d\n", i2cdev, mpu->fd);
#endif

  /* if gyro, accel and DLPF init values provided */
  if (250 <= gyrofs && gyrofs <= 2000 &&
      2 <= accelfs && accelfs <= 16 &&
      dlpf <= 7) {
    if (!mpu_init (mpu, gyrofs, accelfs, dlpf)) {
      fputs ("error: mpu_init - failed.\n", stderr);
      return 1;
    }
  }
  else {  /* otherwise default initialize */
    if (!mpu_init_default (mpu)) {
      fputs ("error: mpu_init_default - failed\n", stderr);
      return 1;
    }
  }
  usleep (50000);

  puts ("mpu_init_default - succeeded - mpu9250 identified and initialized");

#ifdef MPU9250
#ifndef RPIPICO
  if (!ak8963_initialize (mpu, i2cdev)) {
#else
  if (!ak8963_initialize (mpu)) {
#endif
    puts ("ak8963_initialize - failed");
    printf ("I2C Setup failed for ak8963 device '%s', fd: %d\n",
            i2cdev, mpu->magfd);
    return 1;
  }
  puts ("ak8963_initialize - succeeded - ak8963 identified and initialized");
  printf ("I2C Setup succeeded for ak8963 device '%s', fd: %d\n\n",
          i2cdev, mpu->magfd);

  /* setting mag bias and scale causes them to be automaically applied
   * in get_mag() when mag_bias.x is non-zero.
   *
   * NOTE: will need to move MAGBIAS and Accel/Gyro axis scale into mpu_calibrate
   *       based on MPU chip ID and apply from saved values as all require
   *       the mpu be moved through the full range of all axis to compute scale
   *       along with soft/hard iron biases -- which is impractical to do on
   *       every mpu start. Values can be checked periodically.
   */
  set_usr_mag_bias_scale (mpu);

#endif

  return 0;
}


/**
 *  configures and starts timers for mpu read and output after delay of ns_to_start
 */
int timers_start (uint64_t ns_to_start, uint64_t ns_mpu_read, uint64_t ns_mpu_output)
{
  /* initialize interval timers, get real-time signal no.
   *  it0 - mpu_read:   100 times per-second
   *  it1 - oled_write:   5 times per-second
   */
  it0 = itimer_create_timer (ns_to_start, ns_mpu_read, sighdlr_mpu_sample);
  it1 = itimer_create_timer (ns_to_start, ns_mpu_output, sighdlr_mpu_output);

  /* validate realtime signal no. returned */
  if (it0.signo < SIGRTMIN || SIGRTMAX < it0.signo) {
    fputs ("error: itimer_create_timer - mpu_read\n", stderr);
    return 1;
  }

  /* validate realtime signal no. returned */
  if (it1.signo < SIGRTMIN || SIGRTMAX < it1.signo) {
    fputs ("error: itimer_create_timer - oled-write\n", stderr);
    return 1;
  }

  mpuptr->itimer_sample = &it0;     /* update global values */
  mpuptr->itimer_output = &it1;

  if (itimer_start_timer (&it0) == 0) {   /* start reading mpu */
    fputs ("error: itimer_start_timer it0 - mpu read\n", stderr);
    return 1;
  }

  if (itimer_start_timer (&it1) == 0) {   /* start mpu output */
    fputs ("error: itimer_start_timer it1 - oled_write\n", stderr);
    return 1;
  }

  return 0;
}


/**
 *  mpu_stop shut down timers and close file descriptors
 *
 *  \brief stop mpu output and sample timers while file descriptors
 *  are closed and then delete the interval timers.
 *
 *  \return returns 0 on success, -1 on error.
 */
int mpu_stop (mpu_t *mpu)
{
  int rtn = 0;

  /* stop mpu timers */
  itimer_stop_timer (mpu->itimer_output);
  itimer_stop_timer (mpu->itimer_sample);

#ifndef RPIPICO
  int err = 0;

  /* close i2c-smbus file descriptors */
  if ((err = close (mpu->fd)) == -1) {
    rtn = err;
  }
#ifdef MPU9250
  /* close magnetometer file descriptor */
  if ((err = close (mpu->magfd)) == -1) {
    rtn = err;
  }
#endif

#endif

  /* delete mpu timers */
  itimer_delete_timer (mpu->itimer_sample);
  itimer_delete_timer (mpu->itimer_output);

  return rtn;
}
