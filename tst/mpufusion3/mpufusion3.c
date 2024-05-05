#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef IOCTLINIT
#include <wiringx.h>
#endif

#include <stdlib.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <unistd.h>
// #include <stdlib.h>

// #include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>
#include <sys/select.h>
#include <time.h>

#include <fusion.h>

#include "mpu-constants.h"
#include "mpu.h"
#include "mpu_console.h"

#include "itimer.h"
#include "i2c-smbus.h"

// #define MPU9250 1

// static volatile bool mpu_data_rdy;
static volatile bool mpu_raw_data_rdy;
// static volatile bool fusion_data_rdy;
static volatile bool fusion_write_rdy;
// static volatile uint8_t sigrt_mpu_read = 0;
static volatile uint8_t sigrt_mpu_read_raw = 0;
static volatile uint8_t sigrt_fusion_write = 0;

static itimer it0,
              it1;

/* TODO have mpu addr set it WHO_AM_I check during init
 *      after converting fuction internals to switch
 *      allowing assignment to mpu->addr
 */
mpu_t mpu = { .addr = MPU_ADDRESS_AD0_LOW, .id = 0 };

typedef struct {
  int16_t tempc;
  FusionVector3 accel, gyro, vel, pos;
  FusionEulerAngles euler;
} state_t;

/* temp global */
static state_t state = { .tempc = 0 };

/* TODO: get actual chip 1/chip 2 values from Pico i2c-mpu9250-Fusion.c */

const FusionVector3 gyroSens = { { .015258f, .015258f, .015258f } };
const FusionVector3 accelSens = { { 6.103516e-5f,
                                    6.103516e-5f,
                                    6.103516e-5f } };

const FusionVector3 accelBias = {
  // .axis.x =   125.f,  //  (24.6 C)
  // .axis.x =   250.f,  // (raw 2200 - 27.6 C)
  .axis.x =   300.f,  // (raw 2200 - 27.8 C)
  // .axis.y =  -475.f,
  // .axis.y =  -500.f,  // (raw 2200 - 27.6 C)
  .axis.y =  -510.f,  // (raw 2200 - 27.7S C)
  .axis.z =  -130.f,
}; // replace these values with actual accel bias in LSB
const FusionVector3 gyroBias = {
  .axis.x =   15.f,
  .axis.y =   15.f,
  .axis.z =    0.f,
}; // replace these values with actual accel bias in LSB

const FusionRotationMatrix softIronBias = { { 1.0f, 0.0f, 0.0f,
                                              0.0f, 1.0f, 0.0f,
                                              0.0f, 0.0f, 1.0f } };
const FusionVector3 hardIronBias = { { 0.0f, 0.0f, 0.0f } };

const float samplePeriod = 0.01f;             /* sample time 100/sec */
const int samplespersec = 1./samplePeriod;    /* per second */

FusionAhrs ahrs;                    /* fusion structs */
FusionBias bias;

/* make pointer to provide as extern to mpu.c */
#ifdef MILKV
const char *i2cdev = "/dev/i2c-0";
#else
const char *i2cdev = "/dev/i2c-1";
#endif

/* signal handler for coverning mpu read based on itimer signal */
/*
static void sighdlr_mpu_read (int sig, siginfo_t *si, void *uc)
{
  // if (SIGRTMIN <= sig && sig <= SIGRTMAX) {
  if (sig == sigrt_mpu_read) {
    if (get_acc_gyro_tempc (&mpu) && get_mag(&mpu)) {
      mpu_data_rdy = true;
    }
  }

  (void)si;
  (void)uc;
}
*/

static volatile uint16_t readcnt = 0, writecnt = 0;

/* signal handler for coverning mpu read based on itimer signal */
static void sighdlr_mpu_read_raw (int sig, siginfo_t *si, void *uc)
{
  // if (SIGRTMIN <= sig && sig <= SIGRTMAX) {
  if (sig == sigrt_mpu_read_raw) {
      mpu_raw_data_rdy = true;
      readcnt++;
  }

  (void)si;
  (void)uc;
}


/* signal handler for coverning mpu read based on itimer signal */
static void sighdlr_fusion_write (int sig, siginfo_t *si, void *uc)
{
  // if (SIGRTMIN <= sig && sig <= SIGRTMAX) {
  if (sig == sigrt_fusion_write) {
    fusion_write_rdy = true;
    writecnt++;
  }

  (void)si;
  (void)uc;
}


/* pselect with second, nanosecond timout controlling
 * the update of values printed to screen and indicates
 * whether input is available on stdin ('q' quit, etc..)
 */
int pselect_timer (unsigned sec, unsigned nsec)
{
  fd_set set;
  int res;
  struct timespec ts = { sec, nsec };

  /* mask mpu_read_signal */
  sigset_t mask;
  sigemptyset (&mask);
  sigaddset (&mask, sigrt_mpu_read_raw);

  /* Initialize the file descriptor set. */
  FD_ZERO (&set);
  FD_SET (0, &set);

  /* watch STDIN_FILENO with timeout_sec timeout */
  // res = pselect (1, &set, NULL, NULL, &ts, &mask);
  res = pselect (1, &set, NULL, NULL, &ts, NULL);
  (void)mask;

  return res;
}

/* simple empty-stdin function, c must be initialized */
void empty_stdin (int c)
{
  while (c != '\n' || c == EOF) {
    c = getchar();
  }
}

static int mpu_start (int argc, char **argv)
{
  int fdi2c;

  /* initialize required members of C-struct mpu since using
   * implicit constructor to create temporary zeros all and the
   * normal brace initialization generates broken warnings for
   * missing initializer (-Wmissing-field-initializers)
   */
  mpu.addr =  MPU_ADDRESS_AD0_LOW;
  mpu.id = 0;

  /* check if i2cdev passed on command line */
  // i2cdev = argc > 1 ? argv[1] : i2cdev;
  if (argc > 1 && 0 <= *argv[1] - '0' && *argv[1] - '0' <= 9) {
    mpu.id = *argv[1] - '0';
  }
  printf ("MPU ID: %d\n", mpu.id);

  if ((fdi2c = i2c_init_device (i2cdev, MPU_ADDRESS_AD0_LOW)) < 0) {
    fprintf (stderr, "error: IOCTL I2C Setup failed for device '%s' (return: %d)\n",
              i2cdev, fdi2c);
    return 1;
  }
  mpu.fd = fdi2c;

  printf ("I2C Setup succeeded for device '%s', fd: %d\n", i2cdev, mpu.fd);

  if (!mpu_init_default (&mpu)) {
    puts ("mpu_init_default - failed");
    return 1;
  }
  usleep (50000);

  puts ("mpu_init_default - succeeded - mpu9250 identified and initialized");

  if (!ak8963_initialize (&mpu)) {
    puts ("ak8963_initialize - failed");
    printf ("I2C Setup failed for ak8963 device '%s', fd: %d\n",
            i2cdev, mpu.magfd);
    return 1;
  }
  puts ("ak8963_initialize - succeeded - ak8963 identified and initialized");
  printf ("I2C Setup succeeded for ak8963 device '%s', fd: %d\n",
          i2cdev, mpu.magfd);

  return 0;
}

static int timers_start (void)
{
  uint64_t  ns_to_start   = 100000000,
            ns_mpu_read   = 10000000,
            ns_oled_write = 200000000;

  /* initialize interval timers, get real-time signal no.
   *  it0 - mpu_read:   100 times per-second
   *  it1 - oled_write:   5 times per-second
   */
  it0 = itimer_create_timer (ns_to_start, ns_mpu_read, sighdlr_mpu_read_raw);
  it1 = itimer_create_timer (ns_to_start, ns_oled_write, sighdlr_fusion_write);

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

  sigrt_mpu_read_raw = it0.signo;     /* update global values */
  sigrt_fusion_write = it1.signo;

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

/** temporary couner for samples per repeating_timer output period
 *  (output at 200 ms, 20-21 samples per-period => ~100 Hz sample rate.
 */
static volatile uint16_t samples = 0;

void fusion_init (void)
{
  /* Initialise gyroscope bias correction algorithm
   * stationary threshold = 0.5 degrees per second
   */
  FusionBiasInitialise (&bias, 0.5f, samplePeriod);

  /* Initialise AHRS algorithm with gain = 0.5 */
  FusionAhrsInitialise (&ahrs, 0.5f);

  /* magnetic field limits, -70 uT to 70 uT */
  // FusionAhrsSetMagneticField (&ahrs, -70.0f, 70.0f);
  FusionAhrsSetMagneticField (&ahrs, 0.0f, 70.0f);
}

bool fusion_update (void)
{
  int16_t a[3] = {0}, g[3] = {0}, m[3] = {0}, t = 0;

  bool  rawaccgyro = get_raw_acc_gyro_tempc (mpu.fd, a, g, &t),
        rawmag = get_raw_mag (&mpu, m);

  if (!rawaccgyro) {
    puts ("rawaccgyro read failed\n");
    return false;
  }

  if (!rawmag) {
    puts ("rawmag read failed\n");
    return false;
  }
  /* read/validate raw sensor values into arrays */
  // if (get_raw_acc_gyro_tempc (mpu.fd, a, g, &t) && get_raw_mag (&mpu, m)) {
  if (rawaccgyro && rawmag) {

    /* Calibrate gyroscope - parameters are raw values in lsb */
    FusionVector3 uncalibratedGyroscope = {
      .axis.x = g[0],
      .axis.y = g[1],
      .axis.z = g[2]
    };
    FusionVector3 calibratedGyroscope =
                  FusionCalibrationInertial (uncalibratedGyroscope,
                  FUSION_ROTATION_MATRIX_IDENTITY, gyroSens,
                  gyroBias);

    /* Calibrate accelerometer - parameters are raw values in lsb */
    FusionVector3 uncalibratedAccelerometer = {
      .axis.x = a[0],
      .axis.y = a[1],
      .axis.z = a[2]
    };
    FusionVector3 calibratedAccelerometer =
                  FusionCalibrationInertial (uncalibratedAccelerometer,
                  FUSION_ROTATION_MATRIX_IDENTITY, accelSens,
                  accelBias);
                  // FUSION_VECTOR3_ZERO);

    /* Calibrate magnetometer - parameters in lsb */
    FusionVector3 uncalibratedMagnetometer = {
        .axis.x = m[0] /* * 4912. / 32768 */,
        .axis.y = m[1] /* * 4912. / 32768 */,
        .axis.z = m[2] /* * 4912. / 32768 */
    };
    FusionVector3 calibratedMagnetometer =
                  FusionCalibrationMagnetic (uncalibratedMagnetometer,
                  softIronBias, hardIronBias);

    /* Update gyroscope bias correction algorithm */
    calibratedGyroscope = FusionBiasUpdate (&bias, calibratedGyroscope);

    /* Update AHRS algorithm */
    FusionAhrsUpdate (&ahrs, calibratedGyroscope, calibratedAccelerometer,
                              calibratedMagnetometer, samplePeriod);

    /* call fusion update for mpu - not working - t also needed
    FusionVector3 calgyro = fusion_mpu (&ahrs, &bias, &mpu,
                                        &gyroBias, &accelBias,
                                        &gyroSens, &accelSens,
                                        &hardIronBias, &softIronBias);
    */

    /* Get Euler angles */
    FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles (
                                    FusionAhrsGetQuaternion(&ahrs));
    /* Get Acceleration */
    // FusionVector3 accel = FusionAhrsGetEarthAcceleration (&ahrs);
    // FusionVector3 accel = FusionAhrsGetLinearAcceleration (&ahrs);

    /* update state values for timer user_data */
    state.euler = eulerAngles;
    state.accel = FusionAhrsGetLinearAcceleration (&ahrs);
    state.gyro = calibratedGyroscope;

    state.tempc = t;

    samples += 1;

    return true;
  }
  else {
    puts ("fusion update failed");
  }

  return false;
}

// static volatile uint16_t cnt = 0;

void fusion_show_euler (void)
{
  // const state_t *_state = &state;
  uint16_t tempraw = state.tempc;

  if (tempraw > 32768) {
    tempraw -= 65536;
  }

  float room_temp_offset = 0.,
        temp = tempraw / 333.87 - room_temp_offset + 21.;

  printf ("Roll : % 8.1f    Pitch : % 8.1f    Yaw : %8.1f    %5.1f C    (%hu)\n\033[1A",
          state.euler.angle.roll, state.euler.angle.pitch,
          state.euler.angle.yaw, temp, samples);

  samples = 0;
}

/** denoise values with FusionVector3 */
// FusionVector3 fv3_denoise (const FusionVector3 *data, const float limit)
// {
//   /* initialize x, y, z to positive values */
//   FusionVector3 fv = {
//     .axis.x = data->axis.x < 0. ? -data->axis.x : data->axis.x,
//     .axis.y = data->axis.y < 0. ? -data->axis.y : data->axis.y,
//     .axis.z = data->axis.z < 0. ? -data->axis.z : data->axis.z };
//   float lim = limit;
//
//   if (limit < 0.) {           /* ensure limit is positive */
//     lim = -limit;
//   }
//
//   /* denoise x, y & z values */
//   fv.axis.x = fv.axis.x < lim ? 0. : data->axis.x;
//   fv.axis.y = fv.axis.y < lim ? 0. : data->axis.y;
//   fv.axis.z = fv.axis.z < lim ? 0. : data->axis.z;
//
//   return fv;
// }

int main (int argc, char **argv) {

  if (mpu_start (argc, argv) == 1) {    /* start mpu */
    // close_display();
    return 1;
  }
  puts ("mpu started");

  fusion_init();
  puts ("fusion_init done");

  if (timers_start() == 1) {            /* start mpu read and oled write timers */
    // close_display();
    return 1;
  }
  printf ("mpu read & output timers started\n"
          "  sigrt_mpu_read_raw:  %hhu\n"
          "  sigrt_fusion_write:  %hhu\n\n",
          sigrt_mpu_read_raw, sigrt_fusion_write);

  for (int i = 0; i < 500;) {
//     if (get_acc_gyro_tempc (&mpu)) {
//       show_mpu_output (&mpu);
//     }
//     if (fusion_update()) {
//       fusion_show_euler();
//     }
    if (mpu_raw_data_rdy) {
      fusion_update();
      mpu_raw_data_rdy = false;
    }
    if (fusion_write_rdy) {
      fusion_show_euler();
      fusion_write_rdy = false;
      i++;
    }
    if (pselect_timer (0, 1.e8) == 1) {
      int c = getchar();
      if (c == 'q' || c == '\n' || c == EOF) {
        break;
      }
      empty_stdin (c);
    }
    // usleep(100000);
  }

  itimer_delete_timer (&it0);
  itimer_delete_timer (&it1);

  // close_display();

  printf ("\r\033[4B\033[?25h\n");

  printf ("mpu_raw_data_rdy:  %s\n"
          "fusion_write_rdy:  %s\n"
          "samples         :  %hu\n"
          "samples-per-sec :  %d\n",
          mpu_raw_data_rdy ? "true" : "false",
          fusion_write_rdy ? "true" : "false",
          samples, samplespersec);
}
