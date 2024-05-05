#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <inttypes.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>


#define CHANNELS    4

/* ADS1115 Address Pointer Register */
#define APR_CNV_REG           0x00
#define APR_CFG_REG           0x01
#define APR_LO_THRESH         0x02
#define APR_HI_THRESH         0x03

/* ADS1115 Configuration Register
 *  § 9.6.3 Config Register (P[1:0] = 1h) [reset = 8583h]
 *    OS        - operational status (w: 1 - start single conversion)
 *    MUX       - input multiplexer
 *    PGA       - programmable gain amplifier
 *    MODE      - operating mode (0 - continuous, 1 - single-shot)
 *    DR        - data rate - samples per second
 *    COMP_MODE`- comparator mode (0 - traditional, 1 - window)
 *    COMP_POL  - comparator polarity (0 - active low, 1 - active high)
 *    COMP_LAT  - latching comparator (0 - non-latching, 1 - latching)
 *    COMP_QUE  - comparator queue (11 - disable, otherwise enabled)
 */
#define CFG_SET_OS(x)         ((x & 0x01) << 15)
#define CFG_SET_MUX(x)        ((x & 0x07) << 12)
#define CFG_SET_PGA(x)        ((x & 0x07) <<  9)
#define CFG_SET_MODE(x)       ((x & 0x01) <<  8)
#define CFG_SET_DR(x)         ((x & 0x07) <<  5)
#define CFG_SET_COMP_MODE(x)  ((x & 0x01) <<  4)
#define CFG_SET_COMP_POL(x)   ((x & 0x01) <<  3)
#define CFG_SET_COMP_LAT(x)   ((x & 0x01) <<  2)
#define CFG_SET_COMP_QUE(x)   (x & 0x03)

#define CFG_SET_CHAN(x)       ((x & 0x3) << 12)
#define CFG_GET_CHAN(_cfg)    ((_cfg >> 12) & 0x03)

#define I2C_COMMAND(_cfg,_chan)   (_cfg | CFG_SET_CHAN(_chan))

#define CFG_GET_OS(x)         ((x >> 15) & 0x01)
#define CFG_GET_MUX(x)        ((x >> 12) & 0x07)
#define CFG_GET_PGA(x)        ((x >>  9) & 0x07)
#define CFG_GET_MODE(x)       ((x >>  8) & 0x01)
#define CFG_GET_DR(x)         ((x >>  5) & 0x07)
#define CFG_GET_COMP_MODE(x)  ((x >>  4) & 0x01)
#define CFG_GET_COMP_POL(x)   ((x >>  3) & 0x01)
#define CFG_GET_COMP_LAT(x)   ((x >>  2) & 0x01)
#define CFG_GET_COMP_QUE(x)   (x & 0x03)

#define CFG_CNV_START         CFG_SET_OS(0x01)

#define CFG_DIFF_0_1          CFG_SET_MUX(0x00)
#define CFG_DIFF_0_3          CFG_SET_MUX(0x01)
#define CFG_DIFF_1_3          CFG_SET_MUX(0x02)
#define CFG_DIFF_2_3          CFG_SET_MUX(0x03)
#define CFG_SNGL_AIN0         CFG_SET_MUX(0x04)
#define CFG_SNGL_AIN1         CFG_SET_MUX(0x05)
#define CFG_SNGL_AIN2         CFG_SET_MUX(0x06)
#define CFG_SNGL_AIN3         CFG_SET_MUX(0x07)

#define CFG_PGA_6_144         CFG_SET_PGA(0x00)
#define CFG_PGA_4_096         CFG_SET_PGA(0x01)
#define CFG_PGA_2_048         CFG_SET_PGA(0x02)
#define CFG_PGA_1_024         CFG_SET_PGA(0x03)
#define CFG_PGA_0_512         CFG_SET_PGA(0x04)
#define CFG_PGA_0_256         CFG_SET_PGA(0x05)
#define CFG_PGA_0_256a        CFG_SET_PGA(0x06)
#define CFG_PGA_0_256b        CFG_SET_PGA(0x07)

#define CFG_CONTINUOUS        CFG_SET_MODE(0x00)
#define CFG_SINGLE_SHOT       CFG_SET_MODE(0x01)

#define CFG_DR_8              CFG_SET_DR(0x00)
#define CFG_DR_16             CFG_SET_DR(0x01)
#define CFG_DR_32             CFG_SET_DR(0x02)
#define CFG_DR_64             CFG_SET_DR(0x03)
#define CFG_DR_128            CFG_SET_DR(0x04)
#define CFG_DR_250            CFG_SET_DR(0x05)
#define CFG_DR_475            CFG_SET_DR(0x06)
#define CFG_DR_860            CFG_SET_DR(0x07)

#define CFG_QUE_DISABLE       CFG_SET_COMP_QUE (0x03)

#define CNV_READY             CFG_CNV_START

#define ADDR_GND              0x48
#define ADDR_VDD              0x40
#define ADDR_SDA              0x4a
#define ADDR_SCL              0x4b

const char *i2cdev[] = { "/dev/i2c-0", "/dev/i2c-1" };

// Configuration register bit 15 (1 bit long) meaning when reading
// enum readOperation
// {
//     busy          = 0x0 << 15,      // Conversion not ready
//     ready         = 0x1 << 15       // Conversion ready to read
// };

// Configuration register bit 15 (1 bit long) meaning when writing
// enum writeOperation
// {
//     noop          = 0x0 << 15,      // Not requesting anything
//     convert       = 0x1 << 15,      // Requesting conversion
// };

// Configuration register bits 12-14 (3 bits long)
// enum multiplexer
// {
//     diff_0_1      = 0x00 << 12,     // Select diff channel 0 to 1
//     diff_0_3      = 0x01 << 12,     // Select diff channel 0 to 3
//     diff_1_3      = 0x02 << 12,     // Select diff channel 1 to 3
//     diff_2_3      = 0x03 << 12,     // Select diff channel 2 to 3
//     single_0      = 0x04 << 12,     // Select channel 0
//     single_1      = 0x05 << 12,     // Select channel 1
//     single_2      = 0x06 << 12,     // Select channel 2
//     single_3      = 0x07 << 12      // Select channel 4
// };

// Configuration register bits 9-11 (3 bits long)
enum gain
{
    pga_6_144V    = 0x00 << 9,      // +/-6.144V range
    pga_4_096V    = 0x01 << 9,      // +/-4.096V range
    pga_2_048V    = 0x02 << 9,      // +/-2.048V range*
    pga_1_024V    = 0x03 << 9,      // +/-1.024V range
    pga_0_512V    = 0x04 << 9,      // +/-0.512V range
    pga_0_256V    = 0x05 << 9,      // +/-0.256V range
    pga_0_256V2   = 0x06 << 9,      // not used
    pga_0_256V3   = 0x07 << 9       // not used
};

// Configuration register bit 8 (1 bit long)
enum sampleMode
{
    continuous    = 0x0 << 8,       // Continuous conversion mode
    single        = 0x1 << 8        // Power-down single-shot mode*
};

// Configuration register bits 5-7 (3 bits long)
enum sampleRate
{
    sps8          = 0x00 << 5,      // 8 samples per second
    sps16         = 0x01 << 5,      // 16 samples per second
    sps32         = 0x02 << 5,      // 32 samples per second
    sps64         = 0x03 << 5,      // 64 samples per second
    sps128        = 0x04 << 5,      // 128 samples per second*
    sps250        = 0x05 << 5,      // 250 samples per second
    sps475        = 0x06 << 5,      // 475 samples per second
    sps860        = 0x07 << 5       // 860 samples per second
};

// Configuration register bit 4 (1 bit long)
enum comparatorMode
{
    traditional   = 0x0 << 4,       // Traditional comparator*
    window        = 0x1 << 4        // Window comparator
};

// Configuration register bit 3 (1 bit long)
enum alertPolarity
{
    activeLow     = 0x0 << 3,       // ALERT pin active low*
    activeHigh    = 0x1 << 3        // ALERT pin active high
};

// Configuration register bit 2 (1 bit long)
enum latchMode
{
    nonLatching   = 0x0 << 2,       // ALERT does not latch
    latching      = 0x1 << 2        // ALERT latches until read
};

// Configuration register bits 0-1 (2 bits long)
enum alertMode
{
    alert1        = 0x00 << 0,      // Set ALERT after 1 conversion
    alert2        = 0x01 << 0,      // Set ALERT after 2 conversions
    alert4        = 0x02 << 0,      // Set ALERT after 4 conversions
    none          = 0x03 << 0       // Disable the comparator*
};

// Max channels available on each ADS1115 device
// const int CHANNELS = 4;

// How many times to retry polling for conversion result
const int MAX_RETRY = 4;

// I2C addresses for multiple chained ADS1115 devices
const uint8_t i2c_address[] =
{
    ADDR_GND,                       /* ADDR pin tied to GND (default) */
    ADDR_VDD,                       /* ADDR pin tied to VDD */
    ADDR_SDA,                       /* ADDR pin tied to SDA */
    ADDR_SCL                        /* ADDR pin tied to SCL */
};

// Shortcut to select a channel to read from
// const uint16_t SELECT_CHANNEL[] =
// {
//     single_0,      // Select channel 0
//     single_1,      // Select channel 1
//     single_2,      // Select channel 2
//     single_3       // Select channel 3
// };

// Shortcut to look up delay for sample rate
const int DELAYS[] =
{
    1000 + 1000 * 1000 / 8,     // Delay for 8 samples/sec rate
    1000 + 1000 * 1000 / 16,    // Delay for 16 samples/sec rate
    1000 + 1000 * 1000 / 32,    // Delay for 32 samples/sec rate
    1000 + 1000 * 1000 / 64,    // Delay for 64 samples/sec rate
    1000 + 1000 * 1000 / 128,   // Delay for 128 samples/sec rate
    1000 + 1000 * 1000 / 250,   // Delay for 250 samples/sec rate
    1000 + 1000 * 1000 / 475,   // Delay for 475 samples/sec rate
    1000 + 1000 * 1000 / 860    // Delay for 860 samples/sec rate
};

// I2C bus on which ADS1115 device(s) are attached
// #ifdef MILKVFS
// int i2cBus = 0;
// #else
// int i2cBus = 1;
// #endif

// Number of chained ADS1115 devices
int devices = 0;

// Gain mode
enum gain adcgain;

// Sample rate
enum sampleRate rate;

// file descriptor for open i2cdevfs
int i2cfd = -1;

typedef struct {
  int fd;
  uint16_t channel_cfg[CHANNELS];
  uint8_t i2c_address;
} ads_t;

int ads_i2c_init (ads_t *dev, const char *i2cdevfs, uint8_t i2caddr)
{
  if ((dev->fd = open (i2cdevfs, O_RDWR)) == -1) {
    perror ("open-i2cdevfs");
    return -1;
  }

  dev->i2c_address = i2caddr;

  return 0;
}

void ads_channel_cfg (ads_t *dev, uint8_t chan, uint16_t mux,
                      uint16_t pga, uint16_t mode, uint16_t sps)
{
  // dev->channel_cfg[chan] =  CFG_SET_MUX(mux)    |
  //                           CFG_SET_CHAN(chan)  |
  //                           CFG_SET_PGA(pga)    |
  //                           CFG_SET_MODE(mode)  |
  //                           CFG_SET_DR(sps)     |
  //                           CFG_QUE_DISABLE;

  dev->channel_cfg[chan] =  mux | pga | mode | sps| CFG_QUE_DISABLE;
}

/*
  Initialize ADS1115 on Raspberry Pi 4B
  initGain - The gain to configure
  initRate - The sample rate to configure
  initDevices - How many ADS1115 are chained on I2C bus
  initBus - Which I2C bus to use
  returns true if successful, false if failed
*/
bool init (const char *i2cdevfs, enum gain initGain, enum sampleRate initRate, int initDevices)
{
  // Open I2C device
  i2cfd = open (i2cdevfs, O_RDWR);

  // Failed to open I2C bus
  if (i2cfd < 0) {
    return false;
  }

  // Initialize state
  // i2cBus = initBus;
  devices = initDevices;
  adcgain = initGain;
  rate = initRate;

  return true;
}

/*
  Helper method to request conversion on a channel
  device - The index of ADS1115 to talk to (up to 4 can be chained)
  channel - The channel to read
  config - The configuration options
*/
bool configureDevice (int device, int channel, uint16_t config)
{
  // Select device to talk to on I2C bus
  if (i2cfd < 0 || ioctl(i2cfd, I2C_SLAVE, i2c_address[device]) < 0)
  {
    // Failed to select device
    return false;
  }

  // Include a command to switch channel
  // config |= SELECT_CHANNEL[channel];
  config |= CFG_SET_CHAN(channel);

  // Encode command to write to configuration register
  uint8_t command[] = {
    // Select configuration register
    APR_CFG_REG,
    // Configuration register value MSB (two's complement)
    ((uint8_t*)&config)[1],
    // Configuration register value LSB (two's complement)
    ((uint8_t*)&config)[0]
  };

  if (write(i2cfd, command, sizeof(command)) != sizeof(command))
  {
    // If size to write != size written, error
    return false;
  }

  return true;
}

/*
  Helper method to wait for conversion result
  channel - The channel to poll
  returns true if ready to read, false otherwise
*/
bool poll_single_channel (int channel)
{
  uint8_t buffer[2] = {0};
  uint16_t config = 0;
  int retries = 0;

  do {
    // Wait for configuration to take effect
    usleep(DELAYS[rate >> 5]);

    // Read configuration register
    if (read(i2cfd, &buffer, sizeof(buffer)) != sizeof(buffer))
    {
      // Failed to read
      return false;
    }

    // Decode configuration register value from two's complement
    config = buffer[0] << 8 | buffer[1];
  } while (
            // Wait until conversion is ready...
            (config & CNV_READY) == 0 &&
            // for this channel...
            (CFG_GET_CHAN(config) != channel) &&
            // and there are retries left
            retries++ < MAX_RETRY
          );

  return retries < MAX_RETRY;
}

/*
  Helper method to read conversion result when ready
  value - The variable to set to the conversion result
  returns true if read successfully, false otherwise
*/
bool readConversion (uint16_t *value) {
  // Select conversion register
  uint8_t command[] = { APR_CNV_REG };

  if (write(i2cfd, command, sizeof(command)) != sizeof(command)) {
    // Size to write != size written, failed to write
    return false;
  }

  // Read conversion register
  uint8_t reg_conv[2] = {0};

  if (read(i2cfd, reg_conv, sizeof(reg_conv)) != sizeof(reg_conv)) {
    // Size to read != size read, failed to read
    return false;
  }

#ifdef DEBUG
  printf ("  reg_conv: 0x%02hhx, 0x%02hhx\n", reg_conv[0], reg_conv[1]);
#endif

  // Decode reg_conv from two's complement
  *value = reg_conv[0] << 8 | reg_conv[1];

  return true;
}

/*
  Sample a channel on a device
  device - The index of ADS1115 if multiple, or 0
  channel - The channel on ADS1115 0-3
  sample - The value sampled
  returns true if sampled successfully, false otherwise
*/
bool readadc (int device, int channel, uint16_t *sample)
{

  uint16_t config = CFG_CNV_START   |     /* Command to request conversion */
                    CFG_SNGL_AIN0   |     /* AIN0 - GND */
                    adcgain         |     /* set gain */
                    single          |     /* single-shot conversion mode */
                    rate            |     /* sample rate */
                    traditional     |     /* not using comparator */
                    nonLatching     |     /* non-latching */
                    none            |     /* no alerts */
                    activeLow;

#ifdef DEBUG
  printf ("  config: 0x%04hx\n", config | (channel & 0x7) << 12);
#endif

  if (configureDevice (device, channel, config) &&
                        poll_single_channel (channel) &&
                        readConversion (sample)) {
    return true;
  }

  return false;
}

int main()
{
  /* for 0-3.3V nitialize ADS1115 on I2C bus 0 with 4.096 gain & default rate */
  if (!init (i2cdev[1], CFG_PGA_4_096, CFG_DR_128, 1)) {
    return 1;
  }

  // Read all channels
  uint16_t sample;

  for (int device = 0; device < devices; device++)
  {
    for (int channel = 0; channel < CHANNELS; channel++)
    {
      if (readadc (device, channel, &sample)) {
        /* LSB = FSR / 65536 => (2 * 4.096) / 65536 => 4.096 / 32678
         * see datasheet § 9.3.3 Full-Scale Range (FSR) and LSB Size
         */
        if (sample >> 15) {   /* 0 - 3.3v can't be negative */
          sample = 0;         /* see § 9.5.4 Data Format */
        }
        printf ("device %d channel %d = %.2f V\n",
                device, channel, (float)sample * 4.096f / 32768);
      }
    }
  }

  return 0;
}
