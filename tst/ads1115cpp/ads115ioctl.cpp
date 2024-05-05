#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <inttypes.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

/*
  Constants
*/

// ADS1115 registers
enum deviceRegister
{
    conversion    = 0b00,           // Conversion register
    configuration = 0b01,           // Configuration register
    loThreshold   = 0b10,           // Low Threshold register
    hiThreshold   = 0b11            // High Threshold register
};

// Configuration register bit 15 (1 bit long) meaning when reading
enum readOperation
{
    busy          = 0b0 << 15,      // Conversion not ready
    ready         = 0b1 << 15       // Conversion ready to read
};

// Configuration register bit 15 (1 bit long) meaning when writing
enum writeOperation
{
    noop          = 0b0 << 15,      // Not requesting anything
    convert       = 0b1 << 15,      // Requesting conversion
};

// Configuration register bits 12-14 (3 bits long)
enum multiplexer
{
    diff_0_1      = 0b000 << 12,    // Select diff channel 0 to 1
    diff_0_3      = 0b001 << 12,    // Select diff channel 0 to 3
    diff_1_3      = 0b010 << 12,    // Select diff channel 1 to 3
    diff_2_3      = 0b011 << 12,    // Select diff channel 2 to 3
    single_0      = 0b100 << 12,    // Select channel 0
    single_1      = 0b101 << 12,    // Select channel 1
    single_2      = 0b110 << 12,    // Select channel 2
    single_3      = 0b111 << 12     // Select channel 4
};

// Configuration register bits 9-11 (3 bits long)
enum gain
{
    pga_6_144V    = 0b000 << 9,     // +/-6.144V range
    pga_4_096V    = 0b001 << 9,     // +/-4.096V range
    pga_2_048V    = 0b010 << 9,     // +/-2.048V range*
    pga_1_024V    = 0b011 << 9,     // +/-1.024V range
    pga_0_512V    = 0b100 << 9,     // +/-0.512V range
    pga_0_256V    = 0b101 << 9,     // +/-0.256V range
    pga_0_256V2   = 0b110 << 9,     // not used
    pga_0_256V3   = 0b111 << 9      // not used
};

// Configuration register bit 8 (1 bit long)
enum sampleMode
{
    continuous    = 0b0 << 8,       // Continuous conversion mode
    single        = 0b1 << 8        // Power-down single-shot mode*
};

// Configuration register bits 5-7 (3 bits long)
enum sampleRate
{
    sps8          = 0b000 << 5,     // 8 samples per second
    sps16         = 0b001 << 5,     // 16 samples per second
    sps32         = 0b010 << 5,     // 32 samples per second
    sps64         = 0b011 << 5,     // 64 samples per second
    sps128        = 0b100 << 5,     // 128 samples per second*
    sps250        = 0b101 << 5,     // 250 samples per second
    sps475        = 0b110 << 5,     // 475 samples per second
    sps860        = 0b111 << 5,     // 860 samples per second
};

// Configuration register bit 4 (1 bit long)
enum comparatorMode
{
    traditional   = 0b0 << 4,       // Traditional comparator*
    window        = 0b1 << 4        // Window comparator
};

// Configuration register bit 3 (1 bit long)
enum alertPolarity
{
    activeLow     = 0b0 << 3,       // ALERT pin active low*
    activeHigh    = 0b1 << 3        // ALERT pin active high
};

// Configuration register bit 2 (1 bit long)
enum latchMode
{
    nonLatching   = 0b0 << 2,       // ALERT does not latch
    latching      = 0b1 << 2        // ALERT latches until read
};

// Configuration register bits 0-1 (2 bits long)
enum alertMode
{
    alert1        = 0b00 << 0,      // Set ALERT after 1 conversion
    alert2        = 0b01 << 0,      // Set ALERT after 2 conversions
    alert4        = 0b10 << 0,      // Set ALERT after 4 conversions
    none          = 0b11 << 0       // Disable the comparator*
};

// Max channels available on each ADS1115 device
const int CHANNELS = 4;

// How many times to retry polling for conversion result
const int MAX_RETRY = 4;

// I2C addresses for multiple chained ADS1115 devices
const uint8_t DEVICE_ADDRESS[] =
{
    0x48,                       // ADDR pin tied to GND*
    0x49,                       // ADDR pin tied to VDD
    0x4A,                       // ADDR pin tied to SDA
    0x4B                        // ADDR pin tied to SCL
};

// Shortcut to select a channel to read from
const uint16_t SELECT_CHANNEL[] =
{
    multiplexer::single_0,      // Select channel 0
    multiplexer::single_1,      // Select channel 1
    multiplexer::single_2,      // Select channel 2
    multiplexer::single_3       // Select channel 3
};

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

/*
  State
*/

// I2C bus on which ADS1115 device(s) are attached
int i2cBus = 0;

// Number of chained ADS1115 devices
int devices = 0;

// Gain mode
gain adcgain;

// Sample rate
sampleRate rate;

// I2C device handle
int i2c = -1;

/*
  Operations
*/

/*
  Initialize ADS1115 on Raspberry Pi 4B
  initGain - The gain to configure
  initRate - The sample rate to configure
  initDevices - How many ADS1115 are chained on I2C bus
  initBus - Which I2C bus to use
  returns true if successful, false if failed
*/
bool init(
  gain initGain,
  sampleRate  initRate,
  int initDevices,
  int initBus
) {
  // Open I2C bus
  char busName[16] = {0};
  sprintf(busName, "/dev/i2c-%d", initBus);
  i2c = open(busName, O_RDWR);

  // Failed to open I2C bus
  if (i2c < 0) return false;

  // Initialize state
  i2cBus = initBus;
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
  if (i2c < 0 || ioctl(i2c, I2C_SLAVE, DEVICE_ADDRESS[device]) < 0)
  {
    // Failed to select device
    return false;
  }

  // Include a command to switch channel
  config |= SELECT_CHANNEL[channel];

  // Encode command to write to configuration register
  uint8_t command[] = {
    // Select configuration register
    deviceRegister::configuration,
    // Configuration register value MSB (two's complement)
    ((uint8_t*)&config)[1],
    // Configuration register value LSB (two's complement)
    ((uint8_t*)&config)[0]
  };

  if (write(i2c, command, sizeof(command)) != sizeof(command))
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
bool pollDevice (int channel)
{
  uint8_t buffer[2] = {0};
  uint16_t config = 0;
  int retries = 0;

  do
  {
    // Wait for configuration to take effect
    usleep(DELAYS[rate >> 5]);

    // Read configuration register
    if (read(i2c, &buffer, sizeof(buffer)) != sizeof(buffer))
    {
      // Failed to read
      return false;
    }

    // Decode configuration register value from two's complement
    config = buffer[0] << 8 | buffer[1];
  }
  while
  (
    // Wait until conversion is ready...
    (config & readOperation::ready) == 0 &&
    // for this channel...
    ((config >> 12) & 0b111) != (SELECT_CHANNEL[channel] >> 12) &&
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
bool readConversion (uint16_t& value)
{
  // Select conversion register
  uint8_t command[] = { deviceRegister::conversion };

  if (write(i2c, command, sizeof(command)) != sizeof(command))
  {
    // Size to write != size written, failed to write
    return false;
  }

  // Read conversion register
  uint8_t conversion[2] = {0};

  if (read(i2c, &conversion, sizeof(conversion))
      != sizeof(conversion))
  {
    // Size to read != size read, failed to read
    return false;
  }

  // Decode conversion from two's complement
  value = conversion[0] << 8 | conversion[1];
  return true;
}

/*
  Sample a channel on a device
  device - The index of ADS1115 if multiple, or 0
  channel - The channel on ADS1115 0-3
  sample - The value sampled
  returns true if sampled successfully, false otherwise
*/
bool read (int device, int channel, uint16_t& sample)
{
  uint16_t config =
    // Command to request conversion
    writeOperation::convert |
    // Set gain
    adcgain |
    // Set single-shot conversion mode
    sampleMode::single |
    // Set sampling rate
    rate |
    // Not using comparator
    comparatorMode::traditional |
    // Not using latching
    latchMode::nonLatching |
    // Not using alerts
    alertMode::none |
    alertPolarity::activeLow;

  if (configureDevice (device, channel, config) &&
      pollDevice (channel) &&
      readConversion (sample))
  {
    return true;
  }

  return false;
}

/*
  Entry point
*/

int main()
{
  // Initialize one ADS1115 on I2C bus 0 with +/- 4.096 gain & default rate
  if (!init (gain::pga_4_096V, sampleRate::sps128, 1, 1))
    return 1;

  // Read all channels
  uint16_t sample;

  for (int device = 0; device < devices; device++)
  {
    for (int channel = 0; channel < CHANNELS; channel++)
    {
      if (read (device, channel, sample)) {
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
