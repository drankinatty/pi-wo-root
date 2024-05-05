#ifndef adsrewrite_h
#define adsrewrite_h  1

#include <stdint.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include <unistd.h>

/* ADS1115 Address Pointer Register values */
#define CHANNELS              4

#define APR_CNV_REG           0x00
#define APR_CFG_REG           0x01
#define APR_LO_THRESH         0x02
#define APR_HI_THRESH         0x03

#define CFG_SET_OS(x)         ((x & 0x1) << 15)
#define CFG_SET_MUX(x)        ((x & 0x7) << 12)
#define CFG_SET_PGA(x)        ((x & 0x7) <<  9)
#define CFG_SET_MODE(x)       ((x & 0x1) <<  8)
#define CFG_SET_DR(x)         ((x & 0x7) <<  5)
#define CFG_SET_COMP_MODE(x)  ((x & 0x1) <<  4)
#define CFG_SET_COMP_POL(x)   ((x & 0x1) <<  3)
#define CFG_SET_COMP_LAT(x)   ((x & 0x1) <<  2)
#define CFG_SET_COMP_QUE(x)   (x & 0x3)

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

#define CFG_COMP_MODE_TRAD    CFG_SET_COMP_MODE(0x00)
#define CFG_COMP_MODE_WIN     CFG_SET_COMP_MODE(0x01)

#define CFG_COMP_POL_LOW      CFG_SET_COMP_POL(0x00)
#define CFG_COMP_POL_HIGH     CFG_SET_COMP_POL(0x01)

#define CFG_COMP_LAT_NO       CFG_SET_COMP_LAT(0x00)
#define CFG_COMP_LAT          CFG_SET_COMP_LAT(0x01)

#define CFG_QUE_DISABLE       CFG_SET_COMP_QUE (0x03)

#define CNV_READY             CFG_CNV_START

#define DIFF_0_1              0x00
#define DIFF_0_3              0x01
#define DIFF_1_3              0x02
#define DIFF_2_3              0x03
#define SNGL_AIN0             0x04
#define SNGL_AIN1             0x05
#define SNGL_AIN2             0x06
#define SNGL_AIN3             0x07

#define PGA_6_144             0x00
#define PGA_4_096             0x01
#define PGA_2_048             0x02
#define PGA_1_024             0x03
#define PGA_0_512             0x04
#define PGA_0_256             0x05
#define PGA_0_256a            0x06
#define PGA_0_256b            0x07

#define CONTINUOUS            0x00
#define SINGLE_SHOT           0x01

#define DR_8                  0x00
#define DR_16                 0x01
#define DR_32                 0x02
#define DR_64                 0x03
#define DR_128                0x04
#define DR_250                0x05
#define DR_475                0x06
#define DR_860                0x07

#define COMP_MODE_TRAD        0x00
#define COMP_MODE_WIN         0x01

#define COMP_POL_LOW          0x00
#define COMP_POL_HIGH         0x01

#define COMP_LAT_NO           0x00
#define COMP_LAT              0x01

#define ADDR_GND              0x48
#define ADDR_VDD              0x40
#define ADDR_SDA              0x4a
#define ADDR_SCL              0x4b

typedef struct {
  int fd;
  uint16_t  channel_cfg[CHANNELS],
            sample[CHANNELS];
  uint8_t i2c_address;
} ads_t;

/**
 * @brief open i2c devfs device file assigning the file descriptor and
 * i2c address to the device struct.
 * @param dev pointer to device struct.
 * @param i2cdevfs i2c device filesystem note (e.g. "/dev/i2c-0").
 * @param i2caddr i2c address for device on i2c bus (0x48 - 0x4b).
 * @return returns 0 on success, -1 on error.
 */
int ads_i2c_init (ads_t *dev, const char *i2cdevfs, uint8_t i2caddr);

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
                      uint16_t pga, uint16_t mode, uint16_t sps);

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
                          uint16_t mode, uint16_t sps);

/**
 * @brief read analong input (chan) for adc device.
 * @param ads pointer to device struct.
 * @param chan ads1115 analog input channel (0-3).
 * @return returns 0 on success, -1 on error.
 */
int ads_read_channel (ads_t *dev, uint8_t chan);

#endif
