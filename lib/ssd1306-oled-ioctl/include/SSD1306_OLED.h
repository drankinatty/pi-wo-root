/*!
* @file     SSD1306_OLED.h
* @brief    OLED driven by SSD1306 controller.
* @author   Gavin Lyons. David C. Rankin (modified to use Linux ioctl())
* @details  <https://github.com/gavinlyonsrepo/SSD1306_OLED_RPI>
*/

#ifndef SSD1306_H
#define SSD1306_H  1

// Library includes

#include <cstdio>
#include <cstdint>
#include <cstdbool>
// #include <bcm2835.h>
#include "SSD1306_OLED_graphics.h"

extern "C" {

#include <i2c-smbus.h>
}

//  SSD1306 Command Set

// Fundamental Commands
#define SSD1306_SET_CONTRAST_CONTROL    0x81
#define SSD1306_DISPLAY_ALL_ON_RESUME   0xA4
#define SSD1306_DISPLAY_ALL_ON          0xA5
#define SSD1306_NORMAL_DISPLAY          0xA6
#define SSD1306_INVERT_DISPLAY          0xA7
#define SSD1306_DISPLAY_OFF             0xAE
#define SSD1306_DISPLAY_ON              0xAF
#define SSD1306_NOP                     0xE3

// Scrolling Commands
#define SSD1306_RIGHT_HORIZONTAL_SCROLL              0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL               0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A
#define SSD1306_DEACTIVATE_SCROLL                    0x2E
#define SSD1306_ACTIVATE_SCROLL                      0x2F
#define SSD1306_SET_VERTICAL_SCROLL_AREA             0xA3

// Addressing Setting Commands
#define SSD1306_SET_LOWER_COLUMN   0x00
#define SSD1306_SET_HIGHER_COLUMN  0x10
#define SSD1306_MEMORY_ADDR_MODE   0x20
#define SSD1306_SET_COLUMN_ADDR    0x21
#define SSD1306_SET_PAGE_ADDR      0x22

// Hardware Configuration Commands
#define SSD1306_SET_START_LINE        0x40
#define SSD1306_SET_SEGMENT_REMAP     0xA0
#define SSD1306_SET_MULTIPLEX_RATIO   0xA8
#define SSD1306_COM_SCAN_DIR_INC      0xC0
#define SSD1306_COM_SCAN_DIR_DEC      0xC8
#define SSD1306_SET_DISPLAY_OFFSET    0xD3
#define SSD1306_SET_COM_PINS          0xDA
#define SSD1306_CHARGE_PUMP           0x8D

// Timing & Driving Scheme Setting Commands
#define SSD1306_SET_DISPLAY_CLOCK_DIV_RATIO  0xD5
#define SSD1306_SET_PRECHARGE_PERIOD         0xD9
#define SSD1306_SET_VCOM_DESELECT            0xDB

// I2C related
#define SSD1306_COMMAND        0x00
#define SSD1306_DATA           0xC0
#define SSD1306_DATA_CONTINUE  0x40
#define SSD1306_ADDR           0x3C  /**< I2C address alt 0x3D */

#ifdef MILKVFS
#define I2CDEVFS              "/dev/i2c-0"
#else
#define I2CDEVFS              "/dev/i2c-1"
#endif

#define SSD1306_command(Reg)  I2C_Write_Byte(Reg, SSD1306_COMMAND)
#define SSD1306_data(Data)    I2C_Write_Byte(Data, SSD1306_DATA_CONTINUE)
// #define SSD1306_command(cmd) i2c_write_reg_byte_delay (od->fd, SSD1306_COMMAND, cmd)
// #define SSD1306_data(data) i2c_write_reg_byte_delay (od->fd, SSD1306_DATA_CONTINUE, data)

// Pixel color
#define BLACK   0
#define WHITE   1
#define INVERSE 2

// Delays
#define SSD1306_INITDELAY 100 /**< Initialisation delay in mS */


/*!
	@brief class to control OLED and define buffer
*/
class SSD1306 : public SSD1306_graphics  {
 public:
  SSD1306 (int16_t oledwidth, int16_t oledheight);
  ~SSD1306(){};

  virtual void drawPixel (int16_t x, int16_t y, uint8_t color) override;
  void OLEDupdate (void);
  void OLEDclearBuffer (void);
  void OLEDBufferScreen (int16_t x, int16_t y, uint8_t w, uint8_t h,
                          uint8_t* data);
  void OLEDFillScreen (uint8_t pixel, uint8_t mircodelay);
  void OLEDFillPage (uint8_t page_num, uint8_t pixels,uint8_t delay);
  OLED_Return_Codes_e OLEDBitmap (int16_t x, int16_t y,
                                  int16_t w, int16_t h,
                                  const uint8_t* data, bool invert);
  bool OLEDbegin (const char *I2C_devfs  = I2CDEVFS,
                  uint8_t I2c_address = SSD1306_ADDR,
                  bool I2C_debug = false);
  bool OLEDSetBufferPtr (uint8_t width, uint8_t height,
                          uint8_t* pBuffer, uint16_t sizeOfBuffer);
  bool OLED_I2C_ON (void);
  void OLED_I2C_OFF (void);
  void OLEDinit (void);
  void OLEDPowerDown (void);

  void OLEDEnable (uint8_t on);
  void OLEDContrast (uint8_t OLEDcontrast);
  void OLEDInvert (bool on);

  void OLEDStartScrollRight (uint8_t start, uint8_t stop);
  void OLEDStartScrollLeft (uint8_t start, uint8_t stop) ;
  void OLEDStartScrollDiagRight (uint8_t start, uint8_t stop);
  void OLEDStartScrollDiagLeft (uint8_t start, uint8_t stop);
  void OLEDStopScroll (void);

  const char *getI2Cdevfs (void);
  void setI2Cdevfs (const char*);
  int getI2Cfd (void);

  uint8_t OLEDCheckConnection (void);

  uint8_t OLEDI2CErrorGet (void);

  uint16_t OLEDI2CErrorTimeoutGet (void);
  void OLEDI2CErrorTimeoutSet (uint16_t);

  uint8_t OLEDI2CErrorRetryNumGet (void);
  void OLEDI2CErrorRetryNumSet (uint8_t);

  bool OLEDDebugGet (void);
  void OLEDDebugSet (bool);

  uint16_t getLibVerNum (void);

 private:

  void I2C_Write_Byte (uint8_t value, uint8_t Cmd);

  /**< /dev/fs device for I2C bus interface */
  const char *_I2C_devfs =  I2CDEVFS;
  /**< I2C address */
  uint8_t _I2C_address = SSD1306_ADDR;
  /**< open I2C file descriptor (initialized in error condition) */
  int _I2C_fd = -1;
  /**< I2C debug flag default false  */
  bool _I2C_DebugFlag = false;
  /**<I2C delay(in between retry attempts) in event of error in mS*/
  uint16_t _I2C_ErrorDelay = 100;
  /**< In event of I2C error number of retry attempts*/
  uint8_t _I2C_ErrorRetryNum = 3;
  /**< In event of I2C error holds bcm2835 I2C reason code 0x00 = success*/
  uint8_t _I2C_ErrorFlag = 0x00;

  /**< Library version number */
  uint16_t _LibraryVersionNum = 160;

  /**< Width of OLED Screen in pixels */
  int16_t _OLED_WIDTH;
  /**< Height of OLED Screen in pixels */
  int16_t _OLED_HEIGHT;
  /**< Number of byte size pages OLED screen is divided into */
  int8_t _OLED_PAGE_NUM;
  /**< Width of Screen Buffer */
  uint8_t bufferWidth;
  /**< Height of Screen Buffer */
  uint8_t bufferHeight;

  /**< pointer to buffer which holds screen data */
  uint8_t *OLEDbuffer = nullptr;
};

/*
struct i2cdef {
  int fd;
  uint8_t i2caddr;
  i2cdef (int fdi2c, uint8_t addr) : fd {fdi2c}, i2caddr {addr} {}
};

extern i2cdef *od;
*/

#endif
