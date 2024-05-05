

/*!
  @file SSD1306_OLED_RPI/examples/src/SPEED_TEST/main.cpp
  @author Gavin Lyons, modifiend by David C Rankin
  @brief Test file for SSD1306_OLED library,  showing fps frame rate per second

  Project Name: SSD1306_OLED_RPI

  @test
          -# Test 601 FPS test frame rate per second

  @details
          -# results data for test file
          -# I2C_Speed = 0 Fps 7 bcm2835_i2c_set_baudrate(100000)
          -# I2C_Speed = 2500 Fps 6 bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500)
          -# I2C_Speed = 626 Fps 16 bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626)
          -# I2C_Speed = 150 Fps 24 bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_150)
          -# I2C_Speed = 148 FPS 25 bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_148)
*/

#include <ctime>
#include <cstdio>
#include <unistd.h>
#include "SSD1306_OLED.h"

#define myOLEDwidth  128
#define myOLEDheight 64
#define FULLSCREEN (myOLEDwidth * (myOLEDheight/8))
SSD1306 myOLED (myOLEDwidth, myOLEDheight) ; // instantiate  an object

// vars for the test
uint16_t count  = 0;
uint16_t countLimit = 1000;
bool colour = 1;
uint64_t  previousCounter =0;
uint8_t  screenBuffer[FULLSCREEN];


// =============== Function prototype ================
bool SetupTest(void);
void myLoop(void);
uint16_t display_buffer(long , int );
void EndTests(void);
static uint64_t counter( void );


// ======================= Main ===================
int main(void)
{
  double start, stop, elapsed;
  struct timespec ts;

  if (SetupTest() != true) {
    return -1;
  }

  clock_gettime( CLOCK_MONOTONIC, &ts );
  start = ts.tv_sec + ts.tv_nsec / 1e9;

  myLoop();

  clock_gettime( CLOCK_MONOTONIC, &ts );
  stop = ts.tv_sec + ts.tv_nsec / 1e9;

  elapsed = stop - start;
  printf ("%hu frames in %.2f seconds => %.2f fps\n",
          countLimit, elapsed, countLimit / elapsed);

  EndTests();

  return 0;
}
// ======================= End of main  ===================

void EndTests()
{
  myOLED.OLEDPowerDown(); //Switch off display
  myOLED.OLED_I2C_OFF(); // Switch off I2C , optional may effect other programs & devices
  printf("OLED End\r\n");
}

bool SetupTest()
{
  /*
  const uint16_t I2C_Speed = BCM2835_I2C_CLOCK_DIVIDER_626;; // bcm2835I2CClockDivider enum ,see readme.
  const uint8_t I2C_Address = 0x3C;
  bool I2C_debug = false; */
  printf ("OLED Test Begin\r\n");
  // Check if Bcm28235 lib installed and print version.
  // Turn on I2C bus (optional it may already be on)
  if (!myOLED.OLED_I2C_ON())
  {
    printf ("Error 1202: bcm2835_i2c_begin :Cannot start I2C, Running as root?\n");
    return false;
  }

  printf ("SSD1306 library Version Number :: %u\r\n",myOLED.getLibVerNum());
  myOLED.OLEDinit();
  myOLED.OLEDFillScreen (0xF1, 0); // splash screen bars, optional just for effect
  sleep (1);

  return true;
}

void myLoop()
{
  uint16_t fps = 0;
  printf ("OLED Frames Per Second (FPS) test , ends at frame "
          "'countLimit == %hu'\n", countLimit);
  myOLED.setTextColor(WHITE);
  myOLED.setTextSize(1);

  if (!myOLED.OLEDSetBufferPtr (myOLEDwidth, myOLEDheight,
                                screenBuffer, sizeof(screenBuffer))) {
    return;
  }

  while (count < countLimit)
  {
    // uint16_t every_nframes = 100;
    static long framerate = 0;
    fps = display_buffer (framerate, count);
    framerate++;
    count++;
    if (count == 50) {
      uint16_t tmp = countLimit;
      if (fps < 5) { break; }
      else if (fps < 10) { tmp = 100; }
      else if (fps < 15) { tmp = 250; }
      else if (fps < 20) { tmp = 500; }
      /*
      else if (fps < 10) { tmp = 100; every_nframes = 10; }
      else if (fps < 15) { tmp = 250; every_nframes = 25; }
      else if (fps < 20) { tmp = 500; every_nframes = 50; }
      */
      if (tmp != countLimit) {
        printf ("  adjusting countLimit = %hu base on first 50 frames at %hu fps\n"
                "  (I2C bus clock too slow)\n", tmp, fps);
        countLimit = tmp;
      }
    }
    // if (count % every_nframes == 0) {
    if (count % 50 == 0) {
      printf ("  at %4hu frames : %2hu fps\n", count, fps);
    }
  }

  printf ("OLED Framerate   : %hu fps\n", fps);
}

uint16_t display_buffer(long currentFramerate, int count)
{
  // Values to count frame rate per second
  static long lastFramerate = 0;
  static uint16_t fps;
  uint64_t currentCounter = counter();

  if (currentCounter - previousCounter >= 1000000000) {
    //(Rolls over every 1 second)
    fps = currentFramerate - lastFramerate;
    lastFramerate = currentFramerate ;
    previousCounter = currentCounter;
    colour = !colour;
  }

  myOLED.OLEDclearBuffer();

  // ** TEST CODE **
  myOLED.setCursor(0, 0);
  myOLED.print("SSD1306");
  myOLED.setCursor(0, 10);
  myOLED.print("G Lyons");
  myOLED.setCursor(0, 20);
  myOLED.print(myOLED.getLibVerNum());

  myOLED.setCursor(0, 30);
  myOLED.setCursor(0, 40);
  myOLED.print(count);
  myOLED.setCursor(0, 50);
  myOLED.print(fps);
  myOLED.print(" fps");

  myOLED.drawFastVLine(64, 0, 63, WHITE);
  myOLED.fillRect(70, 10, 20, 20, colour);
  myOLED.fillCircle(110, 20, 10, !colour);
  myOLED.drawRoundRect(80, 40, 40, 20, 10, WHITE);
  // ** END of TEST CODE **

  myOLED.OLEDupdate();

  return fps;
}

// This func returns nano-seconds as a 64-bit unsigned number,
// monotonically increasing, since system boot.
// The actual resolution looks like microseconds. returns nanoseconds
static uint64_t counter( void )
{
  struct timespec now;
  clock_gettime( CLOCK_MONOTONIC, &now );
  return  ((uint64_t)now.tv_sec * 1000000000U) + (uint64_t)now.tv_nsec;
}
