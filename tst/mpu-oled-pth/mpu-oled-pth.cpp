/**
 *  Inversense MPU9250 on SSD1306 OLED Display for Raspberry Pi
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */

#include <cstdio>
#include <cstdint>

#include <unistd.h>
/**
 * write to oled from separate thread to maintain 100Hz sample rate
 */
#include <pthread.h>

/**
 * Include oled library header, compile against libssd1306
 */
#include <SSD1306_OLED.h>

extern "C" {
#include <itimer.h>           /* itimer, compile against libitimer*/
#include <i2c-smbus.h>        /* i2c-smbus, compile against libi2c */
#include <mpu.h>              /* MPU, compile against libmpu */
#include <mpu-constants.h>
#include <mpu-calibrate.h>
#include <mpu-console.h>
}

/**
 * pthread validation macros
 */
#define handle_error_en(en, msg) \
  do { if (en) { errno = en; perror(msg); exit(EXIT_FAILURE); }} while (0)

#define handle_error(msg) \
  do { perror(msg); exit(EXIT_FAILURE); } while (0)

/* BUILD_64 */
#if defined(__LP64__) || defined(_LP64)
# define BUILD_64   1
#endif

/* OLED size */
#define myOLEDwidth  128
#define myOLEDheight  64
#define FULLSCREEN (myOLEDwidth * (myOLEDheight/8))

/* number of update iterations at 5 per-second (500 -> 100 second runtime */
#define NUPDATES     500

/* interval timer start delay and intervals for mpu sample, output and
 * oled page 2 display update interval for values not requiring update
 * multiple times per-second (e.g. temperature)
 */
#define ITIMER_START_DELAY    20000000          /* 0.02 sec */
#define MPU_SAMPLE_NSEC       10000000          /* 0.01 sec */
#define MPU_OUTPUT_NSEC      200000000          /* 0.2  sec */
#define OLED_PAGE2_NSEC     1000000000          /* 1.0  sec */

SSD1306 oled (myOLEDwidth ,myOLEDheight);       /* instance of oled class */

extern volatile sig_atomic_t sample_timer_rdy;  /* .01 sec timer ready */
extern volatile sig_atomic_t mpu_data_rdy;      /* .01 sec output handler ready */
extern volatile sig_atomic_t mpu_output_rdy;    /* .2  sec output handler ready */
static volatile sig_atomic_t oled_page2_rdy;    /* 1.  sec output handler ready */

/**
 * pthread variables and thread function for oled output
 */
volatile bool mpurun = false;                   /* stops oled thread */
volatile bool opcnt_rdy = false;                /* output to oled complete */

extern volatile bool autocalibrated;    /* auto-calibrate successfully completed */

static volatile uint8_t sigrt_oled_page2 = 0;   /* RT signal no. for page2 handler */

static volatile uint8_t page = 0;               /* current page displayed */

static itimer it2;        /* temp / statistic info update 1. sec */

/*
 *  Generate default constructor for C-struct mpu and then
 *  set member values directly in main() before initializing
 *  the mpu. See: https://stackoverflow.com/a/1544047/3422102
 */
mpu_t mpu = mpu_t();    /* C++ defautl zero copy-initialize struct */

/* accommodate both MilkV-Duo and Raspberry Pi using default I2C pins */
#ifdef MILKVFS
const char *i2cdev = "/dev/i2c-0";
#else
const char *i2cdev = "/dev/i2c-1";
#endif

/** init display OLED display
 *  @brief Initializes OLED display displaying brief horizontal lines before
 *  clearing.
 *  @return returns true if successfully initialized, false otherwise.
 */
bool init_display()
{
  static uint8_t screenbuf[FULLSCREEN];    /* why not make static and part of init ? */

  puts ("oled setup()");
  if (!oled.OLED_I2C_ON()) {
    fputs ("error: OLED_I2C_ON() - failed.\n", stderr);
    return false;
  }

  printf ("SSD1306 library Version Number : %u\n", oled.getLibVerNum());
  usleep (50000);

  oled.OLEDinit();
  // display_fill (0);
  oled.OLEDFillScreen (0xF0, 0);            /* optional stripes splash screen */
  usleep (250000);

  if (!oled.OLEDSetBufferPtr (myOLEDwidth, myOLEDheight, /* also part of init */
                              screenbuf, sizeof(screenbuf))) {
    return false;
  }

  return true;
}

/** write_display
 *  @brief Writes current buffer contents to oled display.
 */
void write_display (void)
{
  oled.OLEDupdate();
  // usleep (25000);

  // oled.OLEDclearBuffer();
  // oled.OLEDFillScreen (0, 0);
}

/** close_display
 *  @brief Power down and close oled display.
 */
void close_display (void)
{
  oled.OLEDPowerDown();
  oled.OLED_I2C_OFF();
}

/** draw_fixed
 *  @brief Writes fixed/static text for oled display page 1.
 *  @note variable 'page' 0 corresponds to display page 1
 *  (there is no display page 0)
 *  @todo adjust function names page nomenclature to be 0-based
 *  throughout.
 */
void draw_fixed (void)
{
  puts ("drawing fixed text/graphics - screen 1");

  oled.OLEDclearBuffer(); // Clear the buffer

  // oled.setTextWrap (true);
  oled.setFontNum (OLEDFont_Default);

  // oled.setTextColor (WHITE);
  /* NOTE must set bgcolor to have space overwrite CGRAM
   * See SSD1306_graphics::drawChar (...) and else if (bg != color)
   * background is set to WHITE by default with foreground BLACK.
   *
   * this can be used to advantage when drawing entire screen to prevent
   * overwrite of H and V lines without having to reorder the sequence
   * of function calls.
   */
  oled.setTextColor (WHITE, BLACK);
  oled.setTextSize (1);

  oled.setCursor (45, 0);
  oled.print ("MPU-9250");

  oled.setCursor (0, 9);
  oled.print ("    ACC   GYRO    MAG");

  oled.drawFastHLine (0, 17, 128, WHITE);
  oled.drawFastVLine (9, 9, 64, WHITE);

  oled.setCursor (0, 24);
  oled.print ("X");

  oled.setCursor (0, 36);
  oled.print ("Y");

  oled.setCursor (0, 48);
  oled.print ("Z");
}

/** oled_updt_mpu_values
 *  @brief Dynamic text updated on display page 1.
 */
void oled_updt_mpu_values (void)
{
  char buf[32] = "";
  sprintf (buf, "% 5.2f % 6.1f % 6.1f",
		mpu.accel.x, mpu.gyro.x, mpu.mag.x);
  oled.setCursor (13, 24);
  oled.print (buf);

  sprintf (buf, "% 5.2f % 6.1f % 6.1f",
		mpu.accel.y, mpu.gyro.y, mpu.mag.y);
  oled.setCursor (13, 36);
  oled.print (buf);

  sprintf (buf, "% 5.2f % 6.1f % 6.1f",
		mpu.accel.z, mpu.gyro.z, mpu.mag.z);
  oled.setCursor (13, 48);
  oled.print (buf);

  write_display();

}

void oled_updt_mpu_tmp (mpu_t *_mpu)
{
  char buf[32] = "";
  sprintf (buf, "% 5.2f % 6.1f % 6.1f",
		_mpu->accel.x, _mpu->gyro.x, _mpu->mag.x);
  oled.setCursor (13, 24);
  oled.print (buf);

  sprintf (buf, "% 5.2f % 6.1f % 6.1f",
		_mpu->accel.y, _mpu->gyro.y, _mpu->mag.y);
  oled.setCursor (13, 36);
  oled.print (buf);

  sprintf (buf, "% 5.2f % 6.1f % 6.1f",
		_mpu->accel.z, _mpu->gyro.z, _mpu->mag.z);
  oled.setCursor (13, 48);
  oled.print (buf);

  write_display();

}

/**
 * @brief Simple function to display message (s) on oled display
 * @param s string of less than 26 chars for display at center
 * of oled display using size 1 font.
 */
void oled_show_calibrating (const char *s)
{
  oled.setCursor (16, 36);
  oled.print (s);
  write_display();
}

/** draw_fixed2
 *  @brief Fixed/static text for display page 2
 */
void draw_fixed_2 (void)
{
  char buf[32] = "";

  puts ("drawing fixed text/graphics - screen 2");

  oled.OLEDclearBuffer();   /* clear display buffer */

  oled.setFontNum (OLEDFont_Default);

  // oled.setTextColor (WHITE);
  /* NOTE must set bgcolor to have space overwrite CGRAM
   * See SSD1306_graphics::drawChar (...) and else if (bg != color)
   * background is set to WHITE by default with foreground BLACK.
   *
   * this can be used to advantage when drawing entire screen to prevent
   * overwrite of H and V lines without having to reorder the sequence
   * of function calls.
   */
  oled.setTextColor (WHITE, BLACK);
  oled.setTextSize (1);

  oled.setCursor (45, 0);
  oled.print ("MPU-9250");

  sprintf (buf, "ID: %hhu", mpu.id);
  oled.setCursor (0, 9);
  oled.print (buf);

  oled.drawFastHLine (0, 17, 128, WHITE);

  oled.setCursor (1, 24);
  oled.print ("Temp: ");
}

/** text_page2
 *  @brief Dynamic text updated on display page 2
 */
void text_page2 (void)
{
  char buf[32] = "";

  sprintf (buf, "% 6.1fC % 6.1fF", mpu.tempc, mpu.tempc * 9 / 5 + 32);

  oled.setCursor (28, 24);
  oled.print (buf);

  write_display();
}

void text_page2_tmp (mpu_t *_mpu)
{
  char buf[32] = "";

  sprintf (buf, "% 6.1fC % 6.1fF", _mpu->tempc, _mpu->tempc * 9 / 5 + 32);

  oled.setCursor (28, 24);
  oled.print (buf);

  write_display();
}

/* signal handler for updating dynamic text page 2 based on itimer signal */
static void sighdlr_oled_page2 (int sig, siginfo_t *si, void *uc)
{
  if (sig == sigrt_oled_page2 && mpu_data_rdy) {
    oled_page2_rdy = true;
  }

  (void)si;
  (void)uc;
}

/* simple empty-stdin function, c must be initialized */
void empty_stdin (int c)
{
  while (c != '\n' && c != EOF) {
    c = getchar();
  }
}

/* create 3rd timer to provide page2 display with a 1-sec update
 * for information like temperature that doesn't need a more frequent
 * update. mpu-sample (0.01 sec) and mpu-output (0.2 sec) timers are
 * provided in mpu_start() and itimers_start() functions in mpu.c.
 */
static int start_page_2_timer (uint64_t ns_to_start, uint64_t ns_oled_page2)
{
  /* create timer instance of page 2 display (update once per-second)*/
  it2 = itimer_create_timer (ns_to_start, ns_oled_page2, sighdlr_oled_page2);

  /* validate realtime signal no. returned */
  if (it2.signo < SIGRTMIN || SIGRTMAX < it2.signo) {
    fputs ("error: itimer_create_timer - oled-page2\n", stderr);
    return 1;
  }

  sigrt_oled_page2 = it2.signo;   /* set global real-time signal no. */

  if (itimer_start_timer (&it2) == -1) {   /* start page2 output timer */
    fputs ("error: itimer_start_timer it2 - oled_page2\n", stderr);
    return 1;
  }

  return 0;
}


/* function to change message showing when autocalibration complete */
void show_calibrated (void)
{
  printf ("\033[2A%s Output:  (\033[1;32mAUTO-Calibrated\033[0m) "
          "press Enter to quit.\033[0K\n\n", mpu.typenm);
}


/**
 * @brief oled_xy_write will write string (str) to oled display at position
 * (x, y). 26 chars or less for size 1 font.
 * @param x horizontal position (in pixels) for start of string
 * @param y vertical position (in pixels) for start of string
 * @param str string to display at position (x, y) on oled display
 */
static void oled_xy_write (int x, int y, const char *str)
{
  oled.setCursor (x, y);
  oled.print (str);

  write_display();

}

pthread_mutex_t lock;

/**
 * @brief thread_oled is thread function for oled update
 * @param data NULL
 * @return NULL
 */
void *thread_oled (void *data)
{
  volatile bool shown = false;            /* whether calibrated msg shown */
  volatile uint8_t  lastpage = page,      /* last page no. displayed */
                    current = page;       /* current page no. to display */

  while (mpurun) {                        /* until main program loop exit */
    current = page;                       /* set current page from global  */

    if (!shown && autocalibrated) {       /* autocalibrated status */
      show_calibrated();
      shown = true;
    }

    if ((mpu_data_rdy && page == 0) || (oled_page2_rdy && page == 1)) {
      pthread_mutex_lock (&lock);       /* lock mutex */
      mpu_t tmp = mpu;
      pthread_mutex_unlock (&lock);     /* unlock mutex */

      switch (current) {                    /* display current page */
        case 0:                             /* fallthrough intentional */
        default:
          /* if data and output timers ready */
          if (mpu_output_rdy) {                   /* if output timer ready */
            if (current != lastpage) {            /* page changed */
              draw_fixed();                       /* fixed text for page 1 */
              lastpage = current;
            }
#ifdef DEBUGMAG
            if (autocalibrated) {
              int16_t mag[3];
              if (get_raw_mag (&mpu, mag)) {
                printf ("raw: % 6hd, % 6hd, % 6hd\x1b[0K\n",
                        mag[0], mag[1], mag[2]);
              }
              printf ("mag: % 6.1f, % 6.1f, % 6.1f\x1b[0K\n\x1b[2A",
                      tmp.mag.x, tmp.mag.y, tmp.mag.z);
            }
#endif
            oled_updt_mpu_tmp (&tmp);               /* update display values */
            mpu_output_rdy = false;               /* reset ready false */
            opcnt_rdy = true;                     /* output count updt ready */
          }
          break;
        case 1:
          /* page 1 (2nd page to display) */
          if (oled_page2_rdy) {                   /* page 2 timer ready */
            if (current != lastpage) {            /* page changed */
              draw_fixed_2();                     /* fixed text for page 2 */
              lastpage = current;
            }
            text_page2_tmp(&tmp);                         /* update display values */
            oled_page2_rdy = false;               /* reset ready false */
            opcnt_rdy = true;                     /* output count updt ready */
          }
          break;
      }
    }
    else {
      usleep (10000);                     /* max of 100Hz until output ready */
    }
  }

  return data;
}

#ifdef MAGHOFL
extern volatile uint_fast8_t drdymax;
extern volatile uint32_t hoflset;
#endif

int main (void) {

  pthread_t id;                       /* oled thread id */
  pthread_attr_t attr;                /* oled thread attributes */
  void *res;                          /* results pointer */

  if (!init_display()) {              /* initialize oled display */
    return 1;
  }
  draw_fixed();                       /* draw fixed text for page 1 */
  oled_xy_write (16, 24, "Do NOT move MPU");
  oled_show_calibrating ("Starting MPU");
  /* open i2cdev, get i2c-smbus file descriptor,
   * configure mpu and magnetometer (if present).
   * accel and gyro bias are autocalibrated,
   * magnetometer hard and soft iron bias are set
   * if saved values provided in mpu-caldata.[ch]
   */
  if (mpu_start (&mpu, i2cdev, MPU_ADDRESS_AD0_LOW, 0, 0, 0) == 1) {
    close_display();
    return 1;
  }
  mpurun = true;

  printf ("mpu started for mpu-id: %hhu\n", mpu.id);

  /* start interval timers mpu-sample (0.01 sec) and mpu-output (0.2 sec) */
  if (timers_start (ITIMER_START_DELAY,
                    MPU_SAMPLE_NSEC, 150000000) == 1) {
                    // MPU_SAMPLE_NSEC, MPU_OUTPUT_NSEC) == 1) {
    close_display();
    return 1;
  }
  puts ("mpu sample & output timers started");

  /* start display page 2 timer (1 sec interval timer) */
  if (start_page_2_timer(ITIMER_START_DELAY, OLED_PAGE2_NSEC) == 1) {
    close_display();
    return 1;
  }
  printf ("oled page_2 timer started\n\n"
#ifdef BUILD_64
          "updating oled display %d times (~%lu seconds)\n"
#else
          "updating oled display %d times (~%llu seconds)\n\n"
#endif
          "(ENTER alone to exit early, 'n' to cycle page2, page1, ...)\n\n",
          NUPDATES, ((uint64_t)NUPDATES * MPU_OUTPUT_NSEC) / (uint64_t)(1e9));

  /* show calibrating message on oled */
  oled_show_calibrating ("Calibrating...");

  /* show warning not to move until auto-calibration done, hide cursor */
  printf ("%s Output:  (\033[1;31mDO NOT MOVE UNTIL "
          "CALIBRATED\033[0m)\033[?25l\n\n", mpu.typenm);

  /* initialize thread mutex */
  handle_error_en (pthread_mutex_init (&lock, NULL), "pthread_mutex_init");

  /* initialize thread attributes (using defaults) and validate */
  handle_error_en (pthread_attr_init (&attr), "pthread_attr_init");

  /* create thread / validate */
  handle_error_en (pthread_create (&id, &attr, thread_oled, NULL),
                   "pthread_create");

  /* main program loop providing NUPDATES updates on oled display
   * (approx 100 seconds). oled update done in separate thread to
   * allow main program loop to sample mpu at 100Hz.
   */
  for (int i = 0; i < NUPDATES;) {
    /* validate read of accel, gyro and temp */
    bool ga_read_ok = false;
    bool mag_read_ok = false;

    pthread_mutex_lock (&lock);       /* lock mutex */

    ga_read_ok = get_acc_gyro_tempc (&mpu);
 #if defined (MPU9250) || defined (MPU9255)
    mag_read_ok = get_mag (&mpu);
 #endif

    pthread_mutex_unlock (&lock);     /* unlock mutex */

    // if (sample_timer_rdy && get_acc_gyro_tempc (&mpu)
    if (sample_timer_rdy && ga_read_ok
#if defined (MPU9250) || defined (MPU9255)
        /* validate read of magnetometer if provided */
        // && get_mag (&mpu)
        && mag_read_ok
#endif
    ) {
      mpu_data_rdy = true;                  /* set data ready true */
      sample_timer_rdy = false;             /* set sample timer ready false */
      if (!autocalibrated) {                /* if autocal not complete */
        autocalibrate (&mpu);               /* call autocalibrate func */
      }
#ifdef DEBUGMAG_1
      else {
        printf ("mag: % 6.1f, % 6.1f, % 6.1f\r",
                mpu.mag.x, mpu.mag.y, mpu.mag.z);
      }
#endif
    }

    if (mpu_data_rdy) {               /* data ready, 100Hz block (fusion) */
    }

    if (opcnt_rdy == true) {          /* oled output complete, ready to  */
      i += 1;                         /* increment output (loop) counter */
      opcnt_rdy = false;              /* set output count ready false */
    }

    /* pselect blocks until mpu_sample_rdy signal generated
     * check if user input available
     */
    if (pselect_timer (0, 2.e8) == 1) {
      int c = getchar();
      /* check for quit (or empty input or EOF) */
      if (c == 'q' || c == '\n' || c == EOF) {
        break;
      }
      else if (c == 'n' || c == 't') {      /* 'n'ext or 't'oggle display */
        page ^= 0x01;                       /* toggle page 0,1,0,1,0 ... */
        // printf ("page: %hhu\n", page);
      }
      empty_stdin (c);
    }
  }

  /* stop thread function and join */
  mpurun = false;

  handle_error_en (pthread_join (id, &res), "pthread_join");
  handle_error_en (pthread_mutex_destroy (&lock), "pthread_mutex_destroy");

  itimer_stop_timer (&it2);                 /* stop page 2 timer */

  /* stop/shutdown mpu, close file descriptors, stop/delete mpu timers */
  if (mpu_stop (&mpu) == -1) {
    fputs ("error on mpu_stop\n", stderr);
  }

  itimer_delete_timer (&it2);               /* delete page 2 timer */

  close_display();                          /* close/shutdown oled display */

#ifdef MAGHOFL
  printf ("data ready max fails (drdymax)  : %hhu\n"
          "magnetic overflow cnt (hoflset) : %u\n", drdymax, hoflset);
#endif

  printf ("\033[?25h\n");                   /* restore cursor visibility */
}
