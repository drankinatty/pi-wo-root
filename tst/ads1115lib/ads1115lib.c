#include <stdio.h>

#include <ads1115.h>


/* ADS1115 ADC config */
#define RETRIES               4

#ifdef MILKVFS
#define SPIBUS    0
#else
#define SPIBUS    1
#endif


/* i2c devfs devide nodes for each i2c bus */
const char *i2cdev[2] = { "/dev/i2c-0", "/dev/i2c-1" };


int main (void) {

  ads_t ads = { .fd = 0 };
  int samples = 100;

  /* set all channels to single-shot input */
  ads_channel_cfg_all (&ads, CFG_SNGL_AIN0, CFG_PGA_4_096,
                        CFG_SINGLE_SHOT, CFG_DR_128);

  puts ("\nADS1115 write config for each channel (AINx) "
        "single-shot 4.096V gain 128 SPS\n");

  /* output channel_cfg and config command values for each */
  for (int chan = 0; chan < CHANNELS; chan++) {
    printf ("  ads channel_cfg[%d]: 0x%4hx\n"
            "             command: 0x%4hx\n",
            chan, ads.channel_cfg[chan],
            ads.channel_cfg[chan] | CFG_CNV_START);

  }
  putchar ('\n');

  /* open i2c device file save file descriptor and i2c address to struct */
  if (ads_i2c_init (&ads, i2cdev[SPIBUS], ADDR_GND) == -1) {
    return 1;
  }

  /* output samples and approximate run-time, set cursor invisible */
  printf ("%d analog input samples (~%d sec), results and voltages:\033[?25l\n\n",
          samples, samples / 5);

  for (int i = 0; i < samples; i++) {
    /* loop reading each analog input and output result */
    for (uint8_t chan = 0; chan < CHANNELS; chan++) {
      if (ads_read_channel (&ads, chan) == 0) {
        printf ("  channel[%hhu] : 0x%04hx  (%.2f V)\n", chan, ads.sample[chan],
                (ads.sample[chan] >> 15) & 1 ? 0. :
                (float)ads.sample[chan] * 4.096f / 32768);
      }
      else {
        fprintf (stderr, "error: reading channel %hhu\n", chan);
      }
    }
    usleep (200000);          /* 5Hz sample rate */

    if (i + 1 < samples) {    /* up 4 lines except last iteration */
      fputs ("\033[4A", stdout);
    }
  }

  puts ("\033[?25h");         /* restore cursor visibility */
}

/*

$ ./adsrewrite

ADS1115 write config for each channel (AINx) single-shot 4.096V gain 128 SPS

  ads channel_cfg[0]: 0x4383
             command: 0xc383
  ads channel_cfg[1]: 0x5383
             command: 0xd383
  ads channel_cfg[2]: 0x6383
             command: 0xe383
  ads channel_cfg[3]: 0x7383
             command: 0xf383

Analog input channel sample results and voltages:

  channel[0] : 0x6737  (3.30 V)
  channel[1] : 0x3cd2  (1.95 V)
  channel[2] : 0x3132  (1.57 V)
  channel[3] : 0x0002  (0.00 V)


*/
