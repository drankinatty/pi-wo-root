#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#include <tinygpio.h>     /* include header for tinygpio library */

#define GPIOMAX   53

/**
 * @brief simple blink function to blink LED on pin
 * @param pin GPIO pin to use
 * @param on_usec number of microseconds the LED is ON
 * @param off_usec number of microseconds the LED if OFF
 * @param cnt number of times to blink the LED
 */
void blink (unsigned pin,
            unsigned on_usec, unsigned off_usec, unsigned cnt)
{
  int i = 0;

  while (cnt--) {
    gpioWrite (pin, 1);
    usleep (on_usec);

    printf ("blink: %d\r", ++i);
    fflush (stdout);

    gpioWrite (pin, 0);
    usleep (off_usec);
  }

  putchar ('\n');
}

int main (int argc, char **argv)
{
  unsigned  pin = 4,            /* gpio pin               - argv[1] */
            on_usec  = 50000,   /* blink on microseconds  - argv[2] */
            off_usec = 200000,  /* blink off microseconds - argv[3] */
            repeat   = 20;      /* number of blinks       - argv[4] */
  int rtn;                      /* function return */

  if (argc > 1) {   /* if pin argument provided, set pin */
    unsigned tmp;
    if (sscanf (argv[1], "%u", &tmp) != 1) {
      fprintf (stderr, "error: invalid unsigned pin value for argv[1] (%s)\n",
              argv[1]);
      return 1;
    }
    pin = tmp;
  }

  if (argc > 2) {   /* if microseconds ON time provided, set on_usec */
    unsigned tmp;
    if (sscanf (argv[2], "%u", &tmp) != 1) {
      fprintf (stderr, "error: invalid microseconds ON time for argv[2] (%s)\n",
              argv[2]);
      return 1;
    }
    on_usec = tmp;
  }

  if (argc > 3) {   /* if microseconds OFF time provided, set off_usec */
    unsigned tmp;
    if (sscanf (argv[3], "%u", &tmp) != 1) {
      fprintf (stderr, "error: invalid microseconds OFF time for argv[3] (%s)\n",
              argv[3]);
      return 1;
    }
    off_usec = tmp;
  }

  if (argc > 4) {   /* if no. of blinks to repeat provided, set repeat */
    unsigned tmp;
    if (sscanf (argv[4], "%u", &tmp) != 1) {
      fprintf (stderr, "error: invalid unsigned repeat value for argv[4] (%s)\n",
              argv[4]);
      return 1;
    }
    repeat = tmp;
  }

  if (GPIOMAX < pin) {  /* validate pin no. is valid */
    fprintf (stderr, "error: invalid gpio pin '%u' (0-53 allowed)\n", pin);
    return 1;
  }

  rtn = gpioInitialise();   /* initialize tinygpio lib */

  if (rtn < 0) {  /* validate initiaization succeeded */
    fprintf (stderr, "error: gpioInitialise() returned %d\n", rtn);
    return 1;
  }

  printf("Pi model = %u, Pi revision = %u\n", *pimodel, *pirev);

  gpioSetMode (pin, PI_OUTPUT);   /* set output mode on pin */

  if ((rtn = gpioGetMode (pin)) != PI_OUTPUT) {   /* validate mode set */
    fprintf (stderr, "error get_mode != PI_OUTPUT : %d\n", rtn);
    return 1;
  }

  blink (pin, on_usec, off_usec, repeat);   /* blink LED attached to pin */

}

