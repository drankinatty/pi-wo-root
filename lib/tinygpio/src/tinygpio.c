/**
 * Adapted from Tiny GPIO Access
 * http://abyz.me.uk/rpi/pigpio/examples.html
 * Public Domain
 * as modified:
 *
 *  GPIO access for Raspberry Pi through /dev/gpiomem
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "tinygpio.h"

unsigned piModel;
unsigned piRev;

/* pointers to make available via extern in header */
unsigned *pimodel = &piModel;
unsigned *pirev = &piRev;

static volatile uint32_t  *gpioReg = MAP_FAILED;


int gpioInitialise (void)
{
  int fd;

  piRev = gpioHardwareRevision(); /* sets piModel and piRev */

  fd = open ("/dev/gpiomem", O_RDWR | O_SYNC);

  if (fd < 0) {
    fprintf (stderr, "failed to open /dev/gpiomem\n");
    return -1;
  }

  gpioReg = (uint32_t *)mmap (NULL, 0xB4, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);

  close (fd);

  if (gpioReg == MAP_FAILED) {
    fprintf (stderr, "Bad, mmap failed\n");
    return -1;
  }

  return 0;
}

void gpioSetMode (unsigned gpio, unsigned mode)
{
  int reg, shift;

  reg   =  gpio / 10;
  shift = (gpio % 10) * 3;

  gpioReg[reg] = (gpioReg[reg] & ~(7<<shift)) | (mode<<shift);
}

int gpioGetMode (unsigned gpio)
{
  int reg, shift;

  reg   =  gpio / 10;
  shift = (gpio % 10) * 3;

  return (*(gpioReg + reg) >> shift) & 7;
}

void gpioSetPullUpDown (unsigned gpio, unsigned pud)
{
  *(gpioReg + GPPUD) = pud;

  usleep (20);

  *(gpioReg + GPPUDCLK0 + PI_BANK) = PI_BIT;

  usleep (20);

  *(gpioReg + GPPUD) = 0;

  *(gpioReg + GPPUDCLK0 + PI_BANK) = 0;
}

int gpioRead(unsigned gpio)
{
  if ((*(gpioReg + GPLEV0 + PI_BANK) & PI_BIT) != 0) {
    return 1;
  }

  return 0;
}

void gpioWrite (unsigned gpio, unsigned level)
{
  if (level == 0) {
    *(gpioReg + GPCLR0 + PI_BANK) = PI_BIT;
  }
  else {
    *(gpioReg + GPSET0 + PI_BANK) = PI_BIT;
  }
}

void gpioTrigger (unsigned gpio, unsigned pulseLen, unsigned level)
{
  if (level == 0) {
    *(gpioReg + GPCLR0 + PI_BANK) = PI_BIT;
  }
  else {
    *(gpioReg + GPSET0 + PI_BANK) = PI_BIT;
  }

  usleep (pulseLen);

  if (level != 0) {
    *(gpioReg + GPCLR0 + PI_BANK) = PI_BIT;
  }
  else {
    *(gpioReg + GPSET0 + PI_BANK) = PI_BIT;
  }
}

/* Bit (1<<x) will be set if gpio x is high. */

uint32_t gpioReadBank1 (void)
{
  return (*(gpioReg + GPLEV0));
}

uint32_t gpioReadBank2 (void)
{
  return (*(gpioReg + GPLEV1));
}

/* To clear gpio x bit or in (1<<x). */

void gpioClearBank1 (uint32_t bits)
{
  *(gpioReg + GPCLR0) = bits;
}

void gpioClearBank2 (uint32_t bits)
{
  *(gpioReg + GPCLR1) = bits;
}

/* To set gpio x bit or in (1<<x). */

void gpioSetBank1 (uint32_t bits)
{
  *(gpioReg + GPSET0) = bits;
}

void gpioSetBank2 (uint32_t bits)
{
  *(gpioReg + GPSET1) = bits;
}

unsigned gpioHardwareRevision (void)
{
  static unsigned rev = 0;

  FILE * filp;
  char buf[512];
  char term;
  int chars=4; /* number of chars in revision string */

  if (rev) return rev;

  piModel = 0;

  filp = fopen ("/proc/cpuinfo", "r");

  if (filp != NULL)
  {
    while (fgets (buf, sizeof(buf), filp) != NULL)
    {
      if (piModel == 0)
      {
        if (!strncasecmp ("model name", buf, 10))
        {
          if (strstr (buf, "ARMv6") != NULL)
          {
            piModel = 1;
            chars = 4;
          }
          else if (strstr (buf, "ARMv7") != NULL)
          {
            piModel = 2;
            chars = 6;
          }
          else if (strstr (buf, "ARMv8") != NULL)
          {
            piModel = 2;
            chars = 6;
          }
        }
      }

      if (!strncasecmp ("revision", buf, 8))
      {
        if (sscanf (buf + strlen (buf) - (chars + 1), "%x%c", &rev, &term) == 2)
        {
          if (term != '\n') rev = 0;
        }
      }
    }

    fclose(filp);
  }

  return rev;
}

