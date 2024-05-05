#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#include <itimer.h>

static volatile int c = 0;

/* signal handler for coverning mpu read based on itimer signal */
static void handler (int sig, siginfo_t *si, void *uc)
{
  if (SIGRTMIN <= sig && sig <= SIGRTMAX) {
    ++c;
  }

  (void)si;
  (void)uc;
}


int main (void) {

  int last = 0;
  uint64_t  ns_to_start = 250000000,
            ns_interval = 250000000;

  /* initialize timer */
  itimer it0 = itimer_create_timer (ns_to_start, ns_interval, handler);

  if (it0.signo < 0) {  /* validate timer start */
    puts ("itimer_create_timer");
    return 1;
  }

  if (itimer_start_timer (&it0) == 0) {   /* start reading mpu */
    puts ("itimer_start_timer");
    return 1;
  }

  while (1) {
    if (c != last) {
      printf ("%d\n", c);
      last = c;
    }
    if (c >= 50) {
      break;
    }
    usleep (250000);
  }

  itimer_delete_timer (&it0);
}
