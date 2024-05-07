#include <itimer.h>

/**
 * @brief array holding Real-Time Signal No.s as timers are created, and
 * stored Signal No.s are reset zero when timers are deleted.
 */
static int8_t sigrt_pool[RTELEMS] = {0};


/**
 * @brief get avilable RT signal from available pool.
 * The avilable signals that may be returned range from
 * SIGRTMIN to SIGRTMAX. Signals retured are marked used
 * in the global sigrt_pool[] array and are not avaialble
 * again until the timer is deleted returning the signal
 * to the pool where it is marked as available by resetting
 * the value ain the sigrt_pool[] array to zero.
 *
 * @return on success a SIGRTnum is returned in the range
 * SIGRTMIN to SIGRTMAX, -1 is returned when all signals are
 * used.
 */
static int8_t itimer_get_rts_from_pool (void)
{
  for (int i = 0; i < RTELEMS; i++) {
    if (sigrt_pool[i] == 0) {
      return (sigrt_pool[i] = SIGRTMIN + i);
    }
  }

  return -1;
}


/**
 * @brief returns a RT signal previosly handed out by
 * itimer_get_rts_from_pool() to the sigrt_pool marking
 * that signal as once again available in the sigrt_pool[]
 * array.
 *
 * @param sig the SIGRTnum being release for reuse.
 *
 * @return zero is returned on success, -1 if sig is out of range.
 */
static int8_t itimer_return_rts_to_pool (int sig)
{
  if (sig < SIGRTMIN || SIGRTMAX < sig) {
    return -1;
  }

  return sigrt_pool[sig - SIGRTMIN] = 0;
}


/**
 * @brief create an interval timer generating interrupt at each interval.
 * @param ns_to_start nanosecond delay before timer start.
 * @param ns_interval nanosecond interval for timer.
 * @param fn sa_sigaction function called on interrupt.
 * @return returns struct itimer with .signo set to valid real-time signal
 * number on success, .signo set to -1 on failure.
 */
itimer itimer_create_timer (uint64_t ns_to_start, int64_t ns_interval,
                            void (*fn)(int, siginfo_t*, void*))
{
  itimer it = { .signo = 0 };
  struct sigevent     sev;
  struct sigaction    sa = { .sa_sigaction = NULL };

  /* get available RT signal from pool */
  if ((it.signo = itimer_get_rts_from_pool()) == -1) {
    errwarn ("itimer_get_rts_from_pool");
    return it;
  }

  /* initialize sigaction struct to handle timer signal */
  sa.sa_flags = SA_SIGINFO;
  sa.sa_sigaction = fn;
  sigemptyset (&sa.sa_mask);

  /* register sigaction for signum */
  if (sigaction (it.signo, &sa, NULL) == -1) {
    perror ("sigaction-sa");
    itimer_return_rts_to_pool (it.signo);
    it.signo = -1;
    return it;
  }

  /* create timer event and associate with signum */
  sev.sigev_notify = SIGEV_SIGNAL;
  sev.sigev_signo = it.signo;
  sev.sigev_value.sival_ptr = &it.timerid;
  if (timer_create (CLKID, &sev, &it.timerid) == -1) {
    perror ("timer_create");
    itimer_return_rts_to_pool (it.signo);
    it.signo = -1;
    return it;
  }

  /* initialize start time and intervals and associate with timerid
   *
   * NOTE: the function timer_settime() disarms the timer if both
   *       it_value fields are zero. Check ns_to_start and set to
   *       nominal value if zero.
   *
   *  see: man 2 gettimer (settimer) and man 2 timer_settime
   */
  if (ns_to_start == 0) {
    ns_to_start = 100;
  }
  it.its.it_value.tv_sec = ns_to_start / NSECS;
  it.its.it_value.tv_nsec = ns_to_start % NSECS;

  it.its.it_interval.tv_sec = ns_interval / NSECS;
  it.its.it_interval.tv_nsec = ns_interval % NSECS;

  return it;
}


/**
 * @brief start interval timer 'it'.
 * @param it itimer struct returned by itimer_create_timer.
 * @return returns 0 on success, -1 otherwise.
 */
int itimer_start_timer (itimer *it)
{
  /* validate itimer struct assigned valid real-time signal number */
  if (it->signo < 0) {
    fputs ("error: itimer_start_timer struct parameter invalid signo.\n",
            stderr);
    return -1;
  }

  /* start timer */
  if (timer_settime (it->timerid, 0, &it->its, NULL) == -1) {
    perror ("timer_settime - timerid");
    return -1;
  }

  it->enabled = 1;

  return 0;
}


/**
 * @brief stop interval timer 'it'.
 * @param it itimer struct returned by itimer_create_timer.
 * @return returns 0 on success, -1 otherwise.
 */
int itimer_stop_timer (itimer *it)
{
  struct itimerspec its = { .it_interval = 0 };

  /* validate itimer struct assigned valid real-time signal number */
  if (it->signo < 0) {
    fputs ("error: itimer_start_timer struct parameter invalid signo.\n",
            stderr);
    return -1;
  }

  /* stop timer by setting interval time zero */
  if (timer_settime (it->timerid, 0, &its, NULL) == -1) {
    perror ("timer_settime - timerid");
    return -1;
  }

  it->enabled = 0;

  return 0;
}


/**
 * @brief prevent timer signal from firing by masking signal no.
 * @param it itimer struct returned by itimer_create_timer.
 * @return returns 0 on success, -1 otherwise.
 */
int itimer_block_timer (itimer *it)
{
  sigset_t mask;

  sigemptyset (&mask);
  sigaddset (&mask, it->signo);
  if (sigprocmask (SIG_SETMASK, &mask, NULL) == -1) {
    perror ("sigprocmask SIG_SETMASK");
    return -1;
  }

  return 0;
}


/**
 * @brief unblocks timer signal previously blocked by itimer_block_timer.
 * @param it itimer struct returned by itimer_create_timer.
 * @return returns 0 on success, -1 otherwise.
 */
int itimer_unblock_timer (itimer *it)
{
  sigset_t mask;

  sigemptyset (&mask);
  sigaddset (&mask, it->signo);
  if (sigprocmask (SIG_UNBLOCK, &mask, NULL) == -1) {
    perror ("sigprocmask SIG_UNBLOCK");
    return -1;
  }

  return 0;
}


/**
 * @brief disable and delete itimer interval timer and restore signo to pool.
 * @param it itimer struct returned by itimer_create_timer.
 * @return returns 0 on success, -1 otherwise.
 */
int itimer_delete_timer (itimer *it)
{
  it->enabled = 0;

  it->its.it_value.tv_sec = 0;
  it->its.it_value.tv_nsec = 0;

  it->its.it_interval.tv_sec = 0;
  it->its.it_interval.tv_nsec = 0;

  if (timer_delete (it->timerid) == -1) {
    perror ("timer_delete");
    return -1;
  }
  it->timerid = 0;

  itimer_return_rts_to_pool (it->signo);
  it->signo = 0;

  return 0;
}
