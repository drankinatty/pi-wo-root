/**
 *  Interval timer library for Raspberry Pi
 *
 *  Copyright (c) David C. Rankin, 2022-2024
 *  License: GPLv2
 */
#ifndef itimer_h_
#define itimer_h_  1

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>

#define CLKID     CLOCK_MONOTONIC
#define NRTSIGS   (SIGRTMAX - SIGRTMIN + 1)
#define RTELEMS   31
#define NSECS     1000000000

#define errexit(errmsg) { perror (errmsg); exit (EXIT_FAILURE); }
#define errwarn(errmsg) { fputs (errmsg "\n", stderr); }

typedef struct itimer {
  int8_t  signo,
          enabled;
  timer_t timerid;
  struct itimerspec its;
} itimer;

/**
 * @brief create an interval timer generating interrupt at each interval.
 * @param ns_to_start nanosecond delay before timer start.
 * @param ns_interval nanosecond interval for timer.
 * @param fn sa_sigaction function called on interrupt.
 * @return returns struct itimer with .signo set to valid real-time signal
 * number on success, .signo set to -1 on failure.
 */
itimer itimer_create_timer (uint64_t ns_to_start, int64_t ns_interval,
                            void (*fn)(int, siginfo_t*, void*));

/**
 * @brief start interval timer 'it'.
 * @param it itimer struct returned by itimer_create_timer.
 * @return returns 0 on success, -1 otherwise.
 */
int itimer_start_timer (itimer *it);

/**
 * @brief stop interval timer 'it'.
 * @param it itimer struct returned by itimer_create_timer.
 * @return returns 0 on success, -1 otherwise.
 */
int itimer_stop_timer (itimer *it);

/**
 * @brief prevent timer signal from firing by masking signal no.
 * @param it itimer struct returned by itimer_create_timer.
 * @return returns 0 on success, -1 otherwise.
 */
int itimer_block_timer (itimer *it);

/**
 * @brief unblocks timer signal previously blocked by itimer_block_timer.
 * @param it itimer struct returned by itimer_create_timer.
 * @return returns 0 on success, -1 otherwise.
 */
int itimer_unblock_timer (itimer *it);

/**
 * @brief disable and delete itimer interval timer and restore signo to pool.
 * @param it itimer struct returned by itimer_create_timer.
 * @return returns 0 on success, -1 otherwise.
 */
int itimer_delete_timer (itimer *it);

#ifdef __cplusplus
}
#endif

#endif
