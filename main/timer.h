/*----------------------------------------------------------------
 *
 * timer.h
 *
 * Header file for timer functions
 *
 *---------------------------------------------------------------*/
#ifndef _TIMER_H_
#define _TIMER_H_

/*
 * Variables
 */

/*
 * function Prototypes
 */
// TODO(IDF6): was "void *(callback)()". The timers[] struct in timer.c declares the
// field as void (*)(void) and calls it as such, so the prototype was simply wrong.
// C23 (the default in IDF 6.0) no longer treats () as "unspecified arguments".
bool ft_timer_new(time_count_64_t *timer_new, time_count_64_t duration, void (*callback)(void), char *name); // Start a new timer in ms
int  ft_timer_delete(time_count_64_t *timer);                                                            // Stop a running timer
void trace_synchronous(void *pvParameters);                                                              // Synchronou scheduler
void trace_timers(void *pvParameters);                                                                   // Update the free running timers
void show_time(void);                                                                                    // Show the current time
int64_t         run_time_us(void);       // Show how long we have been running for in us
int64_t         run_time_ms(void);       // Show how long we have been running for in us
int64_t         run_time_s(void);        // Show how long we have been running for in us
void            reset_run_time_us(void); // Reset the clock back to zero

/*
 *  Definitions
 */

#endif
