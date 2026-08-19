/*----------------------------------------------------------------
 *
 * trace.h
 *
 * IMU for guns to work with FreeETarget
 *
 *--------------------------------------------------------------*/

#ifndef _TRACE_H
#define _TRACE_H

#include "product.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "serial_io.h"

#ifdef TRACE_C
#define EXTERN
#else
#define EXTERN extern
#endif

#define _DONE_     "\r\nDone\r\n"
#define _GREETING_ "CONNECTED"                        // Message to send on connection
#define _BYE_      "BYE"                              // Message to send on disconnection
#define _HELLO_    "HELLO WORLD"                      // Message to send on reconnection

#define NETWORK_TIME_PERIOD    (15 * 60 * ONE_SECOND) // Expect a time synch every 15 minutes
#define KEEP_ALIVE_TIME_PERIOD (10 * 60 * ONE_SECOND) // Expect a time synch every 15 minutes

#define INIT_DONE 0xabcd                              // NON-VOL Initialization complete signature
#ifndef true
#define true  (1 == 1)
#define false (0 == 1)
#endif

#define IN_STARTUP       0x0001                       // The software is in initialization
#define IN_NO_CAL        (IN_STARTUP << 1)            // The unit has not been calibrated
#define IN_FIFO_FRESH    (IN_NO_CAL << 1)             // New FIFO data
#define IN_FIFO_FULL     (IN_FIFO_FRESH << 1)       // The FIFO is full
#define IN_REDUCTION     (IN_FIFO_FULL << 1)          // The data is being reduced
#define IN_OPERATION     (IN_REDUCTION << 1)          // FIFO has data, unit has been zeroed
#define IN_FATAL_ERROR   (IN_OPERATION << 1)          // A fatal error has occured and cannot be fixed
#define IN_TEST          (IN_FATAL_ERROR << 1)        // Running a test
#define AP_ACTIVE        (IN_TEST << 1)               // Access Point is active
#define STATION_ACTIVE   (AP_ACTIVE << 1)             // Station is active
#define CLIENT_CONNECTED (STATION_ACTIVE << 1)        // We are connected to the target
#define SERVER_CONNECTED (CLIENT_CONNECTED << 1)      // We are connected to the target
#define TIME_VALID       (SERVER_CONNECTED << 1)      // The timebase is valid

#define IF(x)             if ( (run_state & (x)) != 0 )
#define IF_NOT(x)         if ( (run_state & (x)) == 0 )
#define set_status_LED(x)                             // Placeholder for setting the status LED

#define SEND(who, message) {message} serial_to_all(_xs, who);

/*
 * Options
 */
#define LONG_TEXT   (512 + 256) // Long text strings are 512 long
#define MEDIUM_TEXT 256         // Medimum length strings are 256 long
#define SHORT_TEXT  128         // Short text strings are 128 long
#define TINY_TEXT   64          // Tiny text strings are 64 long

/*
 * Oscillator Features
 */
#define TICK_10ms  (1)                           // Minimum timeout 10ms
#define TICK_50ms  (5 * TICK_10ms)               // Minimum timeout 10ms
#define ONE_SECOND (100 * TICK_10ms)             // 10 ms delay per LSB

#define FULL_SCALE     0xffffffff                // Full scale timer
#define MS_TO_TICKS(x) (ONE_SECOND * (x) / 1000) // Convert from time in ms to time ticks

#define PI      3.14159269
#define PI_ON_4 (PI / 4.0d)
#define PI_ON_2 (PI / 2.0d)
#define TWO_PI  (2.0d * PI)

/*
 *  Global Variables
 */
EXTERN char         _xs[1024 + 512];                                   // General purpose string buffer
EXTERN unsigned int is_trace;                                          // Tracing level(s)

EXTERN unsigned int          board_revision;                           // Board revision number
EXTERN volatile unsigned int run_state;                                // Current running state of the software

EXTERN int sample_in;                                                  // Index to entry from sensor (<0 - wraps around)
EXTERN int sample_out;                                                 // Index to output to application  (<0 - wraps around)

#ifdef TRACE_C
EXTERN char           *no_yes[]       = {"No", "Yes"};                 // Yes or No
EXTERN time_count_64_t session_time[] = {1000 * 60, 15 * 60, 75 * 60}; // Time in each session EMPTY, SIGHT, SCORE // Array of shot records
#else
EXTERN char           *no_yes[]; // Yes or No strings
EXTERN time_count_64_t session_time[];
#endif

/*
 * trace functions
 */
void trace_init(void);            // Get the target software ready
void trace_loop(void *arg);       // Target polling loop
void trace_push_button(void);     // Monitor the push button
void trace_reduce(int timestamp); // Reduce the data and send it
void trace_send(int oversample);  // Build and send a trace
void trace_health_monitor(void);  // Check the health of the sensor

#endif
