
/******************************************************************************
 *
 * @file: IMU.h
 *
 * API interface for the IMU functions
 *
 *****************************************************************************
 *
 * See: https://www.analog.com/en/products/BMI270.html
 *
 *****************************************************************************/
#ifndef _IMU_H_
#define _IMU_H_

/*
 *  Memory Calculations (needed for memory allocation)
 *
 *  A frame is a single sample read from the ACCEL/GYRO
 *  A sample buffer is the space to store a single FIFO pull
 *  A trace is the path drawn by the gun on the target
 */
#define APPROACH       5                                  // Go back in time 5 seconds
#define FOLLOW_THROUGH 2                                  // Go forwards 2 seconds
#define OVERSAMPLE     (SAMPLE_RATE / TRACE_RATE)         // Only send 1/8 samples

#define SAMPLE_RATE   (800)                               // Output Data Rate samples per second
#define SAMPLE_PERIOD (APPROACH + FOLLOW_THROUGH)         // Accumulate sampls for 8 seconds
#define SAMPLE_BUFFER_COUNT                                                                                                                \
  (((SAMPLE_RATE * SAMPLE_PERIOD) / RAW_FRAME_COUNT) + 1) // Number of frames needed to store the approach and follow through

#define TRACE_RATE        (100)                           // Trace points per
#define TRACE_FRAME_SIZE  (2 * 4)                         // (24) 6 entries at 4 bytes (32 bits) each
#define TRACE_MEMORY_SIZE (TRACE_RATE * SAMPLE_PERIOD * TRACE_FRAME_SIZE) // (96000)


/*
 *  The trace point structure represents a computed point in the trace.
 */
typedef struct
{
  real_t x;      // X position in mm
  real_t y;      // Y position in mm
} trace_point_t; // computed point

/*
 *  Functions
 */
#if(0)
FIFO_raw_t *trace_first(void);                               // Reset the trace pointers
FIFO_raw_t *fifo_next(FIFO_index_t *index);                  // Go to the next pointer
FIFO_raw_t *trace_previous(FIFO_index_t *index);             // Go to the prior pointer
FIFO_raw_t *trace_FIFO_next(FIFO_index_t *index);            // Point to the next input buffer
#endif 
void        IMU_test(void);                                  // Test the IMU
void        trace_build(time_count_64_t timestamp);          // Build up the trace
void        trace_build_and_send(time_count_64_t timestamp); // Build and send a trace
void        trace_send(int oversample);                      // Build and send a trace
void        IMU_real_time(void);                             // Output the trace in real time

#endif