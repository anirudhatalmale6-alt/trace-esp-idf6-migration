
/******************************************************************************
 *
 * @file: trace_types.h
 *
 * Common interface to trace types.
 *
 *****************************************************************************
 *
 * See: https://www.bosch-sensortec.com/products/motion-sensors/bmi270/
 *
 *****************************************************************************/
#ifndef TRACE_TYPES_H
#define TRACE_TYPES_H

/*
 * Data as pulled in from the BMK270.
 * Note, the register order is LSB then MSB
 *
 * IMPORTANT
 *
 * The SPI driver will deliver the bytes in the order they come out of the ICM45686.
 * On the other hand, the ESP32 will word align any uint16_t entries in the structure
 * meaning that the alignment between what the program thinks is aligned and how they
 * arrive from the ICM45686 will be off by one byte.
 *
 * For example,    typedef struct {uint8_t dummy; uint16_t sample}
 * will have an empty byte inserted between dummy and sample so that sample is word
 * aligned.
 *
 * Any attempt to use sample will result in a mangled value.
 *
 * To get around this issue, an empty byte is added at the beginning of the
 * structure so that dummy is byte aligned, and uint16_t x becomes word aligned
 * and can be accessed in the correct little-big endian that is provided by the
 * ICM45686.
 *
 */

/*
 * A single sample frame as read from the FIFO
 */
typedef struct        // A single raw frame as read from the FIFO
{
  uint8_t buffer[16]; // Just an array of bytes
} FIFO_raw_t;         // Buffer for a single raw frame as read from the FIFO

/*
 * A large buffer to hold an entire WATERMARK of samples
 */
typedef struct           // Large buffer to hold all the FIFO data from own read cycle
{
  int32_t    mean[6];    // Mean of the samples in the FIFO buffer
  int32_t    std_dev[6]; // Standard deviation of the samples in the FIFO buffer
  FIFO_raw_t f[RAW_FRAME_COUNT];
} FIFO_packet_t;         // Value read from sensor via FIFO

#define FIFO_HEADER 0
#define X_DOTDOT    1
#define Y_DOTDOT    3
#define Z_DOTDOT    5
#define RHO_DOT     7
#define THETA_DOT   9
#define PHI_DOT     11
#define TEMPERATURE 13
#define TIMESTAMP   14

/*
 *  Data as presented by FIFO.
 *  IMPORTANT:  The ESP32 does word alignment on uint16_t entries, so care must be taken to correctly interpret the FIFO data.
 */


/*
 *  Data as presented by registers.
 */
typedef struct       // Data for a single sample as interpreted from the raw FIFO frame
{
  int16_t x_dotdot;  // Sample frame from ICM-45686
  int16_t y_dotdot;
  int16_t z_dotdot;
  int16_t rho_dot;
  int16_t theta_dot;
  int16_t phi_dot;   // Z axis rotation speed
  int16_t temperature;
} register_single_t; // Value read from sensor



#endif
