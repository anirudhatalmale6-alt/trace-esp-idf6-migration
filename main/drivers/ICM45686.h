/******************************************************************************
 *
 * @file: ICM45686.h
 *
 * Common interface to the Toshiba ICM45686  3-axis accelerometer.
 *
 *****************************************************************************
 *
 * See: https://www.toshiba.com/taec/components/motion-sensors/icm45686
 *
 *****************************************************************************/
#ifndef ICM45686_H
#define ICM45686_H

#define AVAILABLE_FIFO (8 * 1024)                                // (8192) 8K FIFO available

#define RAW_FRAME_SIZE  (1 + (6 * 2) + 1 + 2)                    // (16)   8 entries @ 2 bytes per entry
#define RAW_FRAME_COUNT (200)                                    // (400)  entries in the FIFO

#define WATERMARK (RAW_FRAME_COUNT)                              // Generate a watermark based on the raw frame count

#if ( (WATERMARK * RAW_FRAME_SIZE) > (AVAILABLE_FIFO * 9 / 10) ) // If the watermark is over 90% of the FIFO
#error "WATERMARK IS TOO HIGH"
#endif

/*
 * Data as pulled in from the ICM45686
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
 * A single frame as read from the device registers.
 */
typedef struct // A single frame as read from the device registers
{
  int16_t x_dotdot;
  int16_t y_dotdot;
  int16_t z_dotdot;
  int16_t rho_dot;
  int16_t theta_dot;
  int16_t phi_dot;
} register_raw_frame_t; // Buffer for a single raw frame as read from the device registers

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
  float      mean[6];    // Mean of the samples in the FIFO buffer
  float      std_dev[6]; // Standard deviation of the samples in the FIFO buffer
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

#define SX_DOTDOT  0
#define SY_DOTDOT  1
#define SZ_DOTDOT  2
#define SRHO_DOT   3
#define STHETA_DOT 4
#define SPHI_DOT   5

/*
 *  Data as presented by FIFO.
 *  IMPORTANT:  The ESP32 does word alignment on uint16_t entries, so care must be taken to correctly interpret the FIFO data.
 */
typedef struct         // Data for a single sample as interpreted from the raw FIFO frame
{
  real_t x_dotdot;     // Sample frame from BMI270
  real_t y_dotdot;
  real_t z_dotdot;
  real_t rho_dot;
  real_t theta_dot;
  real_t phi_dot;      // Z axis rotation speed
} FIFO_real_single_t;  // Value read from sensor

typedef struct         // Unpacked FIFO frame and converted to floats
{
  int8_t  header;      // Header byte for the FIFO frame, not used in this program
  int16_t x_dotdot;    // Sample frame from BMI270
  int16_t y_dotdot;
  int16_t z_dotdot;
  int16_t rho_dot;
  int16_t theta_dot;
  int16_t phi_dot;     // Z axis rotation speed
  int8_t  temperature;
  int16_t timestamp;   // Timestamp for the sample, not used in this program
} FIFO_fixed_single_t; // Value read from sensor
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

/*
 * Pointers to access structures
 */
typedef struct
{
  int16_t inner; // Inner pointer  Points to WHICH FIFO buffer
  int16_t outer; // Outer pointer  Points to sample inside of the FIFO buffer
} FIFO_index_t;

/*
 *  The trace vector structure represents the full state of the system at a given point in time.
 */
typedef struct
{
  real_t x_dotdot;  // X-axis acceleration in g
  real_t y_dotdot;  // Y-axis acceleration in g
  real_t z_dotdot;  // Z-axis acceleration in g
  real_t x_dot;     // Velocity in the X axis
  real_t y_dot;     // Velocity in the Y axis
  real_t z_dot;     // Velocity in the Z axis
  real_t x;         // X position
  real_t y;         // Y position
  real_t z;         // Z position
  real_t rho_dot;   // X angular velocity
  real_t theta_dot; // Y angular velocity
  real_t phi_dot;   // Z angular velocity
  real_t rho;       // X angle
  real_t theta;     // Y angle
  real_t phi;       // Z angle
} trace_vector_t;   // Vector at the point

/*
 *  Functions
 */
void ICM45686_init(unsigned int bmi270_gpio);                                    // Initialize the ICM45686
void ICM45686_read_raw_accel(register_single_t *sample);                         // Read the accelermeter
bool ICM45686_pull_FIFO(void);                                                   // Read all of the samples in the FIFO
void ICM45686_test(void);                                                        // Test the ICM45686
void ICM45686_find_zero(bool automatic_confirm);                                 // Take a zero sample to use for future adjustments
void ICM45686_convert_to_g(FIFO_fixed_single_t *sample, trace_vector_t *actual); // Convert the raw sample to a vector
void ICM45686_oscilliscope(void);                                                // Poor man's oscilliscope
void ICM45686_FIFO_read(void);                                                   // FIFO handler
void ICM45686_SPI_dump(void);                                                    // Dump the ICM45686 registers using SPI.
bool ICM45686_get_next_raw_sample(register_single_t *sample);                    // Pull out the next sample
bool ICM45686_find_index_out(time_count_64_t shot);                              // Set the starting point in the list
void ICM45686_read_temperature(void);                                            // Read the temperature data from the ICM45686
void ICM45686_dump_FIFO(void);                                            // Pull the FIFO data and dump it to the console for testing
void ICM45686_real_unpack(FIFO_raw_t *raw, FIFO_real_single_t *sample);   // Convert raw FIFO frame to structured sample
void ICM45686_fixed_unpack(FIFO_raw_t *raw, FIFO_fixed_single_t *sample); // Convert raw FIFO frame to structured sample
void ICM45686_FIFO_statistics(void);                                      // Compute the statistics for the last FIFO pull

FIFO_raw_t *FIFO_return_first(void);                                      // Return pointer to the oldest sample in the FIFO buffer
FIFO_raw_t *FIFO_return_next(void);                                       // Return pointer to the next sample in the FIFO buffer
FIFO_raw_t *FIFO_return_previous(void);                                   // Return pointer to the previous sample in the FIFO buffer

#endif
