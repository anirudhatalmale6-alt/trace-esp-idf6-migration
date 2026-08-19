/******************************************************************************
 *
 * file: ICM45686.c
 *
 * Toshiba ICM45686 3-axis accelerometer driver
 *
 *****************************************************************************
 *
 * This file contains the driver for the ICM45686 3-axis accelerometer.  The
 * driver is written to be as generic as possible and should work with any
 * implementation of the ICM45686.
 *
 * See: https://www.toshiba.com/taec/components/motion-sensors/icm45686
 *
 *
 *****************************************************************************/
#include "stdio.h"
#include "string.h"
#include "gpio.h"
#include "math.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "common.h"
#include "ICM45686-define.h"

/*
 * Definitions
 */

/*
 *  Typedefs
 */

/*
 *  Variables
 */
FIFO_index_t index_in   = {0, 0};              // Pointer to the input side
FIFO_index_t index_out  = {0, 0};              // Pointer to the output side
FIFO_index_t index_last = {0, 0};              // Pointer to the last FIFO read

time_count_64_t last_FIFO_read;                // Remember when we took the last sample

FIFO_packet_t FIFO_queue[SAMPLE_BUFFER_COUNT]; // Space for 10 seconds of data

static spi_device_handle_t ICM45686_handle;    // Handle for the SPI device

static spi_device_interface_config_t ICM45686_spi_config = {
    // Configuration for the SPI device
    .command_bits     = 0,                   // No command phase
    .address_bits     = 8,                   //
    .dummy_bits       = 0,                   // No dummy bits
    .mode             = 0,                   // SPI mode 0
    .clock_source     = SPI_CLK_SRC_DEFAULT, // Use default clock source
    .duty_cycle_pos   = 128,                 // 50% duty cycle
    .cs_ena_pretrans  = 0,                   // No pre-transaction CS activation
    .cs_ena_posttrans = 0,                   // No post-transaction CS activation
    .clock_speed_hz   = 20 * 1000 * 1000,    // 2 MHz clock speed (do not set higher than 2 MHz)
    .input_delay_ns   = 0,                   // No input delay
    .spics_io_num     = ICM45686_CS,         // CS pin
    .flags            = SPI_DEVICE_NO_DUMMY, // No special flags
    .queue_size       = 1,
    .pre_cb           = NULL,                // Callback to be called before a transmission is started.
    .post_cb          = NULL                 // Callback to be called after a transmission has completed.
};

/*
 *  Local Functions
 */
static void ICM45686_clear_int1_status0(); // Reset the interrupt pending bits.

/*----------------------------------------------------------------
 *
 * @function: ICM45686_init()
 *
 * @brief:    Initalize the ICM45686
 *
 * @return: None
 *
 *----------------------------------------------------------------
 *
 * Setup the accelerometer from the table
 *
 *--------------------------------------------------------------*/
void ICM45686_init(unsigned int ICM45686_gpio)
{
  spi_transaction_t transaction;
  int               i;
  uint8_t           buffer[3];

  DLT(DLT_INFO, SEND(CONSOLE, sprintf(_xs, "ICM45686_init()");))
  PAUSE("Ready")

  ICM45686_spi_config.spics_io_num = ICM45686_gpio;

  /*
   * Add the accelerometer to the bus
   */
  if ( spi_bus_add_device(SPI2_HOST, &ICM45686_spi_config, &ICM45686_handle) != ESP_OK ) // Add the SPI device to the bus
  {
    DLT(DLT_CRITICAL, SEND(CONSOLE, sprintf(_xs, "Failed to add ICM45686 device to SPI bus");))
  }

                                                                                         /*
                                                                                          * Read the device ID
                                                                                          */

  memset(&transaction, 0, sizeof(transaction));       // Clear the transaction structure
  transaction.addr      = 0x80 | WHO_AM_I;            // Register address to read from
  transaction.length    = 1 * 8;                      // Transmit length in bits
  transaction.tx_buffer = NULL;                       // Transmit buffer not used
  transaction.rxlength  = 1 * 8;                      // Receive length in bits
  transaction.flags     = SPI_TRANS_USE_RXDATA;       // Indicate that this is a read operation

  spi_device_transmit(ICM45686_handle, &transaction); // Dummy read to put into SPI mode
  PAUSE("CHIP_ID")
  spi_device_transmit(ICM45686_handle, &transaction); // Transmit the transaction

  if ( transaction.rx_data[0] != WHO_I_SHOULD_BE )    // Check the device ID
  {
    DLT(DLT_FATAL, SEND(CONSOLE, sprintf(_xs, "Failed to read ICM45686 device ID: 0x%02X", transaction.rx_data[0]);))
    return;
  }
  else
  {
    DLT(DLT_INFO, SEND(CONSOLE, sprintf(_xs, "ICM45686 device ID: 0x%02X", transaction.rx_data[0]);))
  }
  vTaskDelay(1);

  /*
   * Program the registers from the configuration table
   */

  i = 0;
  while ( ICM45686_config[i].address != 0x00 )          // Loop through the configuration table until the end is reached
  {
    memset(&transaction, 0, sizeof(transaction));       // Clear the transaction structure
    transaction.addr      = ICM45686_config[i].address; // Register address to write to
    transaction.tx_buffer = &ICM45686_config[i].value;  // Send the value to be written to the register
    transaction.length    = 1 * 8;                      // Transmit length in bits
    transaction.rxlength  = 0 * 8;                      // Receive length in bits
    transaction.flags     = SPI_TRANS_USE_TXDATA;       // Indicate that this is a read operation
    PAUSE("Sending register value");
    DLT(DLT_DEBUG, SEND(CONSOLE, sprintf(_xs, "register 0x%02X: 0x%02X", ICM45686_config[i].address, ICM45686_config[i].value);))
    spi_device_transmit(ICM45686_handle, &transaction); // Transmit the transaction
    vTaskDelay(2);
    i++;
  }

                                                        /*
                                                         * Program the indirect registers from the configuration table
                                                         */
  i = 0;
  while ( ICM45686_indirect_config[i].i_address != 0x00 ) // Loop through the configuration table until the end is reached
  {
    buffer[0] = HI(ICM45686_indirect_config[i].i_address);
    buffer[1] = LO(ICM45686_indirect_config[i].i_address);
    buffer[2] = ICM45686_indirect_config[i].value;

    memset(&transaction, 0, sizeof(transaction));         // Clear the transaction structure
    transaction.addr      = IREG_ADDR_15_8;               // Register address to write to
    transaction.tx_buffer = &buffer[0];
    transaction.length    = 3 * 8;                        // Transmit length in bits
    transaction.rx_buffer = 0;                            // Not used
    transaction.rxlength  = 0 * 8;                        // Receive length in bits
    transaction.flags     = 0;                            // Write directly
    spi_device_transmit(ICM45686_handle, &transaction);   // Transmit the transaction

    DLT(DLT_INFO,
        SEND(CONSOLE, sprintf(_xs, "i_register 0x%04X: 0x%02X", ICM45686_indirect_config[i].i_address, ICM45686_indirect_config[i].value);))
    spi_device_transmit(ICM45686_handle, &transaction);   // Transmit the transaction

    vTaskDelay(1);
    i++;
  }

  /*
   * All done, return
   */
  PAUSE("Finished")
  DLT(DLT_INFO, SEND(CONSOLE, sprintf(_xs, "ICM45686 initialization successful.  Sample Rate: %d", SAMPLE_RATE);))
  return;
}

/*----------------------------------------------------------------
 *
 * @function: ICM45686_pull_FIFO
 *
 * @brief:    Pull all of the samples out of the FIFO and store them in the sample buffer
 *
 * @return:   TRUE if the FIFO is completely full of data
 *
 *----------------------------------------------------------------
 *
 * This function is called if the FIFO watermark is reached.
 *
 * By the time we get here there are AT LEAST WATERMARK bytes of
 * samples in the FIFO.  The function will read one WATERMARKs
 * number of cycles and save them into memory.
 *
 * There may be samples left in the FIFO, but we will get them
 * the next time the watermark interrupt is fired.
 *
 *---------------------------------------------------------------*/
bool ICM45686_pull_FIFO(void)
{
  spi_transaction_t transaction;
  static bool       return_value = false; // Return TRUE if the FIFO is completely full of data
  time_count_64_t   start_time;           // Time function started

  /*
   *  We can read the FIFO
   */
  start_time = NTP_time_ms();

  /*
   *  Read in the next bunch of samples
   */
  index_last = index_in;                                       // Remember where we were before the read
  memset(&transaction, 0, sizeof(transaction));                // Clear the transaction structure{"TEST":24}
  transaction.addr      = 0x80 | FIFO_DATA;                    // Point to the FIFO read regisetrt
  transaction.tx_buffer = NULL;                                // Transmit buffer not used
  transaction.length    = sizeof(FIFO_packet_t) * 8;           // Transmit length in bits
  transaction.rx_buffer = &FIFO_queue[index_in.outer];
  transaction.rxlength  = sizeof(FIFO_packet_t) * 8;           // Receive length in bits
  transaction.flags     = 0;                                   // Indicate that this is a read operation
  spi_device_transmit(ICM45686_handle, &transaction);
  index_in.outer = (index_in.outer + 1) % SAMPLE_BUFFER_COUNT; // Move to the next buffer

  if ( index_in.outer == 0 )                                   // Wrapped around the buffer is full
  {
    run_state |= IN_FIFO_FULL;                                 // Indicate that the FIFO is full
    return_value = true;
  }

  /*
   *  Reset the interrupt status
   */
  ICM45686_clear_int1_status0(); // Reset the interrupt pending bits

  /*
   *  All done, return
   */
  DLT(DLT_TIMING, SEND(CONSOLE, sprintf(_xs, "ICM45686_pull_FIFO(),  delta: %lldms", run_time_ms() - start_time);))
  run_state |= IN_FIFO_FRESH; // Indicate that the FIFO is currently being filled
  return return_value;
}

/*----------------------------------------------------------------
 *
 * @function: ICM45686_test_FIFO
 *
 * @brief:    Pull out the FIFO samples for testing
 *
 * @return:   NONE
 *
 *----------------------------------------------------------------
 *
 * This function is called if the FIFO watermark is reached.
 *
 * By the time we get here there are AT LEAST WATERMARK bytes of
 * samples in the FIFO.  The function will read one WATERMARKs
 * number of cycles and save them into memory.
 *
 * There may be samples left in the FIFO, but we will get them
 * the next time the watermark interrupt is fired.
 *
 *---------------------------------------------------------------*/
#define TO_16A(x) ((int16_t)(((x)[1] << 8) | (x)[0])) // Convert two bytes to a 16-bit integer

char *to_bin(uint8_t value)                           // Convert a byte to a binary string
{
  static char str[9];
  int         i;

  /*
   *  Convert a byte to a binary string
   **/
  for ( i = 0; i != 8; i++ )
  {
    str[7 - i] = (value & (1 << i)) ? '1' : '0';
  }
  str[8] = 0;
  return str;
}

static int spaces[] = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0};
static int swap[]   = {0, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 0, 1, -1};

void ICM45686_dump_FIFO(void)
{
  int i, j;

  /*
   * Show the FIFO contents
   */
  run_state &= ~IN_FIFO_FRESH;             // Clear the FIFO fresh flag before starting the loop

  while ( (run_state & IN_FIFO_FRESH) == 0 )
  {
    vTaskDelay(1);                         // Wait for the FIFO to become fresh
  }

  for ( i = 0; i != RAW_FRAME_COUNT; i++ ) // Convert the whole FIFO buffer
  {
    SEND(CONSOLE, sprintf(_xs, "\r\n%3d:", i);)
    for ( j = 0; j < 16; j++ )
    {
      SEND(CONSOLE, sprintf(_xs, "%02X", FIFO_queue[index_in.outer].f[i].buffer[j + swap[j]]);)
      if ( spaces[j] )
      {
        SEND(CONSOLE, sprintf(_xs, "  ");)
      }
    }
  }
  /*
   *  All done, return
   */
  return;
}

/*----------------------------------------------------------------
 *
 * @function: ICM45686_find_index_out
 *
 * @brief:    Find the place corresponding to the shot sample
 *
 * @return:   TRUE if the data is valid
 *            Starting point of the trace
 *
 *----------------------------------------------------------------
 *
 * The FIFO is a freerunning queue that is updated perfiodically
 * (SAMPLE_RATE).
 *
 * The entry to the queue is index_in and the output is index_out
 * The function picks up the current index_in and subtracts the
 * entries needed to go backwards in time to the desired shot
 * time (index_out).
 *
 * Sample Calculations:
 *
 * shot:           99,627,789
 * last_FIFO_read:101,515,835
 *                -----------
 *                  1,888,046
 *
 * time_delay_s:  1.888,046 seconds
 * sample_delay:  1,510 samples
 *
 * index_in: (6, 0)
 * index_out:(2, 90)
 *
 * 2  -----  3  -----  4  ----- 5  -----  ( 6 not yet used)
 *  90 | 310     400        400      400
 *     |       1510 back in time          |
 *
 *---------------------------------------------------------------*/
bool ICM45686_find_index_out(time_count_64_t shot) // Time shot occured
{
  real_t       time_delay_s;                       // Time shot occured in micro seconds
  unsigned int sample_delay;

  DLT(DLT_DEBUG, SEND(CONSOLE, sprintf(_xs, "ICM45686_find_index_out(%'llu)", shot);))

  if ( shot == 0 )                                 // No shot time, just start at the current point in the FIFO
  {
    index_out.inner = 0;
    index_out.outer = 0;
    return false;
  }

  /*
   * Calculate how much to go backwards in time
   */
  time_delay_s = (real_t)(last_FIFO_read - shot) / (1000000.0); // Time in microseconds (ago)
  sample_delay = time_delay_s * SAMPLE_RATE;                    // This is how many samples behind

  DLT(DLT_DEBUG, SEND(CONSOLE, sprintf(_xs, "last_FIFO_read: %'llu shot: %'llu) => time_delay_s %7.4f   sample_delay:%u  %u",
                                       last_FIFO_read, shot, time_delay_s, sample_delay, sample_delay / RAW_FRAME_COUNT);))

  /*
   * Figure out what indexes to use
   */
  index_out.outer = (index_in.outer - (sample_delay / RAW_FRAME_COUNT) - 1); // Go backwards in time
  if ( index_out.outer < 0 )                                                 // Gone negative, wrap around
  {
    index_out.outer += SAMPLE_BUFFER_COUNT;
  }

  index_out.inner = RAW_FRAME_COUNT - (sample_delay % RAW_FRAME_COUNT);      // Residiue of the timer index;

  DLT(DLT_DEBUG, SEND(CONSOLE, sprintf(_xs, "in:%d %d = out:%d %d", index_in.outer, index_in.inner, index_out.outer, index_out.inner);))

  /*
   * All done, return
   */
  return true;
}

/*----------------------------------------------------------------
 *
 * @function: ICM45686_read_raw_accel()
 *
 * @brief:    Read acceleration data from the ICM45686
 *
 * @return:   Acceleration in FIFO format
 *
 *----------------------------------------------------------------
 *
 * The Acceleration data is read from the ICM45686 in 6 bytes,
 * with the X, Y, and Z axis data each consisting of a low byte
 * followed by a high byte. The raw acceleration data is stored
 * in the provided sample structure.
 *
 * IMPORTANT
 *
 * The ICM45686 brings the acceleration in as LSB and MSB.
 * ie, the bytes need to be swapped before use.
 *
 * The function collects information from the ICM45686 registers
 * and scrambles it into the FIFO format for use by the rest of the program.\
 *
 *--------------------------------------------------------------*/
void ICM45686_read_raw_accel(register_single_t *sample) // Returned values
{
  spi_transaction_t    transaction;
  register_raw_frame_t raw_frame;

  /*
   * Prepare and read a single sample directly from the ICM45686
   */
  memset(&transaction, 0x00, sizeof(transaction));            // Clear the transaction structure
  transaction.addr      = 0x80 | ACCEL_DATA_X1_UI;            // Start at Accel Acceleration Data and read all 6 bytes in one transaction
  transaction.length    = (sizeof(register_raw_frame_t)) * 8; // Transmit length in bits (less the empty)
  transaction.tx_buffer = NULL;                               // Send dummy data to read the acceleration data
  transaction.rxlength  = (sizeof(register_raw_frame_t)) * 8; // Don't count the empty
  transaction.rx_buffer = &raw_frame;                         // Receive buffer to store the raw acceleration data
  transaction.flags     = 0;
  spi_device_transmit(ICM45686_handle, &transaction);         // Transmit the transaction

  DLT(DLT_DEBUG,
      SEND(CONSOLE,
           sprintf(_xs, "raw  x_..: 0X%04X   y_..: 0X%04X   z_..: 0X%04X   rho_.: 0X%04X   theta_.: 0X%04X   phi_.: 0X%04X",
                   raw_frame.x_dotdot, raw_frame.y_dotdot, raw_frame.z_dotdot, raw_frame.rho_dot, raw_frame.theta_dot, raw_frame.phi_dot);))

  /*
   *  Scramble the register values to FIFO position
   */
  sample->x_dotdot  = raw_frame.x_dotdot;
  sample->y_dotdot  = raw_frame.y_dotdot;
  sample->z_dotdot  = raw_frame.z_dotdot;
  sample->rho_dot   = raw_frame.rho_dot;
  sample->theta_dot = raw_frame.theta_dot;
  sample->phi_dot   = raw_frame.phi_dot;

  /*
   *  All done
   */
  return;
}

/*----------------------------------------------------------------
 *
 * @function: ICM45686_read_temperature()
 *
 * @brief:    Read temperature data from the ICM45686
 *
 * @return:   Temperature in FIFO format
 *
 *----------------------------------------------------------------
 *
 * The Temperature data is read from the ICM45686 in 2 bytes,
 * with the temperature data consisting of a high byte followed by a low byte.
 *
 * {"TEST":43}
 *
 *--------------------------------------------------------------*/
void ICM45686_read_temperature(void) // Returned values
{
  spi_transaction_t transaction;
  real_t            temp_raw;        // Signed raw temperature value from the sensor
  real_t            temp_celsius;

  /*
   * Prepare and read a single sample directly from the ICM45686
   */
  memset(&transaction, 0, sizeof(transaction));                              // Clear the transaction structure
  transaction.addr      = 0x80 | TEMP_DATA1_UI;                              // Register address to read from
  transaction.length    = 3 * 8;                                             // Transmit length in bits
  transaction.tx_buffer = NULL;                                              // Transmit buffer not used
  transaction.rxlength  = 3 * 8;                                             // Receive length in bits
  transaction.flags     = SPI_TRANS_USE_RXDATA;                              // Indicate that this is a read operation

  spi_device_transmit(ICM45686_handle, &transaction);                        // Transmit the transaction
  SEND(CONSOLE, sprintf(_xs, "Temperature: 0x%02X %02X %02X", transaction.rx_data[0], transaction.rx_data[1], transaction.rx_data[2]);)
  temp_raw =
      -(((transaction.rx_data[1] << 8) | transaction.rx_data[0]) - 32768.0); // Combine the two bytes into a single raw temperature value
  temp_celsius = (temp_raw / 128.0) + 25.0; // Convert the raw temperature value to degrees Celsius using the formula from the datasheet

  SEND(CONSOLE, sprintf(_xs, "\r\nTemperature: %6.2f (%6.2f °C)", temp_raw, temp_celsius);)
  /*
   *  All done
   */
  return;
}

/*----------------------------------------------------------------
 *
 * @function: ICM45686_find_zero()
 *
 * @brief:    Determine the resting g levels for the ICM45686
 *
 * @return: None
 *
 *----------------------------------------------------------------
 *
 * The acceleration data always contains the earth's gravity,
 * so to get the vector acceleration of the device, we need to
 * zero the data by taking a sample when the device is stationary
 * and subtracting that from future samples.
 *
 * The function computes the AVERAGE acceleration value which
 * must later be SUBRACTED from the raw sensor value
 *
 *--------------------------------------------------------------*/

void ICM45686_find_zero(bool ask_for_confirm) // Ask for save confirmation)
{
  unsigned int       i;                       // Loop counter
  FIFO_real_single_t sample;                  // Read in the order the FIFO returns data
  FIFO_real_single_t min_sample, max_sample;  // Min and max valuse

  DLT(DLT_DEBUG, SEND(CONSOLE, sprintf(_xs, "ICM45686_find_zero()");))

  run_state |= IN_TEST;

  /*
   *  Clear the current offset
   */
  json_x_dotdot_offset  = 0;
  json_y_dotdot_offset  = 0;
  json_z_dotdot_offset  = 0;
  json_rho_dot_offset   = 0;
  json_theta_dot_offset = 0;
  json_phi_dot_offset   = 0;

  min_sample.x_dotdot  = 20000;
  min_sample.y_dotdot  = 20000;
  min_sample.z_dotdot  = 20000;
  min_sample.rho_dot   = 20000;
  min_sample.theta_dot = 20000;
  min_sample.phi_dot   = 20000;

  max_sample.x_dotdot  = 0;
  max_sample.y_dotdot  = 0;
  max_sample.z_dotdot  = 0;
  max_sample.rho_dot   = 0;
  max_sample.theta_dot = 0;
  max_sample.phi_dot   = 0;

  /*
   *  Read the registers to find the average
   */

  run_state &= ~IN_FIFO_FRESH;               // Clear the FIFO fresh flag before starting the loop

  while ( (run_state & IN_FIFO_FRESH) == 0 )
  {
    vTaskDelay(1);                           // Wait for the FIFO to become fresh
  }

  for ( i = 0; i != RAW_FRAME_COUNT; i++ )   // Convert the whole FIFO buffer
  {
    // TODO(IDF6): was ICM45686_fixed_unpack(). Every local in this function
    // (sample, min_sample, max_sample) is declared FIFO_real_single_t, and
    // fixed_unpack() wants a FIFO_fixed_single_t *, so this never matched.
    // Older compilers let the mismatch through as a warning; the 6.0 toolchain
    // makes it an error. Switched to the _real_ variant, which is the one whose
    // signature matches the declarations above. PLEASE CONFIRM this was the intent.
    ICM45686_real_unpack(&FIFO_queue[index_in.outer].f[i], &sample);
    json_x_dotdot_offset += sample.x_dotdot; // Accumulate the X-axis raw acceleration data
    json_y_dotdot_offset += sample.y_dotdot; // Accumulate the Y-axis raw acceleration data
    json_z_dotdot_offset += sample.z_dotdot; // Accumulate the Z-axis raw acceleration data
    json_rho_dot_offset += sample.rho_dot;
    json_theta_dot_offset += sample.theta_dot;
    json_phi_dot_offset += sample.phi_dot;

    if ( sample.x_dotdot < min_sample.x_dotdot )
      min_sample.x_dotdot = sample.x_dotdot;
    if ( sample.y_dotdot < min_sample.y_dotdot )
      min_sample.y_dotdot = sample.y_dotdot;
    if ( sample.z_dotdot < min_sample.z_dotdot )
      min_sample.z_dotdot = sample.z_dotdot;
    if ( sample.rho_dot < min_sample.rho_dot )
      min_sample.rho_dot = sample.rho_dot;
    if ( sample.theta_dot < min_sample.theta_dot )
      min_sample.theta_dot = sample.theta_dot;
    if ( sample.phi_dot < min_sample.phi_dot )
      min_sample.phi_dot = sample.phi_dot;

    if ( sample.x_dotdot > max_sample.x_dotdot )
      max_sample.x_dotdot = sample.x_dotdot;
    if ( sample.y_dotdot > max_sample.y_dotdot )
      max_sample.y_dotdot = sample.y_dotdot;
    if ( sample.z_dotdot > max_sample.z_dotdot )
      max_sample.z_dotdot = sample.z_dotdot;
    if ( sample.rho_dot > max_sample.rho_dot )
      max_sample.rho_dot = sample.rho_dot;
    if ( sample.theta_dot > max_sample.theta_dot )
      max_sample.theta_dot = sample.theta_dot;
    if ( sample.phi_dot > max_sample.phi_dot )
      max_sample.phi_dot = sample.phi_dot;
  }

  /*
   * Average the samples to get a more accurate zero level
   */
  json_x_dotdot_offset /= RAW_FRAME_COUNT;
  json_y_dotdot_offset /= RAW_FRAME_COUNT;
  json_z_dotdot_offset /= RAW_FRAME_COUNT;
  json_rho_dot_offset /= RAW_FRAME_COUNT;
  json_theta_dot_offset /= RAW_FRAME_COUNT;
  json_phi_dot_offset /= RAW_FRAME_COUNT;

  /*
   * Put the results in NONVOL
   */
  DLT(DLT_INFO, SEND(CONSOLE, sprintf(_xs, "MIN  - X_..: %6.4f  Y_..: %6.4f  Z_..: %6.4f   rho_.: %6.4f   theta_.: %6.4f  phi_.: %6.4f ",
                                      min_sample.x_dotdot, min_sample.y_dotdot, min_sample.z_dotdot, min_sample.rho_dot,
                                      min_sample.theta_dot, min_sample.phi_dot);))
  DLT(DLT_INFO, SEND(CONSOLE, sprintf(_xs, "MAX  - X_..: %6.4f  Y_..: %6.4f  Z_..: %6.4f   rho_.: %6.4f   theta_.: %6.4f  phi_.: %6.4f ",
                                      max_sample.x_dotdot, max_sample.y_dotdot, max_sample.z_dotdot, max_sample.rho_dot,
                                      max_sample.theta_dot, max_sample.phi_dot);))

  DLT(DLT_INFO,
      SEND(CONSOLE, sprintf(_xs, "Raw  - X_..: %6.4f  Y_..: %6.4f  Z_..: %6.4f   rho_.: %6.4f   theta_.: %6.4f  phi_.: %6.4f ",
                            sample.x_dotdot, sample.y_dotdot, sample.z_dotdot, sample.rho_dot, sample.theta_dot, sample.phi_dot);))

  DLT(DLT_INFO,
      SEND(CONSOLE, sprintf(_xs, "Zero - X_..: %d  Y_..: %d  Z_..: %d   rho_.: %d   theta_.: %d  phi_.: %d ", json_x_dotdot_offset,
                            json_y_dotdot_offset, json_z_dotdot_offset, json_rho_dot_offset, json_theta_dot_offset, json_phi_dot_offset);))

  if ( ask_for_confirm == true )
  {
    if ( prompt_for_confirm("Commit settings?") == true )
    {
      SEND(CONSOLE, sprintf(_xs, "\r\nZero offset saved");)
      nvs_set_i32(my_handle, NONVOL_X_DOTDOT_OFFSET, json_x_dotdot_offset); // Save the value
      nvs_set_i32(my_handle, NONVOL_Y_DOTDOT_OFFSET, json_y_dotdot_offset);
      nvs_set_i32(my_handle, NONVOL_Z_DOTDOT_OFFSET, json_z_dotdot_offset);
      nvs_set_i32(my_handle, NONVOL_RHO_DOT_OFFSET, json_rho_dot_offset);
      nvs_set_i32(my_handle, NONVOL_THETA_DOT_OFFSET, json_theta_dot_offset);
      nvs_set_i32(my_handle, NONVOL_PHI_DOT_OFFSET, json_phi_dot_offset);
    }
    else
    {
      SEND(CONSOLE, sprintf(_xs, "\r\nZero offset removed");)
      nvs_set_i32(my_handle, NONVOL_X_DOTDOT_OFFSET, 0); // Save the value
      nvs_set_i32(my_handle, NONVOL_Y_DOTDOT_OFFSET, 0);
      nvs_set_i32(my_handle, NONVOL_Z_DOTDOT_OFFSET, 0);
      nvs_set_i32(my_handle, NONVOL_RHO_DOT_OFFSET, 0);
      nvs_set_i32(my_handle, NONVOL_THETA_DOT_OFFSET, 0);
      nvs_set_i32(my_handle, NONVOL_PHI_DOT_OFFSET, 0);
    }
  }
  else
  {
    SEND(CONSOLE, sprintf(_xs, "\r\nZero offset saved");)
    nvs_set_i32(my_handle, NONVOL_X_DOTDOT_OFFSET, json_x_dotdot_offset); // Save the value
    nvs_set_i32(my_handle, NONVOL_Y_DOTDOT_OFFSET, json_y_dotdot_offset);
    nvs_set_i32(my_handle, NONVOL_Z_DOTDOT_OFFSET, json_z_dotdot_offset);
    nvs_set_i32(my_handle, NONVOL_RHO_DOT_OFFSET, json_rho_dot_offset);
    nvs_set_i32(my_handle, NONVOL_THETA_DOT_OFFSET, json_theta_dot_offset);
    nvs_set_i32(my_handle, NONVOL_PHI_DOT_OFFSET, json_phi_dot_offset);
  }

  /*
   *  All done, return
   */
  SEND(CONSOLE, sprintf(_xs, _DONE_);)
  return;
}

/*----------------------------------------------------------------
 *
 * @function: ICM45686_convert_to_g()
 *
 * @brief:    Convert raw acceleration data to g
 *
 * @return:   None
 *
 *----------------------------------------------------------------
 *
 * Multiply the raw acceleration data by the LSB per g for the
 * current range setting to convert to g
 *
 * The fudge factor 2* is because of the scaling inside of the
 * register.
 *
 *--------------------------------------------------------------*/
#define ACCEL_DEAD_BAND 0.005
#define GYRO_DEAD_BAND  0.0001
#define CAL_SCALE       1.0
#define SWAP_ENDIAN(x)  (((x << 8) & 0xFF00) | ((x >> 8) & 0x00FF))

void ICM45686_convert_to_g(FIFO_fixed_single_t *sample, // 16 bit numbers read from BICM45686
                           trace_vector_t      *vector  // Working values
)
{
  /*
   *  Swap the endians
   */
  /*
  sample->x_dotdot  = SWAP_ENDIAN(sample->x_dotdot);
  sample->y_dotdot  = SWAP_ENDIAN(sample->y_dotdot);
  sample->z_dotdot  = SWAP_ENDIAN(sample->z_dotdot);
  sample->rho_dot   = SWAP_ENDIAN(sample->rho_dot);
  sample->theta_dot = SWAP_ENDIAN(sample->theta_dot);
  sample->phi_dot   = SWAP_ENDIAN(sample->phi_dot);
  */
  vector->x_dotdot = CAL_SCALE * (real_t)(sample->x_dotdot - json_x_dotdot_offset) * G_PER_LSB; // Convert raw X-axis data to g
  if ( F_ABS(vector->x_dotdot) < ACCEL_DEAD_BAND )
  {
    vector->x_dotdot = 0;
  }

  vector->y_dotdot = CAL_SCALE * (real_t)(sample->y_dotdot - json_y_dotdot_offset) * G_PER_LSB; // Convert raw Y-axis data to g
  if ( F_ABS(vector->y_dotdot) < ACCEL_DEAD_BAND )
  {
    vector->y_dotdot = 0;
  }

  vector->z_dotdot = CAL_SCALE * (real_t)(sample->z_dotdot - json_z_dotdot_offset) * G_PER_LSB; // Convert raw Z-axis data to g
  if ( F_ABS(vector->z_dotdot) < ACCEL_DEAD_BAND )
  {
    vector->z_dotdot = 0;
  }

  vector->rho_dot = (real_t)(sample->rho_dot - json_rho_dot_offset) * GYRO_PER_LSB;             // Convert raw X-axis data to g
  if ( F_ABS(vector->rho_dot) < GYRO_DEAD_BAND )
  {
    vector->rho_dot = 0;
  }

  vector->theta_dot = (real_t)(sample->theta_dot - json_theta_dot_offset) * GYRO_PER_LSB;       // Convert raw X-axis data to g
  if ( F_ABS(vector->theta_dot) < GYRO_DEAD_BAND )
  {
    vector->theta_dot = 0;
  }

  vector->phi_dot = (real_t)(sample->phi_dot - json_phi_dot_offset) * GYRO_PER_LSB;             // Convert raw X-axis data to g
  if ( F_ABS(vector->phi_dot) < GYRO_DEAD_BAND )
  {
    vector->phi_dot = 0;
  }
  return;
}

/*----------------------------------------------------------------
 *
 * @function: ICM45686_oscilliscope()
 *
 * @brief:    Create a real time oscilliscope for the accelerometer
 *
 * @return:   None
 *
 *----------------------------------------------------------------
 *
 * Poll the ICM45686 and print out the acceleration data
 *
 *--------------------------------------------------------------*/
void ICM45686_oscilliscope(void)
{
  trace_vector_t      trace_vector;
  FIFO_fixed_single_t sample;  // A single sample
  int                 i;
  bool                pause = false;

  if ( prompt_for_confirm("Erase ZERO offset?") == true )
  {
    json_x_dotdot_offset  = 0; // Save the value
    json_y_dotdot_offset  = 0;
    json_z_dotdot_offset  = 0;
    json_rho_dot_offset   = 0;
    json_theta_dot_offset = 0;
    json_phi_dot_offset   = 0;
  }

  memset(&trace_vector, 0, sizeof(trace_vector));
  while ( 1 )
  {
    if ( pause == false )
    {
      run_state &= ~IN_FIFO_FRESH;             // Clear the FIFO fresh flag before starting the loop

      while ( (run_state & IN_FIFO_FRESH) == 0 )
      {
        vTaskDelay(1);                         // Wait for the FIFO to become fresh
      }

      for ( i = 0; i != RAW_FRAME_COUNT; i++ ) // Convert the whole FIFO buffer
      {
        ICM45686_fixed_unpack(&FIFO_queue[index_in.outer].f[i], &sample);

        SEND(CONSOLE, sprintf(_xs, "\r\nH: %02X  x:%04X y:%04X z:%04X    r:%04X  t:%04X p:%04X", sample.header, sample.x_dotdot,
                              sample.y_dotdot, sample.z_dotdot, sample.rho_dot, sample.theta_dot, sample.phi_dot);)
      }

      if ( serial_available(CONSOLE) != 0 )
      {
        char ch = serial_getch(CONSOLE);
        if ( ch == '!' )                  // Exit the test
        {
          break;
        }
        if ( (ch == 'P') || (ch == 'p') ) // Reset the test
        {
          pause = !pause;
        }
      }
    }
  }

  /*
   * All done
   */
  SEND(CONSOLE, sprintf(_xs, _DONE_);)
  return;
}

/*----------------------------------------------------------------
 *
 * @function: ICM45686_SPI_dump()
 *
 * @brief:    Dump the contents of the ICM45686 registers
 *
 * @return:   None
 *
 *----------------------------------------------------------------
 *
 * This function allows the user to enter a register address and a
 * value to write to that register, then reads back the value from the
 * register to verify that the SPI communication is working correctly.
 *
 *--------------------------------------------------------------*/
#define HEADER "\r\n\n          0    1    2    3      4    5    6    7      8    9    A    B      C    D    E    F"
#define TO_16(x)                                                                                                                           \
  (((registers[(x) + 1] << 8) |                                                                                                            \
    registers[(x)]))                // Convert two bytes to a 16 bit value, with the first byte as the MSB and the second byte as the LSB
void ICM45686_SPI_dump(void)
{
  int               address;        // Display address
  uint8_t           registers[128]; // Copy of registers
  spi_transaction_t transaction;

                                    /*
                                     * Read and print the values of all registers
                                     */
  memset(&registers, 0xAB, sizeof(registers));                   // Clear the registers array
  memset(&transaction, 0, sizeof(transaction));                  // Clear the transaction structure

  for ( address = 0; address != 0x80; address++ )
  {
    transaction.addr      = 0x80 | address;                      // Register address to read from
    transaction.length    = (1) * 8;                             // Transmit length in bits
    transaction.tx_buffer = NULL;                                // Transmit buffer not used
    transaction.rxlength  = (1) * 8;                             // Receive length in bits
    transaction.rx_buffer = NULL;                                // Use the pointer as the destination for the read data
    transaction.flags     = SPI_TRANS_USE_RXDATA;                // Indicate that this is a read operation;
    spi_device_transmit(ICM45686_handle, &transaction);          // Transmit the transaction

    registers[address] = transaction.rx_data[0];
  }

  for ( address = 0x00; address < sizeof(registers); address++ ) // Loop through all the registers from 0x00 to 0x7F
  {
    if ( (address % 0x40) == 0x00 )
    {
      SEND(CONSOLE, sprintf(_xs, HEADER);)                       // Print the register address at the start of each line
    }
    if ( (address & 0x0F) == 0x00 )
    {
      SEND(CONSOLE, sprintf(_xs, "\n0x%02X: ", address);)        // Print the register address at the start of each line
    }
    if ( (address % 4) == 0x00 )
    {
      SEND(CONSOLE, sprintf(_xs, "  ");)
    }
    SEND(CONSOLE, sprintf(_xs, "0x%02X ", registers[address]);)  // Print the value read from the register
  }

                                                                 /*
                                                                  *  Display known values
                                                                  */
  SEND(CONSOLE, sprintf(_xs, "\r\n");)
  SEND(CONSOLE, sprintf(_xs, "\r\n0x22: Temperature: %4.2f", ((real_t)TO_16(TEMP_DATA1_UI)) / 128.0 + 25.0);)
  SEND(CONSOLE, sprintf(_xs, "\r\n0x24: FIFO length: %d", ((registers[0x25] << 8) + registers[0x24]));)
  SEND(CONSOLE, sprintf(_xs, "\r\n0x0C: ACC X: %04X", TO_16(ACCEL_DATA_X1_UI));)
  SEND(CONSOLE, sprintf(_xs, "\r\n0x0E: ACC Y: %04X", TO_16(ACCEL_DATA_X1_UI + 2));)
  SEND(CONSOLE, sprintf(_xs, "\r\n0x10: ACC Z: %04X", TO_16(ACCEL_DATA_X1_UI + 4));)
  SEND(CONSOLE, sprintf(_xs, "\r\n0x12: GYRO X: %04X", TO_16(ACCEL_DATA_X1_UI + 6));)
  SEND(CONSOLE, sprintf(_xs, "\r\n0x14: GYRO Y: %04X", TO_16(ACCEL_DATA_X1_UI + 8));)
  SEND(CONSOLE, sprintf(_xs, "\r\n0x16: GYRO Z: %04X", TO_16(ACCEL_DATA_X1_UI + 10));)

  SEND(CONSOLE, sprintf(_xs, _DONE_);)
  return;
}

/*----------------------------------------------------------------
 *
 * @function: ICM45686_clear_int1_status0()
 *
 * @brief:    Clear the INT1_STATUS0 register if it indicates a reset status
 *
 * @return:   None
 *
 *----------------------------------------------------------------
 *
 * This function resets the interrupt pending bits in the
 * INT1_STATUS0 register by writing a zero to it.
 *
 *--------------------------------------------------------------*/
static void ICM45686_clear_int1_status0()
{
  spi_transaction_t transaction;

  DLT(DLT_DEBUG, SEND(CONSOLE, sprintf(_xs, "ICM45686_clear_int1_status0()");))

  memset(&transaction, 0, sizeof(transaction)); // Clear the transaction structure
  transaction.addr      = 0x80 | INT1_STATUS0;  // Point to the FIFO read regisetrt
  transaction.tx_buffer = 0;                    // Send a zero
  transaction.length    = 1 * 8;                // Transmit length in bits
  transaction.rx_buffer = NULL;
  transaction.rxlength  = 1 * 8;                // Receive length in bits
  transaction.flags     = SPI_TRANS_USE_TXDATA; // Indicate that this is a read operation
  spi_device_transmit(ICM45686_handle, &transaction);

  return;
}

                                                /*----------------------------------------------------------------
                                                 *
                                                 * @function: ICM45686_real_unpack()
                                                 *            ICM45686_fixed_unpack()
                                                 *
                                                 * @brief:    Unpack a raw FIFO frame into a structured sample
                                                 *
                                                 * @return:   None
                                                 *
                                                 *----------------------------------------------------------------
                                                 *
                                                 * This function takes a raw FIFO frame and converts it into a
                                                 * FIFO_real_single_t structure for easier access to individual sensor readings.
                                                 *
                                                 *--------------------------------------------------------------*/

#define TO_REAL(x, y)                                                                                                                      \
  temp = ((raw->buffer[x + 1] << 8) | raw->buffer[x]);                                                                                     \
  y    = temp;

void ICM45686_real_unpack(FIFO_raw_t         *raw,    // Raw input buffer
                          FIFO_real_single_t *sample) // Output structured sample
{
  int16_t temp;

  TO_REAL(X_DOTDOT, sample->x_dotdot);
  TO_REAL(Y_DOTDOT, sample->y_dotdot);
  TO_REAL(Z_DOTDOT, sample->z_dotdot);
  TO_REAL(RHO_DOT, sample->rho_dot);
  TO_REAL(THETA_DOT, sample->theta_dot);
  TO_REAL(PHI_DOT, sample->phi_dot);
  return;
}

#define TO_FIXED(x, y)                                                                                                                     \
  temp = ((raw->buffer[x + 1] << 8) | raw->buffer[x]);                                                                                     \
  y    = temp;

void ICM45686_fixed_unpack(FIFO_raw_t          *raw,    // Raw input buffer
                           FIFO_fixed_single_t *sample) // Output structured sample
{
  int16_t temp;

  TO_FIXED(X_DOTDOT, sample->x_dotdot);
  TO_FIXED(Y_DOTDOT, sample->y_dotdot);
  TO_FIXED(Z_DOTDOT, sample->z_dotdot);
  TO_FIXED(RHO_DOT, sample->rho_dot);
  TO_FIXED(THETA_DOT, sample->theta_dot);
  TO_FIXED(PHI_DOT, sample->phi_dot);
  return;
}

/*----------------------------------------------------------------
 *
 * @function: FIFO_return_first()
 *            FIFO_return_next()
 *            FIFO_return_previous()
 *
 * @brief:    Manage indexes
 *
 * @return:   Pointer to next sample in the FIFO buffer
 *
 *----------------------------------------------------------------
 *
 * The raw input is stored in two queues, inner and outer
 *
 * outer           ^ --> outer          ^ --> outer -->
 *                 |                    |
 *   inner --> inner     inner --> inner
 *
 * outer points to the next available FIFO input buffer
 * inner points to an individual sample in the FIFO input buffer
 *
 *--------------------------------------------------------------*/

FIFO_raw_t *FIFO_return_first()     // Find the oldest sample in the FIFO buffer
{
  index_out.outer = index_in.outer; // Start at oldest sample in the FIFO buffer
  index_out.inner = 0;              // Start at the first sample in the FIFO buffer
  return &FIFO_queue[index_out.outer].f[0];
}

FIFO_raw_t *FIFO_return_next(void)
{
  /*
   *  Move to the next sample in the FIFO buffer
   */
  index_out.inner = (index_out.inner + 1) % RAW_FRAME_COUNT; // Go to the next sample in the FIFO buffer

  /*
   *  Check if we have wrapped around to the beginning of the FIFO buffer
   */
  if ( index_out.inner == 0 )                                      // Wrapped around, move to the next FIFO buffer
  {
    index_out.outer = (index_out.outer + 1) % SAMPLE_BUFFER_COUNT; // Move to the next FIFO buffer
  }

  /*
   *  Check if we have caught up to the input point
   */
  if ( (index_out.outer == index_in.outer) && (index_out.inner == index_in.inner) ) // Wrapped around to the input point, no more data
  {
    return NULL;                                                                    // No more data
  }

  return &FIFO_queue[index_out.outer].f[index_out.inner];
}

FIFO_raw_t *FIFO_return_previous(void)
{
  index_out.outer--;         // Go backwards
  if ( index_out.outer < 0 ) // Wrap around
  {
    index_out.outer = SAMPLE_BUFFER_COUNT - 1;
    index_out.inner--;
    if ( index_out.inner < 0 )
    {
      index_out.inner = RAW_FRAME_COUNT - 1;
    }
  }

  return &FIFO_queue[index_out.outer].f[index_out.inner];
}

/*----------------------------------------------------------------
 *
 * @function: ICM45686_FIFO_statistics
 *
 * @brief: Compute the FIFO statistics
 *
 * @return: None
 *
 *----------------------------------------------------------------
 *
 * The FIFO statistics computed on the buffier that has just been
 * read.
 *
 *---------------------------------------------------------------*/
#define I16(x) ((x) & 0xffff)
void ICM45686_FIFO_statistics(void)                               // Display the FIFO diagnostics
{
  int                i;
  FIFO_packet_t     *fifo_packet = &FIFO_queue[index_last.outer]; // Pointer to the current FIFO packet
  FIFO_real_single_t fifo_single;                                 // Single unpacked sample from the FIFO packet
  time_count_64_t    start_time;

  start_time = run_time_ms();

  /*
   *  Work out the average of each component
   */
  for ( i = 0; i != 6; i++ )
  {
    fifo_packet->mean[i]    = 0.0;
    fifo_packet->std_dev[i] = 0.0;
  }

  for ( i = 0; i < RAW_FRAME_COUNT; i++ )
  {
    ICM45686_real_unpack(&fifo_packet->f[i], &fifo_single);
    fifo_packet->mean[SX_DOTDOT] += fifo_single.x_dotdot;
    fifo_packet->mean[SY_DOTDOT] += fifo_single.y_dotdot;
    fifo_packet->mean[SZ_DOTDOT] += fifo_single.z_dotdot;
    fifo_packet->mean[SRHO_DOT] += fifo_single.rho_dot;
    fifo_packet->mean[STHETA_DOT] += fifo_single.theta_dot;
    fifo_packet->mean[SPHI_DOT] += fifo_single.phi_dot;

    DLT(DLT_DEBUG, SEND(CONSOLE, sprintf(_xs, "FIFO: %3d X=%6.0f Y=%6.0f Z=%6.0f RHO=%6.0f THETA=%6.0f PHI=%6.0f", i, fifo_single.x_dotdot,
                                         fifo_single.y_dotdot, fifo_single.z_dotdot, fifo_single.rho_dot, fifo_single.theta_dot,
                                         fifo_single.phi_dot);))
  }

  fifo_packet->mean[SX_DOTDOT] /= (real_t)RAW_FRAME_COUNT;
  fifo_packet->mean[SY_DOTDOT] /= (real_t)RAW_FRAME_COUNT;
  fifo_packet->mean[SZ_DOTDOT] /= (real_t)RAW_FRAME_COUNT;
  fifo_packet->mean[SRHO_DOT] /= (real_t)RAW_FRAME_COUNT;
  fifo_packet->mean[STHETA_DOT] /= (real_t)RAW_FRAME_COUNT;
  fifo_packet->mean[SPHI_DOT] /= (real_t)RAW_FRAME_COUNT;

  DLT(DLT_DEBUG, SEND(CONSOLE, sprintf(_xs, "mean:    X=%6.0f Y=%6.0f Z=%6.0f RHO=%6.0f THETA=%6.0f PHI=%6.0f",
                                       fifo_packet->mean[SX_DOTDOT], fifo_packet->mean[SY_DOTDOT], fifo_packet->mean[SZ_DOTDOT],
                                       fifo_packet->mean[SRHO_DOT], fifo_packet->mean[STHETA_DOT], fifo_packet->mean[SPHI_DOT]);))

  /*
   * Work out the standard deviation
   */
  for ( i = 0; i < RAW_FRAME_COUNT; i++ )
  {
    ICM45686_real_unpack(&fifo_packet->f[i], &fifo_single);
    fifo_packet->std_dev[SX_DOTDOT] += SQ((fifo_single.x_dotdot - fifo_packet->mean[SX_DOTDOT]));
    fifo_packet->std_dev[SY_DOTDOT] += SQ((fifo_single.y_dotdot - fifo_packet->mean[SY_DOTDOT]));
    fifo_packet->std_dev[SZ_DOTDOT] += SQ((fifo_single.z_dotdot - fifo_packet->mean[SZ_DOTDOT]));
    fifo_packet->std_dev[SRHO_DOT] += SQ((fifo_single.rho_dot - fifo_packet->mean[SRHO_DOT]));
    fifo_packet->std_dev[STHETA_DOT] += SQ((fifo_single.theta_dot - fifo_packet->mean[STHETA_DOT]));
    fifo_packet->std_dev[SPHI_DOT] += SQ((fifo_single.phi_dot - fifo_packet->mean[SPHI_DOT]));
  }
  fifo_packet->std_dev[SX_DOTDOT]  = sqrt(fifo_packet->std_dev[SX_DOTDOT] / (real_t)RAW_FRAME_COUNT);
  fifo_packet->std_dev[SY_DOTDOT]  = sqrt(fifo_packet->std_dev[SY_DOTDOT] / (real_t)RAW_FRAME_COUNT);
  fifo_packet->std_dev[SZ_DOTDOT]  = sqrt(fifo_packet->std_dev[SZ_DOTDOT] / (real_t)RAW_FRAME_COUNT);
  fifo_packet->std_dev[SRHO_DOT]   = sqrt(fifo_packet->std_dev[SRHO_DOT] / (real_t)RAW_FRAME_COUNT);
  fifo_packet->std_dev[STHETA_DOT] = sqrt(fifo_packet->std_dev[STHETA_DOT] / (real_t)RAW_FRAME_COUNT);
  fifo_packet->std_dev[SPHI_DOT]   = sqrt(fifo_packet->std_dev[SPHI_DOT] / RAW_FRAME_COUNT);

  DLT(DLT_DEBUG, SEND(CONSOLE, sprintf(_xs, "std_dev: X=%6.0f Y=%6.0f Z=%6.0f RHO=%6.0f THETA=%6.0f PHI=%6.0f",
                                       fifo_packet->std_dev[SX_DOTDOT], fifo_packet->std_dev[SY_DOTDOT], fifo_packet->std_dev[SZ_DOTDOT],
                                       fifo_packet->std_dev[SRHO_DOT], fifo_packet->std_dev[STHETA_DOT], fifo_packet->std_dev[SPHI_DOT]);))

  if ( serial_available(CONSOLE) != 0 )
  {
    char ch = serial_getch(CONSOLE);
    if ( (ch == 'P') || (ch == 'p') ) // Reset the test
    {
      while ( serial_available(CONSOLE) == 0 )
      {
        vTaskDelay(10);
      }
      serial_flush(CONSOLE);
    }
  }

  /*
   *  All done, return
   */
  DLT(DLT_TIMING, SEND(CONSOLE, sprintf(_xs, "ICM45686_FIFO_statistics(), delta_time: %lldms", run_time_ms() - start_time);))
  return;
}
