/*-----------------------------------------------------------
 *
 * common.h
 *
 *------------------------------------------------------------
 *
 * Common definitions for the software
 *
 *-----------------------------------------------------------*/
#ifndef _COMMON_H_
#define _COMMON_H_

/*
 *  Types
 */
typedef unsigned char    byte_t;
typedef volatile int64_t time_count_64_t; // Time in us
typedef volatile int32_t time_count_t;    // Time in s
typedef float            real_t;          // ESP32 has built in single precision floating point (32 bits)

/*
 * Common includes
 */
#include "product.h"        // Defines the overall product configuration
#include "board_assembly.h" // Defines the board assembly configuration
#include "helpers.h"
#include "WiFi.h"
#include "diag_tools.h"
#include "gpio.h"
#include "json.h"
#include "timer.h"
#include "ICM45686.h" // Interface to the ICM45686 3-axis accelerometer

#include "IMU.h"

#include "ntp.h"
#include "client.h"
#include "server.h"
#include "nvs_flash.h"
#include "nonvol.h"
#include "serial_io.h"

#include "trace.h" // Trace control loop

#endif
