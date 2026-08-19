/*----------------------------------------------------------------
 *
 * product.h
 *
 * Definition of how the product is constructed
 *
 *--------------------------------------------------------------*/

#ifndef _PRODUCT_H
#define _PRODUCT_H

/*
 * Configuration
 */
#define USE_ICM45686         (1 == 1) // Use the ICM45686 sensor
#define USE_BMI270           (0 == 1) // Use the BMI270 sensor
#define BUILD_TRACE          (1 == 1) // Build the trace module
#define BUILD_TARGET         (0 == 1) // Build the target module
#define INCLUDE_WIFI_STATION (1 == 1) // Include the code to make a station
#define INCLUDE_WIFI_AP      (0 == 1) // Trace is always a station
#define INCLUDE_CLIENT       (1 == 1) // Include Client network software
#define INCLUDE_SERVER       (0 == 1) // Include Server network software
#define INCLUDE_MDNS         (0 == 1) // Build the mDNS module

#define SOFTWARE_VERSION "\"BOB 1.0.0 August 17, 2026\""

/*
 *  Common typedefs
 */

#endif
