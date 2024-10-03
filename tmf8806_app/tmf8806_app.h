/*****************************************************************************
* Copyright (c) [2024] ams-OSRAM AG                                          *
* All rights are reserved.                                                   *
*                                                                            *
* FOR FULL LICENSE TEXT SEE LICENSE.TXT                                      *
******************************************************************************/
 

/** @file This is the tmf8806 arduino uno driver example console application. 
 */


#ifndef TMF8806_APP_H
#define TMF8806_APP_H

// ---------------------------------------------- includes ----------------------------------------

#include "tmf8806.h"
#include "tmf8806_regs.h"
#include "tmf8806_app.h"


// ---------------------------------------------- compile time defines ----------------------------

// tmf8806 device configuration
#define KILO_ITERATIONS         10    /* number of integrations: range is 10 - 4000 */


// application configuration
// About PERIOD_*OBJECT_IN_MS: if integration time is longer than repeat time, integration time will be used
#define PERIOD_NO_OBJECT_IN_MS  1024  /* 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192 ms range, period if no object is in range */
#define PERIOD_OBJECT_IN_MS     128   /* 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192 ms range, period if object is in range */
#define PERSISTENCE             3     /* 0..255 how many consecutive measurements before the driver averages and reports them, 0 means that thresholds are ignored */
#define LOW_THRESHOLD_MM        50    /* 0..10000 object must be in distance greater than this value in mm */
#define HIGH_THRESHOLD_MM       200   /* 0..10000 object must be in distance smaller than this value in mm */


// select where object detection shall be reported
#define OUTPUT_ON_UART          1     /* set to 1 to print averages on uart, set to 0 will remove the code in the image that prints to uart */
#define OUTPUT_ON_LED           1     /* set to 1 to have led on when there is an object in range and off, when there is none. Set to 0 to remove the code that drives the led */
#define OUTPUT_ON_GPIO          0     /* set to 1 to have a led go high when there is an object in range and low when there is none. Set to 0 to remove the code that drives the gpio */


// some compile time configuration checks that will lead to a compile error
#if ( KILO_ITERATIONS < 10 || KILO_ITERATIONS > 4000 )
  #error "ERROR: KILO_ITERATIONS is out of range, has to be 10..4000"
#endif
#if ( PERIOD_NO_OBJECT_IN_MS < 10 || PERIOD_NO_OBJECT_IN_MS > 10000 )
  #error "ERROR: PERIOD_NO_OBJECT_IN_MS is out of range, has to be 10..10000"
#endif
#if ( PERSISTENCE > 255 || PERSISTENCE < 0 )
  #error "ERROR: PERSISTENCE is out of range, has to be 0..255"
#endif

// automatically select at compile time if distance mode is needed or not. Everything below 20cm can be detected by 
// proximity mode which again saves power
#if HIGH_THRESHOLD_MM > 200
  #define DISTANCE_ENABLED    1
#else
  #define DISTANCE_ENABLED    0
#endif


// --------------------------------------------- for debugging purposes ----------------------------

#define DRIVER_LOG_LEVEL     (TMF8806_LOG_LEVEL_NONE)    /* driver log level - for debugging purposes switch this on, see tmf8806.h for details */
#define APP_LOG_LEVEL        0                           /* set this to 1 to have the application logging, set it to 0 to turn application log off */

#if ( OUTPUT_ON_UART == 0 )
  #if ( DRIVER_LOG_LEVEL != TMF8806_LOG_LEVEL_NONE )
    #error "ERROR: OUTPUT_ON_UART is off. Must set DRIVER_LOG_LEVEL set to TMF8806_LOG_LEVEL_NONE"
  #endif
  #if ( APP_LOG_LEVEL != 0 )
    #error "ERROR: OUTPUT_ON_UART is off. Must set APP_LOG_LEVEL set to 0"
  #endif
#endif


// ---------------------------------------------- functions ---------------------------------------

#if defined( __cplusplus)
extern "C"
{
#endif

/** @brief Arduino setup function is only called once at startup. Do all the initialisation stuff here.
 * @param  baudrate ... for the serial the baudrate
 * @param  i2cClockSpeedInHz ... the i2c frequency
 */
void setupFn( uint32_t baudrate, uint32_t i2cClockSpeedInHz );

/** @brief Arduino main loop function, is executed cyclic
 */
void loopFn( );

#if defined( __cplusplus)
}
#endif

#endif // TMF8806_APP_H
