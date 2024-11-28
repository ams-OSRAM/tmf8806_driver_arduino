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

// ---------------------------------------------- defines -----------------------------------------

/* define to use ROM firmware instead of RAM patch */
#define USE_ROM_FIRMWARE 

// ---------------------------------------------- functions ---------------------------------------

/** @brief Arduino setup function is only called once at startup. Do all the HW initialisation stuff here.
 * @param logLevelIdx ...  the log level index to be used (0..7 -> see logLevels array in tmf8806_app.cpp)
 * @param  baudrate ... for the serial input the baudrate
 * @param  i2cClockSpeedInHz ... the i2c frequency
 */
void setupFn( uint8_t logLevelIdx, uint32_t baudrate, uint32_t i2cClockSpeedInHz );

/** @brief Arduino main loop function, is executed cyclic
 * @return 1 if wants to be called again
 * @return 0 if program should terminate
 */
int8_t loopFn( );

/** @brief Arduino terminate function is only called once when exit key 'q' is pressed. Write a message and wait for shutdown of arduino.
 */
void terminateFn( );

#endif // TMF8806_APP_H
