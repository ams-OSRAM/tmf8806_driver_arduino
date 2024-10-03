/*****************************************************************************
* Copyright (c) [2024] ams-OSRAM AG                                          *
* All rights are reserved.                                                   *
*                                                                            *
* FOR FULL LICENSE TEXT SEE LICENSE.TXT                                      *
******************************************************************************/
 

#ifndef TMF8806_SHIM_H
#define TMF8806_SHIM_H

/** @file This is the shim for the arduino uno. 
 * Any define, macro and/or function herein must be adapted to match your
 * target platform
 */

// ---------------------------------------------- includes ----------------------------------------

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "tmf8806_regs.h"
#include <avr/pgmspace.h>
#include <Arduino.h>

#if defined( __cplusplus)
extern "C"
{
#endif


// ---------------------------------------------- defines -----------------------------------------

#define ARDUINO_MAX_I2C_TRANSFER                  32    /**< Arduino Uno can only handle up to 32 bytes in a single i2c tx/rx */

#define ENABLE_PIN                                6     /**< on the arduino uno the enable pin is connected to digital 6 */
#define INTERRUPT_PIN                             2     /**< interupt to digital 2 */
#define RESULT_PIN                                3     /**< pin 2 is interreupt, pin 6 is enable line, pins 0+1 are uart, so pin 3 is available */
#define LED_PIN                                   LED_BUILTIN

#define USE_INTERRUPT_TO_TRIGGER_READ             1     /**< do not define this, or set to 0 to use i2c polling instead of interrupt pin */

    
// ---------------------------------------------- macros ------------------------------------------


/** @brief macros to cast a pointer to an address - adapt for your machine-word size
 */ 
#define PTR_TO_UINT(ptr)                     ( (intptr_t)(ptr) )

/** @brief macros to replace the platform specific printing
 */ 
#define PRINT_CHAR(c)                         printChar( c )
#define PRINT_INT(i)                          printInt( i )
#define PRINT_UINT(i)                         printUint( i )
#define PRINT_UINT_HEX(i)                     printUintHex( i )
#define PRINT_STR(str)                        printStr( str )  
#define PRINT_CONST_STR(str)                  printConstStr( (const char *)str )  
#define PRINT_LN()                            printLn( )

/** Which character to use to seperate the entries in printing */
#define SEPARATOR                             ','

// for clock correction insert here the number in relation to your host
#define HOST_TICKS_PER_10_US                  10         /**< number of host ticks every 10 microseconds */
#define TMF8806_TICKS_PER_10_US               47         /**< number of tmf8806 ticks every 10 mircoseconds (4.7x faster than host) */


// ---------------------------------------------- functions ---------------------------------------

/** @brief Function to allow to wait for some time in microseconds
 *  @param[in] wait number of microseconds to wait before this function returns
 */
void delayInMicroseconds( uint32_t wait );

/** @brief Function returns the current sys-tick.
 * \return current system tick (granularity is host specific - see macro HOST_TICKS_PER_10_US) 
 */
uint32_t getSysTick( );

/** @brief Function reads a single byte from the given address. This is only needed on 
 * systems that have special memory access methods for constant segments. Like e.g. Arduino Uno
 *  @param[in] ptr memory to read from
 * \return single byte from the given address 
 */
uint8_t readProgramMemoryByte( const uint8_t * ptr );


// ---------------------------------- timer functions ---------------------------------------------

/** @brief Function to start/restart the periodic timer but do not not reset already counted ticks   
 * @param dptr a pointer to a data structure the function needs for timer setup, can
 * be 0-pointer if the function does not need it
 * @param timeInMs the time in ms that the periodic timer shall use
*/
void timerStartPeriodic( void * dptr, uint16_t timeInMs );

/** @brief Function to stop the periodic timer 
 * @param dptr a pointer to a data structure the function needs for timer stop, can
 * be 0-pointer if the function does not need it
*/
void timerStop( void * dptr );


// ---------------------------------- power state functions --------------------------------------

/** @brief Function to enter power down. Wakeup will be through timeout or an interrupt that can 
 * trigger a wakeup (depending on MCU)  
*/
void powerDown( void * dptr );


// ---------------------------------- interrupt functions --------------------------------------

/** @brief Function registers given function handler as Interrupt Handler function for the
 *  interrupt pin
 *  @param[in] handler pointer to the interrupt service routine   
 */
void setInterruptHandler( void (* handler)( void ) );

/** @brief Function removes any Interrupt Handler function
 */
void clrInterruptHandler( void );

/** @brief Function globally disables interrupts
 */
void disableInterrupts( void );

/** @brief Function globally enables interrupts
 */
void enableInterrupts( void );


// ---------------------------------- pin functions ---------------------------------------------

/** @brief Function defines the given pin as output
 * @param[in] dptr ... a pointer to a data structure the function needs for setting the pin direction, can
 * be 0-pointer if the function does not need it
 * @param[in] pin ... pin number that shall be set to output mode
 */
void pinOutput( void * dptr, uint8_t pin );

/** @brief Function defines the given pin as input
 * @param[in] dptr ... a pointer to a data structure the function needs for setting the pin direction, can
 * be 0-pointer if the function does not need it
 * @param[in] pin ... pin number that shall be set to input mode
 */
void pinInput( void * dptr,  uint8_t pin );


/** @brief Function sets the enable pin HIGH. Note that the enable pin must be configured
 * for output (with e.g. function pinOutput)
 * @param[in] dptr ... a pointer to a data structure the function needs for setting the enable pin, can
 * be 0-pointer if the function does not need it
 */
void enablePinHigh( void * dptr );

/** @brief Function sets the enable pin LOW. Note that the enable pin must be configured
 * for output (with e.g. function pinOutput)
 * @param[in] dptr ... a pointer to a data structure the function needs for setting the enable pin, can
 * be 0-pointer if the function does not need it
 */
void enablePinLow( void * dptr );

/** @brief Function sets the result-in-range pin HIGH. Note that the result pin must be configured
 * for output (with e.g. function pinOutput)
 * @param[in] dptr ... a pointer to a data structure the function needs for setting the pin, can
 * be 0-pointer if the function does not need it
 */
void resultPinHigh( void * dptr );

/** @brief Function sets the result-in-range pin LOW. Note that the result pin must be configured
 * for output (with e.g. function pinOutput)
 * @param[in] dptr ... a pointer to a data structure the function needs for setting the pin, can
 * be 0-pointer if the function does not need it
 */
void resultPinLow( void * dptr );

/** @brief Function sets the led pin HIGH. Note that the led pin must be configured
 * for output (with e.g. function pinOutput)
 * @param[in] dptr ... a pointer to a data structure the function needs for setting the pin, can
 * be 0-pointer if the function does not need it
 */
void ledPinHigh( void * dptr );

/** @brief Function sets the result-in-range pin LOW. Note that the led pin must be configured
 * for output (with e.g. function pinOutput)
 * @param[in] dptr ... a pointer to a data structure the function needs for setting the pin, can
 * be 0-pointer if the function does not need it
 */
void ledPinLow( void * dptr );


// ---------------------------------- I2C functions ---------------------------------------------

/**  Return codes for i2c functions: 
 */
#define I2C_SUCCESS             0       /**< successfull execution no error */
#define I2C_ERR_DATA_TOO_LONG   -1      /**< driver cannot handle given amount of data for tx/rx */
#define I2C_ERR_SLAVE_ADDR_NAK  -2      /**< device nak'ed slave address */
#define I2C_ERR_DATA_NAK        -3      /**< device nak'ed written data */
#define I2C_ERR_OTHER           -4      /**< any other error */
#define I2C_ERR_TIMEOUT         -5      /**< timeout in waiting for slave to respond */

/** @brief Function will open the I2C master and configure for the given speed (if possible),
 * else it will reduce the speed to the available frequency
 * @param[in] dptr ... a pointer to a data structure the function may need, can
 * be 0-pointer if the function does not need it
 * @param[in] i2cClockSpeedInHz ... desired i2c clock speed in hertz
 */
void i2cOpen( void * dptr, uint32_t i2cClockSpeedInHz );

/** @brief Function closes the i2c master 
 * @param[in] dptr ... a pointer to a data structure the function may need, can
 * be 0-pointer if the function does not need it
 */
void i2cClose( void * dptr );


/** There are 2 styles of functions available:
 * 1. those that always require a register address to be specified: i2cTxReg, i2cRxReg
 * 2. the more generic (more I2C standard like) that can transmit and/or receive (here the
 *  register address if needed is the first transmitted byte): i2cTxRx
 * Only one set of those two *have to be* available. Both can be available.
 */

/** @brief I2C transmit only function.
 * @param[in] dptr a pointer to a data structure the function needs for transmitting, can
 * be 0-pointer if the function does not need it
 * @param[in] slaveAddr the i2c slave address to be used (7-bit unshifted)
 * @param[in] regAddr the register to start writing to
 * @param[in] toTx number of bytes in the buffer to transmit
 * @param[in] txData pointer to the buffer to transmit
 * \return 0 when successfully transmitted, else an error code
 */ 
int8_t i2cTxReg( void * dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toTx, const uint8_t * txData );

/** @brief I2C transmit register address and receive function.
 * @param[in] dptr a pointer to a data structure the function needs for receiving, can
 * be 0-pointer if the function does not need it
 * @param slaveAddr the i2c slave address to be used (7-bit)
 * @param regAddr the register address to start reading from
 * @param toRx number of bytes in the buffer to receive
 * @param rxData pointer to the buffer to be filled with received bytes
 * \return 0 when successfully received, else an error code
 */ 
int8_t i2cRxReg( void * dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toRx, uint8_t * rxData );

/** @brief I2C transmit and receive function.
 * @param dptr a pointer to a data structure the function needs for transmitting, can
 * be 0-pointer if the function does not need it
 * @param slaveAddr the i2c slave address to be used (7-bit)
 * @param toTx number of bytes in the buffer to transmit (set to 0 if receive only)
 * @param txData pointer to the buffer to transmit
 * @param toRx number of bytes in the buffer to receive (set to 0 if transmit only)
 * @param rxData pointer to the buffer to be filled with received bytes
 * \return 0 when successfully transmitted and received, else an error code
 */ 
int8_t i2cTxRx( void * dptr, uint8_t slaveAddr, uint16_t toTx, const uint8_t * txData, uint16_t toRx, uint8_t * rxData );


// ---------------------------------- UART functions --------------------------------------------

/** @brief Function will open the serial and clear the pipe
 * @param[in] baudrate serial rate in baud
 */
void uartOpen( uint32_t baudrate );

/** @brief Function will close the serial 
 */
void uartClose( );

/** @brief Function outputs a single character. E.g. on a UART.
 *  @param[in] c the character to be printed 
 */
void printChar( char c );

/** @brief Function outputs a signed integer. E.g. on a UART.
 *  @param[in] i the integer to be printed 
 */
void printInt( int32_t i );

/** @brief Function outputs an unsigned integer. E.g. on a UART.
 *  @param[in] i the integer to be printed 
 */
void printUint( uint32_t i );

/** @brief Function outputs an unsigned integer in HEX format. E.g. on a UART.
 *  @param[in] i the integer to be printed 
 */
void printUintHex( uint32_t i );

/** @brief Function outputs a zero terminated string. E.g. on a UART.
 *  @param[in] str pointer to string to be printed  
 */
void printStr( char * str );

/** @brief Function outputs a new-line. E.g. on a UART.
 */
void printLn( void );

/** @brief Function outputs a zero terminated constant string. E.g. on a UART.
 *  @param[in] str pointer to constant string to be printed. On some systems constants can
 * be stored in special memory and require special access for reading.   
 */
void printConstStr( const char * str );


// ------------------------------------------------------------------------------------------------
// readout functions that need special implementation on the Arduino Uno as the RAM is very limited 

/** @brief Function to read a 128 bytes == 1/4 of a 256 bin (2bytes) histogram
 * needed as reading register 0x30 will lead to register bank switching
 * on the device and the arduino uno can only read chunks of 32 bytes
 * So this function reads the quater histogram in reverse order.
 * @param[in] dptr ... a pointer to a data structure the function needs for transmitting, can
 * be 0-pointer if the function does not need it
 * @param[in] slaveAddr ... the i2c slave address to be used (7-bit)
 * @param[in] buffer ... a pointer to a buffer of minimum size of 128 bytes
 * \return 0 when successfully transmitted, else an error code 
 */
int8_t tmf8806ReadQuadHistogram( void * dptr, uint8_t slaveAddr, uint8_t * buffer );

/** @brief Function to print a single TDC channel histogram in a kind of CSV like format 
 * @param dptr a pointer to a data structure the function needs for transmitting, can
 * be 0-pointer if the function does not need it
 * @param histType type of histogram that will is in the buffer data
 * @param id the id for further identification of the histogram's TDC/channel 
 * @param data a buffer of 256 bytes that represent the histogram in little endian encoding for 16-bit
 * @param scale a scaling factor. Each raw 16-bit value needs to be shifted by this amount to the left
 * This function is directly called by the driver to dump a histogram to the UART
 */
void tmf8806ScaleAndPrintHistogram( void * dptr, uint8_t histType, uint8_t id, uint8_t * data, uint8_t scale );


#if defined( __cplusplus)
}
#endif

#endif  // TMF8806_SHIM_H

