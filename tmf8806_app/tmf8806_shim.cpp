/*****************************************************************************
* Copyright (c) [2024] ams-OSRAM AG                                          *
* All rights are reserved.                                                   *
*                                                                            *
* FOR FULL LICENSE TEXT SEE LICENSE.TXT                                      *
******************************************************************************/
 
// Please note that this file as a c++ file. This is due to the fact that the arduino 
// implements some functions that are used here in c++. E.g. the Serial library is
// implemented as a c++ library.
// If you implement all functions in your shim as pure C function, you may name your
// shim layer file as: tmf8806_shim.c and compile it with a C compiler. 
// In this case you should remove the these lines from the tmf8806_shim.h file:
// #if defined( __cplusplus)
// extern "C"
// {
// #endif
// 
// and at the very end of the file:
//
// #if defined( __cplusplus)
// }
// #endif
//
// These macros are only used to allow cross-language (C and C++) compatibility. 
// 


#include "tmf8806_shim.h"
#include "tmf8806.h"
#include "Arduino.h"
#include <time.h>
#include <Wire.h>
#include <avr/wdt.h>

// section with forward declaration of plain C functions
#if defined( __cplusplus)
extern "C"
{
#endif 

void timerInterruptHandler( void ); // forward declarations - function is implemented in a different file

#if defined( __cplusplus)
}
#endif 



// time/ memory access functions -------------------------------------------------------------------------------------------

void delayInMicroseconds ( uint32_t wait )
{
  delayMicroseconds( wait );
}

uint32_t getSysTick ( )
{
  return micros( );
}

// due to the limited RAM of the Arduino Uno all constants are written to flash, therefore a special access method
// is need to read bytes from flash instead of from RAM. (The RAM address range is in a different location.)
uint8_t readProgramMemoryByte ( const uint8_t * ptr )
{
  uint32_t address = (uint32_t)ptr;
  return pgm_read_byte( address );          
}


// ---------------------------------- timer functions ---------------------------------------------
// timer functions - on Uno use the watchdog which can wakeup the arduino uno from power-down state ---------------------

// to keep the arduino uno specific macros in the shim file, the redirection of the ISR to the timerInterruptHandler function which
// is part of tmf8806_app.c
ISR(WDT_vect)
{
  timerInterruptHandler( );
}

// Arduino Uno specific defines for the Watchdog
#define WDP210_MASKED(i)       ( (i) & ((1<<(WDP2+1))-1) )
#define WDP3_MOVED(i)          ( ((i) & (1<<(WDP2+1))) << (WDP3-WDP2-1) )
#define WDT_LIMIT(t)           ( (t) > (16<<9) ? (16<<9) : (t) )          // watchdog can only have these values: 16, 16<<1, 16<<2, .., 16<<9 == 8192 ms

/* function converts the time given to the next equal or bigger timeout the 
   Arduino Uno Watchdog can handle. Values for the ARudino Uno watchdog are:
   16ms, 32ms, 64ms, 128ms, 256ms, 512ms, 1024ms=~1s, 2048ms=~2s, 4096ms=~4s,
   8192ms=~8s. Everything bigger is set to 8192ms. 
   The function then returns the bitfield that is needed for the WD3,WD2,WD1,WD0
   */
static uint8_t timeToWdtIndex( uint16_t timerInMs )
{
  uint8_t i = 0;                                
  uint16_t timerValue = 16;                     // start with the minimum time to wait
  timerInMs = WDT_LIMIT(timerInMs);             // if given time value is greater than 8192, then set it to 8192
  while ( timerValue < timerInMs )              // the Arduino Uno WDT has 10 settings for timeout 0 .. 9
  {
    timerValue *= 2;                            // the timerValue is a power of 2 so we go through them  
    i++;                                        // this is the binary representation of the index: WDP3 WDP2 WDP1 WDP0 (bits)
  }
  i = WDP3_MOVED(i) + WDP210_MASKED(i);         // WDP3 is actually bit 5, make room for bits 3 and 4
  return i;
}

void timerStartPeriodic ( void * dptr, uint16_t timeInMs )
{
  uint8_t r = timeToWdtIndex( timeInMs );
  (void)dptr;
  disableInterrupts();
  WDTCSR = (1 << WDCE) | (1 << WDE); // Watchdog Change Enable + Watchdog enable - both are needed to change the prescaler
  WDTCSR = r;                        // set watchdog prescaler and clear WDCE and clear WDE
  WDTCSR |= (1 << WDIE);             // enable interrupt for watchdog - can be set without Watchdog enabled (=WDE)
  enableInterrupts();
}

void timerStop ( void * dptr )
{
  disableInterrupts();
  WDTCSR = 0 ;                      // Watchdog off (prescaler remains untouched)
  enableInterrupts();
  (void)dptr;
}


// Power down function ---------------------------------------------------------------------------------------------------

void powerDown ( void * dptr )
{
  SMCR = (1<<SM1) | (1<<SE) ;       // setting SM2..0 to value b010 and the SE = Sleep enable bit
  __asm__ __volatile__ ("sleep");   // execute a sleep instruction - there is no C-equivalent to using assembly in this case
}


// interrupt related functions --------------------------------------------------------------------------------

void setInterruptHandler( void (* handler)( void ) )
{
  attachInterrupt( digitalPinToInterrupt( INTERRUPT_PIN ), handler, FALLING );
}

void disableInterruptHandler( uint8_t pin )
{
  detachInterrupt( digitalPinToInterrupt( pin ) );
}

void disableInterrupts ( void )
{
  noInterrupts( );
}

void enableInterrupts ( void )
{
  interrupts( );
}


// pin control functions ------------------------------------------------------------------------------------------------

void pinOutput ( void * dptr, uint8_t pin )
{
  (void)dptr; // not used here
  pinMode( pin, OUTPUT );      /* define a pin as output */
}

void pinInput ( void * dptr, uint8_t pin )
{ 
  (void)dptr; // not used here
  pinMode( pin, INPUT );      /* define a pin as input */
}

void enablePinHigh ( void * dptr )
{
  (void)dptr; // not used here
  digitalWrite( ENABLE_PIN, HIGH );  
}

void enablePinLow ( void * dptr )
{
  (void)dptr; // not used here
  digitalWrite( ENABLE_PIN, LOW );   
}

void resultPinHigh ( void * dptr )
{
  (void)dptr; // not used here
  digitalWrite( RESULT_PIN, HIGH );  
}

void resultPinLow ( void * dptr )
{
  (void)dptr; // not used here
  digitalWrite( RESULT_PIN, LOW );   
}

void ledPinHigh ( void * dptr )
{
  (void)dptr; // not used here
  digitalWrite( LED_PIN, HIGH );  
}

void ledPinLow ( void * dptr )
{
  (void)dptr; // not used here
  digitalWrite( LED_PIN, LOW );   
}


// printing functions ------------------------------------------------------------------------------------------------

void uartOpen ( uint32_t baudrate )
{
  Serial.end( );                                     // this clears any old pending data 
  Serial.begin( baudrate );
}

void uartClose ( )
{
  Serial.end( );
}

void printConstStr ( const char * str )
{
  /* casting back to Arduino specific memory */
  Serial.print( reinterpret_cast<const __FlashStringHelper *>( str ) );
}

void printChar ( char c ) 
{
  Serial.print( c );
}

void printInt ( int32_t i )
{
  Serial.print( i, DEC );
}

void printUint ( uint32_t i )
{
  Serial.print( i, DEC );
}

void printUintHex ( uint32_t i )
{
  Serial.print( i, HEX );
}

void printStr ( char * str )
{
  Serial.print( str );    // use only for printing zero-terminated strings: (const char *)
}

void printLn ( void )
{
  Serial.print( '\n' );
  Serial.flush( );
}


// I2C functions ---------------------------------------------------------------------------------------------------

void i2cOpen ( void * dptr, uint32_t i2cClockSpeedInHz )
{
  (void)dptr; // not used here
  Wire.begin( );
  Wire.setClock( i2cClockSpeedInHz );
}

void i2cClose ( void * dptr )
{
  (void)dptr; // not used here
  Wire.end( );
}

static int8_t i2cTxOnly ( uint8_t logLevel, uint8_t slaveAddr, uint8_t regAddr, uint16_t toTx, const uint8_t * txData )
{  // split long transfers into max of 32-bytes: 1 byte is register address, up to 31 are payload.
  int8_t res = I2C_SUCCESS;
  do 
  {
    uint8_t tx;
    if ( toTx > ARDUINO_MAX_I2C_TRANSFER - 1) 
    {
      tx = ARDUINO_MAX_I2C_TRANSFER - 1;
    }
    else 
    {
      tx = toTx; // less than 31 bytes 
    }
    if ( logLevel & TMF8806_LOG_LEVEL_I2C ) 
    {
      PRINT_STR( "I2C-TX (0x" );
      PRINT_UINT_HEX( slaveAddr );
      PRINT_STR( ")" );
      PRINT_STR( " tx=" );
      PRINT_INT( tx+1 );          // +1 for regAddr
      PRINT_STR( " 0x" );
      PRINT_UINT_HEX( regAddr );
      if ( logLevel >= TMF8806_LOG_LEVEL_DEBUG ) 
      {
        uint8_t dumpTx = tx;
        const uint8_t * dump = txData;
        while ( dumpTx-- )
        {
          PRINT_STR( " 0x" );
          PRINT_UINT_HEX( *dump );
          dump++;
        }
      }
      PRINT_LN( );
    }

    Wire.beginTransmission( slaveAddr );
    Wire.write( regAddr );
    if ( tx )
    {
      Wire.write( txData, tx );
    }
    toTx -= tx;
    txData += tx;
    regAddr += tx;
    res = Wire.endTransmission( );
  } while ( toTx && res == I2C_SUCCESS );
  return I2C_SUCCESS;
}

static int8_t i2cRxOnly ( uint8_t logLevel, uint8_t slaveAddr, uint16_t toRx, uint8_t * rxData )
{   // split long transfers into max of 32-bytes
  uint8_t expected = 0;
  uint8_t rx = 0;
  int8_t res = I2C_SUCCESS;
  do 
  {
    uint8_t * dump = rxData; // in case we dump on uart, we need the pointer
    if ( toRx > ARDUINO_MAX_I2C_TRANSFER ) 
    {
      expected = ARDUINO_MAX_I2C_TRANSFER;
    }
    else 
    {
      expected = toRx; // less than 32 bytes 
    }
    Wire.requestFrom( slaveAddr, expected );
    rx = 0;
    while ( Wire.available() ) 
    {  // read in all available bytes
      *rxData = Wire.read();
      rxData++;
      toRx--;
      rx++;
    }
    if ( logLevel & TMF8806_LOG_LEVEL_I2C ) 
    {
      PRINT_STR( "I2C-RX (0x" );
      PRINT_UINT_HEX( slaveAddr );
      PRINT_STR( ")" );
      PRINT_STR( " toRx=" );
      PRINT_INT( rx );
      if ( logLevel >= TMF8806_LOG_LEVEL_DEBUG ) 
      {
        uint8_t dumpRx = rx;
        while ( dumpRx-- )
        {
          PRINT_STR( " 0x" );
          PRINT_UINT_HEX( *dump );
          dump++;
        }
      }
      PRINT_LN( );
    }
  } while ( toRx && expected == rx );
  if ( toRx || expected != rx )
  {
    res = I2C_ERR_TIMEOUT;
  }
  return res;
}

int8_t i2cTxReg ( void * dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toTx, const uint8_t * txData )
{  // split long transfers into max of 32-bytes
  tmf8806Driver * driver = (tmf8806Driver *)dptr;
  return i2cTxOnly( driver->logLevel, slaveAddr, regAddr, toTx, txData ); 
}

int8_t i2cRxReg ( void * dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toRx, uint8_t * rxData )
{   // split long transfers into max of 32-bytes
  tmf8806Driver * driver = (tmf8806Driver *)dptr;
  int8_t res = i2cTxOnly( driver->logLevel, slaveAddr, regAddr, 0, 0 ); 
  if ( res == I2C_SUCCESS )
  {
    res = i2cRxOnly( driver->logLevel, slaveAddr, toRx, rxData );
  }
  return res;
}

int8_t i2cTxRx ( void * dptr, uint8_t slaveAddr, uint16_t toTx, const uint8_t * txData, uint16_t toRx, uint8_t * rxData )
{
  tmf8806Driver * driver = (tmf8806Driver *)dptr;
  int8_t res = I2C_SUCCESS;
  if ( toTx )
  {
    res = i2cTxOnly( driver->logLevel, slaveAddr, *txData, toTx-1, txData+1 );
  }
  if ( toRx && res == I2C_SUCCESS )
  {
    res = i2cRxOnly( driver->logLevel, slaveAddr, toRx, rxData );
  }
  return res;
}


// read-out functions that require special treatment because the arduino uno has so little memory ---------------------------------------------------

#if ( ( 128 / ARDUINO_MAX_I2C_TRANSFER ) * ARDUINO_MAX_I2C_TRANSFER != 128 )
    #error "ARDUINO_MAX_I2C_TRANSFER size must be a divider of 128"
#endif

int8_t tmf8806ReadQuadHistogram ( void * dptr, uint8_t slaveAddr, uint8_t * buffer )
{
  int8_t res = I2C_SUCCESS;
  // need to read reverse the histogram as a read of register 0x30 will lead to an i2c bank
  // switch at the tmf8806, and mess up the histograms
  for ( int8_t i = 128 - ARDUINO_MAX_I2C_TRANSFER; i >= 0 && res == I2C_SUCCESS; i -= ARDUINO_MAX_I2C_TRANSFER )
  {
    res = i2cRxReg( dptr, slaveAddr, TMF8806_COM_RESULT_NUMBER + i, ARDUINO_MAX_I2C_TRANSFER, buffer+i );
  }
  return res;
}

void tmf8806ScaleAndPrintHistogram ( void * dptr, uint8_t histType, uint8_t id, uint8_t * data, uint8_t scale )
{
  (void)dptr;                               // unused in this implementation
  switch ( histType )
  {
    case TMF8806_DIAG_HIST_ALG_PILEUP:      // Pileup histogram
      { // pileup == 4x 1histogram, shift bin-content by 1
        PRINT_CONST_STR( F( "#TGPUC" ) );
      }
      break;
    case TMF8806_DIAG_HIST_ALG_PU_TDC_SUM:      // SUM histogram
      { // sum = 1x histogram, shift bin-content by 2
        PRINT_CONST_STR( F( "#SUM" ) );
      }
      break;
    case TMF8806_DIAG_HIST_ELECTRICAL_CAL:
      { // 5 histograms
        PRINT_CONST_STR( F( "#CI" ) );
      }
      break;
    case TMF8806_DIAG_HIST_PROXIMITY:
      { // 5 histograms
        PRINT_CONST_STR( F( "#PT" ) );
      }
      break;
    case TMF8806_DIAG_HIST_DISTANCE:
      { // 5 histograms
        PRINT_CONST_STR( F( "#TG" ) );
      }
      break;
    default:
      {
        PRINT_CONST_STR( F( "#UNKNOWN" ) );
      }
      break;
  }
  PRINT_INT( (id&0x1F)/2 ); // TDC+Channel number
  for ( uint8_t i = 0; i < TMF8806_HISTOGRAM_BINS; i++, data += 2 )
  {    
    uint32_t value = tmf8806GetUint16(data);      // little endian
    value = value << scale;                             // print scaled histograms -> can become 32-bit
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( value );
  }
  PRINT_LN( );
}

