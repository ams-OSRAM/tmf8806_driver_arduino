/*
 *****************************************************************************
 * Copyright by ams OSRAM AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */

#include "tmf8806_shim.h"
#include "tmf8806.h"
#include "Arduino.h"
#include <time.h>
#include <Wire.h>


void delayInMicroseconds ( uint32_t wait )
{
  delayMicroseconds( wait );
}

uint32_t getSysTick ( )
{
  return micros( );
}

uint8_t readProgramMemoryByte ( const uint8_t * ptr )
{
  uint32_t address = (uint32_t)ptr;
  return pgm_read_byte( address );
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
}


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

void inputOpen ( uint32_t baudrate )
{
  Serial.end( );                                     // this clears any old pending data 
  Serial.begin( baudrate );
}

void inputClose ( )
{
  Serial.end( );
}

int8_t inputGetKey ( char *c )
{
  if ( Serial.available() )
  {
    *c = Serial.read();
    return 1;
  }
  return 0;
}

void printConstStr ( const char * str )
{
  /* casting back to Arduino specific memory */
  Serial.print( reinterpret_cast<const __FlashStringHelper *>( str ) );
}

void pinOutput ( uint8_t pin )
{
  pinMode( pin, OUTPUT );      /* define a pin as output */
}

void pinInput ( uint8_t pin )
{ 
  pinMode( pin, INPUT );      /* define a pin as input */
}

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

