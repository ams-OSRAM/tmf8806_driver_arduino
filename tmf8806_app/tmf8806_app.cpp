/*****************************************************************************
* Copyright (c) [2024] ams-OSRAM AG                                          *
* All rights are reserved.                                                   *
*                                                                            *
* FOR FULL LICENSE TEXT SEE LICENSE.TXT                                      *
******************************************************************************/
 
//
// tmf8806 arduino uno driver example console application
//

// ---------------------------------------------- includes ----------------------------------------

#include "tmf8806_shim.h"
#include "tmf8806.h"
#include "tmf8806_regs.h"
#include "tmf8806_app.h"
#include "tmf8806_image.h"

// ---------------------------------------------- defines -----------------------------------------

// tmf states
#define TMF8806_STATE_DISABLED      0
#define TMF8806_STATE_STANDBY       1     
#define TMF8806_STATE_STOPPED       2
#define TMF8806_STATE_MEASURE       3
#define TMF8806_STATE_FACTORY_CALIB 4
#define TMF8806_STATE_ERROR         5

// constant that enables all histograms in the bitmask
#define TMF8806_DUMP_ALL_HISTOGRAM  ( (TMF8806_DUMP_HIST_ALG_PU_TDC_SUM<<1) -1)

// number of persistence configurations
#define TMF8806_NUMBER_PERS_CONFIG  3

// number of kilo iterations for factory calibration
#define TMF8806_FACTORY_CALIB_KITERS    4000

// alternate i2c address to be use
#define TMF8806_ALT_SLAVE_ADDR   0x42

// binary command identifiers
#define TMF8806_BINARY_CMD_FACTORY_CALIB 0x30 // sets arbitrary factory calibration (payload: tmf8806FactoryCalibData)
#define TMF8806_BINARY_CMD_CONFIGURE 0x31     // sets arbitrary configuration (payload: tmf8806MeasureCmd)
#define TMF8806_BINARY_CMD_THRESHOLDS 0x32    // sets arbitrary persistence and threshold
#define TMF8806_BINARY_CMD_CHAR_MODE 0x00     // not a valid command identifier, indicates that the application is in character input mode
#define TMF8806_BINARY_CMD_PENDING 0xFF       // not a valid command identifier, indicates that the application is in binary input mode awaiting a command identifier

// maximum binary command payload size
#define TMF8806_BINARY_BUF_SIZE 14


// ---------------------------------------------- constants -----------------------------------------

// log levels to change with keys +/-
const uint8_t logLevels[] = 
{ TMF8806_LOG_LEVEL_NONE
, TMF8806_LOG_LEVEL_ERROR
, TMF8806_LOG_LEVEL_CLK_CORRECTION
, TMF8806_LOG_LEVEL_INFO
, TMF8806_LOG_LEVEL_VERBOSE
, TMF8806_LOG_LEVEL_I2C
, TMF8806_LOG_LEVEL_DEBUG 
};

extern const tmf8806MeasureCmd defaultConfig;           // reuse as a 2nd configuration the default config

// alternate measurement configuration
const tmf8806MeasureCmd alternateConfig = 
{ .data = { .spreadSpecSpadChp = { .amplitude = 0
                                 , .config = 0
                                 , .reserved = 0
                                 }
          , .spreadSpecVcselChp = { .amplitude = 0
                                  , .config = 0
                                  , .singleEdgeMode = 0
                                  , .reserved = 0
                                  }
          , .data = { .factoryCal = 0                               // load factory calib
                    , .algState = 1                                 // load alg state
                    , .reserved = 0                                   
                    , .spadDeadTime = 0                             // 0 = 97ns, 4 = 16ns, 7 = 4ns
                    , .spadSelect = 0                               // all SPAD for prox
                    }
          , .algo = { .reserved0 = 0           
                    , .distanceEnabled = 0                          // prox only
                    , .vcselClkDiv2 = 0
                    , .distanceMode = 0                             // 0=2.5m
                    , .immediateInterrupt = 0
                    , .reserved = 0
                    , .algKeepReady = 0                             // 0 = power saving on
                    }
          , .gpio = { .gpio0 = 0
                    , .gpio1 = 0
                    }
          , .daxDelay100us = 0
          , .snr = { .threshold = 6
                   , .vcselClkSpreadSpecAmplitude = 0  
                   }
          , .repetitionPeriodMs = 0                                 // 0 == single shot mode
          , .kIters  = 100
          , .command = TMF8806_COM_CMD_STAT__cmd_measure
          }
};

const uint8_t appPersistence[ TMF8806_NUMBER_PERS_CONFIG ] =    { 0, 1,   5   };
const uint16_t appLowThreshold[ TMF8806_NUMBER_PERS_CONFIG ] =  { 0, 100, 200 };
const uint16_t appHighThreshold[ TMF8806_NUMBER_PERS_CONFIG ] = { 0, 200, 1500 };

// ---------------------------------------------- variables -----------------------------------------

tmf8806Driver tmf8806;            // instance of tmf8806
tmf8806DistanceResultFrame resultFrame; // a result frame
extern uint8_t logLevel;          // for i2c logging in shim 
int8_t stateTmf8806;              // current state of the device 
int8_t clkCorrectionOn;           // if non-zero clock correction is on
int8_t configNr;                  // which config to choose (default or alternate)
uint8_t dumpHistogramOn;          // histograms selected for dumping
uint8_t persConfigNr;             // which persistence config to choose
volatile uint8_t irqTriggered; 
uint8_t llIdx;                    // index for log verbosity
uint8_t binaryCmd;                // currently active binary command identifier (if any)
uint8_t binaryBufFill;            // fill level of the binary command payload buffer
uint8_t binaryBuf[TMF8806_BINARY_BUF_SIZE]; // binary command payload buffer


// ---------------------------------------------- function declaration ------------------------------

// Function checks the UART for received characters and interprets them
// returns 0 if exit key (='q') was not pressed
// returns 1 if exit key was pressed
int8_t serialInput( );

// Function to print the results in a kind of CSV like format 
// driver ... pointer to the tmf8828 driver structure 
// result ... result frame to be printed to uart
void tmf8806PrintResult( tmf8806Driver * driver, const tmf8806DistanceResultFrame * result );


// ---------------------------------------------- functions -----------------------------------------

// Print the current state (stateTmf8806) in a readable format
void printState ( )
{
  PRINT_CONST_STR( F( "state=" ) );
  switch ( stateTmf8806 )
  {
    case TMF8806_STATE_DISABLED: PRINT_CONST_STR( F( "disabled" ) ); break;
    case TMF8806_STATE_STANDBY: PRINT_CONST_STR( F( "standby" ) ); break;
    case TMF8806_STATE_STOPPED: PRINT_CONST_STR( F( "stopped" ) ); break;
    case TMF8806_STATE_MEASURE: PRINT_CONST_STR( F( "measure" ) ); break;
    case TMF8806_STATE_FACTORY_CALIB: PRINT_CONST_STR( F( "factcalib" ) ); break;
    case TMF8806_STATE_ERROR: PRINT_CONST_STR( F( "error" ) ); break;   
    default: PRINT_CONST_STR( F( "???" ) ); break;
  }
  PRINT_LN( );
}

// print registers either as c-struct or plain
void printRegisters ( uint8_t regAddr, uint16_t len, char seperator )
{
  uint8_t buf[8];
  uint16_t i;
  uint8_t j;
  for ( i = 0; i < len; i += 8 )            // if len is not a multiple of 8, we will print a bit more registers ....
  {
    uint8_t * ptr = buf;    
    i2cRxReg( &tmf8806, tmf8806.i2cSlaveAddress, regAddr, 8, buf );
    if ( seperator == ' ' )
    {
      PRINT_CONST_STR( F( "0x" ) );
      PRINT_UINT_HEX( regAddr );
      PRINT_CONST_STR( F( ": " ) );
    }
    for ( j = 0; j < 8; j++ )
    {
      PRINT_CONST_STR( F( " 0x" ) ); PRINT_UINT_HEX( *ptr++ ); PRINT_CHAR( seperator ); 
    }
    PRINT_LN( );
    regAddr = regAddr + 8;
  }
  if ( seperator == ',' )
  {
    PRINT_CONST_STR( F( "};" ) );
    PRINT_LN( );
  }
}

// Function prints a help screen
void printHelp ( )
{
  PRINT_CONST_STR( F( "TMF8806 Arduino Driver Version " ) );
  PRINT_INT( tmf8806.info.version[0] );
  PRINT_CHAR( '.' );
  PRINT_INT( tmf8806.info.version[1] );
  PRINT_LN( ); PRINT_CONST_STR( F( "a .. dump regs" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "c .. toggle cfg" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "d .. disable dev" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "e .. enable dev" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "f .. fact calib" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "h .. help" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "i .. i2c addr change" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "m .. measure" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "p .. power down" ) );
#ifndef USE_ROM_FIRMWARE
  PRINT_LN( ); PRINT_CONST_STR( F( "r .. remote control mode" ) );
#endif  
  PRINT_LN( ); PRINT_CONST_STR( F( "s .. stop measure" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "t .. set pers+thresholds" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "w .. wakeup" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "x .. clock corr on/off" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "z .. hist. dump" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "+ .. log+" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "- .. log-" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "b\\x30<tmf8806FactoryCalibData data> .. set arbitrary fact calib" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "b\\x31<tmf8806MeasureCmd data> .. set arbitrary cfg" ) );
  PRINT_LN( ); PRINT_CONST_STR( F( "b\\x32<pers,loThres,hiThres> .. set arbitrary persistence / threshold" ) );
  PRINT_LN( ); 
}

void printDeviceInfo ( )
{
  PRINT_CONST_STR( F( "App " ) );
  PRINT_INT( tmf8806.device.appVersion[0] );
  PRINT_CHAR( '.' );
  PRINT_INT( tmf8806.device.appVersion[1] );
  PRINT_CHAR( '.' );
  PRINT_INT( tmf8806.device.appVersion[2] );
  PRINT_CHAR( '.' );
  PRINT_INT( tmf8806.device.appVersion[3] );
  PRINT_LN( );
  PRINT_CONST_STR( F( "Chip " ) );
  PRINT_INT( tmf8806.device.chipVersion[0] );
  PRINT_CHAR( '.' );
  PRINT_INT( tmf8806.device.chipVersion[1] );
  PRINT_LN( );
  PRINT_CONST_STR( F( "SN 0x" ) );
  PRINT_UINT_HEX( tmf8806.device.deviceSerialNumber );
  PRINT_LN( );
}

// Results printing:
// #Obj,<i2c-addr>,<result_number>,<confidence>,<distance_mm>,<clk_corrected_distance_mm>,<systick>,<temperature>,<ref_cnt>,<target_cnt>,<xtalk>
void tmf8806PrintResult ( tmf8806Driver * driver, const tmf8806DistanceResultFrame * result )
{
  uint16_t distance;
  PRINT_CONST_STR( F( "#Obj" ) );
  PRINT_CHAR( SEPARATOR );
  PRINT_INT( driver->i2cSlaveAddress );
  PRINT_CHAR( SEPARATOR );
  PRINT_UINT( result->resultNum );
  PRINT_CHAR( SEPARATOR );
  PRINT_UINT( result->reliability );
  PRINT_CHAR( SEPARATOR );
  PRINT_UINT( result->distPeak );
  PRINT_CHAR( SEPARATOR );
  distance = tmf8806CorrectDistance( driver, result->distPeak );
  PRINT_UINT( distance );
  PRINT_CHAR( SEPARATOR );
  PRINT_UINT( result->sysClock );      
  PRINT_CHAR( SEPARATOR );    
  PRINT_INT( result->temperature );
  PRINT_CHAR( SEPARATOR );
  PRINT_UINT( result->referenceHits );
  PRINT_CHAR( SEPARATOR );
  PRINT_UINT( result->objectHits );
  PRINT_CHAR( SEPARATOR );
  PRINT_UINT( result->xtalk );
  PRINT_LN( );
}

void tmf8806PrintFactoryCalibration ( tmf8806Driver * driver )
{
  PRINT_CONST_STR(F("#Cal"));
  const uint8_t* const cal = (uint8_t*)&(driver->factoryCalib);
  for ( uint8_t b = 0; b < sizeof(tmf8806FactoryCalibData); ++b )
  {
    PRINT_CHAR( SEPARATOR );
    PRINT_CONST_STR(F("0x"));
    PRINT_UINT_HEX(cal[b]);
  }
  PRINT_LN();
}

void resetAppState ( )
{
  stateTmf8806 = TMF8806_STATE_DISABLED;
  clkCorrectionOn = 1;
  configNr = 0;         // default config
  dumpHistogramOn = 0;  // default no histogram dumping
  persConfigNr = 0;     // default state   
  irqTriggered = 0;
}

// ---------------------------- keyboard handler ------------------------------------------------------------------

void clockCorrection ( )
{
  clkCorrectionOn = !clkCorrectionOn;       // toggle clock correction on/off  
  tmf8806ClkCorrection( &tmf8806, clkCorrectionOn );
  PRINT_CONST_STR( F( "Clk corr is " ) );
  PRINT_INT( clkCorrectionOn );
  PRINT_LN( );
}

void configure ( )
{
  if ( stateTmf8806 == TMF8806_STATE_STOPPED )
  {
    configNr = !configNr;
    if ( configNr )
    {
        tmf8806SetConfiguration( &tmf8806, &alternateConfig );
        PRINT_CONST_STR( F( "Alt cfg" ) );
    }
    else 
    {
        tmf8806SetConfiguration( &tmf8806, &defaultConfig );
        PRINT_CONST_STR( F( "Dft cfg" ) );
    }
    PRINT_LN( ); 
  }
}

void disable ( )
{
  tmf8806Disable( &tmf8806 );
  stateTmf8806 = TMF8806_STATE_DISABLED;
}

void enable ( )
{
  if ( stateTmf8806 == TMF8806_STATE_DISABLED )
  {
    tmf8806Enable( &tmf8806 );
    delayInMicroseconds( ENABLE_TIME_MS * 1000 );
    tmf8806ClkCorrection( &tmf8806, clkCorrectionOn ); 
    tmf8806Wakeup( &tmf8806 );
    if ( tmf8806IsCpuReady( &tmf8806, CPU_READY_TIME_MS ) )
    {
#ifdef USE_ROM_FIRMWARE
      if ( tmf8806SwitchToRomApplication( &tmf8806 ) == BL_SUCCESS_OK )
#else      
      if ( tmf8806DownloadFirmware(&tmf8806,tmf8806_image_start, tmf8806_image, tmf8806_image_length) == BL_SUCCESS_OK )      
#endif      
      {
        resetAppState( );                         // set everything to default
        printDeviceInfo( );
        stateTmf8806 = TMF8806_STATE_STOPPED;     // change what differs from default
        configNr = 1;                             // will toggle to 0 -> default config in below function  
        configure();                              // set a configuration
        printHelp();                              // prints on UART usage and waits for user input on serial
        return;                                   // early return, no need to read device info again
      }
      else
      {
        PRINT_CONST_STR( F( "ERROR App Switch" ) );
        PRINT_LN( );
        stateTmf8806 = TMF8806_STATE_ERROR;
      }
    }
    else
    {
      stateTmf8806 = TMF8806_STATE_ERROR;
    }
  } // else device is already enabled
  tmf8806ReadDeviceInfo( &tmf8806 );
  printDeviceInfo( );
}

void factoryCalibration ( )
{
  if ( stateTmf8806 == TMF8806_STATE_STOPPED )
  {
    PRINT_CONST_STR( F( "Fact Cal" ) );
    PRINT_LN( );
    tmf8806ClrAndEnableInterrupts( &tmf8806, TMF8806_INTERRUPT_RESULT );   // use interrupt to wait for factory calibration to be completed
    if ( BL_SUCCESS_OK == tmf8806FactoryCalibration( &tmf8806, TMF8806_FACTORY_CALIB_KITERS ) )
    { // wait for factory calibration to be completed
      stateTmf8806 = TMF8806_STATE_FACTORY_CALIB;
    }
  }
}

void histogramDumping ( )
{
  if ( stateTmf8806 == TMF8806_STATE_STOPPED )
  {
    if ( dumpHistogramOn >= TMF8806_DUMP_ALL_HISTOGRAM )
    {
      dumpHistogramOn = 0; // is off again
    }
    else if ( !dumpHistogramOn )
    {
      dumpHistogramOn = TMF8806_DUMP_HIST_ELECTRICAL_CAL;
    }
    else
    {
      dumpHistogramOn = dumpHistogramOn << 1;       // select histogram type of histogram dumping
      if ( dumpHistogramOn > TMF8806_DUMP_HIST_ALG_PU_TDC_SUM )
      {
        dumpHistogramOn = TMF8806_DUMP_ALL_HISTOGRAM;  // set all bits 
      }
    }
    tmf8806ConfigureHistograms( &tmf8806, dumpHistogramOn );
    PRINT_CONST_STR( F( "Histogram is " ) );
    PRINT_INT( dumpHistogramOn );
    PRINT_LN( );
  }
}

void i2cAddressChange ( )
{
  if ( stateTmf8806 == TMF8806_STATE_STOPPED )
  {
    PRINT_CONST_STR( F( "Addr=0x" ) );
    if ( tmf8806.i2cSlaveAddress == TMF8806_SLAVE_ADDR )
    {
      tmf8806SetI2CSlaveAddress( &tmf8806, TMF8806_ALT_SLAVE_ADDR, 0x0, 0x0 ); // always change the i2c address
      PRINT_UINT_HEX(TMF8806_ALT_SLAVE_ADDR);
    }
    else
    {
      tmf8806SetI2CSlaveAddress( &tmf8806, TMF8806_SLAVE_ADDR, 0x0, 0x0 ); // always change the i2c address
      PRINT_UINT_HEX(TMF8806_SLAVE_ADDR);
    }
    PRINT_LN();
  }
}

void remoteControlMode ( )
{
  /* 0x0F -> use maximum VCSEL current for remote control mode */
  if ( ( stateTmf8806 == TMF8806_STATE_STOPPED ) && ( tmf8806StartRemoteControlMode( &tmf8806, 0x0F ) == APP_SUCCESS_OK ) )
  {
    stateTmf8806 = TMF8806_STATE_MEASURE;
    PRINT_CONST_STR( F( "RC mode on" ) );
    PRINT_LN();
  }
}

void logUp ( )
{
  llIdx++;
  if ( llIdx > sizeof( logLevels ) - 1 )
  {
    llIdx = sizeof( logLevels ) - 1;
  }
  tmf8806SetLogLevel( &tmf8806, logLevels[llIdx] );
  PRINT_CONST_STR( F( "Log=" ) );
  PRINT_INT( logLevels[llIdx] );
  PRINT_LN( );
}

void logDown ( )
{
  if ( llIdx > 0 )
  {
    llIdx--;
  }
  tmf8806SetLogLevel( &tmf8806, logLevels[llIdx] );
  PRINT_CONST_STR( F( "Log=" ) );
  PRINT_INT( logLevels[llIdx] );
  PRINT_LN( );
}

void measure ( )
{
  if ( stateTmf8806 == TMF8806_STATE_STOPPED )
  {
    tmf8806ClrAndEnableInterrupts( &tmf8806, TMF8806_INTERRUPT_RESULT | TMF8806_INTERRUPT_DIAGNOSTIC );   
    if ( BL_SUCCESS_OK == tmf8806StartMeasurement( &tmf8806 ) )
    {
      stateTmf8806 = TMF8806_STATE_MEASURE;
    }
  }
}

void powerDown ( )
{
  if ( stateTmf8806 == TMF8806_STATE_MEASURE )      // stop a measurement first
  {
    tmf8806StopMeasurement( &tmf8806 );
    tmf8806DisableAndClrInterrupts( &tmf8806, 0xFF );               // just disable all
    stateTmf8806 = TMF8806_STATE_STOPPED;
  }
  if ( stateTmf8806 == TMF8806_STATE_STOPPED )
  {
    tmf8806Standby( &tmf8806 );
    stateTmf8806 = TMF8806_STATE_STANDBY;
  }
}

void stop ( )
{
  if ( ( stateTmf8806 != TMF8806_STATE_DISABLED ) && ( stateTmf8806 != TMF8806_STATE_STANDBY ) )
  {
    tmf8806StopMeasurement( &tmf8806 );
    tmf8806DisableAndClrInterrupts( &tmf8806, 0xFF );               // just disable all
    if ( stateTmf8806 != TMF8806_STATE_ERROR )
    {
      stateTmf8806 = TMF8806_STATE_STOPPED;
    }
  }
}

void thresholds ( )
{
  if ( stateTmf8806 == TMF8806_STATE_STOPPED )
  {
    persConfigNr++;
    if ( persConfigNr >= TMF8806_NUMBER_PERS_CONFIG )
    {
      persConfigNr = 0;
    }
    tmf8806SetThresholds( &tmf8806, appPersistence[ persConfigNr ], appLowThreshold[ persConfigNr ], appHighThreshold[ persConfigNr ] );
    PRINT_CONST_STR( F( "Pers=" ) );
    PRINT_INT( appPersistence[ persConfigNr ] );
    PRINT_CONST_STR( F( " LowTh=" ) );
    PRINT_INT( appLowThreshold[ persConfigNr ] );
    PRINT_CONST_STR( F( " HighTh=" ) );
    PRINT_INT( appHighThreshold[ persConfigNr ] );
    PRINT_LN( );
  }
}

void wakeup ( )
{
  if ( stateTmf8806 == TMF8806_STATE_STANDBY )
  {
    tmf8806Wakeup( &tmf8806 );
    if ( tmf8806IsCpuReady( &tmf8806, CPU_READY_TIME_MS ) )
    {
      stateTmf8806 = TMF8806_STATE_STOPPED;
    }
    else
    {
      stateTmf8806 = TMF8806_STATE_ERROR;
    }
  }
}

// returns the expected binary command payload size if a valid identifier is passed, -1 if the identifier is invalid
int8_t binaryCmdPayloadSize( uint8_t cmd ) {
  if ( cmd == TMF8806_BINARY_CMD_FACTORY_CALIB )
  {
    return sizeof( tmf8806FactoryCalibData );
  }
  else if ( cmd == TMF8806_BINARY_CMD_CONFIGURE )
  {
    return sizeof( tmf8806MeasureCmd );
  }
  else if ( cmd == TMF8806_BINARY_CMD_THRESHOLDS )
  { // persistence, low threshold, high threshold       
    return sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t);
  }
  else
  {
    return -1;
  }
}

// enters binary input mode
void enterBinaryInputMode ( )
{
  binaryCmd = TMF8806_BINARY_CMD_PENDING;
  binaryBufFill = 0;
  PRINT_CONST_STR( F( "Binary input mode active" ) );
  PRINT_LN( );
}

// leaves binary input mode
void exitBinaryInputMode ( )
{
  binaryCmd = TMF8806_BINARY_CMD_CHAR_MODE;
  binaryBufFill = 0;
  PRINT_CONST_STR( F( "Binary input mode inactive" ) );
  PRINT_LN( );
  printState( );
}

// returns 1 if in binary input mode, 0 if in character input mode
int8_t isInBinaryInputMode ( )
{
  if ( binaryCmd == TMF8806_BINARY_CMD_CHAR_MODE ) {
    return 0;
  } else {
    return 1;
  }
}

// handles a received binary command with payload of the expected size, returns 1 if program termination is requested
int8_t handleCompleteBinaryCmd ( )
{
  if ( stateTmf8806 == TMF8806_STATE_STOPPED && binaryCmd == TMF8806_BINARY_CMD_FACTORY_CALIB && binaryBufFill == sizeof( tmf8806FactoryCalibData ) )
  {
    tmf8806SetFactoryCalibration( &tmf8806, ( tmf8806FactoryCalibData * ) binaryBuf );
    PRINT_CONST_STR( F( "Factory calib set" ) );
    PRINT_LN( );
  }
  else if ( stateTmf8806 == TMF8806_STATE_STOPPED && binaryCmd == TMF8806_BINARY_CMD_CONFIGURE && binaryBufFill == sizeof( tmf8806MeasureCmd ) )
  {
    tmf8806SetConfiguration( &tmf8806, ( tmf8806MeasureCmd * ) binaryBuf );
    PRINT_CONST_STR( F( "Custom cfg" ) );
    PRINT_LN( );
    configNr = 1; // will switch to the default config on reception of the config toggle character command
  } 
  else if ( stateTmf8806 == TMF8806_STATE_STOPPED && binaryCmd == TMF8806_BINARY_CMD_THRESHOLDS && binaryBufFill == binaryCmdPayloadSize( TMF8806_BINARY_CMD_THRESHOLDS ) ) 
  { 
    const uint8_t pers = *binaryBuf;
    const uint16_t loT = tmf8806GetUint16( binaryBuf + 1 );
    const uint16_t hiT = tmf8806GetUint16( binaryBuf + 3 );
    tmf8806SetThresholds( &tmf8806, pers, loT, hiT );
    PRINT_CONST_STR( F( "Custom thresholds: " ) );
    PRINT_UINT(pers);
    PRINT_CHAR( SEPARATOR );
    PRINT_UINT(loT);
    PRINT_CHAR( SEPARATOR );
    PRINT_UINT(hiT);
    PRINT_LN( );
  }

  return 0;
}

// handles a single incoming byte in binary input mode, returns 1 if program termination is requested
int8_t handleBinaryInput ( uint8_t byte )
{
  if ( binaryCmd == TMF8806_BINARY_CMD_PENDING )
  {
    if ( binaryCmdPayloadSize( byte ) == -1 ) // function returns -1 if command identifier is invalid
    {
      PRINT_CONST_STR( F( "#Err,BinaryCmd," ) );
      PRINT_UINT_HEX( byte );
      PRINT_LN( );
      exitBinaryInputMode( );
    }
    else
    {
      binaryCmd = byte;
    }
  }
  else
  {
    if ( binaryBufFill >= TMF8806_BINARY_BUF_SIZE ) // prevent buffer overflow (this check is not neccessary when TMF8806_BINARY_BUF_SIZE is correctly set to the maximum payload size of all binary commands)
    {
      PRINT_CONST_STR( F( "#Err,BinaryBuf" ) );
      PRINT_LN( );
      exitBinaryInputMode( );
    }
    else
    {
      binaryBuf[binaryBufFill++] = byte;

      if ( binaryBufFill == binaryCmdPayloadSize( binaryCmd ) ) {
        int8_t res = handleCompleteBinaryCmd( ); // handle binary command when expected payload size has been reached
        exitBinaryInputMode( );
        return res;
      }
    }
  }

  return 0;
}

// handles a single incoming character in character input mode, returns 1 if program termination is requested
int8_t handleCharInput ( char key )
{
  if ( key < 33 || key >= 126 ) // skip all control characters and DEL  
  {
    return 0; // nothing to do here
  }
  else
  { 
    if ( key == 'a' )       // all registers shall be printed
    {  
      printRegisters( 0x00, 256, ' ' );  
    }
    else if ( key == 'c' )       // configure
    {  
      configure( );
    }
    else if ( key == 'd' )       // disable
    {  
      disable( );
    }
    else if ( key == 'e' ) // enable
    {  
      enable( );
    }
    else if ( key == 'f' )       // factory calibration shall be run
    {  
      factoryCalibration( );
    }
    else if ( key == 'h' )       // print help
    {
      printHelp(); 
    }
    else if ( key == 'i' )       // change the i2c slave address
    {
      i2cAddressChange( );
    }
    else if ( key == 'm' )       // measure
    {  
      measure( );
    }
    else if ( key == 'p' )       // power down
    {
      powerDown( );
    }
    else if ( key == 'q' )       // quit program
    {
      return 1;                  // fast exit -> want to terminate programm (if possible on the platform)
    }
#ifndef USE_ROM_FIRMWARE         
    // remote control mode needs firmware patch
    else if ( key == 'r' )       // start remote control mode
    {
      remoteControlMode();                 
    }
#endif    
    else if ( key == 's' )       // stop measure
    {
      stop( );
    }
    else if ( key == 't' )       // set thresholds
    {
      thresholds( );
    }
    else if ( key == 'w' )       // wakeup
    {
      wakeup( );
    }
    else if ( key == 'x' )       // toggle clock correction on/off
    {
      clockCorrection( );
    }
    else if ( key == 'z' )       // select which histograms shall be dumped - or none
    {
      histogramDumping( );
    }      
    else if ( key == '+' )       // increase printing
    {  
      logUp( );
    }
    else if ( key == '-' )       // decrease printing
    {  
      logDown( );
    }
    else if ( key == 'b' )       // binary mode
    {  
      enterBinaryInputMode( );
    }
    else 
    {
      PRINT_CONST_STR( F( "#Err" ) );
      PRINT_CHAR( SEPARATOR );
      PRINT_CONST_STR( F( "Cmd" ) );
      PRINT_CHAR( SEPARATOR );
      PRINT_CHAR( key );
      PRINT_LN( );
    }
    printState( );
    return 0;
  }
}

// Function checks the UART for received characters and interprets them, returns 1 if program termination is requested
int8_t serialInput ( )
{
  char rx;
  int8_t read = inputGetKey( &rx );
  while ( read ) {
    int8_t res;
    if ( isInBinaryInputMode( ) )
    {
      res = handleBinaryInput( ( uint8_t ) rx );
    }
    else
    {
      res = handleCharInput( rx );
    }
    if ( res != 0 ) {
      return res;
    }

    read = inputGetKey( &rx );
  }
  return 0;     // rx must be 0 to leave while loop
}


void interruptHandler ( void )
{
  irqTriggered = 1;
}

// -------------------------------------------------------------------------------------------------------------


// Arduino setup function is only called once at startup. Do all the HW initialisation stuff here.
void setupFn ( uint8_t logLevelIdx, uint32_t baudrate, uint32_t i2cClockSpeedInHz )
{
  if ( logLevelIdx >= sizeof( logLevels ) )
  {
    logLevelIdx = sizeof( logLevels ) - 1;
  }
  llIdx = logLevelIdx;                          // this variable is reset only with this function not with FW-dwnl

  // configure ENABLE pin and interupt pin
  pinOutput( ENABLE_PIN );
  pinInput( INTERRUPT_PIN );
  enablePinLow( &tmf8806 );

  // start serial and i2c
  inputOpen( baudrate );
  i2cOpen( &tmf8806, i2cClockSpeedInHz );

  resetAppState( );     // reset application local variables 
  tmf8806Initialise( &tmf8806, logLevels[ llIdx ] );     // reset driver
  setInterruptHandler( interruptHandler );

  tmf8806Disable( &tmf8806 );                        // this resets the I2C address in the device
  delayInMicroseconds(CAP_DISCHARGE_TIME_MS * 1000); // wait for a proper discharge of the cap
  printHelp();
}

// Arduino main loop function, is executed cyclic
int8_t loopFn ( )
{
  int8_t res = APP_SUCCESS_OK;
  uint8_t intStatus = 0;
  int8_t exit = serialInput();                                                            // handle any keystrokes from UART

#if ( defined( USE_INTERRUPT_TO_TRIGGER_READ ) && (USE_INTERRUPT_TO_TRIGGER_READ != 0) )
  if ( irqTriggered )
#else
  if ( stateTmf8806 == TMF8806_STATE_MEASURE || stateTmf8806 == TMF8806_STATE_FACTORY_CALIB )
#endif
  { 
    disableInterrupts( );
    irqTriggered = 0;
    enableInterrupts( );
    intStatus = tmf8806GetAndClrInterrupts( &tmf8806, TMF8806_INTERRUPT_RESULT | TMF8806_INTERRUPT_DIAGNOSTIC );   
    if ( intStatus & TMF8806_INTERRUPT_RESULT )                      // check if a result is available 
    {
      if ( stateTmf8806 == TMF8806_STATE_FACTORY_CALIB )
      {
        res = tmf8806ReadFactoryCalibration( &tmf8806 );      // read back the factory calibration and store it in driver 
        if ( res == APP_SUCCESS_OK )
        {
          tmf8806PrintFactoryCalibration( &tmf8806 );
          tmf8806DisableAndClrInterrupts( &tmf8806, TMF8806_INTERRUPT_RESULT | TMF8806_INTERRUPT_DIAGNOSTIC );
          stateTmf8806 = TMF8806_STATE_STOPPED;
          printState(); 
        }
      }
      else
      {
        res = tmf8806ReadResult( &tmf8806, &resultFrame );
        if ( res == APP_SUCCESS_OK )
        {
          tmf8806PrintResult( &tmf8806, &resultFrame );
          if ( tmf8806.measureConfig.data.repetitionPeriodMs == 0 )          // single shot, host state-machine should enter state stopped
          {
            tmf8806DisableAndClrInterrupts( &tmf8806, TMF8806_INTERRUPT_RESULT | TMF8806_INTERRUPT_DIAGNOSTIC );
            stateTmf8806 = TMF8806_STATE_STOPPED;
            printState(); 
          }
        }
      }
    }
    if ( res == APP_SUCCESS_OK )
    { 
      if ( intStatus & TMF8806_INTERRUPT_DIAGNOSTIC )                      // check if a histogram is available     
      {
        res = tmf8806ReadHistograms( &tmf8806 );
      }
    }
  }

  if ( res != APP_SUCCESS_OK )                         // in case that fails there is some error in programming or on the device, this should not happen
  {
    tmf8806StopMeasurement( &tmf8806 );
    tmf8806DisableAndClrInterrupts( &tmf8806, 0xFF );
    stateTmf8806 = TMF8806_STATE_STOPPED;
    PRINT_CONST_STR( F( "#Err" ) );
    PRINT_CHAR( SEPARATOR );
    PRINT_CONST_STR( F( "inter" ) );
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( intStatus );
    PRINT_CHAR( SEPARATOR );
    PRINT_CONST_STR( F( "but no/wrong data" ) );
    PRINT_LN( );
  }

  return !exit;    // 1 == loop again, 0 == exit
}

// Arduino has no terminate function but PC has.
void terminateFn ( )
{
  tmf8806Disable( &tmf8806 );
  clrInterruptHandler( );

  i2cClose( &tmf8806 );
  inputClose( );
}

