/*****************************************************************************
* Copyright (c) [2024] ams-OSRAM AG                                          *
* All rights are reserved.                                                   *
*                                                                            *
* FOR FULL LICENSE TEXT SEE LICENSE.TXT                                      *
******************************************************************************/
 
//
// tmf8806 arduino uno driver low power example
//

// ---------------------------------------------- includes ----------------------------------------

#include "tmf8806_shim.h"
#include "tmf8806.h"
#include "tmf8806_app.h"

// ---------------------------------------------- defines -----------------------------------------

#define APP_ERR_CPU_NOT_READY                 (-10) // CPU could not get ready. Try power cycling, if issue reamins the device is probably broken
#define APP_ERR_COULD_NOT_SWITCH_TO_ROM_APP   (-11) // CPU could not start ROM application. Try power cycling, if issue reamins the device is probably broken


#define NR_MEASUREMENTS                       ( PERSISTENCE > 0 ? PERSISTENCE : 1 )     /* arrays must have a size > 0 */


// ---------------------------------------------- constants -----------------------------------------

// measurement configuration
const tmf8806MeasureCmd config = 
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
                    , .spadDeadTime = 4                             // 0 = 97ns, 4 = 16ns, 7 = 4ns
                    , .spadSelect = 0                               // all SPAD for prox
                    }
          , .algo = { .reserved0 = 0           
                    , .distanceEnabled = DISTANCE_ENABLED           // prox only or distance too
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
          , .kIters  = KILO_ITERATIONS
          , .command = TMF8806_COM_CMD_STAT__cmd_measure
          }
};


// ---------------------------------------------- variables -----------------------------------------

tmf8806Driver tmf8806;                  // instance of tmf8806 driver
tmf8806DistanceResultFrame resultFrame; // a result frame
tmf8806StateData stateData;             // keep a copy of the last state data (to avoid unnecessary BDV search) 

volatile int8_t resultIrqTriggered;     // result interrupt has been triggered, read result
volatile int8_t timerIrqTriggered;      // timer interrupt has been triggered, start a measurement
uint8_t tmf8806IsOff;                   // disabled then true and can start new measurment, else on and cannot start new measurement
uint16_t timerPeriod;                   // period with which single shot measurements are done

// data for averaging result distances
uint8_t idx;                            // index for averaging measurements
uint16_t distances[ NR_MEASUREMENTS ];  // queue of distances for fast sum calculation
uint8_t snrs[ NR_MEASUREMENTS ];        // queue of snrs for for fast sum calculation
uint32_t sumDistance;                   // the sum of all distances in the queue
uint16_t sumSnr;                        // the sum of all snrs in the queue
int8_t queueIsFull;                     // is set to 1 as soon as the queue gets full == average can be calcualted


// ---------------------------------------------- function declaration ------------------------------

// Function to print information about device and driver 
void printDeviceInfo( );

// Function to print the results in a kind of CSV like format 
// driver ... pointer to the tmf8828 driver structure 
// result ... result frame to be printed to uart
void tmf8806PrintResult( tmf8806Driver * driver, const tmf8806DistanceResultFrame * result );

// Function to calculate and print the averaged result in CSV like format
void printAveragedResult( );

// Function to debug queue issues - not needed for regular reporting
void printQueue( );


// Function to reset the averaging arrays, indices and sums
void resetAverages( );

// Function to insert a result in the array of results for average calculation
// distance ... the distance in mm
// snr ........ the SNR (0..63)
void insertResult( uint16_t distance, uint8_t snr );


// Function starts a single shot measurement 
void turnOnAndStartMeasure( );

// Function reads a result record, inserts the distance and snr in array, calculates average if 
// enough elements are in array.
// powers down tmf8806.
void readResultAndPowerDown( );


// Function is called when a result interrupt is received. Will wake-up the MCU if it was powered down.
void resultInterruptHandler( void );

// Function is called when a timer interrupt is received. Will wake-up the MCU if it was powered down.
void timerInterruptHandler( void ); 


// --------------------------------------- print functions -----------------------------------------

void printDeviceInfo ( )
{
  PRINT_STR( "App " );
  PRINT_INT( tmf8806.device.appVersion[0] );
  PRINT_CHAR( '.' );
  PRINT_INT( tmf8806.device.appVersion[1] );
  PRINT_CHAR( '.' );
  PRINT_INT( tmf8806.device.appVersion[2] );
  PRINT_CHAR( '.' );
  PRINT_INT( tmf8806.device.appVersion[3] );
  PRINT_LN( );
  PRINT_STR( "Chip " );
  PRINT_INT( tmf8806.device.chipVersion[0] );
  PRINT_CHAR( '.' );
  PRINT_INT( tmf8806.device.chipVersion[1] );
  PRINT_LN( );
  PRINT_STR( "SN 0x" );
  PRINT_UINT_HEX( tmf8806.device.deviceSerialNumber );
  PRINT_LN( );
}


// Results printing:
// #Obj,<i2c-addr>,<result_number>,<confidence>,<distance_mm>,<clk_corrected_distance_mm>,<systick>,<temperature>,<ref_cnt>,<target_cnt>,<xtalk>
void tmf8806PrintResult ( tmf8806Driver * driver, const tmf8806DistanceResultFrame * result )
{
  uint16_t distance;
  PRINT_STR( "#Obj" );
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

// Results printing:
// #Avr,<time_in_ms>,<confidence>,<distance_mm>
void printAveragedResult ( )
{
  uint32_t tickMs = getSysTick() / 1000;
  uint16_t distance = sumDistance / NR_MEASUREMENTS;
  uint8_t snr = sumSnr / NR_MEASUREMENTS;
  PRINT_STR( "#Avr" );
  PRINT_CHAR( SEPARATOR );
  PRINT_UINT( tickMs );
  PRINT_CHAR( SEPARATOR );
  PRINT_UINT( snr );
  PRINT_CHAR( SEPARATOR );
  PRINT_UINT( distance );
  PRINT_LN( );
}


// Debug function, queue printing:
// #Que,<sumDistance>,<sumSnr>,<idx>,<queueIsFull>,<distance_mm>,<snr>,<distance_mm>,<snr>,<distance_mm>,<snr>
void printQueue ( )
{
  uint8_t i;
  PRINT_STR( "#Que" );
  PRINT_CHAR( SEPARATOR );
  PRINT_UINT( sumDistance );
  PRINT_CHAR( SEPARATOR );
  PRINT_UINT( sumSnr );
  PRINT_CHAR( SEPARATOR );
  PRINT_UINT( idx );
  PRINT_CHAR( SEPARATOR );
  PRINT_UINT( queueIsFull );
  for ( i = 0; i < NR_MEASUREMENTS; i++ )
  {
    PRINT_CHAR( SEPARATOR );
    PRINT_UINT( distances[ i ] );
    PRINT_CHAR( SEPARATOR );
    PRINT_UINT( snrs[ i ] );
  }
  PRINT_LN( );
}


// ---------------------------- average and queue functions  ------------------------------------------------------------

void resetAverages ( )
{
  if ( sumDistance > 0 || sumSnr > 0 )    // only need to reset the queue if there are elements in the queue 
  {
    sumDistance = 0;
    sumSnr = 0;
    for ( idx = 0; idx < NR_MEASUREMENTS; idx++ )
    {
      distances[ idx ] = 0;
      snrs[ idx ] = 0;
    }
  }
  queueIsFull = 0;                        // not enough results to build average 
  idx = 0;                                  
}

void insertResult ( uint16_t distance, uint8_t snr )
{
  if (  ( PERSISTENCE == 0 ) 
      || ( ( snr > 0 ) && ( distance >= LOW_THRESHOLD_MM && distance <= HIGH_THRESHOLD_MM ) )
      )
  {
    sumDistance -= distances[ idx ];      // replace oldest element in queue 
    sumSnr -= snrs[ idx ];
    sumDistance += distance;
    sumSnr += snr;
    distances[ idx ] = distance;
    snrs[ idx ] = snr;
    idx++;
    if ( idx == NR_MEASUREMENTS )         // handle wrap over 
    {
      queueIsFull = 1;                    // have enough results to build average 
      idx = 0;
    }
    if ( timerPeriod != PERIOD_OBJECT_IN_MS )      // change detection period if object came into range of interest
    {
      timerPeriod = PERIOD_OBJECT_IN_MS;
      timerStop( &tmf8806 );
      timerStartPeriodic( &tmf8806, timerPeriod );       
    }
  }
  else
  {
    resetAverages ( );                    // no object or object is out of range
#if OUTPUT_ON_LED
    ledPinLow( &tmf8806 );
#endif
#if OUTPUT_ON_GPIO
    resultPinLow( &tmf8806 );
#endif              
    if ( timerPeriod != PERIOD_NO_OBJECT_IN_MS )      // change detection period if object came out of range of interest
    {
      timerPeriod = PERIOD_NO_OBJECT_IN_MS;
      timerStop( &tmf8806 );
      timerStartPeriodic( &tmf8806, timerPeriod );       
    }
  }
}


// ---------------------------- measurement functions  ------------------------------------------------------------------

void turnOnAndStartMeasure ( void )
{
  int8_t res = APP_ERR_CPU_NOT_READY;       

  disableInterrupts( );
  tmf8806IsOff = 0;                           // sensor is running
  enableInterrupts( );                   

  tmf8806Enable( &tmf8806 );
  delayInMicroseconds( ENABLE_TIME_MS * 1000 );
  tmf8806ClkCorrection( &tmf8806, 0 /* clock correction is auto-reset with power-enable, and single shot mode does not have enough samples to do clk-correction */ ); 
  tmf8806Wakeup( &tmf8806 );
  if ( tmf8806IsCpuReady( &tmf8806, CPU_READY_TIME_MS ) )
  {
    if ( tmf8806SwitchToRomApplication( &tmf8806 ) == APP_SUCCESS_OK )
    {
      resultIrqTriggered = 0;
      tmf8806ClrAndEnableInterrupts( &tmf8806, TMF8806_INTERRUPT_RESULT );    // use interrupt to wait for results
      tmf8806SetConfiguration( &tmf8806, &config );                           // use config from this file
      tmf8806SetStateData( &tmf8806, &stateData );                            // use last recorded state data, this avoids unnecessary recalibration
      res = tmf8806StartMeasurement( &tmf8806 );
    }
    else
    {
      res = APP_ERR_COULD_NOT_SWITCH_TO_ROM_APP;
    }
  }
 if ( res != APP_SUCCESS_OK )
 {
#if OUTPUT_ON_UART  
    PRINT_STR( "ERROR start app " );
    PRINT_INT( res );
    PRINT_LN( );
#endif
    disableInterrupts( );
    tmf8806IsOff = 1;             // could not start, not running
    enableInterrupts( );                   
  }
}

void readResultAndTurnOff ( void )
{
  if ( tmf8806GetAndClrInterrupts( &tmf8806, TMF8806_INTERRUPT_RESULT ) & TMF8806_INTERRUPT_RESULT )                      // check if a result is available 
  {
    uint16_t distance;
    uint8_t snr;
    int8_t res = tmf8806ReadResult( &tmf8806, &resultFrame );
    if ( res == APP_SUCCESS_OK )
    {
      distance = resultFrame.distPeak;
      snr = resultFrame.reliability;
      stateData = tmf8806.stateData;                                  // as tmf8806 is switched off, keep a local copy of the state data
#if ( OUTPUT_ON_UART > 0 )
      tmf8806PrintResult( &tmf8806, &resultFrame );
#endif
      
      // switch off tmf8806
      tmf8806DisableAndClrInterrupts( &tmf8806, 0xFF );               // just disable all
      tmf8806Disable( &tmf8806 );
      disableInterrupts( );
      tmf8806IsOff = 1;     // is disabled
      enableInterrupts( );

      // now insert result in array and publish if an average could be calculated    
      insertResult( distance, snr );
      if ( queueIsFull )
      {
#if OUTPUT_ON_UART  
        printAveragedResult( );
        // printQueue();            // for debugging purposes you may want to uncomment this function
#endif
#if OUTPUT_ON_LED
        ledPinHigh( &tmf8806 );
#endif
#if OUTPUT_ON_GPIO
        resultPinHigh( &tmf8806 );
#endif        
      }
    }
    else  // read result failed
    {
#if OUTPUT_ON_UART  
      PRINT_STR( "ERROR Read res " );
      PRINT_INT( res );
      PRINT_LN( );
#endif
#if OUTPUT_ON_LED
      ledPinLow( &tmf8806 );
#endif
#if OUTPUT_ON_GPIO
      resultPinLow( &tmf8806 );
#endif            
    }
  }
}


// ---------------------------- interrupt service routines (priviledged execution level) ----------------------------------------

void resultInterruptHandler ( void )
{
  resultIrqTriggered = 1;
#if ( APP_LOG_LEVEL > 0 )
  PRINT_STR( "INT0" );
  PRINT_LN();
#endif
}

void timerInterruptHandler ( void )
{
  timerIrqTriggered = 1;
#if ( APP_LOG_LEVEL > 0 )
  PRINT_STR( "WDT" );
  PRINT_LN();
#endif
}


// --------------------------------- init function (called only once) ----------------------------------------------------------------

// Arduino setup function is only called once at startup. Do all the HW initialisation stuff here.
void setupFn ( uint32_t baudrate, uint32_t i2cClockSpeedInHz )
{
  timerStop( &tmf8806 );                                // reset arduino timer hardware  
  tmf8806Initialise( &tmf8806, DRIVER_LOG_LEVEL );      // reset driver
  stateData = tmf8806.stateData;                        // copy the default state data for first measurements

  // configure ENABLE pin and interupt pin
  enablePinLow( &tmf8806 );
  pinOutput( &tmf8806, ENABLE_PIN );
  pinInput( &tmf8806, INTERRUPT_PIN );

#if OUTPUT_ON_UART  
  uartOpen( baudrate );
#endif  
#if OUTPUT_ON_LED
  ledPinLow( &tmf8806 );
  pinOutput( &tmf8806, LED_PIN );
#endif
#if OUTPUT_ON_GPIO
  resultPinLow( &tmf8806 );       // start with no object           
  pinOutput( &tmf8806, RESULT_PIN );
#endif    

  // start i2c
  i2cOpen( &tmf8806, i2cClockSpeedInHz );

  // variable initialisation
  resultIrqTriggered = 0;
  timerIrqTriggered = 0;
  tmf8806IsOff = 0;     // is running
  sumDistance = 1;                                        // force a clearing of the arrays within funciton resetAverages
  resetAverages( );
  setInterruptHandler( resultInterruptHandler );

  tmf8806Enable( &tmf8806 );
  delayInMicroseconds( ENABLE_TIME_MS * 1000 );
  tmf8806Wakeup( &tmf8806 );
  if ( tmf8806IsCpuReady( &tmf8806, CPU_READY_TIME_MS ) )
  {
    if ( tmf8806SwitchToRomApplication( &tmf8806 ) == APP_SUCCESS_OK )
    {
      tmf8806ReadDeviceInfo( &tmf8806 );
#if OUTPUT_ON_UART  
      printDeviceInfo( );
#endif
    }
  }
  tmf8806Disable( &tmf8806 );                           // make sure device is off
  delayInMicroseconds( CAP_DISCHARGE_TIME_MS * 1000 );    // wait for a proper discharge of the cap

  delayInMicroseconds( 3000 ); // in case I did something stupid with the watchdog

  tmf8806IsOff = 1;     // is off

  timerPeriod = PERIOD_OBJECT_IN_MS;            // default we assume there is an object in range of interest                         
  timerStartPeriodic( &tmf8806, timerPeriod );          
  powerDown( &tmf8806 );
}


// --------------------------------- loop function (called periodically) -------------------------------------------------------------

void loopFn ( )
{
  if ( resultIrqTriggered )
  {
    uint16_t distance;
    uint8_t snr;
    disableInterrupts( );
    resultIrqTriggered = 0;
    enableInterrupts( );
    readResultAndTurnOff( );
  }

  if ( timerIrqTriggered )
  {
    if ( tmf8806IsOff )
    {
      disableInterrupts( );
      timerIrqTriggered = 0;
      enableInterrupts( );
      turnOnAndStartMeasure( );
    } // else cannot start as last measruement is still ongoing
#if ( APP_LOG_LEVEL > 0 )
    else
    {
      uint32_t tickMs = getSysTick() / 1000;    
      PRINT_STR( "Warning: last measurement not done " );
      PRINT_UINT( tickMs );
      PRINT_LN();
    }
#endif
#if ( APP_LOG_LEVEL > 0 )
      PRINT_STR( "PD" );
      PRINT_LN();
#endif
  }
  powerDown( &tmf8806 );
}

