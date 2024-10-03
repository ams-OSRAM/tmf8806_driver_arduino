/*****************************************************************************
* Copyright (c) [2024] ams-OSRAM AG                                          *
* All rights are reserved.                                                   *
*                                                                            *
* FOR FULL LICENSE TEXT SEE LICENSE.TXT                                      *
******************************************************************************/
 
//
// tmf8806 arduino uno driver example program
//

// ---------------------------------------------- includes ----------------------------------------

#include "tmf8806.h"
#include "tmf8806_app.h"


// ---------------------------------------------- defines -----------------------------------------

#define UART_BAUD_RATE              115200

// arduino uno can only do 400 KHz I2C
#define I2C_CLK_SPEED               400000

// -------------------------------------------------------------------------------------------------------------

// Arduino setup function is only called once at startup. Do all the HW initialisation stuff here.
void setup ( )
{
  setupFn( UART_BAUD_RATE, I2C_CLK_SPEED ); 
}

// Arduino main loop function, is executed cyclic
void loop ( )
{
  loopFn( );    // Arduino cannot stop, so we ignore it here
}
