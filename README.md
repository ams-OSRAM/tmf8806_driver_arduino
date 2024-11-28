# TMF8806 (Time-of-flight) Arduino Uno driver  

This is a simple universal driver to show the capabilies of the time-of-flight device TMF8806. 

## Requirements  

- an Arduino Uno R3 board 
- an ams TMF8806 Arduino Shield Board [TMF8806_EVM_EB_SHIELD](https://ams-osram.com/search?q=tmf8806)
- a USB B cable
- the [Arduino IDE](https://www.arduino.cc/en/software) to compile and download the Arduino Uno example application


## Files  

The arduino project in the folder: **tmf8806_app** is a very simple single character command line interpreter listening/printing on UART
It contains the following files:

- **tmf8806_app.ino** - the arduino specific wrapper for the application
- **tmf8806_app.h** and **tmf8806_app.cpp** - the TMF8806 command line application
- **tmf8806.h** and **tmf8806.cpp** - the TMF8806 driver 
- **tmf8806_shim.h** and **tmf8806_shim.cpp** - a shim to abstract the arduino specific I2C, UART and GPIO functions
- **tmf8806_image.h** and **tmf8806_image.c** - the TMF8806 firmware that is downloaded by the driver as a c-struct

You can use application interrupt driven. For this you need to set the following define to 1: **USE_INTERRUPT_TO_TRIGGER_READ**

### Porting to another MCU

To port the driver and application to another platform you need to adapt the following files:
- **tmf8806_a.ino** - the arduino specific wrapper for the application, replace this with the requirements for your selected platform
- **tmf8806_shim.h** and **tmf8806_shim.cpp** - a shim to abstract the arduino specific I2C, UART and GPIO functions

## UART and command line interpreter  

The project listens and talks on the UART. Baud rate is 115200, 8-bit, no parity, 1 stop bit.

The command line interpreter uses single characters followed by ENTER as input commands. 
For interfacing with a zeromq server the command line interpreter also supports complex command strings with binary payload.

UART commands (single character)

- a ... dump registers
- c ... toggle configuration
- d ... disable device
- e ... enable device
- f ... factory calibration
- h ... help
- i ... I2C address change
- m ... start measure
- p ... power down
- r ... remote control mode
- s ... stop measure
- t ... switch persistence and thresholds
- w ... wakeup
- x ... clock correction on/off
- z ... histogram dump
- + ... log level+
- - ... log level-

UART commands (binary payload)

- b\x30<tmf8806FactoryCalibData data> .. set arbitrary factory calibration data
- b\x31<tmf8806MeasureCmd data>       .. set arbitrary configuration
- b\x32<pers,loThres,hiThres>         .. set arbitrary persistence / threshold

## Command line interpreter application  

The command line interpreter application mimics a simple host application. It allows the user to switch between 2 pre-defined configurations:  

- Configuration 0: Period 33 ms, KiloIterations = 400
- Configuration 1: single shot mode, KiloIterations = 100

You can modify the configurations (e.g. choose different period or KiloIterations) in the application source file, recompile and download your own configuration application to the Arduino. 

## Examples  

### Power up device  

Before the device can be used the host must power the device. The arduino uno setup function will pull the enable line to the TMF8806 low. I.e. the TMF8806 will be powered down.

Enter the character

- e  

followed by ENTER to enable the device. If necessary the driver will automatically download the the firmware patch file to the TMF8806 RAM. The arduino uno will publish the FW version in the terminal:  

e.g. version 192.4.11.0

### Power up device and do measurements  

Type the following commands on UART:

- e 
- m 

The header for the results looks like this:  
 \#Obj,i2c_slave_address,result_number,reliability,measured_distance,corrected_distance,sys_clock,temperature,reference_hits,object_hits,crosstalk

You will see measurement result records on the UART for the default configuration.  

\#Obj,65,165,63,295,295,735639833,27,69216,27384,829
\#Obj,65,166,63,295,295,735796255,27,69243,27459,830
\#Obj,65,167,63,294,294,735951813,27,69362,27297,799

# Factory calibration  

Factory calibration must be done for each device. 

The simplest way is to do a live factory calibration. I.e. do the following steps:
1. Connect your Arduino Uno and tmf8806 to the PC via USB
2. Start a terminal program 
3. Configure and connect the terminal program to the arduino uno
4. Make sure you have a cover glass on top of the TMF8806. This is needed for the crosstalk.
5. Make sure there is no object in front of the TMF8806 within 40 cm.
6. Enter the following commands in your terminal console:  
    - e 
    - f  
7. The application reports the calibration data: e.g. #Cal,0x2,0x0,0x0,0xA,0xB0,0xBE,0xBD,0x7C,0xF5,0xF0,0xF3,0xF7,0x7,0x4
8. If the application does not send the calibration data the calibration has failed. Check your cover glass and check if there is no object (within 40cm) in the field-of-view of the sensor.

To use the obtained factory calibration data as default add this code to the sketch:

```
static const uint8_t deviceFactoryCalibration[sizeof(tmf8806FactoryCalibData)] = 
{
  0x2,0x0,0x0,0xA,0xB0,0xBE,0xBD,0x7C,0xF5,0xF0,0xF3,0xF7,0x7,0x4
};

void tmf8806Initialise ( tmf8806Driver * driver, uint8_t logLevel )
{
  tmf8806ResetClockCorrection( driver );
  driver->i2cSlaveAddress = TMF8806_SLAVE_ADDR;
  driver->clkCorrectionEnable = 1;                  // default is on
  driver->logLevel = logLevel;
  driver->measureConfig = defaultConfig;
  // driver->factoryCalib = defaultFactoryCalib;
  tmf8806DeserializeFactoryCalibration(deviceFactoryCalibration,&(driver->factoryCalib));
  driver->stateData = defaultStateData;
  driver->info = tmf8806DriverInfoReset;
  driver->device = tmf8806DeviceInfoReset;
}
```

Put the device calibration data into the array **deviceFactoryCalibration** and replace the assignment of the default factory 
calibration with a call to deserialize the calibration data from that array.

## Crosstalk readout

The application reports the crosstalk with each measurement result. E.g.:

\#Obj,65,167,63,294,294,735951813,27,69362,27297,799

The crosstalk is the last field in the line.

## I2C slave address changing

Send the command "i" to the application to change the sensor I2C address. The application will report the new I2C address. E.g.: Addr=0x42

# Histogram dumping  

The device can also dump histograms. The order the Arduino Uno driver reports these histograms is exactly the same as the TMF8806 reports them on I2C.  

There are five different types of histograms available  

- electrical calibration histograms (ID=1)
- proximity histograms (ID=2)
- distance histograms (ID=4)
- pileup-corrected distance histograms (ID=8)
- summed histogram (ID=16)

Type "z" and ENTER until you have the desired value for histogram dumping. E.g if the application sends:

Histogram is 4

distance histograms are configured for dumping. The application dumps ALL histograms if:

Histogram is 31

For all histograms, the first number after the marker (e.g. #TG7) will give the channel this histogram contains.

There are always 10 histograms reported. 

- Number  0 == TDC0, Channel0
- Number  1 == TDC0, Channel1
- ... 
- Number  9 == TDC4, Channel0
- Number 10 == TDC0, Channel1
 
The header for the raw histograms looks like this:  
\#<histogram_type>,bin_0,bin_1,...,bin_127

Line Tag  | Histogram Type
----------|-----------------------
  \#CI    | electrical calibration
  \#PT    | proximity
  \#TG    | distance
  \#TGPUC | pileup-corrected distance
  \#SUM   | summed

## Remote Control Mode

This is a new operation mode for the TMF8806 VCSEL (laser diode) to act as an infrared LED controlled 
by the input signal on GPIO1. GPIO1 == LO -> VCSEL OFF, GPIO1 == HI -> VCSEL ON. 
This function requires firmware patch 4.14.1.x or newer. It does not work with standard ROM firmware.
To exit the remote control mode send the command "s" (stop measure).
