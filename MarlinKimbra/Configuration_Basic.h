#ifndef CONFIGURATION_BASIC_H
#define CONFIGURATION_BASIC_H
/*
 * This configuration file contains basic settings.
 *
 * - Serial comunication type
 * - Board type
 * - Mechanism type
 * - Extruders number
 * - Thermistor type
 * - Temperature limits
 * - UI Language
 *
 * Mechanisms-settings can be found in Configuration_Xxxxxx.h (where Xxxxxx can be: Cartesian - Delta - Core - Scara)
 * Feature-settings can be found in Configuration_Feature.h
 * Pins-settings can be found in "Configuration_Pins.h"
 */

/***********************************************************************
 ********************** Serial comunication type ***********************
 ***********************************************************************/
// SERIAL_PORT selects which serial port should be used for communication with the host.
// This allows the connection of wireless adapters (for instance) to non-default port pins.
// Serial port 0 is still used by the Arduino bootloader regardless of this setting.
#define SERIAL_PORT 0

// Enable the Bluetooth serial interface on AT90USB devices
//#define BLUETOOTH

// This determines the communication speed of the printer
// 2400,9600,19200,38400,57600,115200,250000
#define BAUDRATE 115200

// User-specified version info of this build to display in [Pronterface, etc] terminal window during
// startup. Implementation of an idea by Prof Braino to inform user that any changes made to this
// build by the user have been successfully uploaded into firmware.
#define STRING_CONFIG_H_AUTHOR "(none, default config)"   // Who made the changes.

// Define this to set a unique identifier for this printer, (Used by some programs to differentiate between machines)
// You can use an online service to generate a random UUID. (eg http://www.uuidgenerator.net/version4)
#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

// If you want test the firmware uncomment below. Use Serial arduino monitor...
//#define FIRMWARE_TEST // ONLY BAUDRATE 115200

// Some particular clients re-start sending commands only after receiving a 'wait' when there is a bed serial-connection.
//#define NO_TIMEOUTS 1000 // Milliseconds
//#define ADVANCED_OK // Uncomment to include more info in ok command
/***********************************************************************/


/*****************************************************************************************
 *************************************** Board type **************************************
 *****************************************************************************************
 *                                                                                       *
 * Either an numeric ID or name defined in boards.h is valid.                            *
 * See: https://github.com/MagoKimbra/MarlinKimbra/blob/master/Documentation/Hardware.md *
 *                                                                                       *
 *****************************************************************************************/
#define MOTHERBOARD BOARD_RAMPS_13_HFB
/*****************************************************************************************/


/***********************************************************************
 *************************** Mechanism type ****************************
 ***********************************************************************/
#define MECHANISM MECH_CARTESIAN
//#define MECHANISM MECH_COREXY
//#define MECHANISM MECH_COREXZ
//#define MECHANISM MECH_DELTA
//#define MECHANISM MECH_SCARA
/***********************************************************************/


/*************************************************************************************
 ************************************ Power supply ***********************************
 *************************************************************************************
 *                                                                                   *
 * The following define selects which power supply you have.                         *
 * Please choose the one that matches your setup and set to POWER_SUPPLY:            *
 * 0 Normal power                                                                    *
 * 1 ATX                                                                             *
 * 2 X-Box 360 203 Watts (the blue wire connected to PS_ON and the red wire to VCC)  *
 *                                                                                   *
 *************************************************************************************/
#define POWER_SUPPLY 0
// Define this to have the electronics keep the power supply off on startup. If you don't know what this is leave it.
//#define PS_DEFAULT_OFF
/*************************************************************************************/


/***********************************************************************
 ************************** Extruders number ***************************
 ***********************************************************************/
// This defines the number of extruder real or virtual
#define EXTRUDERS 1

// This defines the number of Driver extruder you have and use
#define DRIVER_EXTRUDERS 1
/***********************************************************************/


/*****************************************************************************************************
 ************************************** Thermistor type **********************************************
 *****************************************************************************************************
 *                                                                                                   *
 * 4.7kohm PULLUP!                                                                                   *
 * This is a normal value, if you use a 1k pullup thermistor see below                               *
 * Please choose the one that matches your setup and set to TEMP_SENSOR_.                            *
 *                                                                                                   *
 * Temperature sensor settings (4.7kohm PULLUP):                                                     *
 *  -2 is thermocouple with MAX6675 (only for sensor 0)                                              *
 *  -1 is thermocouple with AD595                                                                    *
 *   0 is not used                                                                                   *
 *   1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)                                 *
 *   2 is 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)                                        *
 *   3 is Mendel-parts thermistor (4.7k pullup)                                                      *
 *   4 is 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!     *
 *   5 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)              *
 *   6 is 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup) *
 *   7 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)                                     *
 *  71 is 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)                                     *
 *   8 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)                                        *
 *   9 is 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)                                          *
 *  10 is 100k RS thermistor 198-961 (4.7k pullup)                                                   *
 *  11 is 100k beta 3950 1% thermistor (4.7k pullup)                                                 *
 *  12 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)       *
 *  13 is 100k Hisens 3950  1% up to 300°C for hotend "Simple ONE " & "Hotend "All In ONE"           *
 *  20 is the PT100 circuit found in the Ultimainboard V2.x                                          *
 *  60 is 100k Maker's Tool Works Kapton Bed Thermistor beta=3950                                    *
 *                                                                                                   *
 * 1kohm PULLUP!                                                                                     *
 * This is not normal, you would have to have changed out your 4.7k for 1k                           *
 * (but gives greater accuracy and more stable PID)                                                  *
 * Please choose the one that matches your setup.                                                    *
 *                                                                                                   *
 * Temperature sensor settings (4.7kohm PULLUP):                                                     *
 *  51 is 100k thermistor - EPCOS (1k pullup)                                                        *
 *  52 is 200k thermistor - ATC Semitec 204GT-2 (1k pullup)                                          *
 *  55 is 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)                *
 *                                                                                                   *
 *  1047 is Pt1000 with 4k7 pullup                                                                   *
 *  1010 is Pt1000 with 1k pullup (non standard)                                                     *
 *  147 is Pt100 with 4k7 pullup                                                                     *
 *  110 is Pt100 with 1k pullup (non standard)                                                       *
 *  998 and 999 are Dummy Tables. ALWAYS read 25°C or DUMMY_THERMISTOR_998_VALUE temperature         *
 *                                                                                                   *
 *****************************************************************************************************/
#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_BED 1

//These 2 defines help to calibrate the AD595 sensor in case you get wrong temperature measurements.
//The measured temperature is defined as "actualTemp = (measuredTemp * TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET"
#define TEMP_SENSOR_AD595_OFFSET 0.0
#define TEMP_SENSOR_AD595_GAIN   1.0

// Use it for Testing or Development purposes. NEVER for production machine.
//#define DUMMY_THERMISTOR_998_VALUE 25
//#define DUMMY_THERMISTOR_999_VALUE 25

//Show Temperature ADC value
//The M105 command return, besides traditional information, the ADC value read from temperature sensors.
//#define SHOW_TEMP_ADC_VALUES
/*****************************************************************************************************/


/***********************************************************************
 ************************* Temperature limits ***************************
 ***********************************************************************/
// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 275 // (degC)
#define HEATER_1_MAXTEMP 275 // (degC)
#define HEATER_2_MAXTEMP 275 // (degC)
#define HEATER_3_MAXTEMP 275 // (degC)
#define BED_MAXTEMP      150 // (degC)

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5 // (degC)
#define HEATER_1_MINTEMP 5 // (degC)
#define HEATER_2_MINTEMP 5 // (degC)
#define HEATER_3_MINTEMP 5 // (degC)
#define BED_MINTEMP      5 // (degC)

//Preheat Constants
#define PLA_PREHEAT_HOTEND_TEMP 190
#define PLA_PREHEAT_HPB_TEMP 60
#define PLA_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255

#define ABS_PREHEAT_HOTEND_TEMP 240
#define ABS_PREHEAT_HPB_TEMP 100
#define ABS_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255

#define GUM_PREHEAT_HOTEND_TEMP 230
#define GUM_PREHEAT_HPB_TEMP 60
#define GUM_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255
/*****************************************************************************************************/


/***********************************************************************
 *************************** UI Language  ******************************
 ***********************************************************************
 *                                                                     *
 * Select the language that you prefer and change LANGUAGE_CHOICE      *
 *                                                                     *
 * 1  English                                                          *
 * 2  Polish                                                           * 
 * 3  French                                                           * 
 * 4  German                                                           * 
 * 5  Spanish                                                          * 
 * 6  Russian                                                          * 
 * 7  Italian                                                          * 
 * 8  Portuguese                                                       * 
 * 9  Finnish                                                          * 
 * 10 Aragonese                                                        * 
 * 11 Dutch                                                            * 
 * 12 Catalan                                                          * 
 * 13 Basque-Euskera                                                   * 
 * 14 Portuguese (Brazil)                                              * 
 *                                                                     * 
 ***********************************************************************/
#define LANGUAGE_CHOICE 1
/***********************************************************************/
#endif
