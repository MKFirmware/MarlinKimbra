#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// This configuration file contains basic settings. Select your:
//  - board type
//  - temperature sensor type
//  - Mechanism type (cartesian-corexy-delta-scara)
//
// Mechanisms-settings can be found in configuration_xxx.h
// Advanced settings can be found in Configuration_adv.h

// Choose your board type.
// Either an numeric ID or name defined in boards.h is valid.
// See: https://github.com/MagoKimbra/MarlinKimbra/blob/master/Documentation/Hardware.md
#include "boards.h"
#ifndef MOTHERBOARD
  #define MOTHERBOARD BOARD_RAMPS_13_EFB
#endif

// User-specified version info of this build to display in during startup.
// Implementation of an idea by Prof Braino to inform user that any changes
// made to this build by the user have been successfully uploaded into firmware.
#define STRING_VERSION "v4.0.2"
#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__                 // build date and time
#define STRING_URL "reprap.org"
#define STRING_CONFIG_H_AUTHOR "(MagoKimbra: magokimbra@hotmail.com)" // Who made the changes.
#define STRING_SPLASH STRING_VERSION " - " STRING_URL                 // will be shown during bootu

// Select which serial port should be used for communication with the host.
// This allows the connection of wireless adapters (for instance) to non-default port pins.
// Serial port 0 is still used by the Arduino bootloader regardless of this setting.
#define SERIAL_PORT 0

// This determines the communication speed of the printer
// 115200 - 250000
#define BAUDRATE 115200

// This enables the serial port associated to the Bluetooth interface on AT90USB devices
//#define BTENABLED

// Define this to set a unique identifier for this printer, (Used by some programs to differentiate between machines)
// You can use an online service to generate a random UUID. (eg http://www.uuidgenerator.net/version4)
//#define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

// If you want test the firmware uncomment below. Use Serial arduino monitor...
//#define FIRMWARE_TEST
#ifdef FIRMWARE_TEST
  #undef BAUDRATE
  #define BAUDRATE 115200  // Baudrate setting to 115200 because serial monitor arduino fuction at max 115200 baudrate.
#endif

/***********************************************************************\
 **************************** Define type printer **********************
 ***********************************************************************/
#define CARTESIAN
//#define COREXY
//#define DELTA
//#define SCARA

#if defined(CARTESIAN)
  #include "Configuration_Cartesian.h"
#elif defined(COREXY)
  #include "Configuration_Corexy.h"
#elif defined(DELTA)
  #include "Configuration_Delta.h"
#elif defined(SCARA)
  #include "Configuration_Scara.h"
#endif
/***********************************************************************/

// This defines the number of extruders real or virtual
#define EXTRUDERS 1

// This is used for singlenozzled multiple extrusion configuration
// Uncomment below to enable (One Hotend)
//#define SINGLENOZZLE

/***********************************************************************
 *********************** Multiextruder MKR4  ***************************
 ***********************************************************************
 *                                                                     *
 * Setting for more extruder width rele system                         *
 *                                                                     *
 * Firmware by MagoKimbra magokimbra@hotmail.com                       *
 *                                                                     *
 * See pins.h for pin command rele                                     *
 *                                                                     *
 ***********************************************************************/
//#define MKR4
#ifdef MKR4
#define DELAY_R 500          // Delay for switch rele
#define DRIVER_EXTRUDERS 1   // This defines the number of Driver extruders
#endif // END MKR4

//**********************************************************************

/***********************************************************************
 *********************** Multiextruder NPr2  ***************************
 ***********************************************************************
 *                                                                     *
 * Setting fot color meccanism NPr2 by NicolaP (www.3dmakerlab.it)     *
 *                                                                     *
 * Firmware by MagoKimbra magokimbra@hotmail.com                       *
 *                                                                     *
 * Find angle setting by g-Code "M997 Cxxx"                            *
 *                                                                     *
 ***********************************************************************/
//#define NPR2
#ifdef NPR2
#define COLOR_STEP {120,25,-65,-155} // CARTER ANGLE
#define COLOR_SLOWRATE 170           // MICROSECOND delay for carter motor routine (Carter Motor Feedrate: upper value-slow feedrate)  
#define COLOR_HOMERATE 4             // FEEDRATE for carter home
#define MOTOR_ANGLE 1.8              // Nema angle for single step 
#define DRIVER_MICROSTEP 4           // Microstep moltiplicator driver (set jumper MS1-2-3) off-on-off 1/4 microstepping.
#define CARTER_MOLTIPLICATOR 14.22   // CARTER MOLTIPLICATOR (gear ratio 13/31-10/31)
#define DRIVER_EXTRUDERS 2           // This defines the number of Driver extruders
#endif
//**********************************************************************


#if !defined(MKR4) && !defined(NPR2)
#define DRIVER_EXTRUDERS EXTRUDERS // This defines the number of Driver extruders
#endif

// The following define selects which power supply you have. Please choose the one that matches your setup
// 0 = Normal power
// 1 = ATX
// 2 = X-Box 360 203 Watts (the blue wire connected to PS_ON and the red wire to VCC)
#define POWER_SUPPLY 0

// Define this to have the electronics keep the power supply off on startup. If you don't know what this is leave it.
//#define PS_DEFAULT_OFF

//===========================================================================
//============================= Thermal Settings ============================
//===========================================================================

//================================ Thermistor ===============================
// Standard 4.7kohm pull up tables
//
//   -2 is thermocouple with MAX6675 (only for sensor 0)
//   -1 is thermocouple with AD595
//    0  is not used
//    1  is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
//    2  is 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
//    3  is Mendel-parts thermistor (4.7k pullup)
//    4  is 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
//    5  is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
//    6  is 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
//    7  is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
//   71  is 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)
//    8  is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
//    9  is 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
//   10 is 100k RS thermistor 198-961 (4.7k pullup)
//   11 is 100k beta 3950 1% thermistor (4.7k pullup)
//   12 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)
//   13 is 100k Hisens 3950  1% up to 300�C for hotend "Simple ONE " & "Hotend "All In ONE" 
//   20 is the PT100 circuit found in the Ultimainboard V2.x
//   60 is 100k Maker's Tool Works Kapton Bed Thermistor beta=3950
// 1047 is Pt1000 with 4k7 pullup
// 1010 is Pt1000 with 1k pullup (non standard)
//  147 is Pt100 with 4k7 pullup
//  110 is Pt100 with 1k pullup (non standard)

// 1 kohm pullup tables
// ATTENTION: This is not normal, you would have to have changed out your 4.7k for 1k
// This gives greater accuracy and more stable PID
//
//   51 is 100k thermistor - EPCOS (1k pullup)
//   52 is 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
//   55 is 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)

#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_BED 1

// This makes temp sensor 1 a redundant sensor for sensor 0.
// If the temperatures difference between these sensors is to high the print will be aborted.
//#define TEMP_SENSOR_1_AS_REDUNDANT
#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10 // degC

#ifdef SINGLENOZZLE
#undef TEMP_SENSOR_1_AS_REDUNDANT
#endif

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10 // seconds
// Range of +/- temperatures considered "close" to the target one
#define TEMP_HYSTERESIS 3 // degC
// Window around target to start the residency timer x degC early.
#define TEMP_WINDOW 1 // degC

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5 // degC
#define HEATER_1_MINTEMP 5 // degC
#define HEATER_2_MINTEMP 5 // degC
#define HEATER_3_MINTEMP 5 // degC
#define BED_MINTEMP      5 // degC

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 275 // degC
#define HEATER_1_MAXTEMP 275 // degC
#define HEATER_2_MAXTEMP 275 // degC
#define HEATER_3_MAXTEMP 275 // degC
#define BED_MAXTEMP      150 // degC

// If your bed has low resistance e.g. 0.6 ohm and throws the fuse you can duty cycle it to reduce the
// average current. The value should be an integer and the heat bed will be turned on for 1 interval of
// HEATER_BED_DUTY_CYCLE_DIVIDER intervals.
//#define HEATER_BED_DUTY_CYCLE_DIVIDER 4

// If you want the M105 heater power reported in watts, define the BED_WATTS, and (shared for all extruders) HOTEND_WATTS
//#define HOTEND_WATTS (12.0*12.0/6.7)  //  P=I^2/R
//#define BED_WATTS (12.0*12.0/1.1)     // P=I^2/R

//=============================== PID settings ==============================
// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP
#ifdef PIDTEMP
  // Limits current to nozzle while in bang-bang mode
  #define BANG_MAX 255 // 255 is full current
  // Limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below)
  #define PID_MAX BANG_MAX // 255 is full current
  // Sends debug data to the serial port
  //#define PID_DEBUG
  // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
  //#define PID_OPENLOOP 1
  // PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of
  // approximately 1s useful for heaters driven by a relay
  //#define SLOW_PWM_HEATERS

  // If the temperature difference between the target temperature and the actual temperature
  // is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
  #define PID_FUNCTIONAL_RANGE 10 // degC
  
  // Limit for the integral term
  #define PID_INTEGRAL_DRIVE_MAX PID_MAX
  // Smoothing factor within the PID
  #define K1 0.95

//             HotEnd{HE0,HE1,HE2,HE3}
  #define DEFAULT_Kp {41,41,41,41}     // Kp for E0, E1, E2, E3
  #define DEFAULT_Ki {07,07,07,07}     // Ki for E0, E1, E2, E3
  #define DEFAULT_Kd {59,59,59,59}     // Kd for E0, E1, E2, E3

#endif // PIDTEMP

//========================= Bed Temperature Control =========================
// Select PID or bang-bang with PIDTEMPBED. If bang-bang, BED_LIMIT_SWITCHING will enable hysteresis
// Uncomment this to enable PID on the bed. It uses the same frequency PWM as the extruder.
// If your PID_dT above is the default, and correct for your hardware/configuration, that means 7.689Hz,
// which is fine for driving a square wave into a resistive load and does not significantly impact you FET heating.
// This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W heater.
// If your configuration is significantly different than this and you don't understand the issues involved, you probably
// shouldn't use bed PID until someone else verifies your hardware works.
// If this is enabled, find your own PID constants below.
//#define PIDTEMPBED
//#define BED_LIMIT_SWITCHING

// This sets the max power delivered to the bed, and replaces the HEATER_BED_DUTY_CYCLE_DIVIDER option.
// all forms of bed control obey this (PID, bang-bang, bang-bang with hysteresis)
// setting this to anything other than 255 enables a form of PWM to the bed just like HEATER_BED_DUTY_CYCLE_DIVIDER did,
// so you shouldn't use it unless you are OK with PWM on your bed.  (see the comment on enabling PIDTEMPBED)
// Limits duty cycle to bed
#define MAX_BED_POWER 255 // 255 is full current

// FIND YOUR OWN: "M303 E-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.
#ifdef PIDTEMPBED
  // 120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
  // from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, aggressive factor of 0.15 (vs 0.1, 1, 10)
  #define DEFAULT_bedKp 10.00
  #define DEFAULT_bedKi 0.023
  #define DEFAULT_bedKd 305.4

  // 120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
  // From pidautotune:
  //#define DEFAULT_bedKp 97.1
  //#define DEFAULT_bedKi 1.41
  //#define DEFAULT_bedKd 1675.16
#endif // PIDTEMPBED

//this prevents dangerous Extruder moves, i.e. if the temperature is under the limit
//can be software-disabled for whatever purposes by
#define PREVENT_DANGEROUS_EXTRUDE
//if PREVENT DANGEROUS EXTRUDE is on, you can still disable (uncomment) very long bits of extrusion separately.
#define PREVENT_LENGTHY_EXTRUDE

#define EXTRUDE_MINTEMP 170 // degC
// Prevent extrusion of very large distances.
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH+Y_MAX_LENGTH) // mm

//================== Thermal Runaway Protection ==============================
// This is a feature to protect your printer from burn up in flames if it has
// a thermistor coming off place (this happened to a friend of mine recently and
// motivated me writing this feature).
// The issue: If a thermistor come off, it will read a lower temperature than actual.
// The system will turn the heater on forever, burning up the filament and anything
// else around.
// After the temperature reaches the target for the first time, this feature will 
// start measuring for how long the current temperature stays below the target 
// minus _HYSTERESIS (set_temperature - THERMAL_RUNAWAY_PROTECTION_HYSTERESIS).
// If it stays longer than _PERIOD, it means the thermistor temperature
// cannot catch up with the target, so something *may be* wrong. Then, to be on the
// safe side, the system will he halt.
// Bear in mind the count down will just start AFTER the first time the 
// thermistor temperature is over the target, so you will have no problem if
// your extruder heater takes 2 minutes to hit the target on heating.

// If you want to enable this feature for all your extruder heaters,
// uncomment the 2 defines below:
// Parameters for all extruder heaters
//#define THERMAL_RUNAWAY_PROTECTION_PERIOD 40 //in seconds
//#define THERMAL_RUNAWAY_PROTECTION_HYSTERESIS 4 // in degree Celsius

// If you want to enable this feature for your bed heater,
// uncomment the 2 defines below:
// Parameters for the bed heater
//#define THERMAL_RUNAWAY_PROTECTION_BED_PERIOD 20 //in seconds
//#define THERMAL_RUNAWAY_PROTECTION_BED_HYSTERESIS 2 // in degree Celsius
//===========================================================================



//===========================================================================
//============================ User Interfaces ==============================
//===========================================================================

//============================== LCD and SD support =========================
//#define ULTRA_LCD  //general LCD support, also 16x2
//#define DOGLCD  // Support for SPI LCD 128x64 (Controller ST7565R graphic Display Family)
//#define SDSUPPORT // Enable SD Card Support in Hardware Console
//#define SDSLOW // Use slower SD transfer mode (not normally needed - uncomment if you're getting volume init error)
//#define SD_CHECK_AND_RETRY // Use CRC checks and retries on the SD communication
//#define ENCODER_PULSES_PER_STEP 1 // Increase if you have a high resolution encoder
//#define ENCODER_STEPS_PER_MENU_ITEM 5 // Set according to ENCODER_PULSES_PER_STEP or your liking
//#define ULTIMAKERCONTROLLER //as available from the Ultimaker online store.
//#define ULTIPANEL  //the UltiPanel as on Thingiverse
//#define LCD_FEEDBACK_FREQUENCY_HZ 1000	// this is the tone frequency the buzzer plays when on UI feedback. ie Screen Click
//#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100 // the duration the buzzer plays the UI feedback sound. ie Screen Click

// The MaKr3d Makr-Panel with graphic controller and SD support
// http://reprap.org/wiki/MaKr3d_MaKrPanel
//#define MAKRPANEL

// The RepRapDiscount Smart Controller (white PCB)
// http://reprap.org/wiki/RepRapDiscount_Smart_Controller
//#define REPRAP_DISCOUNT_SMART_CONTROLLER

// The GADGETS3D G3D LCD/SD Controller (blue PCB)
// http://reprap.org/wiki/RAMPS_1.3/1.4_GADGETS3D_Shield_with_Panel
//#define G3D_PANEL

// The RepRapDiscount FULL GRAPHIC Smart Controller (quadratic white PCB)
// http://reprap.org/wiki/RepRapDiscount_Full_Graphic_Smart_Controller
//
// ==> REMEMBER TO INSTALL U8glib to your ARDUINO library folder: http://code.google.com/p/u8glib/wiki/u8glib
//#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

// The RepRapWorld REPRAPWORLD_KEYPAD v1.1
// http://reprapworld.com/?products_details&products_id=202&cPath=1591_1626
//#define REPRAPWORLD_KEYPAD
//#define REPRAPWORLD_KEYPAD_MOVE_STEP 10.0 // how much should be moved when a key is pressed, eg 10.0 means 10mm per click

// The Elefu RA Board Control Panel
// http://www.elefu.com/index.php?route=product/product&product_id=53
// REMEMBER TO INSTALL LiquidCrystal_I2C.h in your ARDUINO library folder: https://github.com/kiyoshigawa/LiquidCrystal_I2C
//#define RA_CONTROL_PANEL

//automatic expansion
#if defined (MAKRPANEL)
  #define DOGLCD
  #define SDSUPPORT
  #define ULTIPANEL
  #define NEWPANEL
  #define DEFAULT_LCD_CONTRAST 17
#endif

#if defined (REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
  #define DOGLCD
  #define U8GLIB_ST7920
  #define REPRAP_DISCOUNT_SMART_CONTROLLER
#endif

#if defined(ULTIMAKERCONTROLLER) || defined(REPRAP_DISCOUNT_SMART_CONTROLLER) || defined(G3D_PANEL)
  #define ULTIPANEL
  #define NEWPANEL
#endif

#if defined(REPRAPWORLD_KEYPAD)
  #define NEWPANEL
  #define ULTIPANEL
#endif
#if defined(RA_CONTROL_PANEL)
  #define ULTIPANEL
  #define NEWPANEL
  #define LCD_I2C_TYPE_PCA8574
  #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
#endif

//I2C PANELS

//#define LCD_I2C_SAINSMART_YWROBOT
#ifdef LCD_I2C_SAINSMART_YWROBOT
  // This uses the LiquidCrystal_I2C library ( https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home )
  // Make sure it is placed in the Arduino libraries directory.
  #define LCD_I2C_TYPE_PCF8575
  #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
  #define NEWPANEL
  #define ULTIPANEL
#endif

// PANELOLU2 LCD with status LEDs, separate encoder and click inputs
//#define LCD_I2C_PANELOLU2
#ifdef LCD_I2C_PANELOLU2
  // This uses the LiquidTWI2 library v1.2.3 or later ( https://github.com/lincomatic/LiquidTWI2 )
  // Make sure the LiquidTWI2 directory is placed in the Arduino or Sketchbook libraries subdirectory.
  // (v1.2.3 no longer requires you to define PANELOLU in the LiquidTWI2.h library header file)
  // Note: The PANELOLU2 encoder click input can either be directly connected to a pin
  //       (if BTN_ENC defined to != -1) or read through I2C (when BTN_ENC == -1).
  #define LCD_I2C_TYPE_MCP23017
  #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
  #define LCD_USE_I2C_BUZZER //comment out to disable buzzer on LCD
  #define NEWPANEL
  #define ULTIPANEL

  #ifndef ENCODER_PULSES_PER_STEP
	#define ENCODER_PULSES_PER_STEP 4
  #endif

  #ifndef ENCODER_STEPS_PER_MENU_ITEM
	#define ENCODER_STEPS_PER_MENU_ITEM 1
  #endif


  #ifdef LCD_USE_I2C_BUZZER
	#define LCD_FEEDBACK_FREQUENCY_HZ 1000
	#define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100
  #endif

#endif

// Panucatt VIKI LCD with status LEDs, integrated click & L/R/U/P buttons, separate encoder inputs
//#define LCD_I2C_VIKI
#ifdef LCD_I2C_VIKI
  // This uses the LiquidTWI2 library v1.2.3 or later ( https://github.com/lincomatic/LiquidTWI2 )
  // Make sure the LiquidTWI2 directory is placed in the Arduino or Sketchbook libraries subdirectory.
  // Note: The pause/stop/resume LCD button pin should be connected to the Arduino
  //       BTN_ENC pin (or set BTN_ENC to -1 if not used)
  #define LCD_I2C_TYPE_MCP23017
  #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
  #define LCD_USE_I2C_BUZZER //comment out to disable buzzer on LCD (requires LiquidTWI2 v1.2.3 or later)
  #define NEWPANEL
  #define ULTIPANEL
#endif

// Shift register panels
// ---------------------
// 2 wire Non-latching LCD SR from:
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/schematics#!shiftregister-connection 

//#define SAV_3DLCD
#ifdef SAV_3DLCD
  #define SR_LCD_2W_NL    // Non latching 2 wire shiftregister
  #define NEWPANEL
  #define ULTIPANEL
#endif


#ifdef ULTIPANEL
//  #define NEWPANEL  //enable this if you have a click-encoder panel
  #define SDSUPPORT
  #define ULTRA_LCD
  #ifdef DOGLCD // Change number of lines to match the DOG graphic display
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 5
  #else
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 4
  #endif
#else //no panel but just LCD
  #ifdef ULTRA_LCD
  #ifdef DOGLCD // Change number of lines to match the 128x64 graphics display
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 5
  #else
    #define LCD_WIDTH 16
    #define LCD_HEIGHT 2
  #endif
  #endif
#endif

// default LCD contrast for dogm-like LCD displays
#ifdef DOGLCD
  #ifndef DEFAULT_LCD_CONTRAST
    #define DEFAULT_LCD_CONTRAST 32
  #endif
#endif

// option for invert rotary switch
//#define INVERT_ROTARY_SWITCH

//============================== Languages UI =========================
// 1  English
// 2  Polish
// 3  French
// 4  German
// 5  Spanish
// 6  Russian
// 7  Italian
// 8  Portuguese
// 9  Finnish
// 10 Aragonese
// 11 Dutch
// 12 Catalan
// 13 Basque-Euskera

#define LANGUAGE_CHOICE 7


//===========================================================================
//=============================Additional Features===========================
//===========================================================================

//=================================== EEPROM ================================
// The microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores parameters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//define this to enable EEPROM support
//#define EEPROM_SETTINGS
//to disable EEPROM Serial responses and decrease program space by ~1700 byte: comment this out:
// please keep turned on if you can.
//#define EEPROM_CHITCHAT

//========================= Bowden Filament management ======================
//#define EASY_LOAD
#ifdef EASY_LOAD
  #define BOWDEN_LENGTH 560       // mm
  #define LCD_PURGE_LENGTH 3      // mm
  #define LCD_RETRACT_LENGTH 3    // mm
  #define LCD_PURGE_FEEDRATE 3    // mm/s
  #define LCD_RETRACT_FEEDRATE 10 // mm/s
  #define LCD_LOAD_FEEDRATE 8     // mm/s
  #define LCD_UNLOAD_FEEDRATE 8   // mm/s
#endif


//============================== Preheat Constants ==========================
#define PLA_PREHEAT_HOTEND_TEMP 190
#define PLA_PREHEAT_HPB_TEMP 60
#define PLA_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255

#define ABS_PREHEAT_HOTEND_TEMP 240
#define ABS_PREHEAT_HPB_TEMP 100
#define ABS_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255

#define GUM_PREHEAT_HOTEND_TEMP 230
#define GUM_PREHEAT_HPB_TEMP 60
#define GUM_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255


//============================= R/C Servo support ===========================
// Number of servos
// If you select a configuration below, this will receive a default value and does not need to be set manually
// set it manually if you have more servos than extruders and wish to manually control some
// leaving it defining as 0 will disable the servo subsystem
#define NUM_SERVOS 0      // Servo index starts with 0 for M280 command

// Servo Endstops
// This allows for servo actuated endstops, primary usage is for the Z Axis to eliminate calibration or bed height changes.
// Use M666 command to correct for switch height offset to actual nozzle height. Store that setting with M500.
//
#define SERVO_ENDSTOPS {-1,-1,0}            // Servo index for X, Y, Z. Disable with -1
#define SERVO_ENDSTOP_ANGLES {0,0,0,0,90,0} // X,Y,Z Axis Extend and Retract angles


//============================== Filament Sensor ============================
// Enter the diameter of the filament generally used (3.0 mm or 1.75 mm)
// This is then used in the slicer software. Used for sensor reading validation
#define DEFAULT_NOMINAL_FILAMENT_DIA 3.0 // mm

// Also allows adjustment of diameter at print time (vs  at slicing)
// Single extruder only at this point (extruder 0)
// Motherboards
// 34 - RAMPS1.4 - uses Analog input 5 on the AUX2 connector 
// 81 - Printrboard - Uses Analog input 2 on the Exp1 connector (version B,C,D,E)
// 301 - Rambo  - uses Analog input 3
// Note may require analog pins to be defined for different motherboards
//#define FILAMENT_SENSOR

#ifdef FILAMENT_SENSOR
  //The number of the extruder that has the filament sensor (0,1,2)
  #define FILAMENT_SENSOR_EXTRUDER_NUM 0
  // Measurement delay in cm.  This is the distance from filament sensor to middle of barrel
  #define MEASUREMENT_DELAY_CM 14 // cm
  // Upper limit factor used for sensor reading validation
  #define MEASURED_UPPER_LIMIT 3.30 // mm
  // Lower limit factor for sensor reading validation
  #define MEASURED_LOWER_LIMIT 1.90 // mm
  // Delay buffer size in bytes (1 byte = 1cm)
  // Limits maximum measurement delay allowable (must be larger than MEASUREMENT_DELAY_CM  and lower number saves RAM)
  #define MAX_MEASUREMENT_DELAY	20 // cm
  // When using an LCD, uncomment the line below to display the Filament
  // sensor data on the last line instead of status.  Status will appear for 5 sec.
  //#define FILAMENT_LCD_DISPLAY

  //defines used in the code
  #define DEFAULT_MEASURED_FILAMENT_DIA DEFAULT_NOMINAL_FILAMENT_DIA  //set measured to nominal initially 
#endif // FILAMENT_SENSOR


//=================================== Misc =================================

// Temperature status LEDs that display the hotend and bet temperature.
// If all hotends and bed temperature and temperature setpoint are < 54C then the BLUE led is on.
// Otherwise the RED led is on. There is 1C hysteresis.
//#define TEMP_STAT_LEDS

// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
//#define FAST_PWM_FAN

// Use software PWM to drive the fan, as for the heaters. This uses a very low frequency
// which is not ass annoying as with the hardware PWM. On the other hand, if this frequency
// is too low, you should also increment SOFT_PWM_SCALE.
//#define FAN_SOFT_PWM

// Incrementing this by 1 will double the software PWM frequency,
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.
// However, control resolution will be halved for each increment;
// at zero value, there are 128 effective control positions.
#define SOFT_PWM_SCALE 0

// M240  Triggers a camera by emulating a Canon RC-1 Remote
// Data from: http://www.doc-diy.net/photo/rc-1_hacked/
//#define PHOTOGRAPH_PIN 23

// SF send wrong arc g-codes when using Arc Point as fillet procedure
//#define SF_ARC_FIX

// Support for the BariCUDA Paste Extruder.
//#define BARICUDA

// Support for BlinkM/CyzRgb
//#define BLINKM

// Support for STEPPERS TOSHIBA
//#define CONFIG_STEPPERS_TOSHIBA

// Setting firmware for filament end switch
//#define FILAMENT_END_SWITCH

// define laser beam support
//#define LASERBEAM



#include "Configuration_adv.h"

#endif //__CONFIGURATION_H
