/**
 * pins.h
 */

/****************************************************************************************
* 10 BOARD_GEN7_CUSTOM - Gen7 custom (Alfons3 Version)
* 11 BOARD_GEN7_12 - Gen7 v1.1, v1.2
* 12 BOARD_GEN7_13 - Gen7 v1.3
* 12 BOARD_GEN7_14 - Gen7 v1.4
*
*  2 BOARD_CHEAPTRONIC - Cheaptronic v1.0
* 20 BOARD_SETHI - Sethi 3D_1
* 21 BOARD_ELEFU_3 - Elefu Ra Board (v3)
* 22 BOARD_GEN3_MONOLITHIC - Gen3 Monolithic Electronics
*
*  3 BOARD_RAMPS_OLD - MEGA/RAMPS up to 1.2
* 33 BOARD_RAMPS_13_HFB - RAMPS 1.3 / 1.4 (Power outputs: Hotend0, Fan, Bed)
* 34 BOARD_RAMPS_13_HHB - RAMPS 1.3 / 1.4 (Power outputs: Hotend0, Hotend1, Bed)
* 35 BOARD_RAMPS_13_HFF - RAMPS 1.3 / 1.4 (Power outputs: Hotend0, Fan, Fan)
* 36 BOARD_RAMPS_13_HHF - RAMPS 1.3 / 1.4 (Power outputs: Hotend0, Hotend1, Fan)
* 37 BOARD_RAMPS_13_HHH - RAMPS 1.3 / 1.4 (Power outputs: Hotend0, Hotend1, Hotend2)
*
*301 BOARD_RAMBO - Rambo
*302 BOARD_MINIRAMBO - Mini Rambo
*
*  4 BOARD_DUEMILANOVE_328P - Duemilanove w/ ATMega328P pin assignment
* 40 BOARD_MKS_BASE    - Arduino Mega2560 with RAMPS v1.4 pin assignments
*401 BOARD_RADDS       - Radds ARM 32 Arduino DUE
*403 BOARD_RAMPS_FD_V1 - Ramps FD version 1 ARM 32 Arduino DUE
*404 BOARD_RAMPS_FD_V2 - Ramps FD version 2 ARM 32 Arduino DUE
*408 BOARD_SMART_RAMPS - Smart Ramps ARM 32 Arduino DUE
*433 BOARD_RAMPS4DUE   - Ramps ARM 32 Arduino DUE
*
*  5 BOARD_GEN6 - Gen6
* 51 BOARD_GEN6_DELUXE - Gen6 deluxe
*502 BOARD_ALLIGATOR   - Alligator R2 ARM 32 Arduino DUE

*
*  6 BOARD_SANGUINOLOLU_11 - Sanguinololu < 1.2
* 62 BOARD_SANGUINOLOLU_12 - Sanguinololu 1.2 and above
* 63 BOARD_MELZI - Melzi
* 64 BOARD_STB_11 - STB V1.1
* 65 BOARD_AZTEEG_X1 - Azteeg X1
* 66 BOARD_MELZI_MAKR3D - Melzi with ATmega1284 (MaKr3d version)
* 67 BOARD_AZTEEG_X3 - Azteeg X3
* 68 BOARD_AZTEEG_X3_PRO - Azteeg X3 Pro
*
*  7 BOARD_ULTIMAKER - Ultimaker
*
* 70 BOARD_MEGATRONICS   - Megatronics
*701 BOARD_MEGATRONICS_2 - Megatronics v2.0
*702 BOARD_MINITRONICS   - Minitronics v1.0
*703 BOARD_MEGATRONICS_3 - Megatronics v3.0
*705 BOARD_ULTRATRONICS  - Ultratronics v1 ARM 32 Arduino DUE
* 71 BOARD_ULTIMAKER_OLD - Ultimaker (Older electronics. Pre 1.5.4. This is rare)
* 72 BOARD_ULTIMAIN_2 - Ultimainboard 2.x (Uses TEMP_SENSOR 20)
* 77 BOARD_3DRAG - 3Drag Controller
* 78 BOARD_K8200 - Vellemann K8200 Controller (derived from 3Drag Controller)
*
*  8 BOARD_TEENSYLU - Teensylu
* 80 BOARD_RUMBA - Rumba
* 81 BOARD_PRINTRBOARD - Printrboard (AT90USB1286)
* 82 BOARD_BRAINWAVE - Brainwave (AT90USB646)
* 83 BOARD_SAV_MKI - SAV Mk-I (AT90USB1286)
* 84 BOARD_TEENSY2 - Teensy++2.0 (AT90USB1286)
* 88 BOARD_5DPRINT - 5DPrint D8 Driver Board
*
*  9 BOARD_GEN3_PLUS - Gen3+
* 90 BOARD_OMCA_A - Alpha OMCA board
* 91 BOARD_OMCA - Final OMCA board
*999 BOARD_LEAPFROG - Leapfrog
*
*99 BOARD_99 - Custom motherboard
****************************************************************************************/

#ifndef PINS_H
#define PINS_H


/******************************************************************************
* 10
* Gen7 Alfons
* These Pins are assigned for the modified GEN7
* Board from Alfons3 Please review the pins and adjust it for your needs
******************************************************************************/
#if MB(GEN7_CUSTOM)
  #define KNOWN_BOARD

  #if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__) && !defined(__AVR_ATmega1284P__)
    #error Oops!  Make sure you have 'Gen7' selected from the 'Tools -> Boards' menu.
  #endif

  //x axis pins
  #define ORIG_X_STEP_PIN       21  // different from standard GEN7
  #define ORIG_X_DIR_PIN        20  // different from standard GEN7
  #define ORIG_X_ENABLE_PIN     24
  #define X_STOP_PIN        0

  //y axis pins
  #define ORIG_Y_STEP_PIN       23
  #define ORIG_Y_DIR_PIN        22
  #define ORIG_Y_ENABLE_PIN     24
  #define Y_STOP_PIN        1

  //z axis pins
  #define ORIG_Z_STEP_PIN       26
  #define ORIG_Z_DIR_PIN        25
  #define ORIG_Z_ENABLE_PIN     24
  #define Z_STOP_PIN        2
  
  //extruder pins
  #define ORIG_E0_STEP_PIN      28
  #define ORIG_E0_DIR_PIN       27
  #define ORIG_E0_ENABLE_PIN    24
  
  #define ORIG_TEMP_0_PIN        2
  #define ORIG_TEMP_1_PIN       -1
  #define ORIG_TEMP_2_PIN       -1
  #define ORIG_TEMP_BED_PIN      1  // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 34 bed)
 
  #define ORIG_HEATER_0_PIN      4
  #define ORIG_HEATER_1_PIN     -1
  #define ORIG_HEATER_2_PIN     -1
  #define ORIG_HEATER_BED_PIN    3  // (bed)
  
  #define SDPOWER               -1
  #define SDSS                  31  // SCL pin of I2C header || CS Pin for SD Card support
  #define LED_PIN               -1
  
  #define ORIG_FAN_PIN          -1
  #define ORIG_PS_ON_PIN        19
  
  //our pin for debugging.
  #define DEBUG_PIN             -1
  
  //our RS485 pins
  //#define TORIG_X_ENABLE_PIN  12
  //#define RORIG_X_ENABLE_PIN  13
  
  #define ORIG_BEEPER_PIN  -1
  #define SD_DETECT_PIN    -1
  #define SUICIDE_PIN      -1  //has to be defined; otherwise Power_off doesn't work
  
  #define KILL_PIN         -1
  //Pins for 4bit LCD Support
  #define LCD_PINS_RS      18
  #define LCD_PINS_ENABLE  17
  #define LCD_PINS_D4      16
  #define LCD_PINS_D5      15
  #define LCD_PINS_D6      13
  #define LCD_PINS_D7      14
  
  //buttons are directly attached
  #define BTN_EN1          11
  #define BTN_EN2          10
  #define BTN_ENC          12  //the click

#endif // GEN7_CUSTOM
/****************************************************************************************/



/****************************************************************************************
* 11
* Gen7 v1.1, v1.2
****************************************************************************************/
#if MB(GEN7_12)
  #define KNOWN_BOARD
  
  #if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__) && !defined(__AVR_ATmega1284P__)
    #error Oops! Make sure you have 'Gen7' selected from the 'Tools -> Boards' menu.
  #endif
  
  #if DISABLED(GEN7_VERSION)
    #define GEN7_VERSION   12  // v1.x
  #endif
  
  //X axis pins
  #define ORIG_X_STEP_PIN       19
  #define ORIG_X_DIR_PIN        18
  #define ORIG_X_ENABLE_PIN     24
  #define X_STOP_PIN            7
  
  //Y axis pins
  #define ORIG_Y_STEP_PIN       23
  #define ORIG_Y_DIR_PIN        22
  #define ORIG_Y_ENABLE_PIN     24
  #define Y_STOP_PIN            5
  
  //Z axis pins
  #define ORIG_Z_STEP_PIN       26
  #define ORIG_Z_DIR_PIN        25
  #define ORIG_Z_ENABLE_PIN     24
  #define ORIG_Z_MIN_PIN        1
  #define ORIG_Z_MAX_PIN        0
  
  //extruder pins
  #define ORIG_E0_STEP_PIN      28
  #define ORIG_E0_DIR_PIN       27
  #define ORIG_E0_ENABLE_PIN    24
  
  #define ORIG_TEMP_0_PIN        1
  #define ORIG_TEMP_1_PIN       -1
  #define ORIG_TEMP_2_PIN       -1
  #define ORIG_TEMP_BED_PIN      2
  
  #define ORIG_HEATER_0_PIN      4
  #define ORIG_HEATER_1_PIN     -1
  #define ORIG_HEATER_2_PIN     -1
  #define ORIG_HEATER_BED_PIN    3
  
  #define KILL_PIN         -1
  
  #define SDPOWER          -1
  #define SDSS             -1  // SCL pin of I2C header
  #define LED_PIN          -1
  
  #define ORIG_FAN_PIN     31
  #define ORIG_PS_ON_PIN   15
  
  //All these generations of Gen7 supply thermistor power
  //via PS_ON, so ignore bad thermistor readings
  #define BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  
  //our pin for debugging.
  #define DEBUG_PIN         0
  
  //our RS485 pins
  #define TORIG_X_ENABLE_PIN    12
  #define RORIG_X_ENABLE_PIN    13

#endif
/****************************************************************************************/



/****************************************************************************************
* 12
* Gen7 v1.3
****************************************************************************************/
#if MB(GEN7_13)
  #define KNOWN_BOARD
  
  #if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__) && !defined(__AVR_ATmega1284P__)
    #error Oops! Make sure you have 'Gen7' selected from the 'Tools -> Boards' menu.
  #endif
  
  #if DISABLED(GEN7_VERSION)
    #define GEN7_VERSION   13  // v1.x
  #endif
  
  //X axis pins
  #define ORIG_X_STEP_PIN       19
  #define ORIG_X_DIR_PIN        18
  #define ORIG_X_ENABLE_PIN     24
  #define X_STOP_PIN        7
  
  //Y axis pins
  #define ORIG_Y_STEP_PIN       23
  #define ORIG_Y_DIR_PIN        22
  #define ORIG_Y_ENABLE_PIN     24
  #define Y_STOP_PIN        5
  
  //Z axis pins
  #define ORIG_Z_STEP_PIN       26
  #define ORIG_Z_DIR_PIN        25
  #define ORIG_Z_ENABLE_PIN     24
  #define ORIG_Z_MIN_PIN    1
  #define ORIG_Z_MAX_PIN    0
  
  //extruder pins
  #define ORIG_E0_STEP_PIN      28
  #define ORIG_E0_DIR_PIN       27
  #define ORIG_E0_ENABLE_PIN    24
  
  #define ORIG_TEMP_0_PIN        1
  #define ORIG_TEMP_1_PIN       -1
  #define ORIG_TEMP_2_PIN       -1
  #define ORIG_TEMP_BED_PIN      2
  
  #define ORIG_HEATER_0_PIN      4
  #define ORIG_HEATER_1_PIN     -1
  #define ORIG_HEATER_2_PIN     -1
  #define ORIG_HEATER_BED_PIN    3
  
  #define KILL_PIN         -1
  
  #define SDPOWER          -1
  #define SDSS             -1  // SCL pin of I2C header
  #define LED_PIN          -1
  
  #define ORIG_FAN_PIN          -1  // Gen7 v1.3 removed the fan pin
  #define ORIG_PS_ON_PIN        15
  
  //All these generations of Gen7 supply thermistor power
  //via PS_ON, so ignore bad thermistor readings
  #define BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  
  //our pin for debugging.
  #define DEBUG_PIN         0
  
  //our RS485 pins
  #define TORIG_X_ENABLE_PIN    12
  #define RORIG_X_ENABLE_PIN    13

#endif
/****************************************************************************************/



/****************************************************************************************
* 13
* Gen7 v1.4 pin assignment
****************************************************************************************/
#if MB(GEN7_14)
  #define GEN7_VERSION     14 // v1.4
  #define KNOWN_BOARD
  
  #if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__) && !defined(__AVR_ATmega1284P__)
    #error Oops! Make sure you have 'Gen7' selected from the 'Tools -> Boards' menu.
  #endif
  
  //X axis pins
  #define ORIG_X_STEP_PIN       29
  #define ORIG_X_DIR_PIN        28
  #define ORIG_X_ENABLE_PIN     25
  #define X_STOP_PIN        0
  
  //Y axis pins
  #define ORIG_Y_STEP_PIN       27
  #define ORIG_Y_DIR_PIN        26
  #define ORIG_Y_ENABLE_PIN     25
  #define Y_STOP_PIN        1
  
  //Z axis pins
  #define ORIG_Z_STEP_PIN       23
  #define ORIG_Z_DIR_PIN        22
  #define ORIG_Z_ENABLE_PIN     25
  #define Z_STOP_PIN        2
  
  //extruder pins
  #define ORIG_E0_STEP_PIN      19
  #define ORIG_E0_DIR_PIN       18
  #define ORIG_E0_ENABLE_PIN    25
  
  #define ORIG_TEMP_0_PIN        1
  #define ORIG_TEMP_1_PIN       -1
  #define ORIG_TEMP_2_PIN       -1
  #define ORIG_TEMP_BED_PIN      0
  
  #define ORIG_HEATER_0_PIN      4
  #define ORIG_HEATER_1_PIN     -1
  #define ORIG_HEATER_2_PIN     -1
  #define ORIG_HEATER_BED_PIN    3
  
  #define KILL_PIN         -1

  #define SDPOWER          -1
  #define SDSS             -1  // SCL pin of I2C header
  #define LED_PIN          -1

  #define ORIG_FAN_PIN          -1

  #define ORIG_PS_ON_PIN        15

  //our pin for debugging.
  #define DEBUG_PIN         0

  //our RS485 pins
  #define TORIG_X_ENABLE_PIN    12
  #define RORIG_X_ENABLE_PIN    13

#endif // GEN7
/****************************************************************************************/



/****************************************************************************************
* 2
* Cheaptronic v1.0
****************************************************************************************/
#if MB(CHEAPTRONIC)
  #define KNOWN_BOARD 1
  
  #ifndef __AVR_ATmega2560__
    #error Oops! Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
  #endif
  
  #define LARGE_FLASH        true
  
  //X motor stepper
  #define ORIG_X_STEP_PIN 14
  #define ORIG_X_DIR_PIN 15
  #define ORIG_X_ENABLE_PIN 24
  
  //X endstop
  #define ORIG_X_MIN_PIN 3
  #define ORIG_X_MAX_PIN -1
  
  //Y motor stepper
  #define ORIG_Y_STEP_PIN 35
  #define ORIG_Y_DIR_PIN 36
  #define ORIG_Y_ENABLE_PIN 31
  
  //Y endstop
  #define ORIG_Y_MIN_PIN 2
  #define ORIG_Y_MAX_PIN -1
  
  //Z motor stepper
  #define ORIG_Z_STEP_PIN 40
  #define ORIG_Z_DIR_PIN 41
  #define ORIG_Z_ENABLE_PIN 37
  
  //Z endstop
  #define ORIG_Z_MIN_PIN 5
  #define ORIG_Z_MAX_PIN -1
  
  //Extruder 0 stepper
  #define ORIG_E0_STEP_PIN 26
  #define ORIG_E0_DIR_PIN 28
  #define ORIG_E0_ENABLE_PIN 25
  
  //Extruder 1 stepper
  #define ORIG_E1_STEP_PIN 33
  #define ORIG_E1_DIR_PIN 34
  #define ORIG_E1_ENABLE_PIN 30
  
  #define SDPOWER -1
  #define SDSS -1
  #define LED_PIN -1
  
  //FAN
  #define ORIG_FAN_PIN -1
  
  #define ORIG_PS_ON_PIN -1
  #define KILL_PIN -1
  
  #define ORIG_HEATER_0_PIN 19 // EXTRUDER 1
  #define ORIG_HEATER_1_PIN 23 // EXTRUDER 2
  //HeatedBad
  #define ORIG_HEATER_BED_PIN 22
  //Cheaptronic v1.0 hasent EXTRUDER 3
  #define ORIG_HEATER_2_PIN -1
  
  //Temperature sensors
  #define ORIG_TEMP_0_PIN 15
  #define ORIG_TEMP_1_PIN 14
  #define ORIG_TEMP_2_PIN -1
  #define ORIG_TEMP_BED_PIN 13
  
  //Cheaptronic v1.0 dont support LCD
  #define LCD_PINS_RS -1
  #define LCD_PINS_ENABLE -1
  #define LCD_PINS_D4 -1
  #define LCD_PINS_D5 -1
  #define LCD_PINS_D6 -1
  #define LCD_PINS_D7 -1
  
  //Cheaptronic v1.0 dont support keypad
  #define BTN_EN1 -1
  #define BTN_EN2 -1
  #define BTN_ENC -1
  #define BLEN_C 2
  #define BLEN_B 1
  #define BLEN_A 0
  
  //Cheaptronic v1.0 does not use this port
  #define SD_DETECT_PIN -1
  
#endif // CHEAPTRONIC
/****************************************************************************************/



/****************************************************************************************
* 20
* Sethi 3D_1 pin assignment - www.sethi3d.com.br
****************************************************************************************/
#if MB(SETHI)
  #define KNOWN_BOARD
  
  #if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__) && !defined(__AVR_ATmega1284P__)
    #error Oops! Make sure you have 'Sethi 3D' selected from the 'Tools -> Boards' menu.
  #endif
  
  #if DISABLED(GEN7_VERSION)
  #define GEN7_VERSION 12 // v1.x
  #endif
  
  //x axis pins
  #define ORIG_X_STEP_PIN 19
  #define ORIG_X_DIR_PIN 18
  #define ORIG_X_ENABLE_PIN 24
  #define X_STOP_PIN 2
  
  //y axis pins
  #define ORIG_Y_STEP_PIN 23
  #define ORIG_Y_DIR_PIN 22
  #define ORIG_Y_ENABLE_PIN 24
  #define Y_STOP_PIN 0
  
  //z axis pins
  #define ORIG_Z_STEP_PIN 26
  #define ORIG_Z_DIR_PIN 25
  #define ORIG_Z_ENABLE_PIN 24
  #define ORIG_Z_MIN_PIN 1
  #define ORIG_Z_MAX_PIN 0
  
  //extruder pins
  #define ORIG_E0_STEP_PIN 28
  #define ORIG_E0_DIR_PIN 27
  #define ORIG_E0_ENABLE_PIN 24
  
  #define ORIG_TEMP_0_PIN 1
  #define ORIG_TEMP_1_PIN -1
  #define ORIG_TEMP_2_PIN -1
  #define ORIG_TEMP_BED_PIN 2
  
  #define ORIG_HEATER_0_PIN 4
  #define ORIG_HEATER_1_PIN -1
  #define ORIG_HEATER_2_PIN -1
  #define ORIG_HEATER_BED_PIN 3
  
  #define KILL_PIN -1
  
  #define SDPOWER -1
  #define SDSS -1 // SCL pin of I2C header
  #define LED_PIN -1
  
  #if (GEN7_VERSION >= 13)
  // Gen7 v1.3 removed the fan pin
  #define ORIG_FAN_PIN -1
  #else
  #define ORIG_FAN_PIN 31
  #endif
  #define ORIG_PS_ON_PIN 15
  
  //All these generations of Gen7 supply thermistor power
  //via PS_ON, so ignore bad thermistor readings
  #define BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  
  //our pin for debugging.
  #define DEBUG_PIN 0
  
  //our RS485 pins
  #define TORIG_X_ENABLE_PIN 12
  #define RORIG_X_ENABLE_PIN 13

#endif // SETHI
/****************************************************************************************/



/****************************************************************************************
* 21
* Elefu RA Board
****************************************************************************************/
#if MB(ELEFU_3)
  #define KNOWN_BOARD 1
  
  #ifndef __AVR_ATmega2560__
    #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
  #endif
  
  
  #define ORIG_X_STEP_PIN       49
  #define ORIG_X_DIR_PIN        13
  #define ORIG_X_ENABLE_PIN     48
  #define ORIG_X_MIN_PIN        35
  #define ORIG_X_MAX_PIN        -1 // 34
  
  #define ORIG_Y_STEP_PIN       11
  #define ORIG_Y_DIR_PIN        9
  #define ORIG_Y_ENABLE_PIN     12
  #define ORIG_Y_MIN_PIN        33
  #define ORIG_Y_MAX_PIN        -1 // 32
  
  #define ORIG_Z_STEP_PIN       7
  #define ORIG_Z_DIR_PIN        6
  #define ORIG_Z_ENABLE_PIN     8
  #define ORIG_Z_MIN_PIN        31
  #define ORIG_Z_MAX_PIN        -1 // 30
  
  #define ORIG_E2_STEP_PIN      43
  #define ORIG_E2_DIR_PIN       47
  #define ORIG_E2_ENABLE_PIN    42
  
  #define ORIG_E1_STEP_PIN      18
  #define ORIG_E1_DIR_PIN       19
  #define ORIG_E1_ENABLE_PIN    38
  
  #define ORIG_E0_STEP_PIN      40
  #define ORIG_E0_DIR_PIN       41
  #define ORIG_E0_ENABLE_PIN    37
  
  #define SDPOWER               -1
  #define LED_PIN               -1  // Use +12V Aux port for LED Ring
  
  #define ORIG_FAN_PIN          16  // 5V PWM
  
  #define ORIG_PS_ON_PIN        10  // Set to -1 if using a manual switch on the PWRSW Connector
  #define SLEEP_WAKE_PIN        26  // This feature still needs work
  
  #define ORIG_HEATER_0_PIN     45  // 12V PWM1
  #define ORIG_HEATER_1_PIN     46  // 12V PWM2
  #define ORIG_HEATER_2_PIN     17  // 12V PWM3
  #define ORIG_HEATER_BED_PIN   44  // DOUBLE 12V PWM
  #define ORIG_TEMP_0_PIN       3   // ANALOG NUMBERING
  #define ORIG_TEMP_1_PIN       2   // ANALOG NUMBERING
  #define ORIG_TEMP_2_PIN       1   // ANALOG NUMBERING
  #define ORIG_TEMP_BED_PIN     0   // ANALOG NUMBERING
  
  #define ORIG_BEEPER_PIN       36
  
  #define KILL_PIN              -1
  
  // M240  Triggers a camera by emulating a Canon RC-1 Remote
  // Data from: http://www.doc-diy.net/photo/rc-1_hacked/
  #define PHOTOGRAPH_PIN        29
  
  #if ENABLED(RA_CONTROL_PANEL)
    #define SDSS                53
    #define SD_DETECT_PIN       28

    #define BTN_EN1             14
    #define BTN_EN2             39
    #define BTN_ENC             15  // the click

    #define BLEN_C              2
    #define BLEN_B              1
    #define BLEN_A              0
  #endif // RA_CONTROL_PANEL
  
  #if ENABLED(RA_DISCO)
    //variables for which pins the TLC5947 is using
    #define TLC_CLOCK_PIN       25
    #define TLC_BLANK_PIN       23
    #define TLC_XLAT_PIN        22
    #define TLC_DATA_PIN        24
    
    //We also need to define pin to port number mapping for the 2560 to match the pins listed above. If you change the TLC pins, update this as well per the 2560 datasheet!
    //This currently only works with the RA Board.
    #define TLC_CLOCK_BIT 3 //bit 3 on port A
    #define TLC_CLOCK_PORT &PORTA //bit 3 on port A
    
    #define TLC_BLANK_BIT 1 //bit 1 on port A
    #define TLC_BLANK_PORT &PORTA //bit 1 on port A
    
    #define TLC_DATA_BIT 2 //bit 2 on port A
    #define TLC_DATA_PORT &PORTA //bit 2 on port A
    
    #define TLC_XLAT_BIT 0 //bit 0 on port A
    #define TLC_XLAT_PORT &PORTA //bit 0 on port A
    
    //change this to match your situation. Lots of TLCs takes up the arduino SRAM very quickly, so be careful
    //Leave it at at least 1 if you have enabled RA_LIGHTING
    //The number of TLC5947 boards chained together for use with the animation, additional ones will repeat the animation on them, but are not individually addressable and mimic those before them. You can leave the default at 2 even if you only have 1 TLC5947 module.
    #define NUM_TLCS 2
    
    //These TRANS_ARRAY values let you change the order the LEDs on the lighting modules will animate for chase functions.
    //Modify them according to your specific situation.
    //NOTE: the array should be 8 long for every TLC you have. These defaults assume (2) TLCs.
    #define TRANS_ARRAY {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8} //forwards
    //#define TRANS_ARRAY {7, 6, 5, 4, 3, 2, 1, 0, 8, 9, 10, 11, 12, 13, 14, 15} //backwards
  #endif //RA_LIGHTING

#endif // ELEFU_3
/****************************************************************************************/



/****************************************************************************************
* 22
* Gen3  Monolithic Electronics
****************************************************************************************/
#if MB(GEN3_MONOLITHIC)
  #define KNOWN_BOARD 1

  #ifndef __AVR_ATmega644P__
    #error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
  #endif

  #define DEBUG_PIN 0

  // x axis
  #define ORIG_X_STEP_PIN       15
  #define ORIG_X_DIR_PIN        18
  #define ORIG_X_MIN_PIN        20
  //Alex Checar #define X_STOP_PIN         20
  #define ORIG_X_ENABLE_PIN     24  // actually uses ORIG_Y_DIR_PIN
  #define ORIG_X_MAX_PIN        -1

  // y axes
  #define ORIG_Y_STEP_PIN       23
  #define ORIG_Y_DIR_PIN        22
  #define ORIG_Y_MIN_PIN        25
  //Alex Checar #define Y_STOP_PIN         25
  #define ORIG_Y_ENABLE_PIN     24  // shared with ORIG_X_ENABLE_PIN
  #define ORIG_Y_MAX_PIN        -1

  // z axes
  #define ORIG_Z_STEP_PIN       27
  #define ORIG_Z_DIR_PIN        28
  #define ORIG_Z_MIN_PIN        30
  //Alex Checar #define Z_STOP_PIN         30
  #define ORIG_Z_ENABLE_PIN     29
  #define ORIG_Z_MAX_PIN        -1

  //extruder pins
  #define ORIG_E0_STEP_PIN      12
  #define ORIG_E0_DIR_PIN       17
  #define ORIG_E0_ENABLE_PIN    3

  #define ORIG_HEATER_0_PIN     16
  #define ORIG_TEMP_0_PIN       0

  #define ORIG_FAN_PIN -1

  //bed pins
  #define ORIG_HEATER_BED_PIN   -1
  #define ORIG_TEMP_BED_PIN     -1

  #define SDSS                  -1
  #define SDPOWER               -1
  #define LED_PIN               -1

  //pin for controlling the PSU.
  #define ORIG_PS_ON_PIN        14

  //Alex extras from Gen3+
  #define KILL_PIN              -1
  #define ORIG_TEMP_1_PIN       -1
  #define ORIG_TEMP_2_PIN       -1
  #define ORIG_HEATER_2_PIN     -1

#endif // GEN3_MONOLITHIC
/****************************************************************************************/



/****************************************************************************************
* 3 
* RAMPS OLD
****************************************************************************************/
#if MB(RAMPS_OLD)
  #define KNOWN_BOARD 1
  
  #if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
    #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
  #endif
  
  // Uncomment the following line for RAMPS v1.0
  //#define RAMPS_V_1_0
  
  #define ORIG_X_STEP_PIN         26
  #define ORIG_X_DIR_PIN          28
  #define ORIG_X_ENABLE_PIN       24
  #define ORIG_X_MIN_PIN          3
  #define ORIG_X_MAX_PIN          2
  
  #define ORIG_Y_STEP_PIN         38
  #define ORIG_Y_DIR_PIN          40
  #define ORIG_Y_ENABLE_PIN       36
  #define ORIG_Y_MIN_PIN          16
  #define ORIG_Y_MAX_PIN          17
  
  #define ORIG_Z_STEP_PIN         44
  #define ORIG_Z_DIR_PIN          46
  #define ORIG_Z_ENABLE_PIN       42
  #define ORIG_Z_MIN_PIN          18
  #define ORIG_Z_MAX_PIN          19
  
  #define ORIG_E0_STEP_PIN        32
  #define ORIG_E0_DIR_PIN         34
  #define ORIG_E0_ENABLE_PIN      30
  
  #define SDPOWER                 48
  #define SDSS                    53
  #define LED_PIN                 13
  #define ORIG_PS_ON_PIN          -1
  #define KILL_PIN                -1
  
  #if ENABLED(RAMPS_V_1_0) // RAMPS_V_1_0
    #define ORIG_HEATER_0_PIN     12    // RAMPS 1.0
    #define ORIG_HEATER_BED_PIN   -1    // RAMPS 1.0
    #define ORIG_FAN_PIN          11    // RAMPS 1.0
  #else // RAMPS_V_1_1 or RAMPS_V_1_2
    #define ORIG_HEATER_0_PIN     10    // RAMPS 1.1
    #define ORIG_HEATER_BED_PIN    8    // RAMPS 1.1
    #define ORIG_FAN_PIN           9    // RAMPS 1.1
  #endif

  #define ORIG_HEATER_1_PIN       -1
  #define ORIG_HEATER_2_PIN       -1
  #define ORIG_TEMP_0_PIN          2    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
  #define ORIG_TEMP_1_PIN         -1
  #define ORIG_TEMP_2_PIN         -1
  #define ORIG_TEMP_BED_PIN        1    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
  
  // SPI for Max6675 Thermocouple
  #if DISABLED(SDSUPPORT)
    #define MAX6675_SS            66  // Do not use pin 53 if there is even the remote possibility of using Display/SD card
  #else
    #define MAX6675_SS            66  // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present
  #endif

#endif // RAMPS_OLD
/****************************************************************************************/



/****************************************************************************************
* 33
* RAMPS 1.3 / 1.4
* RAMPS_13_HFB (Hotend0, Fan, Bed)
****************************************************************************************/
#if MB(RAMPS_13_HFB)
  #define KNOWN_BOARD 1
  
  #if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
    #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
  #endif
  
  #define LARGE_FLASH true
  
  // X axis pins
  #define ORIG_X_STEP_PIN         54
  #define ORIG_X_DIR_PIN          55
  #define ORIG_X_ENABLE_PIN       38
  #define ORIG_X_MIN_PIN          3
  #define ORIG_X_MAX_PIN          2
  
  // Y axis pins
  #define ORIG_Y_STEP_PIN         60
  #define ORIG_Y_DIR_PIN          61
  #define ORIG_Y_ENABLE_PIN       56
  #define ORIG_Y_MIN_PIN          14
  #define ORIG_Y_MAX_PIN          15
  
  #define Y2_STEP_PIN             36
  #define Y2_DIR_PIN              34
  #define Y2_ENABLE_PIN           30

  // Z axis pins
  #define ORIG_Z_STEP_PIN         46
  #define ORIG_Z_DIR_PIN          48
  #define ORIG_Z_ENABLE_PIN       62
  #define ORIG_Z_MIN_PIN          18
  #define ORIG_Z_MAX_PIN          19

  #define Z2_STEP_PIN             36
  #define Z2_DIR_PIN              34
  #define Z2_ENABLE_PIN           30

  // E axis pins
  #define ORIG_E0_STEP_PIN        26
  #define ORIG_E0_DIR_PIN         28
  #define ORIG_E0_ENABLE_PIN      24

  #define ORIG_E1_STEP_PIN        36
  #define ORIG_E1_DIR_PIN         34
  #define ORIG_E1_ENABLE_PIN      30
  
  #define SDPOWER                 -1
  #define SDSS                    53
  #define LED_PIN                 13
  
  #define ORIG_FAN_PIN            9
  #define ORIG_PS_ON_PIN          12
  
  #define ORIG_HEATER_0_PIN       10  // Hotend 1
  #define ORIG_HEATER_1_PIN       -1
  #define ORIG_HEATER_2_PIN       -1
  #define ORIG_HEATER_3_PIN       -1

  #define ORIG_TEMP_0_PIN         13  // ANALOG NUMBERING
  #define ORIG_TEMP_1_PIN         15  // ANALOG NUMBERING
  #define ORIG_TEMP_2_PIN         -1  // ANALOG NUMBERING
  #define ORIG_TEMP_3_PIN         -1  // ANALOG NUMBERING
  
  #define ORIG_HEATER_BED_PIN     8   // BED
  #define ORIG_TEMP_BED_PIN       14  // ANALOG NUMBERING

  #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) || ENABLED(G3D_PANEL)
    #define KILL_PIN              41
  #else
    #define KILL_PIN              -1
  #endif

  #if NUM_SERVOS > 0
    #define SERVO0_PIN            11
    #if NUM_SERVOS > 1
      #define SERVO1_PIN          6
      #if NUM_SERVOS > 2
        #define SERVO2_PIN        5
        #if NUM_SERVOS > 3
          #define SERVO3_PIN      4
        #endif
      #endif
    #endif
  #endif

  #if ENABLED(ULTRA_LCD)
    #if ENABLED(NEWPANEL)
      #if ENABLED(PANEL_ONE)
        #define LCD_PINS_RS       40
        #define LCD_PINS_ENABLE   42
        #define LCD_PINS_D4       65
        #define LCD_PINS_D5       66
        #define LCD_PINS_D6       44
        #define LCD_PINS_D7       64
      #else
        #define LCD_PINS_RS       16
        #define LCD_PINS_ENABLE   17
        #define LCD_PINS_D4       23
        #define LCD_PINS_D5       25
        #define LCD_PINS_D6       27
        #define LCD_PINS_D7       29
      #endif // PANEL_ONE
  
      #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
        #define ORIG_BEEPER_PIN   37

        #define BTN_EN1           31
        #define BTN_EN2           33
        #define BTN_ENC           35

        #define SD_DETECT_PIN     49
      #elif ENABLED(LCD_I2C_PANELOLU2)
        #define BTN_EN1           47  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           43
        #define BTN_ENC           32
        #define LCD_SDSS          53
        #define SD_DETECT_PIN     -1
        #define KILL_PIN          41
      #elif ENABLED(LCD_I2C_VIKI)
        #define BTN_EN1           22  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           7
        #define BTN_ENC           -1
        #define LCD_SDSS          53
        #define SD_DETECT_PIN     49
      #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
        #define BTN_EN1           35  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           37
        #define BTN_ENC           31
        #define SD_DETECT_PIN     49
        #define LCD_SDSS          53
        #define KILL_PIN          41
        #define ORIG_BEEPER_PIN   23
        #define DOGLCD_CS         29
        #define DOGLCD_A0         27
        #define LCD_PIN_BL        33
      #else
        // arduino pin which triggers an piezzo beeper
        #define ORIG_BEEPER_PIN   33  // Beeper on AUX-4

        // buttons are directly attached using AUX-2
        #if ENABLED(REPRAPWORLD_KEYPAD)
          #define BTN_EN1         64  // encoder
          #define BTN_EN2         59  // encoder
          #define BTN_ENC         63  // enter button
          #define SHIFT_OUT       40  // shift register
          #define SHIFT_CLK       44  // shift register
          #define SHIFT_LD        42  // shift register
        #elif ENABLED(PANEL_ONE)
          #define BTN_EN1         59  // AUX2 PIN 3
          #define BTN_EN2         63  // AUX2 PIN 4
          #define BTN_ENC         49  // AUX3 PIN 7
        #else
          #define BTN_EN1         37
          #define BTN_EN2         35
          #define BTN_ENC         31  // the click
        #endif
  
        #if ENABLED(G3D_PANEL)
          #define SD_DETECT_PIN   49
        #else
          #define SD_DETECT_PIN   -1  // Ramps does not use this port
        #endif

      #endif
  
    #else // old style panel with shift register
      // arduino pin witch triggers an piezo beeper
      #define ORIG_BEEPER_PIN     33  // No Beeper added

      //buttons are attached to a shift register
      // Not wired this yet
      //#define SHIFT_CLK         38
      //#define SHIFT_LD          42
      //#define SHIFT_OUT         40
      //#define SHIFT_EN          17
  
      #define LCD_PINS_RS         16
      #define LCD_PINS_ENABLE     17
      #define LCD_PINS_D4         23
      #define LCD_PINS_D5         25
      #define LCD_PINS_D6         27
      #define LCD_PINS_D7         29
    #endif // NEWPANEL
  #endif // ULTRA_LCD

  // SPI for Max6675 Thermocouple
  #define MAX6675_SS              66  // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present

#endif // RAMPS_13_HFB
/****************************************************************************************/



/****************************************************************************************
* 34
* RAMPS 1.3 / 1.4
* RAMPS_13_HHB (Hotend0, Hotend1, Bed)
****************************************************************************************/
#if MB(RAMPS_13_HHB)
  #define KNOWN_BOARD 1

  #if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
    #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
  #endif

  #define LARGE_FLASH true

  // X axis pins
  #define ORIG_X_STEP_PIN         54
  #define ORIG_X_DIR_PIN          55
  #define ORIG_X_ENABLE_PIN       38
  #define ORIG_X_MIN_PIN          3
  #define ORIG_X_MAX_PIN          2

  // Y axis pins
  #define ORIG_Y_STEP_PIN         60
  #define ORIG_Y_DIR_PIN          61
  #define ORIG_Y_ENABLE_PIN       56
  #define ORIG_Y_MIN_PIN          14
  #define ORIG_Y_MAX_PIN          15

  #define Y2_STEP_PIN             36
  #define Y2_DIR_PIN              34
  #define Y2_ENABLE_PIN           30

  // Z axis pins
  #define ORIG_Z_STEP_PIN         46
  #define ORIG_Z_DIR_PIN          48
  #define ORIG_Z_ENABLE_PIN       62
  #define ORIG_Z_MIN_PIN          18
  #define ORIG_Z_MAX_PIN          19

  #define Z2_STEP_PIN             36
  #define Z2_DIR_PIN              34
  #define Z2_ENABLE_PIN           30

  // E axis pins
  #define ORIG_E0_STEP_PIN        26
  #define ORIG_E0_DIR_PIN         28
  #define ORIG_E0_ENABLE_PIN      24

  #define ORIG_E1_STEP_PIN        36
  #define ORIG_E1_DIR_PIN         34
  #define ORIG_E1_ENABLE_PIN      30

  #define SDPOWER                 -1
  #define SDSS                    53
  #define LED_PIN                 13

  #define ORIG_FAN_PIN            4
  #define ORIG_PS_ON_PIN          12

  #define ORIG_HEATER_0_PIN       10  // HOTEND 1
  #define ORIG_HEATER_1_PIN       9   // HOTEND 2
  #define ORIG_HEATER_2_PIN       -1
  #define ORIG_HEATER_3_PIN       -1

  #define ORIG_TEMP_0_PIN         13  // ANALOG NUMBERING
  #define ORIG_TEMP_1_PIN         15  // ANALOG NUMBERING
  #define ORIG_TEMP_2_PIN         -1  // ANALOG NUMBERING
  #define ORIG_TEMP_3_PIN         -1  // ANALOG NUMBERING

  #define ORIG_HEATER_BED_PIN     8   // BED
  #define ORIG_TEMP_BED_PIN       14  // ANALOG NUMBERING

  #if NUM_SERVOS > 0
    #define SERVO0_PIN            11
    #if NUM_SERVOS > 1
      #define SERVO1_PIN          6
      #if NUM_SERVOS > 2
        #define SERVO2_PIN        5
        #if NUM_SERVOS > 3
          #define SERVO3_PIN      4
        #endif
      #endif
    #endif
  #endif

  #if ENABLED(ULTRA_LCD)
    #if ENABLED(NEWPANEL)
      #if ENABLED(PANEL_ONE)
        #define LCD_PINS_RS       40
        #define LCD_PINS_ENABLE   42
        #define LCD_PINS_D4       65
        #define LCD_PINS_D5       66
        #define LCD_PINS_D6       44
        #define LCD_PINS_D7       64
      #else
        #define LCD_PINS_RS       16
        #define LCD_PINS_ENABLE   17
        #define LCD_PINS_D4       23
        #define LCD_PINS_D5       25
        #define LCD_PINS_D6       27
        #define LCD_PINS_D7       29
      #endif // PANEL_ONE

      #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
        #define ORIG_BEEPER_PIN   37

        #define BTN_EN1           31
        #define BTN_EN2           33
        #define BTN_ENC           35

        #define SD_DETECT_PIN     49
      #elif ENABLED(LCD_I2C_PANELOLU2)
        #define BTN_EN1           47  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           43
        #define BTN_ENC           32
        #define LCD_SDSS          53
        #define SD_DETECT_PIN     -1
        #define KILL_PIN          41
      #elif ENABLED(LCD_I2C_VIKI)
        #define BTN_EN1           22  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           7
        #define BTN_ENC           -1
        #define LCD_SDSS          53
        #define SD_DETECT_PIN     49
      #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
        #define BTN_EN1           35  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           37
        #define BTN_ENC           31
        #define SD_DETECT_PIN     49
        #define LCD_SDSS          53
        #define KILL_PIN          41
        #define ORIG_BEEPER_PIN   23
        #define DOGLCD_CS         29
        #define DOGLCD_A0         27
        #define LCD_PIN_BL        33
      #else
        // arduino pin which triggers an piezzo beeper
        #define ORIG_BEEPER_PIN   33  // Beeper on AUX-4

        // buttons are directly attached using AUX-2
        #if ENABLED(REPRAPWORLD_KEYPAD)
          #define BTN_EN1         64  // encoder
          #define BTN_EN2         59  // encoder
          #define BTN_ENC         63  // enter button
          #define SHIFT_OUT       40  // shift register
          #define SHIFT_CLK       44  // shift register
          #define SHIFT_LD        42  // shift register
        #elif ENABLED(PANEL_ONE)
          #define BTN_EN1         59  // AUX2 PIN 3
          #define BTN_EN2         63  // AUX2 PIN 4
          #define BTN_ENC         49  // AUX3 PIN 7
        #else
          #define BTN_EN1         37
          #define BTN_EN2         35
          #define BTN_ENC         31  // the click
        #endif

        #if ENABLED(G3D_PANEL)
          #define SD_DETECT_PIN   49
        #else
          #define SD_DETECT_PIN   -1  // Ramps does not use this port
        #endif

      #endif

    #else // old style panel with shift register
      // arduino pin witch triggers an piezo beeper
      #define ORIG_BEEPER_PIN     33  // No Beeper added

      //buttons are attached to a shift register
      // Not wired this yet
      //#define SHIFT_CLK         38
      //#define SHIFT_LD          42
      //#define SHIFT_OUT         40
      //#define SHIFT_EN          17

      #define LCD_PINS_RS         16
      #define LCD_PINS_ENABLE     17
      #define LCD_PINS_D4         23
      #define LCD_PINS_D5         25
      #define LCD_PINS_D6         27
      #define LCD_PINS_D7         29
    #endif // NEWPANEL
  #endif // ULTRA_LCD

  // SPI for Max6675 Thermocouple
  #define MAX6675_SS              66  // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present

#endif // RAMPS_13_HHB
/****************************************************************************************/



/****************************************************************************************
* 35
* RAMPS 1.3 / 1.4
* RAMPS_13_HFF (Hotend0, Fan, Fan)
****************************************************************************************/
#if MB(RAMPS_13_HFF)
  #define KNOWN_BOARD 1

  #if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
    #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
  #endif

  #define LARGE_FLASH true

  // X axis pins
  #define ORIG_X_STEP_PIN         54
  #define ORIG_X_DIR_PIN          55
  #define ORIG_X_ENABLE_PIN       38
  #define ORIG_X_MIN_PIN          3
  #define ORIG_X_MAX_PIN          2

  // Y axis pins
  #define ORIG_Y_STEP_PIN         60
  #define ORIG_Y_DIR_PIN          61
  #define ORIG_Y_ENABLE_PIN       56
  #define ORIG_Y_MIN_PIN          14
  #define ORIG_Y_MAX_PIN          15

  #define Y2_STEP_PIN             36
  #define Y2_DIR_PIN              34
  #define Y2_ENABLE_PIN           30

  // Z axis pins
  #define ORIG_Z_STEP_PIN         46
  #define ORIG_Z_DIR_PIN          48
  #define ORIG_Z_ENABLE_PIN       62
  #define ORIG_Z_MIN_PIN          18
  #define ORIG_Z_MAX_PIN          19

  #define Z2_STEP_PIN             36
  #define Z2_DIR_PIN              34
  #define Z2_ENABLE_PIN           30

  // E axis pins
  #define ORIG_E0_STEP_PIN        26
  #define ORIG_E0_DIR_PIN         28
  #define ORIG_E0_ENABLE_PIN      24

  #define ORIG_E1_STEP_PIN        36
  #define ORIG_E1_DIR_PIN         34
  #define ORIG_E1_ENABLE_PIN      30

  #define SDPOWER                 -1
  #define SDSS                    53
  #define LED_PIN                 13

  #define ORIG_FAN_PIN            9
  #define ORIG_PS_ON_PIN          12

  #define ORIG_HEATER_0_PIN       10  // HOTEND 1
  #define ORIG_HEATER_1_PIN       -1
  #define ORIG_HEATER_2_PIN       -1
  #define ORIG_HEATER_3_PIN       -1

  #define ORIG_TEMP_0_PIN         13  // ANALOG NUMBERING
  #define ORIG_TEMP_1_PIN         15  // ANALOG NUMBERING
  #define ORIG_TEMP_2_PIN         -1  // ANALOG NUMBERING
  #define ORIG_TEMP_3_PIN         -1  // ANALOG NUMBERING

  #define ORIG_HEATER_BED_PIN     -1  // BED
  #define ORIG_TEMP_BED_PIN       14  // ANALOG NUMBERING

  #if NUM_SERVOS > 0
    #define SERVO0_PIN            11
    #if NUM_SERVOS > 1
      #define SERVO1_PIN          6
      #if NUM_SERVOS > 2
        #define SERVO2_PIN        5
        #if NUM_SERVOS > 3
          #define SERVO3_PIN      4
        #endif
      #endif
    #endif
  #endif

  #if ENABLED(ULTRA_LCD)
    #if ENABLED(NEWPANEL)
      #if ENABLED(PANEL_ONE)
        #define LCD_PINS_RS       40
        #define LCD_PINS_ENABLE   42
        #define LCD_PINS_D4       65
        #define LCD_PINS_D5       66
        #define LCD_PINS_D6       44
        #define LCD_PINS_D7       64
      #else
        #define LCD_PINS_RS       16
        #define LCD_PINS_ENABLE   17
        #define LCD_PINS_D4       23
        #define LCD_PINS_D5       25
        #define LCD_PINS_D6       27
        #define LCD_PINS_D7       29
      #endif // PANEL_ONE
  
      #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
        #define ORIG_BEEPER_PIN   37

        #define BTN_EN1           31
        #define BTN_EN2           33
        #define BTN_ENC           35

        #define SD_DETECT_PIN     49
      #elif ENABLED(LCD_I2C_PANELOLU2)
        #define BTN_EN1           47  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           43
        #define BTN_ENC           32
        #define LCD_SDSS          53
        #define SD_DETECT_PIN     -1
        #define KILL_PIN          41
      #elif ENABLED(LCD_I2C_VIKI)
        #define BTN_EN1           22  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           7
        #define BTN_ENC           -1
        #define LCD_SDSS          53
        #define SD_DETECT_PIN     49
      #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
        #define BTN_EN1           35  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           37
        #define BTN_ENC           31
        #define SD_DETECT_PIN     49
        #define LCD_SDSS          53
        #define KILL_PIN          41
        #define ORIG_BEEPER_PIN   23
        #define DOGLCD_CS         29
        #define DOGLCD_A0         27
        #define LCD_PIN_BL        33
      #else
        // arduino pin which triggers an piezzo beeper
        #define ORIG_BEEPER_PIN   33  // Beeper on AUX-4

        // buttons are directly attached using AUX-2
        #if ENABLED(REPRAPWORLD_KEYPAD)
          #define BTN_EN1         64  // encoder
          #define BTN_EN2         59  // encoder
          #define BTN_ENC         63  // enter button
          #define SHIFT_OUT       40  // shift register
          #define SHIFT_CLK       44  // shift register
          #define SHIFT_LD        42  // shift register
        #elif ENABLED(PANEL_ONE)
          #define BTN_EN1         59  // AUX2 PIN 3
          #define BTN_EN2         63  // AUX2 PIN 4
          #define BTN_ENC         49  // AUX3 PIN 7
        #else
          #define BTN_EN1         37
          #define BTN_EN2         35
          #define BTN_ENC         31  // the click
        #endif

        #if ENABLED(G3D_PANEL)
          #define SD_DETECT_PIN   49
        #else
          #define SD_DETECT_PIN   -1  // Ramps does not use this port
        #endif

      #endif

    #else // old style panel with shift register
      // arduino pin witch triggers an piezo beeper
      #define ORIG_BEEPER_PIN     33  // No Beeper added

      //buttons are attached to a shift register
      // Not wired this yet
      //#define SHIFT_CLK         38
      //#define SHIFT_LD          42
      //#define SHIFT_OUT         40
      //#define SHIFT_EN          17

      #define LCD_PINS_RS         16
      #define LCD_PINS_ENABLE     17
      #define LCD_PINS_D4         23
      #define LCD_PINS_D5         25
      #define LCD_PINS_D6         27
      #define LCD_PINS_D7         29
    #endif // NEWPANEL
  #endif // ULTRA_LCD

  // SPI for Max6675 Thermocouple
  #define MAX6675_SS              66  // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present

#endif // RAMPS_13_HFF
/****************************************************************************************/



/****************************************************************************************
* 36
* RAMPS 1.3 / 1.4
* RAMPS_13_HHF (Hotend0, Hotend1, Fan)
****************************************************************************************/
#if MB(RAMPS_13_HHF)
  #define KNOWN_BOARD 1

  #if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
    #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
  #endif

  #define LARGE_FLASH true

  // X axis pins
  #define ORIG_X_STEP_PIN         54
  #define ORIG_X_DIR_PIN          55
  #define ORIG_X_ENABLE_PIN       38
  #define ORIG_X_MIN_PIN           3
  #define ORIG_X_MAX_PIN           2

  // Y axis pins
  #define ORIG_Y_STEP_PIN         60
  #define ORIG_Y_DIR_PIN          61
  #define ORIG_Y_ENABLE_PIN       56
  #define ORIG_Y_MIN_PIN          14
  #define ORIG_Y_MAX_PIN          15

  #define Y2_STEP_PIN             36
  #define Y2_DIR_PIN              34
  #define Y2_ENABLE_PIN           30

  // Z axis pins
  #define ORIG_Z_STEP_PIN         46
  #define ORIG_Z_DIR_PIN          48
  #define ORIG_Z_ENABLE_PIN       62
  #define ORIG_Z_MIN_PIN          18
  #define ORIG_Z_MAX_PIN          19

  #define Z2_STEP_PIN             36
  #define Z2_DIR_PIN              34
  #define Z2_ENABLE_PIN           30

  // E axis pins
  #define ORIG_E0_STEP_PIN        26
  #define ORIG_E0_DIR_PIN         28
  #define ORIG_E0_ENABLE_PIN      24

  #define ORIG_E1_STEP_PIN        36
  #define ORIG_E1_DIR_PIN         34
  #define ORIG_E1_ENABLE_PIN      30

  #define SDPOWER                 -1
  #define SDSS                    53
  #define LED_PIN                 13

  #define ORIG_FAN_PIN            8
  #define ORIG_PS_ON_PIN          12

  #define ORIG_HEATER_0_PIN       10   // HOTEND 1
  #define ORIG_HEATER_1_PIN        9   // HOTEND 2
  #define ORIG_HEATER_2_PIN       -1
  #define ORIG_HEATER_3_PIN       -1

  #define ORIG_TEMP_0_PIN         13   // ANALOG NUMBERING
  #define ORIG_TEMP_1_PIN         15   // ANALOG NUMBERING
  #define ORIG_TEMP_2_PIN         -1   // ANALOG NUMBERING
  #define ORIG_TEMP_3_PIN         -1   // ANALOG NUMBERING

  #define ORIG_HEATER_BED_PIN      8   // BED
  #define ORIG_TEMP_BED_PIN       14   // ANALOG NUMBERING

  #if NUM_SERVOS > 0
    #define SERVO0_PIN            11
    #if NUM_SERVOS > 1
      #define SERVO1_PIN          6
      #if NUM_SERVOS > 2
        #define SERVO2_PIN        5
        #if NUM_SERVOS > 3
          #define SERVO3_PIN      4
        #endif
      #endif
    #endif
  #endif

  #if ENABLED(ULTRA_LCD)
    #if ENABLED(NEWPANEL)
      #if ENABLED(PANEL_ONE)
        #define LCD_PINS_RS       40
        #define LCD_PINS_ENABLE   42
        #define LCD_PINS_D4       65
        #define LCD_PINS_D5       66
        #define LCD_PINS_D6       44
        #define LCD_PINS_D7       64
      #else
        #define LCD_PINS_RS       16
        #define LCD_PINS_ENABLE   17
        #define LCD_PINS_D4       23
        #define LCD_PINS_D5       25
        #define LCD_PINS_D6       27
        #define LCD_PINS_D7       29
      #endif // PANEL_ONE
  
      #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
        #define ORIG_BEEPER_PIN   37

        #define BTN_EN1           31
        #define BTN_EN2           33
        #define BTN_ENC           35

        #define SD_DETECT_PIN     49
      #elif ENABLED(LCD_I2C_PANELOLU2)
        #define BTN_EN1           47  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           43
        #define BTN_ENC           32
        #define LCD_SDSS          53
        #define SD_DETECT_PIN     -1
        #define KILL_PIN          41
      #elif ENABLED(LCD_I2C_VIKI)
        #define BTN_EN1           22  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           7
        #define BTN_ENC           -1
        #define LCD_SDSS          53
        #define SD_DETECT_PIN     49
      #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
        #define BTN_EN1           35  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           37
        #define BTN_ENC           31
        #define SD_DETECT_PIN     49
        #define LCD_SDSS          53
        #define KILL_PIN          41
        #define ORIG_BEEPER_PIN   23
        #define DOGLCD_CS         29
        #define DOGLCD_A0         27
        #define LCD_PIN_BL        33
      #else
        // arduino pin which triggers an piezzo beeper
        #define ORIG_BEEPER_PIN   33  // Beeper on AUX-4

        // buttons are directly attached using AUX-2
        #if ENABLED(REPRAPWORLD_KEYPAD)
          #define BTN_EN1         64  // encoder
          #define BTN_EN2         59  // encoder
          #define BTN_ENC         63  // enter button
          #define SHIFT_OUT       40  // shift register
          #define SHIFT_CLK       44  // shift register
          #define SHIFT_LD        42  // shift register
        #elif ENABLED(PANEL_ONE)
          #define BTN_EN1         59  // AUX2 PIN 3
          #define BTN_EN2         63  // AUX2 PIN 4
          #define BTN_ENC         49  // AUX3 PIN 7
        #else
          #define BTN_EN1         37
          #define BTN_EN2         35
          #define BTN_ENC         31  // the click
        #endif

        #if ENABLED(G3D_PANEL)
          #define SD_DETECT_PIN   49
        #else
          #define SD_DETECT_PIN   -1  // Ramps does not use this port
        #endif

      #endif

    #else // old style panel with shift register
      // arduino pin witch triggers an piezo beeper
      #define ORIG_BEEPER_PIN     33  // No Beeper added

      // buttons are attached to a shift register
      // Not wired this yet
      //#define SHIFT_CLK         38
      //#define SHIFT_LD          42
      //#define SHIFT_OUT         40
      //#define SHIFT_EN          17

      #define LCD_PINS_RS         16
      #define LCD_PINS_ENABLE     17
      #define LCD_PINS_D4         23
      #define LCD_PINS_D5         25
      #define LCD_PINS_D6         27
      #define LCD_PINS_D7         29
    #endif // NEWPANEL
  #endif // ULTRA_LCD

  // SPI for Max6675 Thermocouple
  #define MAX6675_SS              66  // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present

#endif // RAMPS_13_HHF
/****************************************************************************************/



/****************************************************************************************
* 37
* RAMPS 1.3 / 1.4
* RAMPS_13_HHH (Hotend0, Hotend1, Hotend2)
****************************************************************************************/
#if MB(RAMPS_13_HHH)
  #define KNOWN_BOARD 1

  #if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
    #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
  #endif

  #define LARGE_FLASH true

  // X axis pins
  #define ORIG_X_STEP_PIN         54
  #define ORIG_X_DIR_PIN          55
  #define ORIG_X_ENABLE_PIN       38
  #define ORIG_X_MIN_PIN          3
  #define ORIG_X_MAX_PIN          2

  // Y axis pins
  #define ORIG_Y_STEP_PIN         60
  #define ORIG_Y_DIR_PIN          61
  #define ORIG_Y_ENABLE_PIN       56
  #define ORIG_Y_MIN_PIN          14
  #define ORIG_Y_MAX_PIN          15

  #define Y2_STEP_PIN             36
  #define Y2_DIR_PIN              34
  #define Y2_ENABLE_PIN           30

  // Z axis pins
  #define ORIG_Z_STEP_PIN         46
  #define ORIG_Z_DIR_PIN          48
  #define ORIG_Z_ENABLE_PIN       62
  #define ORIG_Z_MIN_PIN          18
  #define ORIG_Z_MAX_PIN          19

  #define Z2_STEP_PIN             36
  #define Z2_DIR_PIN              34
  #define Z2_ENABLE_PIN           30

  // E axis pins
  #define ORIG_E0_STEP_PIN        26
  #define ORIG_E0_DIR_PIN         28
  #define ORIG_E0_ENABLE_PIN      24

  #define ORIG_E1_STEP_PIN        36
  #define ORIG_E1_DIR_PIN         34
  #define ORIG_E1_ENABLE_PIN      30

  #define SDPOWER                 -1
  #define SDSS                    53
  #define LED_PIN                 13

  #define ORIG_FAN_PIN            8
  #define ORIG_PS_ON_PIN          12

  #define ORIG_HEATER_0_PIN       10  // HOTEND 1
  #define ORIG_HEATER_1_PIN       9   // HOTEND 2
  #define ORIG_HEATER_2_PIN       8   // HOTEND 3
  #define ORIG_HEATER_3_PIN       -1

  #define ORIG_TEMP_0_PIN         13  // ANALOG NUMBERING
  #define ORIG_TEMP_1_PIN         15  // ANALOG NUMBERING
  #define ORIG_TEMP_2_PIN         14  // ANALOG NUMBERING
  #define ORIG_TEMP_3_PIN         -1  // ANALOG NUMBERING

  #define ORIG_HEATER_BED_PIN     -1  // BED
  #define ORIG_TEMP_BED_PIN       -1  // ANALOG NUMBERING

  #if NUM_SERVOS > 0
    #define SERVO0_PIN            11
    #if NUM_SERVOS > 1
      #define SERVO1_PIN          6
      #if NUM_SERVOS > 2
        #define SERVO2_PIN        5
        #if NUM_SERVOS > 3
          #define SERVO3_PIN      4
        #endif
      #endif
    #endif
  #endif

  #if ENABLED(ULTRA_LCD)
    #if ENABLED(NEWPANEL)
      #if ENABLED(PANEL_ONE)
        #define LCD_PINS_RS       40
        #define LCD_PINS_ENABLE   42
        #define LCD_PINS_D4       65
        #define LCD_PINS_D5       66
        #define LCD_PINS_D6       44
        #define LCD_PINS_D7       64
      #else
        #define LCD_PINS_RS       16
        #define LCD_PINS_ENABLE   17
        #define LCD_PINS_D4       23
        #define LCD_PINS_D5       25
        #define LCD_PINS_D6       27
        #define LCD_PINS_D7       29
      #endif // PANEL_ONE

      #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
        #define ORIG_BEEPER_PIN   37

        #define BTN_EN1           31
        #define BTN_EN2           33
        #define BTN_ENC           35

        #define SD_DETECT_PIN     49
      #elif ENABLED(LCD_I2C_PANELOLU2)
        #define BTN_EN1           47  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           43
        #define BTN_ENC           32
        #define LCD_SDSS          53
        #define SD_DETECT_PIN     -1
        #define KILL_PIN          41
      #elif ENABLED(LCD_I2C_VIKI)
        #define BTN_EN1           22  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           7
        #define BTN_ENC           -1
        #define LCD_SDSS          53
        #define SD_DETECT_PIN     49
      #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
        #define BTN_EN1           35  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           37
        #define BTN_ENC           31
        #define SD_DETECT_PIN     49
        #define LCD_SDSS          53
        #define KILL_PIN          41
        #define ORIG_BEEPER_PIN   23
        #define DOGLCD_CS         29
        #define DOGLCD_A0         27
        #define LCD_PIN_BL        33
      #else
        // arduino pin which triggers an piezzo beeper
        #define ORIG_BEEPER_PIN   33  // Beeper on AUX-4

        // buttons are directly attached using AUX-2
        #if ENABLED(REPRAPWORLD_KEYPAD)
          #define BTN_EN1         64  // encoder
          #define BTN_EN2         59  // encoder
          #define BTN_ENC         63  // enter button
          #define SHIFT_OUT       40  // shift register
          #define SHIFT_CLK       44  // shift register
          #define SHIFT_LD        42  // shift register
        #elif ENABLED(PANEL_ONE)
          #define BTN_EN1         59  // AUX2 PIN 3
          #define BTN_EN2         63  // AUX2 PIN 4
          #define BTN_ENC         49  // AUX3 PIN 7
        #else
          #define BTN_EN1         37
          #define BTN_EN2         35
          #define BTN_ENC         31  // the click
        #endif

        #if ENABLED(G3D_PANEL)
          #define SD_DETECT_PIN   49
        #else
          #define SD_DETECT_PIN   -1  // Ramps does not use this port
        #endif

      #endif

    #else // old style panel with shift register
      // arduino pin witch triggers an piezo beeper
      #define ORIG_BEEPER_PIN     33  // No Beeper added

      // buttons are attached to a shift register
      // Not wired this yet
      //#define SHIFT_CLK         38
      //#define SHIFT_LD          42
      //#define SHIFT_OUT         40
      //#define SHIFT_EN          17
  
      #define LCD_PINS_RS         16
      #define LCD_PINS_ENABLE     17
      #define LCD_PINS_D4         23
      #define LCD_PINS_D5         25
      #define LCD_PINS_D6         27
      #define LCD_PINS_D7         29
    #endif // NEWPANEL
  #endif // ULTRA_LCD

  // SPI for Max6675 Thermocouple
  #define MAX6675_SS              66  // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present

#endif // RAMPS_13_HHH
/****************************************************************************************/



/****************************************************************************************
* 301
* Rambo
****************************************************************************************/
#if MB(RAMBO)
#define KNOWN_BOARD

#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH true

#define ORIG_X_STEP_PIN 37
#define ORIG_X_DIR_PIN 48
#define ORIG_X_MIN_PIN 12
#define ORIG_X_MAX_PIN 24
#define ORIG_X_ENABLE_PIN 29
#define X_MS1_PIN 40
#define X_MS2_PIN 41

#define ORIG_Y_STEP_PIN 36
#define ORIG_Y_DIR_PIN 49
#define ORIG_Y_MIN_PIN 11
#define ORIG_Y_MAX_PIN 23
#define ORIG_Y_ENABLE_PIN 28
#define Y_MS1_PIN 69
#define Y_MS2_PIN 39

#define ORIG_Z_STEP_PIN 35
#define ORIG_Z_DIR_PIN 47
#define ORIG_Z_MIN_PIN 10
#define ORIG_Z_MAX_PIN 30
#define ORIG_Z_ENABLE_PIN 27
#define Z_MS1_PIN 68
#define Z_MS2_PIN 67

#define ORIG_HEATER_BED_PIN 3
#define ORIG_TEMP_BED_PIN 2

#define ORIG_HEATER_0_PIN  9
#define ORIG_TEMP_0_PIN 0

#define ORIG_HEATER_1_PIN 7
#define ORIG_TEMP_1_PIN 1

#if ENABLED(BARICUDA)
#define ORIG_HEATER_2_PIN 6
#else
#define ORIG_HEATER_2_PIN -1
#endif
#define ORIG_TEMP_2_PIN -1

#define ORIG_E0_STEP_PIN         34
#define ORIG_E0_DIR_PIN          43
#define ORIG_E0_ENABLE_PIN       26
#define E0_MS1_PIN 65
#define E0_MS2_PIN 66

#define ORIG_E1_STEP_PIN         33
#define ORIG_E1_DIR_PIN          42
#define ORIG_E1_ENABLE_PIN       25
#define E1_MS1_PIN 63
#define E1_MS2_PIN 64

#define DIGIPOTSS_PIN 38
#define DIGIPOT_CHANNELS {4,5,3,0,1} // X Y Z E0 E1 digipot channels to stepper driver mapping

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13
#define ORIG_FAN_PIN        8
#define ORIG_PS_ON_PIN      4
#define KILL_PIN           -1 // 80 with Smart Controller LCD
#define SUICIDE_PIN        -1 // PIN that has to be turned on right after start, to keep power flowing.

#if ENABLED(ULTRA_LCD)
  #define KILL_PIN 80
  #if ENABLED(NEWPANEL)
   // arduino pin which triggers an piezzo beeper
    #define ORIG_BEEPER_PIN 79      // Beeper on AUX-4
    #define LCD_PINS_RS 70
    #define LCD_PINS_ENABLE 71
    #define LCD_PINS_D4 72
    #define LCD_PINS_D5 73
    #define LCD_PINS_D6 74
    #define LCD_PINS_D7 75

    //buttons are directly attached using AUX-2
    #define BTN_EN1 76
    #define BTN_EN2 77
    #define BTN_ENC 78  //the click

    #define BLEN_C 2
    #define BLEN_B 1
    #define BLEN_A 0

    #define SD_DETECT_PIN 81    // Ramps does not use this port

  #else //old style panel with shift register
    //arduino pin witch triggers an piezzo beeper
    #define ORIG_BEEPER_PIN 33    No Beeper added
    //buttons are attached to a shift register
    // Not wired this yet
    // #define SHIFT_CLK 38
    // #define SHIFT_LD 42
    // #define SHIFT_OUT 40
    // #define SHIFT_EN 17

    #define LCD_PINS_RS 75
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29

    //bits in the shift register that carry the buttons for:
    // left up center down right red
    #define BL_LE 7
    #define BL_UP 6
    #define BL_MI 5
    #define BL_DW 4
    #define BL_RI 3
    #define BL_ST 2
    #define BLEN_B 1
    #define BLEN_A 0
  #endif
#endif //ULTRA_LCD

#endif // RAMBO
/****************************************************************************************/



/****************************************************************************************
* 4
* Duemilanove w/ ATMega328P
****************************************************************************************/
#if MB(DUEMILANOVE_328P)
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega328P__
#error Oops!  Make sure you have 'Arduino Duemilanove w/ ATMega328' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN         19
#define ORIG_X_DIR_PIN          18
#define ORIG_X_ENABLE_PIN       -1
#define X_STOP_PIN              17

#define ORIG_Y_STEP_PIN         10
#define ORIG_Y_DIR_PIN           7
#define ORIG_Y_ENABLE_PIN       -1
#define Y_STOP_PIN               8

#define ORIG_Z_STEP_PIN         13
#define ORIG_Z_DIR_PIN           3
#define ORIG_Z_ENABLE_PIN        2
#define Z_STOP_PIN               4

#define ORIG_E0_STEP_PIN         11
#define ORIG_E0_DIR_PIN          12
#define ORIG_E0_ENABLE_PIN       -1

#define SDPOWER                  -1
#define SDSS                     -1
#define LED_PIN                  -1
#define ORIG_FAN_PIN              5
#define ORIG_PS_ON_PIN           -1
#define KILL_PIN                 -1

#define ORIG_HEATER_0_PIN         6
#define ORIG_HEATER_1_PIN        -1
#define ORIG_HEATER_2_PIN        -1
#define ORIG_TEMP_0_PIN           0    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define ORIG_TEMP_1_PIN          -1
#define ORIG_TEMP_2_PIN          -1
#define ORIG_HEATER_BED_PIN      -1
#define ORIG_TEMP_BED_PIN        -1

#endif // DUEMILANOVE_328P
/****************************************************************************************/



/****************************************************************************************
* 40
* MKS_BASE 1.0 
****************************************************************************************/
#if MB(MKS_BASE)
  #define KNOWN_BOARD 1
  
  #if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
    #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
  #endif
  
  #define LARGE_FLASH true
  
  // X axis pins
  #define ORIG_X_STEP_PIN         54
  #define ORIG_X_DIR_PIN          55
  #define ORIG_X_ENABLE_PIN       38
  #define ORIG_X_MIN_PIN          3
  #define ORIG_X_MAX_PIN          2
  
  // Y axis pins
  #define ORIG_Y_STEP_PIN         60
  #define ORIG_Y_DIR_PIN          61
  #define ORIG_Y_ENABLE_PIN       56
  #define ORIG_Y_MIN_PIN          14
  #define ORIG_Y_MAX_PIN          15
  
  #define Y2_STEP_PIN             36
  #define Y2_DIR_PIN              34
  #define Y2_ENABLE_PIN           30

  // Z axis pins
  #define ORIG_Z_STEP_PIN         46
  #define ORIG_Z_DIR_PIN          48
  #define ORIG_Z_ENABLE_PIN       62
  #define ORIG_Z_MIN_PIN          18
  #define ORIG_Z_MAX_PIN          19

  #define Z2_STEP_PIN             36
  #define Z2_DIR_PIN              34
  #define Z2_ENABLE_PIN           30

  // E axis pins
  #define ORIG_E0_STEP_PIN        26
  #define ORIG_E0_DIR_PIN         28
  #define ORIG_E0_ENABLE_PIN      24

  #define ORIG_E1_STEP_PIN        36
  #define ORIG_E1_DIR_PIN         34
  #define ORIG_E1_ENABLE_PIN      30
  
  #define SDPOWER                 -1
  #define SDSS                    53
  #define LED_PIN                 13
  
  #define ORIG_FAN_PIN            9
  #define PS_ON_PIN               12
  
  #define ORIG_HEATER_0_PIN       10  // Hotend 1
  #define ORIG_HEATER_1_PIN        7  // Hotend 2
  #define ORIG_HEATER_2_PIN       -1
  #define ORIG_HEATER_3_PIN       -1

  #define ORIG_TEMP_0_PIN         13  // ANALOG NUMBERING
  #define ORIG_TEMP_1_PIN         15  // ANALOG NUMBERING
  #define ORIG_TEMP_2_PIN         -1  // ANALOG NUMBERING
  #define ORIG_TEMP_3_PIN         -1  // ANALOG NUMBERING
  
  #define ORIG_HEATER_BED_PIN     8   // BED
  #define ORIG_TEMP_BED_PIN       14  // ANALOG NUMBERING

  #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) || ENABLED(G3D_PANEL)
    #define KILL_PIN              41
  #else
    #define KILL_PIN              -1
  #endif

  #if NUM_SERVOS > 0
    #define SERVO0_PIN            11
    #if NUM_SERVOS > 1
      #define SERVO1_PIN          6
      #if NUM_SERVOS > 2
        #define SERVO2_PIN        5
        #if NUM_SERVOS > 3
          #define SERVO3_PIN      4
        #endif
      #endif
    #endif
  #endif

  #if ENABLED(ULTRA_LCD)
    #if ENABLED(NEWPANEL)
      #if ENABLED(PANEL_ONE)
        #define LCD_PINS_RS       40
        #define LCD_PINS_ENABLE   42
        #define LCD_PINS_D4       65
        #define LCD_PINS_D5       66
        #define LCD_PINS_D6       44
        #define LCD_PINS_D7       64
      #else
        #define LCD_PINS_RS       16
        #define LCD_PINS_ENABLE   17
        #define LCD_PINS_D4       23
        #define LCD_PINS_D5       25
        #define LCD_PINS_D6       27
        #define LCD_PINS_D7       29
      #endif // PANEL_ONE
  
      #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
        #define ORIG_BEEPER_PIN   37

        #define BTN_EN1           31
        #define BTN_EN2           33
        #define BTN_ENC           35

        #define SD_DETECT_PIN     49
      #elif ENABLED(LCD_I2C_PANELOLU2)
        #define BTN_EN1           47  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           43
        #define BTN_ENC           32
        #define LCD_SDSS          53
        #define SD_DETECT_PIN     -1
        #define KILL_PIN          41
      #elif ENABLED(LCD_I2C_VIKI)
        #define BTN_EN1           22  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           7
        #define BTN_ENC           -1
        #define LCD_SDSS          53
        #define SD_DETECT_PIN     49
      #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
        #define BTN_EN1           35  // reverse if the encoder turns the wrong way.
        #define BTN_EN2           37
        #define BTN_ENC           31
        #define SD_DETECT_PIN     49
        #define LCD_SDSS          53
        #define KILL_PIN          41
        #define ORIG_BEEPER_PIN   23
        #define DOGLCD_CS         29
        #define DOGLCD_A0         27
        #define LCD_PIN_BL        33
      #else
        // arduino pin which triggers an piezzo beeper
        #define ORIG_BEEPER_PIN   33  // Beeper on AUX-4

        // buttons are directly attached using AUX-2
        #if ENABLED(REPRAPWORLD_KEYPAD)
          #define BTN_EN1         64  // encoder
          #define BTN_EN2         59  // encoder
          #define BTN_ENC         63  // enter button
          #define SHIFT_OUT       40  // shift register
          #define SHIFT_CLK       44  // shift register
          #define SHIFT_LD        42  // shift register
        #elif ENABLED(PANEL_ONE)
          #define BTN_EN1         59  // AUX2 PIN 3
          #define BTN_EN2         63  // AUX2 PIN 4
          #define BTN_ENC         49  // AUX3 PIN 7
        #else
          #define BTN_EN1         37
          #define BTN_EN2         35
          #define BTN_ENC         31  // the click
        #endif
  
        #if ENABLED(G3D_PANEL)
          #define SD_DETECT_PIN   49
        #else
          #define SD_DETECT_PIN   -1  // Ramps does not use this port
        #endif

      #endif
  
    #else // old style panel with shift register
      // arduino pin witch triggers an piezo beeper
      #define ORIG_BEEPER_PIN     33  // No Beeper added

      //buttons are attached to a shift register
      // Not wired this yet
      //#define SHIFT_CLK         38
      //#define SHIFT_LD          42
      //#define SHIFT_OUT         40
      //#define SHIFT_EN          17
  
      #define LCD_PINS_RS         16
      #define LCD_PINS_ENABLE     17
      #define LCD_PINS_D4         23
      #define LCD_PINS_D5         25
      #define LCD_PINS_D6         27
      #define LCD_PINS_D7         29
    #endif // NEWPANEL
  #endif // ULTRA_LCD

  // SPI for Max6675 Thermocouple
  #define MAX6675_SS              66  // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present

#endif // MKS_BASE
/****************************************************************************************/



/****************************************************************************************
* 401
*
* RADDS
****************************************************************************************/
#if MB(RADDS)
#define KNOWN_BOARD

#ifndef __SAM3X8E__
  #error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define RADDS

#define ORIG_X_STEP_PIN         24
#define ORIG_X_DIR_PIN          23
#define ORIG_X_ENABLE_PIN       26

#define ORIG_Y_STEP_PIN         17
#define ORIG_Y_DIR_PIN          16
#define ORIG_Y_ENABLE_PIN       22

#define ORIG_Z_STEP_PIN          2
#define ORIG_Z_DIR_PIN           3
#define ORIG_Z_ENABLE_PIN       15

#define ORIG_X_MIN_PIN          28
#define ORIG_X_MAX_PIN          34
#define ORIG_Y_MIN_PIN          30
#define ORIG_Y_MAX_PIN          36
#define ORIG_Z_MIN_PIN          32
#define ORIG_Z_MAX_PIN          38

#define ORIG_E0_STEP_PIN        61
#define ORIG_E0_DIR_PIN         60
#define ORIG_E0_ENABLE_PIN      62

#define ORIG_E1_STEP_PIN        64
#define ORIG_E1_DIR_PIN         63
#define ORIG_E1_ENABLE_PIN      65

#define ORIG_E2_STEP_PIN        51
#define ORIG_E2_DIR_PIN         53
#define ORIG_E2_ENABLE_PIN      49

#define ORIG_E3_STEP_PIN        35
#define ORIG_E3_DIR_PIN         33
#define ORIG_E3_ENABLE_PIN      37

#define ORIG_E4_STEP_PIN        29
#define ORIG_E4_DIR_PIN         27
#define ORIG_E4_ENABLE_PIN      31

#define SDPOWER                 -1
#define SDSS                     4
#define LED_PIN                 -1

#define ORIG_BEEPER_PIN         41

#define ORIG_FAN_PIN 	           9
#define ORIG_FAN2_PIN            8

#define ORIG_PS_ON_PIN          40

#define KILL_PIN                -1

#define ORIG_HEATER_BED_PIN      7    // BED
#define ORIG_HEATER_0_PIN       13
#define ORIG_HEATER_1_PIN       12
#define ORIG_HEATER_2_PIN       11

#define ORIG_TEMP_BED_PIN        4   // ANALOG NUMBERING
#define ORIG_TEMP_0_PIN          0   // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN         -1  // 1   // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN         -1  // 2   // ANALOG NUMBERING
#define ORIG_TEMP_3_PIN         -1  // 3   // ANALOG NUMBERING

#if NUM_SERVOS > 0
  #define SERVO0_PIN           5
  #if NUM_SERVOS > 1
    #define SERVO1_PIN         6
    #if NUM_SERVOS > 2
      #define SERVO2_PIN      39
      #if NUM_SERVOS > 3
        #define SERVO3_PIN    40
      #endif
    #endif
  #endif
#endif

#if ENABLED(ULTRA_LCD)
  // RADDS LCD panel
  #if ENABLED(RADDS_DISPLAY)
    #define LCD_PINS_RS 		42
    #define LCD_PINS_ENABLE 43
    #define LCD_PINS_D4 		44
    #define LCD_PINS_D5 		45
    #define LCD_PINS_D6 		46
    #define LCD_PINS_D7 		47

    #define BEEPER          41

    #define BTN_EN1         50
    #define BTN_EN2         52
    #define BTN_ENC         48
			
    #define BTN_BACK        71
    
    #undef SDSS
    #define SDSS            10
    #define SDCARDDETECT    14

  #elif ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define LCD_PINS_RS     46
    #define LCD_PINS_ENABLE 47
    #define LCD_PINS_D4     44

    #define ORIG_BEEPER_PIN 41

    #define BTN_EN1         50
    #define BTN_EN2         52
    #define BTN_ENC         48

    #if UI_VOLTAGE_LEVEL != 1
      #undef UI_VOLTAGE_LEVEL
      #define UI_VOLTAGE_LEVEL  1
    #endif

  #elif ENABLED(SSD1306_OLED_I2C_CONTROLLER)
    #define BTN_EN1         50
    #define BTN_EN2         52
    #define BTN_ENC         48
    #define BEEPER          41
    #define LCD_SDSS        10
    #define SDCARDDETECT    14
    #define KILL_PIN        -1

  #elif ENABLED(SPARK_FULL_GRAPHICS)
    #define LCD_PINS_D4     29
    #define LCD_PINS_ENABLE 27
    #define LCD_PINS_RS     25

    #define BTN_EN1         35
    #define BTN_EN2         33
    #define BTN_ENC         37

    #define KILL_PIN        -1
    #undef BEEPER
    #define BEEPER          -1
	#endif // SPARK_FULL_GRAPHICS
#endif // ULTRA_LCD

#endif //RADDS
/****************************************************************************************/



/****************************************************************************************
* 403 - 404
* Arduino pin assignment
* Ramps - FD v1 & v2
****************************************************************************************/
#if MB(RAMPS_FD_V1) || MB(RAMPS_FD_V2)
#define KNOWN_BOARD 1

#if MB(RAMPS_FD_V1)
  #define RAMPS_FD_V1
  #define INVERTED_HEATER_PINS
  #define INVERTED_BED_PINS
  // No EEPROM
  // Use 4k7 thermistor tables
#else
  #define RAMPS_FD_V2
  // EEPROM supported
  // Use 1k thermistor tables
#endif

#define ORIG_X_STEP_PIN         63
#define ORIG_X_DIR_PIN          62
#define ORIG_X_ENABLE_PIN       48
#define ORIG_X_MIN_PIN          22
#define ORIG_X_MAX_PIN          30

#define ORIG_Y_STEP_PIN         65
#define ORIG_Y_DIR_PIN          64
#define ORIG_Y_ENABLE_PIN       46
#define ORIG_Y_MIN_PIN          24
#define ORIG_Y_MAX_PIN          38

#define ORIG_Z_STEP_PIN         67
#define ORIG_Z_DIR_PIN          66
#define ORIG_Z_ENABLE_PIN       44
#define ORIG_Z_MIN_PIN          26
#define ORIG_Z_MAX_PIN          34

#define ORIG_E0_STEP_PIN        36
#define ORIG_E0_DIR_PIN         28
#define ORIG_E0_ENABLE_PIN      42

#define ORIG_E1_STEP_PIN        43
#define ORIG_E1_DIR_PIN         41
#define ORIG_E1_ENABLE_PIN      39

#define ORIG_E2_STEP_PIN        32
#define ORIG_E2_DIR_PIN         47
#define ORIG_E2_ENABLE_PIN      45

#define SDPOWER                 -1
#define SDSS                     4
#define LED_PIN                 13

#define ORIG_BEEPER_PIN         -1

#define ORIG_FAN_PIN            -1

#define CONTROLLER_FAN_PIN      -1

#define ORIG_PS_ON_PIN          -1

#define KILL_PIN                -1


#define ORIG_HEATER_BED_PIN      8    // BED

#define ORIG_HEATER_0_PIN        9
#define ORIG_HEATER_1_PIN       10
#define ORIG_HEATER_2_PIN       11

#define ORIG_TEMP_BED_PIN        7   // ANALOG NUMBERING

#define ORIG_TEMP_0_PIN          6   // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN          5   // 2    // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN          4   // 3     // ANALOG NUMBERING
#define ORIG_TEMP_3_PIN          3   // ANALOG NUMBERING

#if NUM_SERVOS > 0
  #define SERVO0_PIN            11
  #if NUM_SERVOS > 1
    #define SERVO1_PIN           6
    #if NUM_SERVOS > 2
      #define SERVO2_PIN         5
      #if NUM_SERVOS > 3
        #define SERVO3_PIN       4
      #endif
    #endif
  #endif
#endif

#if ENABLED(ULTRA_LCD)
  #if ENABLED(NEWPANEL)
    // ramps-fd lcd adaptor
    #define LCD_PINS_RS         16
    #define LCD_PINS_ENABLE     17
    #define LCD_PINS_D4         23
    #define LCD_PINS_D5         25
    #define LCD_PINS_D6         27
    #define LCD_PINS_D7         29

    #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
      #define ORIG_BEEPER_PIN   37

      #define BTN_EN1           33
      #define BTN_EN2           31
      #define BTN_ENC           35

      #define SD_DETECT_PIN     49
    #endif
  #endif
#endif //ULTRA_LCD

// SPI for Max6675 Thermocouple
#define MAX6675_SS              53

#endif //RAMPS-FD
/****************************************************************************************/



/****************************************************************************************
* 408
* Arduino pin assignment
* for SMART_RAMPS
****************************************************************************************/
#if MB(SMART_RAMPS)
#define KNOWN_BOARD

#define ORIG_X_STEP_PIN       54
#define ORIG_X_DIR_PIN        55
#define ORIG_X_ENABLE_PIN     38
#define ORIG_X_MIN_PIN         3
#define ORIG_X_MAX_PIN         2

#define ORIG_Y_STEP_PIN       60
#define ORIG_Y_DIR_PIN        61
#define ORIG_Y_ENABLE_PIN     56
#define ORIG_Y_MIN_PIN        14
#define ORIG_Y_MAX_PIN        15

#define ORIG_Z_STEP_PIN       46
#define ORIG_Z_DIR_PIN        48
#define ORIG_Z_ENABLE_PIN     62
#define ORIG_Z_MIN_PIN        18
#define ORIG_Z_MAX_PIN        19

#define ORIG_HEATER_0_PIN     10
#define ORIG_HEATER_1_PIN      9
#define ORIG_HEATER_BED_PIN    8

#define ORIG_TEMP_0_PIN        9  // Due analog pin #
#define ORIG_TEMP_1_PIN        8  // Due analog pin #
#define ORIG_TEMP_BED_PIN     10  // Due analog pin #

#define ORIG_E0_STEP_PIN      26
#define ORIG_E0_DIR_PIN       28
#define ORIG_E0_ENABLE_PIN    24

#define ORIG_E1_STEP_PIN      36
#define ORIG_E1_DIR_PIN       34
#define ORIG_E1_ENABLE_PIN    30

#define SDPOWER               -1
#define SDSS                  53  // 10 if using HW SPI. 53 if using SW SPI
#define LED_PIN               13
#define ORIG_FAN_PIN           9
#define ORIG_PS_ON_PIN        12
#define KILL_PIN              -1
#define SUICIDE_PIN           -1  // PIN that has to be turned on right after start, to keep power flowing

#endif
/****************************************************************************************/



/****************************************************************************************
* 433
* Arduino Due pin assignment
* for RAMPS4DUE (http://forums.reprap.org/read.php?219,479626,page=1)
****************************************************************************************/
#if MB(RAMPS4DUE)
#define KNOWN_BOARD

#ifndef __SAM3X8E__
  #error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN         54
#define ORIG_X_DIR_PIN          55
#define ORIG_X_ENABLE_PIN       38
#define ORIG_X_MIN_PIN           3
#define ORIG_X_MAX_PIN           2

#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       56
#define ORIG_Y_MIN_PIN          14
#define ORIG_Y_MAX_PIN          15

#define ORIG_Z_STEP_PIN         46
#define ORIG_Z_DIR_PIN          48
#define ORIG_Z_ENABLE_PIN       62
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          19

#define Y2_STEP_PIN             36
#define Y2_DIR_PIN              34
#define Y2_ENABLE_PIN           30

#define Z2_STEP_PIN             36
#define Z2_DIR_PIN              34
#define Z2_ENABLE_PIN           30
    
#define ORIG_E0_STEP_PIN        26
#define ORIG_E0_DIR_PIN         28
#define ORIG_E0_ENABLE_PIN      24

#define ORIG_E1_STEP_PIN        36
#define ORIG_E1_DIR_PIN         34
#define ORIG_E1_ENABLE_PIN      30

#define ORIG_HEATER_0_PIN       10
#define ORIG_HEATER_1_PIN       -1
#define ORIG_HEATER_2_PIN       -1
#define ORIG_HEATER_BED_PIN      8    // BED

#define ORIG_TEMP_0_PIN          9   // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN         -1   // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN         -1   // ANALOG NUMBERING
#define ORIG_TEMP_BED_PIN       10   // ANALOG NUMBERING

#define ORIG_FAN_PIN             9
#define ORIG_PS_ON_PIN          12
#define SDPOWER                 -1
#define SDSS                    53
#define LED_PIN                 13

#endif
/****************************************************************************************/



/****************************************************************************************
* 5 - 51
* Gen6 - Gen6 Deluxe
****************************************************************************************/
#if MB(GEN6) || MB(GEN6_DELUXE)
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega644P__
#ifndef __AVR_ATmega1284P__
#error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
#endif
#endif

//x axis pins
    #define ORIG_X_STEP_PIN      15
    #define ORIG_X_DIR_PIN       18
    #define ORIG_X_ENABLE_PIN    19
    #define X_STOP_PIN      20

    //y axis pins
    #define ORIG_Y_STEP_PIN      23
    #define ORIG_Y_DIR_PIN       22
    #define ORIG_Y_ENABLE_PIN    24
    #define Y_STOP_PIN      25

    //z axis pins
    #define ORIG_Z_STEP_PIN      27
    #define ORIG_Z_DIR_PIN       28
    #define ORIG_Z_ENABLE_PIN    29
    #define Z_STOP_PIN      30

    //extruder pins
    #define ORIG_E0_STEP_PIN      4    //Edited @ EJE Electronics 20100715
    #define ORIG_E0_DIR_PIN       2    //Edited @ EJE Electronics 20100715
    #define ORIG_E0_ENABLE_PIN    3    //Added @ EJE Electronics 20100715
    #define ORIG_TEMP_0_PIN       5    //changed @ rkoeppl 20110410
    #define ORIG_TEMP_1_PIN      -1    //changed @ rkoeppl 20110410


    #define ORIG_TEMP_2_PIN      -1    //changed @ rkoeppl 20110410
    #define ORIG_HEATER_0_PIN    14    //changed @ rkoeppl 20110410
    #define ORIG_HEATER_1_PIN    -1
    #define ORIG_HEATER_2_PIN    -1
    #if MOTHERBOARD == 5
    #define ORIG_HEATER_BED_PIN  -1    //changed @ rkoeppl 20110410
    #define ORIG_TEMP_BED_PIN    -1    //changed @ rkoeppl 20110410
    #else
    #define ORIG_HEATER_BED_PIN   1    //changed @ rkoeppl 20110410
    #define ORIG_TEMP_BED_PIN     0    //changed @ rkoeppl 20110410
    #endif
    #define SDPOWER              -1
    #define SDSS                 17
    #define LED_PIN              -1    //changed @ rkoeppl 20110410
    #define ORIG_FAN_PIN         -1    //changed @ rkoeppl 20110410
    #define ORIG_PS_ON_PIN       -1    //changed @ rkoeppl 20110410
    #define KILL_PIN             -1    //changed @ drakelive 20120830
    //our pin for debugging.

    #define DEBUG_PIN             0

    //our RS485 pins
    #define TORIG_X_ENABLE_PIN   12
    #define RORIG_X_ENABLE_PIN   13

#endif // GEN6 || GEN6_DELUXE
/****************************************************************************************/



/****************************************************************************************
* 502
* Alligator R2
* http://www.3dartists.org/
****************************************************************************************/
#if MB(ALLIGATOR)
#define KNOWN_BOARD 1

#ifndef __SAM3X8E__
  #error Oops!  Make sure you have 'Alligator 3D Printer Board' selected from the 'Tools -> Boards' menu.
#endif

#define ALLIGATOR
#define SPI_CHAN_DAC 1

// X AXIS
#define ORIG_X_STEP_PIN       96  // PB24
#define ORIG_X_DIR_PIN         2  // PB25
#define ORIG_X_ENABLE_PIN     24  // PA15, motor RESET pin
#define ORIG_X_MIN_PIN        33  // PC1
#define ORIG_X_MAX_PIN        34  // PC2
#define X_MS1_PIN             99  // PC10

// Y AXIS
#define ORIG_Y_STEP_PIN       94  // PB22
#define ORIG_Y_DIR_PIN        95  // PB23
#define ORIG_Y_ENABLE_PIN     24  // PA15, motor RESET pin
#define ORIG_Y_MIN_PIN        35  // PC3
#define ORIG_Y_MAX_PIN        37  // PC5
#define Y_MS1_PIN             10  // PC29

// Z AXIS
#define ORIG_Z_STEP_PIN       98  // PC27
#define ORIG_Z_DIR_PIN         3  // PC28
#define ORIG_Z_ENABLE_PIN     24  // PA15, motor RESET pin
#define ORIG_Z_MIN_PIN        38  // PC6
#define ORIG_Z_MAX_PIN        39  // PC7
#define Z_MS1_PIN             44  // PC19

// E0 AXIS
#define ORIG_E0_STEP_PIN       5  // PC25
#define ORIG_E0_DIR_PIN        4  // PC26
#define ORIG_E0_ENABLE_PIN    24  // PA15, motor RESET pin
#define E0_MS1_PIN            45  // PC18

// E1 AXIS
#define ORIG_E1_STEP_PIN      28  // PD3 on piggy
#define ORIG_E1_DIR_PIN       27  // PD2 on piggy
#define ORIG_E1_ENABLE_PIN    24  // PA15, motor RESET pin

// E2 AXIS
#define ORIG_E2_STEP_PIN      11 // PD7 on piggy
#define ORIG_E2_DIR_PIN       29 // PD6 on piggy
#define ORIG_E2_ENABLE_PIN    24 // PA15, motor RESET pin

// E3 AXIS
#define ORIG_E3_STEP_PIN      30 // PD9 on piggy
#define ORIG_E3_DIR_PIN       12 // PD8 on piggy
#define ORIG_E3_ENABLE_PIN    24 // PA15, motor RESET pin

#define MOTOR_FAULT_PIN       22 // PB26 , motor X-Y-Z-E0 motor FAULT

#define SDPOWER               -1
#define SDSS                  77 // PA28
#define SD_DETECT_PIN         87 // PA29
#define LED_PIN               -1

#define ORIG_FAN_PIN          92 // PA5
#define ORIG_FAN2_PIN         31 // PA7

#define ORIG_PS_ON_PIN        -1
#define KILL_PIN              -1
#define SUICIDE_PIN           -1 //PIN that has to be turned on right after start, to keep power flowing.

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define ORIG_HEATER_BED_PIN   69 // PA0
#define ORIG_HEATER_0_PIN     68 // PA1
#define ORIG_HEATER_1_PIN      8 // PC22 on piggy
#define ORIG_HEATER_2_PIN      9 // PC21 on piggy
#define ORIG_HEATER_3_PIN     97 // PC20 on piggy

#define ORIG_TEMP_BED_PIN      0 // PA16
#define ORIG_TEMP_0_PIN        1 // PA24, analog pin
#define ORIG_TEMP_1_PIN        2 // PA23 analog pin on piggy
#define ORIG_TEMP_2_PIN        3 // PA22, analog pin on piggy
#define ORIG_TEMP_3_PIN        4 // PA6, analog on piggy

#define LED_PWM1_PIN          40 // PC8
#define LED_PWM2_PIN          41 // PC9
#define LED_PWM3_PIN          36 // PC4

#define EXP_VOLTAGE_LEVEL_PIN 65

#define DAC0_SYNC             53 // PB14
#define DAC1_SYNC              6 // PC24

//64K SPI EEPROM
#define SPI_CHAN_EEPROM1       2
#define SPI_EEPROM1_CS        25 // PD0

//2K SPI EEPROM
#define SPI_EEPROM2_CS        26 // PD1

//** FLASH SPI**/
//32Mb
#define SPI_FLASH_CS          23 //PA14

/** Display **/

// GLCD on expansion port
#if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)

  #define LCD_PINS_RS         18
  #define LCD_PINS_ENABLE     15
  #define LCD_PINS_D4         19
  #define ORIG_BEEPER_PIN     64

  #define BTN_EN1             14
  #define BTN_EN2             16
  #define BTN_ENC             17
  
  #if UI_VOLTAGE_LEVEL != 1
    #undef UI_VOLTAGE_LEVEL
    #define UI_VOLTAGE_LEVEL  1
  #endif
     
#endif //REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

#if NUM_SERVOS > 0
  #define SERVO0_PIN          36
  #if NUM_SERVOS > 1
    #define SERVO1_PIN        40
    #if NUM_SERVOS > 2
      #define SERVO2_PIN      41
      #if NUM_SERVOS > 3
        #define SERVO3_PIN    -1
      #endif
    #endif
  #endif
#endif

#endif //ALLIGATOR
/****************************************************************************************/



/****************************************************************************************
* 6 - 62 - 63 - 64 - 65
*  6 - Sanguinololu <1.2
* 62 - Sanguinololu 1.2 and above
* 63 - Melzi
* 64 - STB 1.1
* 65 - Azteeg X1
* 66 - MELZI 1284
****************************************************************************************/
#if MB(SANGUINOLOLU_11) || MB(SANGUINOLOLU_12) || MB(MELZI) || MB(STB_11) || MB(AZTEEG_X1) || MB(MELZI_1284)

  #if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega1284P__)
    #error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
  #endif

  #define KNOWN_BOARD 1

  #if !MB(SANGUINOLOLU_11)
    #define SANGUINOLOLU_V_1_2
  #endif

  #if defined(__AVR_ATmega1284P__)
    #define LARGE_FLASH true
  #endif

  #define ORIG_X_STEP_PIN         15
  #define ORIG_X_DIR_PIN          21
  #define X_STOP_PIN              18

  #define ORIG_Y_STEP_PIN         22
  #define ORIG_Y_DIR_PIN          23
  #define Y_STOP_PIN              19

  #define ORIG_Z_STEP_PIN          3
  #define ORIG_Z_DIR_PIN           2
  #define Z_STOP_PIN              20

  #define ORIG_E0_STEP_PIN         1
  #define ORIG_E0_DIR_PIN          0

  #define LED_PIN                 -1

  #define ORIG_FAN_PIN            -1

  #if ORIG_FAN_PIN == 12 || ORIG_FAN_PIN ==13
    #define FAN_SOFT_PWM
  #endif

  #if MB(AZTEEG_X1) || MB(STB_11) || MB(MELZI)
    #define ORIG_FAN_PIN           4 // Works for Panelolu2 too
    #if MB(MELZI)
      #define LED_PIN             27
    #elif MB(STB_11)
      #define LCD_PIN_BL          17 // LCD backlight LED
    #endif
  #endif

  #if NUM_SERVOS > 0
    #define SERVO0_PIN            -1
    #if NUM_SERVOS > 1
      #define SERVO1_PIN          -1
      #if NUM_SERVOS > 2
        #define SERVO2_PIN        -1
        #if NUM_SERVOS > 3
          #define SERVO3_PIN      -1
        #endif
      #endif
    #endif
  #endif

  #define ORIG_PS_ON_PIN          -1
  #define KILL_PIN                -1

  #define ORIG_HEATER_0_PIN       13 // (extruder)
  #define ORIG_HEATER_1_PIN       -1
  #define ORIG_HEATER_2_PIN       -1

  #if ENABLED(SANGUINOLOLU_V_1_2)

    #define ORIG_HEATER_BED_PIN   12 // (bed)
    #define ORIG_X_ENABLE_PIN     14
    #define ORIG_Y_ENABLE_PIN     14
    #define ORIG_Z_ENABLE_PIN     26
    #define ORIG_E0_ENABLE_PIN    14

    #if ENABLED(LCD_I2C_PANELOLU2)
      #define ORIG_FAN_PIN         4 // Uses Transistor1 (PWM) on Panelolu2's Sanguino Adapter Board to drive the fan
    #endif

  #else

    #define ORIG_HEATER_BED_PIN   14  // (bed)
    #define ORIG_X_ENABLE_PIN     -1
    #define ORIG_Y_ENABLE_PIN     -1
    #define ORIG_Z_ENABLE_PIN     -1
    #define ORIG_E0_ENABLE_PIN    -1

  #endif

    #define ORIG_TEMP_0_PIN        7   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 33 extruder)
    #define ORIG_TEMP_1_PIN       -1
    #define ORIG_TEMP_2_PIN       -1
    #define ORIG_TEMP_BED_PIN      6   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 34 bed)
    #define SDPOWER               -1
    #define SDSS                  31

  /**
   * On some broken versions of the Sanguino libraries the pin definitions are wrong,
   * which then needs SDSS as pin 24. But you should upgrade your Sanguino libraries! See #368.
   */
  //#define SDSS               24

  #if ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL)
  
    // No buzzer installed
    #define ORIG_BEEPER_PIN -1

    //LCD Pins
    #if ENABLED(DOGLCD)

      #if ENABLED(U8GLIB_ST7920) //SPI GLCD 12864 ST7920 ( like [www.digole.com] ) For Melzi V2.0
        #if MB(MELZI) // Melzi board
          #define LCD_PINS_RS     30 //CS chip select /SS chip slave select
          #define LCD_PINS_ENABLE 29 //SID (MOSI)
          #define LCD_PINS_D4     17 //SCK (CLK) clock
          #define ORIG_BEEPER_PIN 27 // Pin 27 is taken by LED_PIN, but Melzi LED does nothing with Marlin so this can be used for ORIG_BEEPER_PIN. You can use this pin with M42 instead of ORIG_BEEPER_PIN.
        #else         // Sanguinololu 1.3
          #define LCD_PINS_RS      4
          #define LCD_PINS_ENABLE 17
          #define LCD_PINS_D4     30
          #define LCD_PINS_D5     29
          #define LCD_PINS_D6     28
          #define LCD_PINS_D7     27
        #endif
      #else // DOGM SPI LCD Support

        #define DOGLCD_A0         30
        #define DOGLCD_CS         29
        #define LCD_CONTRAST       1
      #endif

      // Uncomment screen orientation
      #define LCD_SCREEN_ROT_0
      //#define LCD_SCREEN_ROT_90
      //#define LCD_SCREEN_ROT_180
      //#define LCD_SCREEN_ROT_270

    #else // !DOGLCD - Standard Hitachi LCD controller
      #define LCD_PINS_RS          4
      #define LCD_PINS_ENABLE     17
      #define LCD_PINS_D4         30
      #define LCD_PINS_D5         29
      #define LCD_PINS_D6         28
      #define LCD_PINS_D7         27
    #endif // !DOGLCD

    //The encoder and click button
    #define BTN_EN1               11
    #define BTN_EN2               10
    #if ENABLED(LCD_I2C_PANELOLU2)
      #if MB(MELZI)
        #define BTN_ENC           29
        #define LCD_SDSS          30 // Panelolu2 SD card reader rather than the Melzi
      #else
        #define BTN_ENC           30
      #endif
    #else
      #define BTN_ENC             16
      #define LCD_SDSS            28 // Smart Controller SD card reader rather than the Melzi
    #endif //Panelolu2

    #define SD_DETECT_PIN         -1

  #elif ENABLED(MAKRPANEL)
    #define ORIG_BEEPER_PIN       29

    // Pins for DOGM SPI LCD Support
    #define DOGLCD_A0             30
    #define DOGLCD_CS             17
    #define LCD_PIN_BL            28 // backlight LED on PA3
    // GLCD features
    #define LCD_CONTRAST           1
    // Uncomment screen orientation
    #define LCD_SCREEN_ROT_0
    //#define LCD_SCREEN_ROT_90
    //#define LCD_SCREEN_ROT_180
    //#define LCD_SCREEN_ROT_270
    //The encoder and click button
    #define BTN_EN1               11
    #define BTN_EN2               10
    #define BTN_ENC               16

    #define SD_DETECT_PIN         -1

  #endif // MAKRPANEL

#endif // SANGUINOLOLU_11
/****************************************************************************************/



/****************************************************************************************
* 67
* AZTEEG X3
****************************************************************************************/
#if MB(AZTEEG_X3)
#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH true

#define ORIG_X_STEP_PIN         54
#define ORIG_X_DIR_PIN          55
#define ORIG_X_ENABLE_PIN       38
#define ORIG_X_MIN_PIN           3
#define ORIG_X_MAX_PIN           2

#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       56
#define ORIG_Y_MIN_PIN          14
#define ORIG_Y_MAX_PIN          15

#define ORIG_Z_STEP_PIN         46
#define ORIG_Z_DIR_PIN          48
#define ORIG_Z_ENABLE_PIN       62
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          19

#define Y2_STEP_PIN             36
#define Y2_DIR_PIN              34
#define Y2_ENABLE_PIN           30

#define Z2_STEP_PIN             36
#define Z2_DIR_PIN              34
#define Z2_ENABLE_PIN           30

#define ORIG_E0_STEP_PIN        26
#define ORIG_E0_DIR_PIN         28
#define ORIG_E0_ENABLE_PIN      24

#define ORIG_E1_STEP_PIN        36
#define ORIG_E1_DIR_PIN         34
#define ORIG_E1_ENABLE_PIN      30

#define SDPOWER                 -1
#define SDSS                    53
#define LED_PIN                 13

#define ORIG_FAN_PIN             9
#define ORIG_PS_ON_PIN          12

#if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) || ENABLED(G3D_PANEL)
  #define KILL_PIN              41
#else
  #define KILL_PIN              -1
#endif

#define ORIG_HEATER_0_PIN       10   // HOTEND 1
#define ORIG_HEATER_1_PIN       -1
#define ORIG_HEATER_2_PIN       -1
#define ORIG_HEATER_3_PIN       -1

#define ORIG_TEMP_0_PIN         13   // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN         15   // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN         -1   // ANALOG NUMBERING

#define ORIG_HEATER_BED_PIN      8   // BED

#define ORIG_TEMP_BED_PIN       14   // ANALOG NUMBERING

#if NUM_SERVOS > 0
  #define SERVO0_PIN            11
#endif

#if NUM_SERVOS > 0
  #define SERVO0_PIN            11
  #if NUM_SERVOS > 1
    #define SERVO1_PIN           6
    #if NUM_SERVOS > 2
      #define SERVO2_PIN         5
      #if NUM_SERVOS > 3
        #define SERVO3_PIN       4
      #endif
    #endif
  #endif
#endif

#if ENABLED(TEMP_STAT_LEDS)
  #define STAT_LED_RED       6
  #define STAT_LED_BLUE     11
#endif

#if ENABLED(ULTRA_LCD)

  #if ENABLED(NEWPANEL)
    #define LCD_PINS_RS 16
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29

    #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
      #define ORIG_BEEPER_PIN 37

      #define BTN_EN1 31
      #define BTN_EN2 33
      #define BTN_ENC 35

      #define SD_DETECT_PIN 49
    #elif ENABLED(LCD_I2C_PANELOLU2)
      #define BTN_EN1 47  //reverse if the encoder turns the wrong way.
      #define BTN_EN2 43
      #define BTN_ENC 32
      #define LCD_SDSS 53
      #define SD_DETECT_PIN -1
      #define KILL_PIN 41
    #elif ENABLED(LCD_I2C_VIKI)
      #define BTN_EN1 22  //reverse if the encoder turns the wrong way.
      #define BTN_EN2 7
      #define BTN_ENC -1
      #define LCD_SDSS 53
      #define SD_DETECT_PIN 49
    #else
      //arduino pin which triggers an piezzo beeper
      #define ORIG_BEEPER_PIN 33  // Beeper on AUX-4

      //buttons are directly attached using AUX-2
      #if ENABLED(REPRAPWORLD_KEYPAD)
        #define BTN_EN1 64 // encoder
        #define BTN_EN2 59 // encoder
        #define BTN_ENC 63 // enter button
        #define SHIFT_OUT 40 // shift register
        #define SHIFT_CLK 44 // shift register
        #define SHIFT_LD 42 // shift register
      #else
        #define BTN_EN1 37
        #define BTN_EN2 35
        #define BTN_ENC 31  //the click
      #endif

      #if ENABLED(G3D_PANEL)
        #define SD_DETECT_PIN 49
      #else
        #define SD_DETECT_PIN -1  // Ramps does not use this port
      #endif

    #endif

  #else //old style panel with shift register
    //arduino pin witch triggers an piezzo beeper
    #define ORIG_BEEPER_PIN 33   //No Beeper added

    //buttons are attached to a shift register
    // Not wired this yet
    //#define SHIFT_CLK 38
    //#define SHIFT_LD 42
    //#define SHIFT_OUT 40
    //#define SHIFT_EN 17

    #define LCD_PINS_RS 16
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29
  #endif

#endif //ULTRA_LCD

// SPI for Max6675 Thermocouple
#define MAX6675_SS       66 // Do not use pin 53 if there is even the remote possibility of using Display/SD card

#endif // AZTEEG X3
/****************************************************************************************/



/****************************************************************************************
* 68 
* AZTEEG X3 PRO
****************************************************************************************/
#if MB(AZTEEG_X3_PRO)
#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH true

#define LARGE_FLASH true

#define ORIG_X_STEP_PIN         54
#define ORIG_X_DIR_PIN          55
#define ORIG_X_ENABLE_PIN       38
#define ORIG_X_MIN_PIN          3
#define ORIG_X_MAX_PIN          2

#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       56
#define ORIG_Y_MIN_PIN          14
#define ORIG_Y_MAX_PIN          15

#define ORIG_Z_STEP_PIN         46
#define ORIG_Z_DIR_PIN          48
#define ORIG_Z_ENABLE_PIN       62
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          19

#define Y2_STEP_PIN             36
#define Y2_DIR_PIN              34
#define Y2_ENABLE_PIN           30

#define Z2_STEP_PIN             36
#define Z2_DIR_PIN              34
#define Z2_ENABLE_PIN           30

#define ORIG_E0_STEP_PIN        26
#define ORIG_E0_DIR_PIN         28
#define ORIG_E0_ENABLE_PIN      24

#define ORIG_E1_STEP_PIN        36
#define ORIG_E1_DIR_PIN         34
#define ORIG_E1_ENABLE_PIN      30

#define ORIG_E2_STEP_PIN        23
#define ORIG_E2_DIR_PIN         25
#define ORIG_E2_ENABLE_PIN      40

#define ORIG_E3_STEP_PIN        27
#define ORIG_E3_DIR_PIN         29
#define ORIG_E3_ENABLE_PIN      41

#define SDPOWER                 -1
#define SDSS                    53
#define LED_PIN                 13

#define ORIG_FAN_PIN             6
#define ORIG_BEEPER_PIN         33
#define ORIG_PS_ON_PIN          12

#if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) || ENABLED(G3D_PANEL)
  #define KILL_PIN              41
#else
  #define KILL_PIN              -1
#endif

#define ORIG_HEATER_0_PIN       10   // HOTEND 1
#define ORIG_HEATER_1_PIN        9   // HOTEND 2
#define ORIG_HEATER_2_PIN       16   // HOTEND 3
#define ORIG_HEATER_3_PIN       17   // HOTEND 4

#define ORIG_TEMP_0_PIN         13   // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN         15   // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN         12   // ANALOG NUMBERING
#define ORIG_TEMP_3_PIN         11   // ANALOG NUMBERING

#define TC1                      4    // ANALOG NUMBERING Thermo couple on Azteeg X3Pro
#define TC2                      5    // ANALOG NUMBERING Thermo couple on Azteeg X3Pro

#define ORIG_HEATER_BED_PIN      8   // BED

#define ORIG_TEMP_BED_PIN       14   // ANALOG NUMBERING

#if NUM_SERVOS > 0
  #define SERVO0_PIN            47
  #if NUM_SERVOS > 1
    #define SERVO1_PIN          -1
    #if NUM_SERVOS > 2
      #define SERVO2_PIN        -1
      #if NUM_SERVOS > 3
        #define SERVO3_PIN      -1
      #endif
    #endif
  #endif
#endif
  
#if ENABLED(TEMP_STAT_LEDS)
  #define STAT_LED_RED      32
  #define STAT_LED_BLUE     35
#endif

#if ENABLED(ULTRA_LCD)

  #if ENABLED(NEWPANEL)
    #define LCD_PINS_RS 16
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29

    #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
      #define ORIG_BEEPER_PIN 37

      #define BTN_EN1 31
      #define BTN_EN2 33
      #define BTN_ENC 35

      #define SD_DETECT_PIN 49
    #elif ENABLED(LCD_I2C_PANELOLU2)
      #define BTN_EN1 47  //reverse if the encoder turns the wrong way.
      #define BTN_EN2 43
      #define BTN_ENC 32
      #define LCD_SDSS 53
      #define SD_DETECT_PIN -1
      #define KILL_PIN 41
    #elif ENABLED(LCD_I2C_VIKI)
      #define BTN_EN1 22  //reverse if the encoder turns the wrong way.
      #define BTN_EN2 7
      #define BTN_ENC -1
      #define LCD_SDSS 53
      #define SD_DETECT_PIN 49
    #else
      //arduino pin which triggers an piezzo beeper
      #define ORIG_BEEPER_PIN 33  // Beeper on AUX-4

      //buttons are directly attached using AUX-2
      #if ENABLED(REPRAPWORLD_KEYPAD)
        #define BTN_EN1 64 // encoder
        #define BTN_EN2 59 // encoder
        #define BTN_ENC 63 // enter button
        #define SHIFT_OUT 40 // shift register
        #define SHIFT_CLK 44 // shift register
        #define SHIFT_LD 42 // shift register
      #else
        #define BTN_EN1 37
        #define BTN_EN2 35
        #define BTN_ENC 31  //the click
      #endif

      #if ENABLED(G3D_PANEL)
        #define SD_DETECT_PIN 49
      #else
        #define SD_DETECT_PIN -1  // Ramps does not use this port
      #endif

    #endif

  #else //old style panel with shift register
    //arduino pin witch triggers an piezzo beeper
    #define ORIG_BEEPER_PIN 33   //No Beeper added

    //buttons are attached to a shift register
    // Not wired this yet
    //#define SHIFT_CLK 38
    //#define SHIFT_LD 42
    //#define SHIFT_OUT 40
    //#define SHIFT_EN 17

    #define LCD_PINS_RS 16
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29
  #endif

#endif //ULTRA_LCD

#if ENABLED(VIKI2) || ENABLED(miniVIKI)
  #define ORIG_BEEPER_PIN  33
 // Pins for DOGM SPI LCD Support
  #define DOGLCD_A0        44
  #define DOGLCD_CS        45
  #define LCD_SCREEN_ROT_180

 //The encoder and click button
  #define BTN_EN1           22
  #define BTN_EN2            7
  #define BTN_ENC           39  // the click switch

  #define SDSS              53
  #define SD_DETECT_PIN     49

  #define KILL_PIN          31
#endif
 
#define MAX6675_SS          66  // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present

#endif // AZTEEG X3 PRO
/****************************************************************************************/



/****************************************************************************************
* 7
* Ultimaker pin assignment
****************************************************************************************/
#if MB(ULTIMAKER)
#define KNOWN_BOARD

#ifndef __AVR_ATmega1280__
 #ifndef __AVR_ATmega2560__
 #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
 #endif
#endif

#define LARGE_FLASH true

#define ORIG_X_STEP_PIN 25
#define ORIG_X_DIR_PIN 23
#define ORIG_X_MIN_PIN 22
#define ORIG_X_MAX_PIN 24
#define ORIG_X_ENABLE_PIN 27

#define ORIG_Y_STEP_PIN 31
#define ORIG_Y_DIR_PIN 33
#define ORIG_Y_MIN_PIN 26
#define ORIG_Y_MAX_PIN 28
#define ORIG_Y_ENABLE_PIN 29

#define ORIG_Z_STEP_PIN 37
#define ORIG_Z_DIR_PIN 39
#define ORIG_Z_MIN_PIN 30
#define ORIG_Z_MAX_PIN 32
#define ORIG_Z_ENABLE_PIN 35

#define ORIG_HEATER_BED_PIN 4
#define ORIG_TEMP_BED_PIN 10

#define ORIG_HEATER_0_PIN  2
#define ORIG_TEMP_0_PIN 8

#define ORIG_HEATER_1_PIN 3
#define ORIG_TEMP_1_PIN 9

#define ORIG_HEATER_2_PIN -1
#define ORIG_TEMP_2_PIN -1

#define ORIG_E0_STEP_PIN         43
#define ORIG_E0_DIR_PIN          45
#define ORIG_E0_ENABLE_PIN       41

#define ORIG_E1_STEP_PIN         49
#define ORIG_E1_DIR_PIN          47
#define ORIG_E1_ENABLE_PIN       48

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13
#define ORIG_FAN_PIN        7
#define ORIG_PS_ON_PIN     12
#define KILL_PIN           -1
#define SUICIDE_PIN        54  //PIN that has to be turned on right after start, to keep power flowing.

#if ENABLED(ULTRA_LCD)

  #if ENABLED(NEWPANEL)
  //arduino pin witch triggers an piezzo beeper
    #define ORIG_BEEPER_PIN 18

    #define LCD_PINS_RS 20
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 16
    #define LCD_PINS_D5 21
    #define LCD_PINS_D6 5
    #define LCD_PINS_D7 6

    //buttons are directly attached
    #define BTN_EN1 40
    #define BTN_EN2 42
    #define BTN_ENC 19  //the click

    #define SD_DETECT_PIN 38

  #else //old style panel with shift register
    //arduino pin witch triggers an piezzo beeper
    #define ORIG_BEEPER_PIN 18

    //buttons are attached to a shift register
    #define SHIFT_CLK 38
    #define SHIFT_LD 42
    #define SHIFT_OUT 40
    #define SHIFT_EN 17

    #define LCD_PINS_RS 16
    #define LCD_PINS_ENABLE 5
    #define LCD_PINS_D4 6
    #define LCD_PINS_D5 21
    #define LCD_PINS_D6 20
    #define LCD_PINS_D7 19

    #define SD_DETECT_PIN -1
  #endif
#endif //ULTRA_LCD

#endif // ULTIMAKER
/****************************************************************************************/



/****************************************************************************************
* 70
* MegaTronics
****************************************************************************************/
#if MB(MEGATRONICS)
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega2560__
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH        true

#define ORIG_X_STEP_PIN         26
#define ORIG_X_DIR_PIN          28
#define ORIG_X_ENABLE_PIN       24
#define ORIG_X_MIN_PIN          41
#define ORIG_X_MAX_PIN          37

#define ORIG_Y_STEP_PIN         60 // A6
#define ORIG_Y_DIR_PIN          61 // A7
#define ORIG_Y_ENABLE_PIN       22
#define ORIG_Y_MIN_PIN          14
#define ORIG_Y_MAX_PIN          15

#define ORIG_Z_STEP_PIN         54 // A0
#define ORIG_Z_DIR_PIN          55 // A1
#define ORIG_Z_ENABLE_PIN       56 // A2
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          19

#define ORIG_E0_STEP_PIN        31
#define ORIG_E0_DIR_PIN         32
#define ORIG_E0_ENABLE_PIN      38

#define ORIG_E1_STEP_PIN        34
#define ORIG_E1_DIR_PIN         36
#define ORIG_E1_ENABLE_PIN      30

#define SDPOWER                 -1
#define SDSS                    53
#define LED_PIN                 13


#define ORIG_FAN_PIN             7 // IO pin. Buffer needed
#define ORIG_PS_ON_PIN          12
#define KILL_PIN                -1

#define ORIG_HEATER_0_PIN        9    // EXTRUDER 1
#define ORIG_HEATER_1_PIN        8    // EXTRUDER 2 (FAN On Sprinter)
#define ORIG_HEATER_2_PIN       -1

#if TEMP_SENSOR_0 == -1
#define ORIG_TEMP_0_PIN          8   // ANALOG NUMBERING
#else
#define ORIG_TEMP_0_PIN         13   // ANALOG NUMBERING

#endif

#define ORIG_TEMP_1_PIN         15   // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN         -1   // ANALOG NUMBERING
#define ORIG_HEATER_BED_PIN     10   // BED
#define ORIG_TEMP_BED_PIN       14   // ANALOG NUMBERING

#define ORIG_BEEPER_PIN         33   // Beeper on AUX-4

#if ENABLED(ULTRA_LCD)
  #if ENABLED(NEWPANEL)
  //arduino pin which triggers an piezzo beeper

    #define LCD_PINS_RS 16
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29

    //buttons are directly attached using AUX-2
    #define BTN_EN1 59
    #define BTN_EN2 64
    #define BTN_ENC 43  //the click

    #define BLEN_C 2
    #define BLEN_B 1
    #define BLEN_A 0

    #define SD_DETECT_PIN -1   // Ramps does not use this port
  #endif //NEWPANEL

#endif //ULTRA_LCD

#endif // MEGATRONICS
/****************************************************************************************/



/****************************************************************************************
* 701
* MegaTronics v2.0
****************************************************************************************/
#if MB(MEGATRONICS_2)
  #define KNOWN_BOARD 1
  
  #ifndef __AVR_ATmega2560__
    #error Oops! Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
  #endif

  #define LARGE_FLASH        true

  #define ORIG_X_STEP_PIN 26
  #define ORIG_X_DIR_PIN 27
  #define ORIG_X_ENABLE_PIN 25
  #define ORIG_X_MIN_PIN 37
  #define ORIG_X_MAX_PIN 40 //2 //Max endstops default to disabled "-1", set to commented value to enable.

  #define ORIG_Y_STEP_PIN 4 // A6
  #define ORIG_Y_DIR_PIN 54 // A0
  #define ORIG_Y_ENABLE_PIN 5
  #define ORIG_Y_MIN_PIN 41
  #define ORIG_Y_MAX_PIN 38 //15

  #define ORIG_Z_STEP_PIN 56 // A2
  #define ORIG_Z_DIR_PIN 60 // A6
  #define ORIG_Z_ENABLE_PIN 55 // A1
  #define ORIG_Z_MIN_PIN 18
  #define ORIG_Z_MAX_PIN 19

  #define ORIG_E0_STEP_PIN 35
  #define ORIG_E0_DIR_PIN 36
  #define ORIG_E0_ENABLE_PIN 34

  #define ORIG_E1_STEP_PIN 29
  #define ORIG_E1_DIR_PIN 39
  #define ORIG_E1_ENABLE_PIN 28

  #define ORIG_E2_STEP_PIN 23
  #define ORIG_E2_DIR_PIN 24
  #define ORIG_E2_ENABLE_PIN 22

  #define SDPOWER -1
  #define SDSS 53
  #define LED_PIN 13

  #define ORIG_FAN_PIN 7
  #define ORIG_FAN2_PIN 6
  #define ORIG_PS_ON_PIN 12
  #define KILL_PIN -1

  #define ORIG_HEATER_0_PIN 9 // EXTRUDER 1
  #define ORIG_HEATER_1_PIN 8 // EXTRUDER 2
  #define ORIG_HEATER_2_PIN -1

  #define SHIFT_CLK 63
  #define SHIFT_LD 42
  #define SHIFT_OUT 17
  #define SHIFT_EN 17

  #if TEMP_SENSOR_0 == -1
    #define ORIG_TEMP_0_PIN 4 // ANALOG NUMBERING
  #else
    #define ORIG_TEMP_0_PIN 13 // ANALOG NUMBERING
  #endif

  #if TEMP_SENSOR_1 == -1
    #define ORIG_TEMP_1_PIN 8 // ANALOG NUMBERING
  #else
    #define ORIG_TEMP_1_PIN 15 // ANALOG NUMBERING
  #endif

  #define ORIG_TEMP_2_PIN -1 // ANALOG NUMBERING

  #define ORIG_HEATER_BED_PIN 10 // BED

  #if TEMP_SENSOR_BED == -1
    #define ORIG_TEMP_BED_PIN 8 // ANALOG NUMBERING
  #else
    #define ORIG_TEMP_BED_PIN 14 // ANALOG NUMBERING
  #endif

  #define ORIG_BEEPER_PIN 64

  #define LCD_PINS_RS 14
  #define LCD_PINS_ENABLE 15
  #define LCD_PINS_D4 30
  #define LCD_PINS_D5 31
  #define LCD_PINS_D6 32
  #define LCD_PINS_D7 33

  //buttons are directly attached using keypad
  #define BTN_EN1 61
  #define BTN_EN2 59
  #define BTN_ENC 43 //the click

  #define BLEN_C 2
  #define BLEN_B 1
  #define BLEN_A 0

  #define SD_DETECT_PIN -1  // Megatronics does not use this port

#endif // MEGATRONICS_2
/****************************************************************************************/



/****************************************************************************************
* 702
* Minitronics v1.0
****************************************************************************************/
#if MB(MINITRONICS)
  #define KNOWN_BOARD 1

  #ifndef __AVR_ATmega1281__
    #error Oops! Make sure you have 'Minitronics ' selected from the 'Tools -> Boards' menu.
  #endif

  #define LARGE_FLASH        true

  #define ORIG_X_STEP_PIN 48
  #define ORIG_X_DIR_PIN 47
  #define ORIG_X_ENABLE_PIN 49
  #define ORIG_X_MIN_PIN 5
  #define ORIG_X_MAX_PIN -1 //2 //Max endstops default to disabled "-1", set to commented value to enable.

  #define ORIG_Y_STEP_PIN 39 // A6
  #define ORIG_Y_DIR_PIN 40 // A0
  #define ORIG_Y_ENABLE_PIN 38
  #define ORIG_Y_MIN_PIN 2
  #define ORIG_Y_MAX_PIN -1 //15

  #define ORIG_Z_STEP_PIN 42 // A2
  #define ORIG_Z_DIR_PIN 43 // A6
  #define ORIG_Z_ENABLE_PIN 41 // A1
  #define ORIG_Z_MIN_PIN 6
  #define ORIG_Z_MAX_PIN -1

  #define ORIG_E0_STEP_PIN 45
  #define ORIG_E0_DIR_PIN 44
  #define ORIG_E0_ENABLE_PIN 27

  #define ORIG_E1_STEP_PIN 36
  #define ORIG_E1_DIR_PIN 35
  #define ORIG_E1_ENABLE_PIN 37

  #define ORIG_E2_STEP_PIN -1
  #define ORIG_E2_DIR_PIN -1
  #define ORIG_E2_ENABLE_PIN -1
  
  #define SDPOWER -1
  #define SDSS               53

  #define LED_PIN 46

  #define ORIG_FAN_PIN 9
  #define ORIG_FAN2_PIN -1
  #define ORIG_PS_ON_PIN -1
  #define KILL_PIN -1

  #define ORIG_HEATER_0_PIN 7 // EXTRUDER 1
  #define ORIG_HEATER_1_PIN 8 // EXTRUDER 2
  #define ORIG_HEATER_2_PIN 9 // thermo couple

  #if TEMP_SENSOR_0 == -1
    #define ORIG_TEMP_0_PIN 5 // ANALOG NUMBERING
  #else
    #define ORIG_TEMP_0_PIN 7 // ANALOG NUMBERING
  #endif
  #define ORIG_TEMP_1_PIN 6   // ANALOG NUMBERING
  #define ORIG_TEMP_2_PIN -1  // ANALOG NUMBERING

  #define ORIG_HEATER_BED_PIN 3 // BED
  #define ORIG_TEMP_BED_PIN 6   // ANALOG NUMBERING

  #define ORIG_BEEPER_PIN -1

  #define LCD_PINS_RS -1
  #define LCD_PINS_ENABLE -1
  #define LCD_PINS_D4 -1
  #define LCD_PINS_D5 -1
  #define LCD_PINS_D6 -1
  #define LCD_PINS_D7 -1

  //buttons are directly attached using keypad
  #define BTN_EN1 -1
  #define BTN_EN2 -1
  #define BTN_ENC -1 //the click

  #define BLEN_C 2
  #define BLEN_B 1
  #define BLEN_A 0

  #define SD_DETECT_PIN -1  // Megatronics does not use this port

#endif // MEGATRONICS_1
/****************************************************************************************/



/****************************************************************************************
* 703
* MegaTronics v3.0
****************************************************************************************/
#if MB(MEGATRONICS_3)
  #define KNOWN_BOARD 1

  #ifndef __AVR_ATmega2560__
    #error Oops! Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
  #endif

  #define LARGE_FLASH        true

  #define ORIG_X_STEP_PIN         58
  #define ORIG_X_DIR_PIN          57
  #define ORIG_X_ENABLE_PIN       59
  #define ORIG_X_MIN_PIN               37
  #define ORIG_X_MAX_PIN               40   //2 //Max endstops default to disabled "-1", set to commented value to enable.

  #define ORIG_Y_STEP_PIN          5 // A6
  #define ORIG_Y_DIR_PIN          17 // A0
  #define ORIG_Y_ENABLE_PIN        4
  #define ORIG_Y_MIN_PIN               41
  #define ORIG_Y_MAX_PIN               38   //15

  #define ORIG_Z_STEP_PIN         16 // A2
  #define ORIG_Z_DIR_PIN          11 // A6
  #define ORIG_Z_ENABLE_PIN       3 // A1
  #define ORIG_Z_MIN_PIN               18
  #define ORIG_Z_MAX_PIN               19

  #define ORIG_E0_STEP_PIN        28
  #define ORIG_E0_DIR_PIN         27
  #define ORIG_E0_ENABLE_PIN      29

  #define ORIG_E1_STEP_PIN        25
  #define ORIG_E1_DIR_PIN         24
  #define ORIG_E1_ENABLE_PIN      26

  #define ORIG_E2_STEP_PIN        22
  #define ORIG_E2_DIR_PIN         60
  #define ORIG_E2_ENABLE_PIN      23

  #define ORIG_E3_STEP_PIN        54
  #define ORIG_E3_DIR_PIN         55
  #define ORIG_E3_ENABLE_PI       55

  #define SDPOWER -1
  #define SDSS 53
  #define LED_PIN 13

  #if NUM_SERVOS > 0
    #define SERVO0_PIN            46 //AUX3-6
    #if NUM_SERVOS > 1
      #define SERVO1_PIN          47 //AUX3-5
      #if NUM_SERVOS > 2
        #define SERVO2_PIN        48 //AUX3-4
        #if NUM_SERVOS > 3
          #define SERVO2_PIN      49 //AUX3-3
        #endif
      #endif
    #endif
  #endif

  #define ORIG_PS_ON_PIN 12
  #define KILL_PIN -1

  #define ORIG_HEATER_0_PIN 2 
  #define ORIG_HEATER_1_PIN 8 
  #define ORIG_HEATER_2_PIN 9 
  #define ORIG_HEATER_BED_PIN 10 
  #define ORIG_FAN_PIN 6
  #define ORIG_FAN2_PIN 7

  #if TEMP_SENSOR_0 == -1
    #define ORIG_TEMP_0_PIN 11 // ANALOG NUMBERING
  #else
    #define ORIG_TEMP_0_PIN 15 // ANALOG NUMBERING
  #endif

  #if TEMP_SENSOR_1 == -1
    #define ORIG_TEMP_1_PIN 10 // ANALOG NUMBERING
  #else
    #define ORIG_TEMP_1_PIN 13 // ANALOG NUMBERING
  #endif

  #if TEMP_SENSOR_2 == -1
    #define ORIG_TEMP_2_PIN 9 // ANALOG NUMBERING
  #else
    #define ORIG_TEMP_2_PIN 12 // ANALOG NUMBERING
  #endif

  #if TEMP_SENSOR_BED == -1
    #define ORIG_TEMP_BED_PIN 8 // ANALOG NUMBERING
  #else 
    #define ORIG_TEMP_BED_PIN 14 // ANALOG NUMBERING
  #endif

  #define ORIG_BEEPER_PIN 61

  #define LCD_PINS_RS 32 
  #define LCD_PINS_ENABLE 31
  #define LCD_PINS_D4 14
  #define LCD_PINS_D5 30 
  #define LCD_PINS_D6 39
  #define LCD_PINS_D7 15

  #define SHIFT_CLK 43
  #define SHIFT_LD 35
  #define SHIFT_OUT 34
  #define SHIFT_EN 44

  //buttons are directly attached using keypad
  #define BTN_EN1 44
  #define BTN_EN2 45
  #define BTN_ENC 33 //the click

  #define BLEN_C 2
  #define BLEN_B 1
  #define BLEN_A 0

  #define SD_DETECT_PIN -1  // Megatronics does not use this port

#endif  // MEGATRONICS_3
/****************************************************************************************/



/****************************************************************************************
* 705
* ULTRATRONICS
*****************************************************************************************/
#if MB(ULTRATRONICS)
#define KNOWN_BOARD

#ifndef __SAM3X8E__
  #error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

// X AXIS
#define ORIG_X_STEP_PIN       35  
#define ORIG_X_DIR_PIN        34  
#define ORIG_X_ENABLE_PIN     37  
#define ORIG_X_MIN_PIN        -1  
#define ORIG_X_MAX_PIN        -1  
#define X_MS1_PIN             -1  

// Y AXIS
#define ORIG_Y_STEP_PIN       22  
#define ORIG_Y_DIR_PIN        23  
#define ORIG_Y_ENABLE_PIN     9  
#define ORIG_Y_MIN_PIN        -1  
#define ORIG_Y_MAX_PIN        -1  
#define Y_MS1_PIN             -1  

// Z AXIS
#define ORIG_Z_STEP_PIN       25  
#define ORIG_Z_DIR_PIN        26  
#define ORIG_Z_ENABLE_PIN     24  
#define ORIG_Z_MIN_PIN        -1  
#define ORIG_Z_MAX_PIN        -1  
#define Z_MS1_PIN             -1  

// E0 AXIS
#define ORIG_E0_STEP_PIN      47  
#define ORIG_E0_DIR_PIN       46  
#define ORIG_E0_ENABLE_PIN    48  
#define E0_MS1_PIN            -1  

// E1 AXIS
#define ORIG_E1_STEP_PIN      44  
#define ORIG_E1_DIR_PIN       36  
#define ORIG_E1_ENABLE_PIN    45  

// E2 AXIS
#define ORIG_E2_STEP_PIN      42 
#define ORIG_E2_DIR_PIN       41 
#define ORIG_E2_ENABLE_PIN    43 

// E3 AXIS
#define ORIG_E3_STEP_PIN      39 
#define ORIG_E3_DIR_PIN       38 
#define ORIG_E3_ENABLE_PIN    40 

#define MOTOR_FAULT_PIN       -1 

#define SDPOWER               -1
#define SDSS                  -1 
#define SD_DETECT_PIN         -1 
#define LED_PIN               -1

#define ORIG_FAN_PIN          -1 
#define FAN2_PIN              -1 

#define ORIG_PS_ON_PIN        -1
#define KILL_PIN              -1
#define SUICIDE_PIN           -1 //PIN that has to be turned on right after start, to keep power flowing.

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define ORIG_HEATER_BED_PIN   69 
#define ORIG_HEATER_0_PIN     68 
#define ORIG_HEATER_1_PIN      8 
#define ORIG_HEATER_2_PIN      9 
#define ORIG_HEATER_3_PIN     97 

#define ORIG_TEMP_BED_PIN      0 
#define ORIG_TEMP_0_PIN        1 
#define ORIG_TEMP_1_PIN       52 
#define ORIG_TEMP_2_PIN       51 
#define ORIG_TEMP_3_PIN       50 

#define LED_RED_PIN           40 
#define LED_GREEN_PIN         41 
#define CASE_LIGHTS_PIN       36 

#define EXP_VOLTAGE_LEVEL_PIN 65

#define DAC0_SYNC             53 
#define DAC1_SYNC              6 

//64K SPI EEPROM
#define SPI_CHAN_EEPROM1       2
#define SPI_EEPROM1_CS        25 

//2K SPI EEPROM
#define SPI_EEPROM2_CS        26 

//** FLASH SPI**/
//32Mb
#define SPI_FLASH_CS          23 

/** Display **/

// GLCD on expansion port
#if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)

  #define LCD_PINS_RS         18
  #define LCD_PINS_ENABLE     15
  #define LCD_PINS_D4         19
  #define BEEPER_PIN          64

  #define BTN_EN1             14
  #define BTN_EN2             16
  #define BTN_ENC             17
  
  #if UI_VOLTAGE_LEVEL != 1
    #undef UI_VOLTAGE_LEVEL
    #define UI_VOLTAGE_LEVEL  1
  #endif
     
#endif //REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

#if NUM_SERVOS > 0
  #define SERVO0_PIN          -1
  #if NUM_SERVOS > 1
    #define SERVO1_PIN        -1
    #if NUM_SERVOS > 2
      #define SERVO2_PIN      -1
      #if NUM_SERVOS > 3
        #define SERVO3_PIN    -1
      #endif
    #endif
  #endif
#endif

#endif // ULTRATRONICS
/****************************************************************************************/



/****************************************************************************************
* 71
* Ultimaker pin assignment (Old electronics)
****************************************************************************************/
#if MB(ULTIMAKER_OLD)
  #define KNOWN_BOARD

  #ifndef __AVR_ATmega1280__
    #ifndef __AVR_ATmega2560__
      #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
    #endif
  #endif

  #define LARGE_FLASH true

  #define ORIG_X_STEP_PIN 25
  #define ORIG_X_DIR_PIN 23
  #define ORIG_X_MIN_PIN 15
  #define ORIG_X_MAX_PIN 14
  #define ORIG_X_ENABLE_PIN 27

  #define ORIG_Y_STEP_PIN 31
  #define ORIG_Y_DIR_PIN 33
  #define ORIG_Y_MIN_PIN 17
  #define ORIG_Y_MAX_PIN 16
  #define ORIG_Y_ENABLE_PIN 29

  #define ORIG_Z_STEP_PIN 37
  #define ORIG_Z_DIR_PIN 39
  #define ORIG_Z_MIN_PIN 19
  #define ORIG_Z_MAX_PIN 18
  #define ORIG_Z_ENABLE_PIN 35

  #define ORIG_HEATER_BED_PIN -1
  #define ORIG_TEMP_BED_PIN -1

  #define ORIG_HEATER_0_PIN  2
  #define ORIG_TEMP_0_PIN 8

  #define ORIG_HEATER_1_PIN 1
  #define ORIG_TEMP_1_PIN 1

  #define ORIG_HEATER_2_PIN -1
  #define ORIG_TEMP_2_PIN -1

  #define ORIG_E0_STEP_PIN         43
  #define ORIG_E0_DIR_PIN          45
  #define ORIG_E0_ENABLE_PIN       41

  #define ORIG_E1_STEP_PIN         -1
  #define ORIG_E1_DIR_PIN          -1
  #define ORIG_E1_ENABLE_PIN       -1

  #define SDPOWER                  -1
  #define SDSS                     -1
  #define LED_PIN                  -1
  #define ORIG_FAN_PIN             -1
  #define ORIG_PS_ON_PIN           -1
  #define KILL_PIN                 -1
  #define SUICIDE_PIN              -1  //PIN that has to be turned on right after start, to keep power flowing.

  #define LCD_PINS_RS 24
  #define LCD_PINS_ENABLE 22
  #define LCD_PINS_D4 36
  #define LCD_PINS_D5 34
  #define LCD_PINS_D6 32
  #define LCD_PINS_D7 30

#endif // ULTIMAKER_OLD
/****************************************************************************************/



/****************************************************************************************
* 72
* Ultiboard v2.0 pin assignment
****************************************************************************************/
#if MB(ULTIMAIN_2)
#define KNOWN_BOARD

#ifndef __AVR_ATmega2560__
 #error Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN 25
#define ORIG_X_DIR_PIN 23
#define X_STOP_PIN 22
#define ORIG_X_ENABLE_PIN 27

#define ORIG_Y_STEP_PIN 32
#define ORIG_Y_DIR_PIN 33
#define Y_STOP_PIN 26
#define ORIG_Y_ENABLE_PIN 31

#define ORIG_Z_STEP_PIN 35
#define ORIG_Z_DIR_PIN 36
#define Z_STOP_PIN 29
#define ORIG_Z_ENABLE_PIN 34

#define ORIG_HEATER_BED_PIN 4
#define ORIG_TEMP_BED_PIN 10

#define ORIG_HEATER_0_PIN  2
#define ORIG_TEMP_0_PIN 8

#define ORIG_HEATER_1_PIN 3
#define ORIG_TEMP_1_PIN 9

#define ORIG_HEATER_2_PIN -1
#define ORIG_TEMP_2_PIN -1

#define ORIG_E0_STEP_PIN         42
#define ORIG_E0_DIR_PIN          43
#define ORIG_E0_ENABLE_PIN       37

#define ORIG_E1_STEP_PIN         49
#define ORIG_E1_DIR_PIN          47
#define ORIG_E1_ENABLE_PIN       48

#define SDPOWER                  -1
#define SDSS                     53
#define LED_PIN                   8
#define ORIG_FAN_PIN              7
#define ORIG_PS_ON_PIN           -1
#define KILL_PIN                 -1
#define SUICIDE_PIN              -1  //PIN that has to be turned on right after start, to keep power flowing.
#define SAFETY_TRIGGERED_PIN     28 //PIN to detect the safety circuit has triggered
#define MAIN_VOLTAGE_MEASURE_PIN 14 //Analogue PIN to measure the main voltage, with a 100k - 4k7 resitor divider.

#define MOTOR_CURRENT_PWM_XY_PIN 44
#define MOTOR_CURRENT_PWM_Z_PIN  45
#define MOTOR_CURRENT_PWM_E_PIN  46
//Motor current PWM conversion, PWM value = MotorCurrentSetting * 255 / range
#define MOTOR_CURRENT_PWM_RANGE 2000
#define DEFAULT_PWM_MOTOR_CURRENT  {1300, 1300, 1250}

//arduino pin witch triggers an piezzo beeper
#define ORIG_BEEPER_PIN 18

#define LCD_PINS_RS 20
#define LCD_PINS_ENABLE 15
#define LCD_PINS_D4 14
#define LCD_PINS_D5 21
#define LCD_PINS_D6 5
#define LCD_PINS_D7 6

//buttons are directly attached
#define BTN_EN1 40
#define BTN_EN2 41
#define BTN_ENC 19  //the click

#define BLEN_C 2
#define BLEN_B 1
#define BLEN_A 0

#define SD_DETECT_PIN 39

#endif // ULTIMAIN_2
/****************************************************************************************/



/****************************************************************************************
* 77
* 3DRAG
****************************************************************************************/

#if MB(3DRAG)
#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH true

#define ORIG_X_STEP_PIN         54
#define ORIG_X_DIR_PIN          55
#define ORIG_X_ENABLE_PIN       38
#define ORIG_X_MIN_PIN           3
#define ORIG_X_MAX_PIN           2

#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       56
#define ORIG_Y_MIN_PIN          14
#define ORIG_Y_MAX_PIN          15

#define ORIG_Z_STEP_PIN         46
#define ORIG_Z_DIR_PIN          48
#define ORIG_Z_ENABLE_PIN       63
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          -1

#define Y2_STEP_PIN             36
#define Y2_DIR_PIN              34
#define Y2_ENABLE_PIN           30

#define Z2_STEP_PIN             36
#define Z2_DIR_PIN              34
#define Z2_ENABLE_PIN           30

#define ORIG_E0_STEP_PIN        26
#define ORIG_E0_DIR_PIN         28
#define ORIG_E0_ENABLE_PIN      24

#define ORIG_E1_STEP_PIN        36
#define ORIG_E1_DIR_PIN         34
#define ORIG_E1_ENABLE_PIN      30

#define SDPOWER                 -1
#define SDSS                    25
#define LED_PIN                 13


#define ORIG_FAN_PIN             8 // IO pin. Buffer needed

#define ORIG_PS_ON_PIN          12

#if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) || ENABLED(G3D_PANEL)
  #define KILL_PIN               41
#else
  #define KILL_PIN               -1
#endif

#define ORIG_HEATER_0_PIN        10  // HOTEND 1
#define ORIG_HEATER_1_PIN        12  // HOTEND 2
#define ORIG_HEATER_2_PIN         6  // HOTEND 3
#define ORIG_HEATER_3_PIN        -1

#define ORIG_TEMP_0_PIN          13   // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN          15   // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN          -1   // ANALOG NUMBERING

#define ORIG_HEATER_BED_PIN       9   // NO BED

#define ORIG_TEMP_BED_PIN        14   // ANALOG NUMBERING

#if NUM_SERVOS > 0
  #define SERVO0_PIN             11
  #if NUM_SERVOS > 1
    #define SERVO1_PIN            6
    #if NUM_SERVOS > 2
      #define SERVO2_PIN          5
      #if NUM_SERVOS > 3
        #define SERVO3_PIN        4
      #endif
    #endif
  #endif
#endif

#define ORIG_BEEPER_PIN          33

#if ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL)
  #define ORIG_BEEPER_PIN        -1

  #define LCD_PINS_RS 27
  #define LCD_PINS_ENABLE 29
  #define LCD_PINS_D4 37
  #define LCD_PINS_D5 35
  #define LCD_PINS_D6 33
  #define LCD_PINS_D7 31

  // Buttons
  #define BTN_EN1 16
  #define BTN_EN2 17
  #define BTN_ENC 23 //the click
#endif // ULTRA_LCD && NEWPANEL

// SPI for Max6675 Thermocouple
#define MAX6675_SS            66  // Do not use pin 53 if there is even the remote possibility of using Display/SD card

#endif // 3DRAG
/****************************************************************************************/



/****************************************************************************************
* 78
* K8200
****************************************************************************************/

#if MB(K8200)
#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH true

#define ORIG_X_STEP_PIN         54
#define ORIG_X_DIR_PIN          55
#define ORIG_X_ENABLE_PIN       38
#define ORIG_X_MIN_PIN           3
#define ORIG_X_MAX_PIN           2

#define ORIG_Y_STEP_PIN         60
#define ORIG_Y_DIR_PIN          61
#define ORIG_Y_ENABLE_PIN       56
#define ORIG_Y_MIN_PIN          14
#define ORIG_Y_MAX_PIN          15

#define ORIG_Z_STEP_PIN         46
#define ORIG_Z_DIR_PIN          48
#define ORIG_Z_ENABLE_PIN       62
#define ORIG_Z_MIN_PIN          18
#define ORIG_Z_MAX_PIN          -1

#define Y2_STEP_PIN             36
#define Y2_DIR_PIN              34
#define Y2_ENABLE_PIN           30

#define Z2_STEP_PIN             36
#define Z2_DIR_PIN              34
#define Z2_ENABLE_PIN           30

#define ORIG_E0_STEP_PIN        26
#define ORIG_E0_DIR_PIN         28
#define ORIG_E0_ENABLE_PIN      24

#define ORIG_E1_STEP_PIN        36
#define ORIG_E1_DIR_PIN         34
#define ORIG_E1_ENABLE_PIN      30

#define SDPOWER                 -1
#define SDSS                    25
#define LED_PIN                 13


#define ORIG_FAN_PIN             8 // IO pin. Buffer needed

#define ORIG_PS_ON_PIN          12

#if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) || ENABLED(G3D_PANEL)
  #define KILL_PIN               41
#else
  #define KILL_PIN               -1
#endif

#define ORIG_HEATER_0_PIN        10  // HOTEND 1
#define ORIG_HEATER_1_PIN        12  // HOTEND 2
#define ORIG_HEATER_2_PIN         6  // HOTEND 3
#define ORIG_HEATER_3_PIN        -1

#define ORIG_TEMP_0_PIN          13   // ANALOG NUMBERING
#define ORIG_TEMP_1_PIN          15   // ANALOG NUMBERING
#define ORIG_TEMP_2_PIN          -1   // ANALOG NUMBERING

#define ORIG_HEATER_BED_PIN       9   // NO BED

#define ORIG_TEMP_BED_PIN        14   // ANALOG NUMBERING

#if NUM_SERVOS > 0
  #define SERVO0_PIN             11
  #if NUM_SERVOS > 1
    #define SERVO1_PIN            6
    #if NUM_SERVOS > 2
      #define SERVO2_PIN          5
      #if NUM_SERVOS > 3
        #define SERVO3_PIN        4
      #endif
    #endif
  #endif
#endif

#define ORIG_BEEPER_PIN           33

#if ENABLED(ULTRA_LCD) && ENABLED(NEWPANEL)
  #define ORIG_BEEPER_PIN -1

  #define LCD_PINS_RS 27
  #define LCD_PINS_ENABLE 29
  #define LCD_PINS_D4 37
  #define LCD_PINS_D5 35
  #define LCD_PINS_D6 33
  #define LCD_PINS_D7 31

  // Buttons
  #define BTN_EN1 16
  #define BTN_EN2 17
  #define BTN_ENC 23 //the click
#endif // ULTRA_LCD && NEWPANEL

// SPI for Max6675 Thermocouple
#define MAX6675_SS          66  // Do not use pin 53 if there is even the remote possibility of using Display/SD card

#endif // K8200
/****************************************************************************************/



/****************************************************************************************
* 8 - 81
* Teensylu 0.7 / Printrboard pin assignments (AT90USB1286)
* Requires the Teensyduino software with Teensy++ 2.0 selected in Arduino IDE!
  http://www.pjrc.com/teensy/teensyduino.html
* See http://reprap.org/wiki/Printrboard for more info
****************************************************************************************/

#if MB(TEENSYLU) || MB(PRINTRBOARD)
#define KNOWN_BOARD 1
#define AT90USB 1286  // Disable MarlinSerial etc.

#ifndef __AVR_AT90USB1286__
#error Oops!  Make sure you have 'Teensy++ 2.0' selected from the 'Tools -> Boards' menu.
#endif

#ifdef AT90USBxx_TEENSYPP_ASSIGNMENTS  // use Teensyduino Teensy++2.0 pin assignments instead of Marlin traditional.
#error These Teensylu/Printrboard assignments depend on traditional Marlin assignments, not AT90USBxx_TEENSYPP_ASSIGNMENTS in fastio.h
#endif

#define LARGE_FLASH        true

#define ORIG_X_STEP_PIN          0
#define ORIG_X_DIR_PIN           1
#define ORIG_X_ENABLE_PIN       39

#define ORIG_Y_STEP_PIN          2
#define ORIG_Y_DIR_PIN           3
#define ORIG_Y_ENABLE_PIN       38

#define ORIG_Z_STEP_PIN          4
#define ORIG_Z_DIR_PIN           5
#define ORIG_Z_ENABLE_PIN       23

#define ORIG_E0_STEP_PIN         6
#define ORIG_E0_DIR_PIN          7
#define ORIG_E0_ENABLE_PIN      19

#define ORIG_HEATER_0_PIN       21  // Extruder
#define ORIG_HEATER_1_PIN       -1
#define ORIG_HEATER_2_PIN       -1
#define ORIG_HEATER_BED_PIN     20  // Bed
#define ORIG_FAN_PIN            22  // Fan
// You may need to change ORIG_FAN_PIN to 16 because Marlin isn't using fastio.h
// for the fan and Teensyduino uses a different pin mapping.

#if MB(TEENSYLU)  // Teensylu
  #define X_STOP_PIN         13
  #define Y_STOP_PIN         14
  #define Z_STOP_PIN         15
  #define ORIG_TEMP_0_PIN          7  // Extruder / Analog pin numbering
  #define ORIG_TEMP_BED_PIN        6  // Bed / Analog pin numbering
#else  // Printrboard
  #define X_STOP_PIN         35
  #define Y_STOP_PIN          8
  #define Z_STOP_PIN         36
  #define ORIG_TEMP_0_PIN          1  // Extruder / Analog pin numbering
  #define ORIG_TEMP_BED_PIN        0  // Bed / Analog pin numbering
#endif

#define ORIG_TEMP_1_PIN         -1
#define ORIG_TEMP_2_PIN         -1

#define SDPOWER            -1
#define SDSS                8
#define LED_PIN            -1
#define ORIG_PS_ON_PIN     -1
#define KILL_PIN           -1
#define ALARM_PIN          -1

#endif // TEENSYLU || PRINTRBOARD
/****************************************************************************************/




/****************************************************************************************
* 80
* RUMBA
****************************************************************************************/

#if MB(RUMBA)
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN         17
#define ORIG_X_DIR_PIN          16
#define ORIG_X_ENABLE_PIN       48
#define ORIG_X_MIN_PIN               37
#define ORIG_X_MAX_PIN               36

#define ORIG_Y_STEP_PIN         54
#define ORIG_Y_DIR_PIN          47
#define ORIG_Y_ENABLE_PIN       55
#define ORIG_Y_MIN_PIN               35
#define ORIG_Y_MAX_PIN               34

#define Y2_STEP_PIN             26
#define Y2_DIR_PIN              25
#define Y2_ENABLE_PIN           27

#define ORIG_Z_STEP_PIN         57
#define ORIG_Z_DIR_PIN          56
#define ORIG_Z_ENABLE_PIN       62
#define ORIG_Z_MIN_PIN               33
#define ORIG_Z_MAX_PIN               32

#define Z2_STEP_PIN             26
#define Z2_DIR_PIN              25
#define Z2_ENABLE_PIN           27

#define ORIG_E0_STEP_PIN        23
#define ORIG_E0_DIR_PIN         22
#define ORIG_E0_ENABLE_PIN      24

#define ORIG_E1_STEP_PIN        26
#define ORIG_E1_DIR_PIN         25
#define ORIG_E1_ENABLE_PIN      27

#define ORIG_E2_STEP_PIN        29
#define ORIG_E2_DIR_PIN         28
#define ORIG_E2_ENABLE_PIN      39

#define LED_PIN                 13

#define ORIG_FAN_PIN            7
//additional FAN1 PIN (e.g. useful for electronics fan or light on/off) on PIN 8

#define ORIG_PS_ON_PIN          45
#define KILL_PIN                46

#if (TEMP_SENSOR_0==0)
 #define ORIG_TEMP_0_PIN             -1
 #define ORIG_HEATER_0_PIN           -1
#else
 #define ORIG_HEATER_0_PIN           2    // EXTRUDER 1
 #if (TEMP_SENSOR_0==-1)
  #define ORIG_TEMP_0_PIN            6    // ANALOG NUMBERING - connector *K1* on RUMBA thermocouple ADD ON is used
 #else
  #define ORIG_TEMP_0_PIN            15   // ANALOG NUMBERING - default connector for thermistor *T0* on rumba board is used
 #endif
#endif

#if (TEMP_SENSOR_1==0)
 #define ORIG_TEMP_1_PIN             -1
 #define ORIG_HEATER_1_PIN           -1
#else
 #define ORIG_HEATER_1_PIN           3    // EXTRUDER 2
 #if (TEMP_SENSOR_1==-1)
  #define ORIG_TEMP_1_PIN            5    // ANALOG NUMBERING - connector *K2* on RUMBA thermocouple ADD ON is used
 #else
  #define ORIG_TEMP_1_PIN            14   // ANALOG NUMBERING - default connector for thermistor *T1* on rumba board is used
 #endif
#endif

#if (TEMP_SENSOR_2==0)
 #define ORIG_TEMP_2_PIN         -1
 #define ORIG_HEATER_2_PIN       -1
#else
 #define ORIG_HEATER_2_PIN        6    // EXTRUDER 3
 #if (TEMP_SENSOR_2==-1)
  #define ORIG_TEMP_2_PIN         7    // ANALOG NUMBERING - connector *K3* on RUMBA thermocouple ADD ON is used <-- this can not be used when TEMP_SENSOR_BED is defined as thermocouple
 #else
  #define ORIG_TEMP_2_PIN         13   // ANALOG NUMBERING - default connector for thermistor *T2* on rumba board is used
 #endif
#endif

//optional for extruder 4 or chamber: #define TEMP_X_PIN         12   // ANALOG NUMBERING - default connector for thermistor *T3* on rumba board is used
//optional FAN1 can be used as 4th heater output: #define ORIG_HEATER_3_PIN       8    // EXTRUDER 4

#if (TEMP_SENSOR_BED==0)
 #define ORIG_TEMP_BED_PIN       -1
 #define ORIG_HEATER_BED_PIN     -1
#else
 #define ORIG_HEATER_BED_PIN      9    // BED
 #if (TEMP_SENSOR_BED==-1)
  #define ORIG_TEMP_BED_PIN       7    // ANALOG NUMBERING - connector *K3* on RUMBA thermocouple ADD ON is used <-- this can not be used when TEMP_SENSOR_2 is defined as thermocouple
 #else
  #define ORIG_TEMP_BED_PIN       11   // ANALOG NUMBERING - default connector for thermistor *THB* on rumba board is used
 #endif
#endif

#define SDPOWER            -1
#define SDSS               53
#define SD_DETECT_PIN      49
#define ORIG_BEEPER_PIN    44
#define LCD_PINS_RS        19
#define LCD_PINS_ENABLE    42
#define LCD_PINS_D4        18
#define LCD_PINS_D5        38
#define LCD_PINS_D6        41
#define LCD_PINS_D7        40
#define BTN_EN1            11
#define BTN_EN2            12
#define BTN_ENC            43

#endif // RUMBA
/****************************************************************************************/




/****************************************************************************************
* 82
* Brainwave 1.0 pin assignments (AT90USB646)
* Requires hardware bundle for Arduino:
*  https://github.com/unrepentantgeek/brainwave-arduino
****************************************************************************************/

#if MB(BRAINWAVE)
#define KNOWN_BOARD 1

#define AT90USB 646  // Disable MarlinSerial etc.

#ifndef __AVR_AT90USB646__
#error Oops!  Make sure you have 'Brainwave' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN         27
#define ORIG_X_DIR_PIN          29
#define ORIG_X_ENABLE_PIN       28
#define X_STOP_PIN          7
#define X_ATT_PIN          26

#define ORIG_Y_STEP_PIN         31
#define ORIG_Y_DIR_PIN          33
#define ORIG_Y_ENABLE_PIN       32
#define Y_STOP_PIN          6
#define Y_ATT_PIN          30

#define ORIG_Z_STEP_PIN         17
#define ORIG_Z_DIR_PIN          19
#define ORIG_Z_ENABLE_PIN       18
#define Z_STOP_PIN          5
#define Z_ATT_PIN          16

#define ORIG_E0_STEP_PIN        21
#define ORIG_E0_DIR_PIN         23
#define ORIG_E0_ENABLE_PIN      22
#define E0_ATT_PIN         20

#define ORIG_HEATER_0_PIN        4  // Extruder
#define ORIG_HEATER_1_PIN       -1
#define ORIG_HEATER_2_PIN       -1
#define ORIG_HEATER_BED_PIN     38  // Bed
#define ORIG_FAN_PIN             3  // Fan

#define ORIG_TEMP_0_PIN          7  // Extruder / Analog pin numbering
#define ORIG_TEMP_1_PIN         -1
#define ORIG_TEMP_2_PIN         -1
#define ORIG_TEMP_BED_PIN        6  // Bed / Analog pin numbering

#define SDPOWER            -1
#define SDSS               -1
#define LED_PIN            39
#define ORIG_PS_ON_PIN     -1
#define KILL_PIN           -1
#define ALARM_PIN          -1

#endif // BRAINWAVE
/****************************************************************************************/



/****************************************************************************************
* 83
* SAV MkI pin assignments (AT90USB1286)
* Requires the Teensyduino software with Teensy++ 2.0 selected in Arduino IDE!
* http://www.pjrc.com/teensy/teensyduino.html
* RepRap Clone Wars project board.
****************************************************************************************/

#if MB(SAV_MKI)  // SAV Mk-I
#define KNOWN_BOARD 1
#define AT90USB 1286  // Disable MarlinSerial etc.

#ifndef __AVR_AT90USB1286__
#error Oops!  Make sure you have 'Teensy++ 2.0' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH        true


#define ORIG_X_STEP_PIN         0
#define ORIG_X_DIR_PIN          1
#define ORIG_X_ENABLE_PIN       39

#define ORIG_Y_STEP_PIN         2
#define ORIG_Y_DIR_PIN          3
#define ORIG_Y_ENABLE_PIN       38

#define ORIG_Z_STEP_PIN         4
#define ORIG_Z_DIR_PIN          5
#define ORIG_Z_ENABLE_PIN       23

#define ORIG_E0_STEP_PIN         6
#define ORIG_E0_DIR_PIN          7
#define ORIG_E0_ENABLE_PIN       19

#define ORIG_HEATER_0_PIN       21  // Extruder
#define ORIG_HEATER_1_PIN       -1
#define ORIG_HEATER_2_PIN       -1
#define ORIG_HEATER_BED_PIN     20  // Bed
#define ORIG_FAN_PIN            16  // Fan   -- from Teensyduino environment.
                                    // For the fan and Teensyduino uses a different pin mapping.
#define X_STOP_PIN              13
#define Y_STOP_PIN              14
#define Z_STOP_PIN              15
//#define Z_STOP_PIN            36  // For inductive sensor.

#define ORIG_TEMP_0_PIN          7  // Extruder / Analog pin numbering
#define ORIG_TEMP_BED_PIN        6  // Bed / Analog pin numbering

#define ORIG_TEMP_1_PIN         -1
#define ORIG_TEMP_2_PIN         -1

#define SDPOWER                 -1
#define SDSS                    20  // PB0 - 8 in marlin env.
#define LED_PIN                 -1
#define ORIG_PS_ON_PIN          -1
#define ALARM_PIN               -1
#define SD_DETECT_PIN           -1

#define ORIG_BEEPER_PIN         -1
#define LCD_PINS_RS             -1
#define LCD_PINS_ENABLE         -1
#define LCD_PINS_D4             -1
#define LCD_PINS_D5             -1
#define LCD_PINS_D6             -1
#define LCD_PINS_D7             -1

#if ENABLED(SAV_3DLCD)
// For LCD SHIFT register LCD
#define SR_DATA_PIN         1
#define SR_CLK_PIN          0

#define BTN_EN1            41
#define BTN_EN2            40
#define BTN_ENC            12

#define KILL_PIN           42 // A2 = 42 - teensy = 40
#define HOME_PIN          -1 // A4 = marlin 44 - teensy = 42

#if NUM_SERVOS > 0
  #define SERVO0_PIN       41 // In teensy's pin definition for pinMode (in Servo.cpp)
#endif

#endif

#endif // SAV_MKI
/****************************************************************************************/




/****************************************************************************************
* 84
* Teensy++ 2.0 Breadboard pin assignments (AT90USB1286)
* Requires the Teensyduino software with Teensy++ 2.0 selected in Arduino IDE!
*  http://www.pjrc.com/teensy/teensyduino.html
* See http://reprap.org/wiki/Printrboard for more info
* CLI build: DEFINES=AT90USBxx_TEENSYPP_ASSIGNMENTS HARDWARE_MOTHERBOARD=84  make
****************************************************************************************/

#if MB(TEENSY2)
#define KNOWN_BOARD 1
#define AT90USB 1286  // Disable MarlinSerial etc.

#ifndef __AVR_AT90USB1286__
#error Oops!  Make sure you have 'Teensy++ 2.0' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH        true

/* 
DaveX plan for Teensylu/printrboard-type pinouts (ref teensylu & sprinter) for a TeensyBreadboard:

                               USB
           GND       GND |-----#####-----| +5V              ATX +5SB
     ATX PS_ON    PWM 27 |b7   #####   b6| 26    PWM*       Stepper Enable 
                  PWM  0 |d0           b5| 25    PWM*        
                  PWM  1 |d1           b4| 24    PWM        
         X_MIN         2 |d2           b3| 23               MISO_PIN
         Y_MIN         3 |d3           b2| 22               MOSI_PIN
         Z_MIN         4 |d4  * *      b1| 21               SCK_PIN       
                       5 |d5  e e      b0| 20               SDSS              
                LED    6 |d6  5 4      e7| 19               
                       7 |d7           e6| 18               
       LCD  RS         8 |e0             | GND              
       LCD  EN         9 |e1   a4 a0    R| AREF             
       LCD  D4        10 |c0   a5 a1   f0| 38 A0            ENC_1           
       LCD  D5        11 |c1   a6 a2   f1| 39 A1            ENC_2
       LCD  D6        12 |c2   a7 a3   f2| 40 A2            ENC_CLK
       LCD  D6        13 |c3           f3| 41 A3            
      Bed Heat    PWM 14 |c4   V G R   f4| 42 A4            
 Extruder Heat    PWM 15 |c5   c n S   f5| 43 A5            
           Fan    PWM 16 |c6   c d T   f6| 44 A6            Bed TC
                      17 |c7   * * *   f7| 45 A7            Extruder TC * 4.7k * +5        
                         -----------------                  

      Interior E4: 36, INT4
      Interior E5: 37, INT5
      Interior PA0-7: 28-35  -- Printrboard and Teensylu use these pins for step & direction:
             T++ PA Signal  Marlin
    
       Z STEP  32 a4  a0 28 X STEP
       Z DIR   33 a5  a1 29 X DIR
       E STEP  34 a6  a2 30 Y STEP
       E DIR   35 a7  a3 31 Y DIR

*/

#ifndef AT90USBxx_TEENSYPP_ASSIGNMENTS  // use Teensyduino Teensy++2.0 pin assignments instead of Marlin alphabetical.
  #error  Uncomment #define AT90USBxx_TEENSYPP_ASSIGNMENTS in fastio.h for this config
  // or build from command line with:  DEFINES=AT90USBxx_TEENSYPP_ASSIGNMENTS HARDWARE_MOTHERBOARD=84  make
#endif

#define ORIG_X_STEP_PIN         28 //  0 Marlin
#define ORIG_X_DIR_PIN          29 //  1 Marlin
#define ORIG_X_ENABLE_PIN       26 

#define ORIG_Y_STEP_PIN         30 //  2 Marlin
#define ORIG_Y_DIR_PIN          31 //  3
#define ORIG_Y_ENABLE_PIN       26     // Shared w/x

#define ORIG_Z_STEP_PIN         32 //  4
#define ORIG_Z_DIR_PIN          33 //  5
#define ORIG_Z_ENABLE_PIN       26 // Shared w/x

#define ORIG_E0_STEP_PIN        34 //  6
#define ORIG_E0_DIR_PIN         35 //  7
#define ORIG_E0_ENABLE_PIN      26 // Shared w/x

#define ORIG_HEATER_0_PIN       15 //  21  // Extruder
#define ORIG_HEATER_1_PIN       -1
#define ORIG_HEATER_2_PIN       -1
#define ORIG_HEATER_BED_PIN     14 // 20  // Bed
#define ORIG_FAN_PIN            16 // 22  // Fan

#define X_STOP_PIN          2
#define Y_STOP_PIN          3
#define Z_STOP_PIN          4

#define ORIG_TEMP_0_PIN          7 // Extruder / Analog pin numbering
#define ORIG_TEMP_BED_PIN        6 // Bed / Analog pin numbering
#define ORIG_TEMP_1_PIN         -1
#define ORIG_TEMP_2_PIN         -1

#define SDPOWER            -1
#define SD_DETECT_PIN      -1
#define SDSS               20 // 8
#define LED_PIN             6
#define ORIG_PS_ON_PIN     27
#define KILL_PIN           -1
#define ALARM_PIN          -1

#if ENABLED(ULTIPANEL)
#define LCD_PINS_RS         8
#define LCD_PINS_ENABLE     9
#define LCD_PINS_D4        10
#define LCD_PINS_D5        11
#define LCD_PINS_D6        12
#define LCD_PINS_D7        13
#define BTN_EN1            38
#define BTN_EN2            39
#define BTN_ENC            40
#endif

#endif // TEENSY2
/****************************************************************************************/




/****************************************************************************************
* 88
* 5DPrint D8 Driver board
* https://bitbucket.org/makible/5dprint-d8-controller-board
****************************************************************************************/

#if MB(5DPRINT)

#define KNOWN_BOARD 1
#define AT90USB 1286  // Disable MarlinSerial etc.

#ifndef __AVR_AT90USB1286__
#error Oops!  Make sure you have 'Teensy++ 2.0' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH        true

#define ORIG_X_STEP_PIN          0
#define ORIG_X_DIR_PIN           1
#define ORIG_X_ENABLE_PIN       23
#define X_STOP_PIN              37

#define ORIG_Y_STEP_PIN          2
#define ORIG_Y_DIR_PIN           3
#define ORIG_Y_ENABLE_PIN       19
#define Y_STOP_PIN              36

#define ORIG_Z_STEP_PIN          4
#define ORIG_Z_DIR_PIN           5
#define ORIG_Z_ENABLE_PIN       18
#define Z_STOP_PIN              39

#define ORIG_E0_STEP_PIN         6
#define ORIG_E0_DIR_PIN          7
#define ORIG_E0_ENABLE_PIN      17

#define ORIG_HEATER_0_PIN       21  // Extruder
#define ORIG_HEATER_1_PIN       -1
#define ORIG_HEATER_2_PIN       -1
#define ORIG_HEATER_BED_PIN     20  // Bed
// You may need to change ORIG_FAN_PIN to 16 because Marlin isn't using fastio.h
// for the fan and Teensyduino uses a different pin mapping.
#define ORIG_FAN_PIN            16  // Fan

#define ORIG_TEMP_0_PIN          1  // Extruder / Analog pin numbering
#define ORIG_TEMP_BED_PIN        0  // Bed / Analog pin numbering

#define ORIG_TEMP_1_PIN         -1
#define ORIG_TEMP_2_PIN         -1

#define SDPOWER                 -1
#define LED_PIN                 -1
#define ORIG_PS_ON_PIN          -1
#define KILL_PIN                -1
#define ALARM_PIN               -1

// The SDSS pin uses a different pin mapping from file Sd2PinMap.h
#define SDSS                    20

// Microstepping pins
// Note that the pin mapping is not from fastio.h
// See Sd2PinMap.h for the pin configurations
#define X_MS1_PIN 25
#define X_MS2_PIN 26
#define Y_MS1_PIN 9
#define Y_MS2_PIN 8
#define Z_MS1_PIN 7
#define Z_MS2_PIN 6
#define E0_MS1_PIN 5
#define E0_MS2_PIN 4

#endif // 5DPRINT
/****************************************************************************************/




/****************************************************************************************
* 9
* Gen3+
****************************************************************************************/
#if MB(GEN3_PLUS)
#define MOTHERBOARD BOARD_SANGUINOLOLU_11   /*TODO: Figure out, Why is this done?*/
#define KNOWN_BOARD 1
#ifndef __AVR_ATmega644P__
#ifndef __AVR_ATmega1284P__
#error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu.
#endif
#endif

#define ORIG_X_STEP_PIN         15
#define ORIG_X_DIR_PIN          18
#define X_STOP_PIN         20

#define ORIG_Y_STEP_PIN         23
#define ORIG_Y_DIR_PIN          22
#define Y_STOP_PIN         25

#define ORIG_Z_STEP_PIN         27
#define ORIG_Z_DIR_PIN          28
#define Z_STOP_PIN         30

#define ORIG_E0_STEP_PIN        17
#define ORIG_E0_DIR_PIN         21

#define LED_PIN            -1

#define ORIG_FAN_PIN            -1

#define ORIG_PS_ON_PIN         14
#define KILL_PIN           -1

#define ORIG_HEATER_0_PIN       12 // (extruder)

#define ORIG_HEATER_BED_PIN     16 // (bed)
#define ORIG_X_ENABLE_PIN       19
#define ORIG_Y_ENABLE_PIN       24
#define ORIG_Z_ENABLE_PIN       29
#define ORIG_E0_ENABLE_PIN      13

#define ORIG_TEMP_0_PIN          0   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 33 extruder)
#define ORIG_TEMP_1_PIN         -1   
#define ORIG_TEMP_2_PIN         -1
#define ORIG_TEMP_BED_PIN        5   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 34 bed)  
#define SDPOWER            -1
#define SDSS               4
#define ORIG_HEATER_2_PIN       -1

#endif // GEN3_PLUS
/****************************************************************************************/




/****************************************************************************************
* 90 - 91
* Open Motion controller with enable based extruders
*
*                        ATMega644
*
*                        +---\/---+
*            (D 0) PB0  1|        |40  PA0 (AI 0 / D31)
*            (D 1) PB1  2|        |39  PA1 (AI 1 / D30)
*       INT2 (D 2) PB2  3|        |38  PA2 (AI 2 / D29)
*        PWM (D 3) PB3  4|        |37  PA3 (AI 3 / D28)
*        PWM (D 4) PB4  5|        |36  PA4 (AI 4 / D27)
*       MOSI (D 5) PB5  6|        |35  PA5 (AI 5 / D26)
*       MISO (D 6) PB6  7|        |34  PA6 (AI 6 / D25)
*        SCK (D 7) PB7  8|        |33  PA7 (AI 7 / D24)
*                  RST  9|        |32  AREF
*                  VCC 10|        |31  GND
*                  GND 11|        |30  AVCC
*                XTAL2 12|        |29  PC7 (D 23)
*                XTAL1 13|        |28  PC6 (D 22)
*       RX0 (D 8)  PD0 14|        |27  PC5 (D 21) TDI
*       TX0 (D 9)  PD1 15|        |26  PC4 (D 20) TDO
*  INT0 RX1 (D 10) PD2 16|        |25  PC3 (D 19) TMS
*  INT1 TX1 (D 11) PD3 17|        |24  PC2 (D 18) TCK
*       PWM (D 12) PD4 18|        |23  PC1 (D 17) SDA
*       PWM (D 13) PD5 19|        |22  PC0 (D 16) SCL
*       PWM (D 14) PD6 20|        |21  PD7 (D 15) PWM
*                        +--------+
*
****************************************************************************************/

#if MB(OMCA_A) //Alpha OMCA board
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega644__
#error Oops!  Make sure you have 'SanguinoA' selected from the 'Tools -> Boards' menu.
#endif

#define ORIG_X_STEP_PIN         21
#define ORIG_X_DIR_PIN          20
#define ORIG_X_ENABLE_PIN       24
#define X_STOP_PIN         0

#define ORIG_Y_STEP_PIN         23
#define ORIG_Y_DIR_PIN          22
#define ORIG_Y_ENABLE_PIN       24
#define Y_STOP_PIN         1

#define ORIG_Z_STEP_PIN         26
#define ORIG_Z_DIR_PIN          25
#define ORIG_Z_ENABLE_PIN       24
#define Z_STOP_PIN         2

#define ORIG_E0_STEP_PIN         28
#define ORIG_E0_DIR_PIN          27
#define ORIG_E0_ENABLE_PIN       24

#define ORIG_E1_STEP_PIN         -1 // 19
#define ORIG_E1_DIR_PIN          -1 // 18
#define ORIG_E1_ENABLE_PIN       24

#define ORIG_E2_STEP_PIN         -1 // 17
#define ORIG_E2_DIR_PIN          -1 // 16
#define ORIG_E2_ENABLE_PIN       24

#define SDPOWER            -1
#define SDSS               11
#define SD_DETECT_PIN      -1 // 10 optional also used as mode pin
#define LED_PIN            -1
#define ORIG_FAN_PIN        3
#define ORIG_PS_ON_PIN     -1
#define KILL_PIN           -1

#define ORIG_HEATER_0_PIN       4
#define ORIG_HEATER_1_PIN       -1 // 12
#define ORIG_HEATER_2_PIN       -1 // 13
#define ORIG_TEMP_0_PIN          0 //D27   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define ORIG_TEMP_1_PIN         -1 // 1
#define ORIG_TEMP_2_PIN         -1 // 2
#define ORIG_HEATER_BED_PIN     -1 // 14/15
#define ORIG_TEMP_BED_PIN       -1 // 1,2 or I2C
/*  Unused (1) (2) (3) 4 5 6 7 8 9 10 11 12 13 (14) (15) (16) 17 (18) (19) (20) (21) (22) (23) 24 (25) (26) (27) 28 (29) (30) (31)  */

#endif // OMCA_A

#if MB(OMCA)  // Final OMCA board -- REF http://sanguino.cc/hardware
#define KNOWN_BOARD 1

#if !defined(__AVR_ATmega644P__) && !defined(__AVR_ATmega644__)
#error Oops!  Make sure you have 'Sanguino' selected from the 'Tools -> Boards' menu. (Final OMCA board)
#endif

#define ORIG_X_STEP_PIN         26
#define ORIG_X_DIR_PIN          25
#define ORIG_X_ENABLE_PIN       10
#define X_STOP_PIN         0

#define ORIG_Y_STEP_PIN         28
#define ORIG_Y_DIR_PIN          27
#define ORIG_Y_ENABLE_PIN       10
#define Y_STOP_PIN         1

#define ORIG_Z_STEP_PIN         23
#define ORIG_Z_DIR_PIN          22
#define ORIG_Z_ENABLE_PIN       10
#define Z_STOP_PIN         2

#define ORIG_E0_STEP_PIN         24
#define ORIG_E0_DIR_PIN          21
#define ORIG_E0_ENABLE_PIN       10

/* future proofing */
#define __FS  20
#define __FD  19
#define __GS  18
#define __GD  13

#define UNUSED_PWM           14 /* PWM on LEFT connector */

#define ORIG_E1_STEP_PIN         -1 // 21
#define ORIG_E1_DIR_PIN          -1 // 20
#define ORIG_E1_ENABLE_PIN       -1 // 19

#define ORIG_E2_STEP_PIN         -1 // 21
#define ORIG_E2_DIR_PIN          -1 // 20
#define ORIG_E2_ENABLE_PIN       -1 // 18

#define SDPOWER            -1
#define SDSS               11
#define SD_DETECT_PIN      -1 // 10 optional also used as mode pin
#define LED_PIN            -1
#define ORIG_FAN_PIN       14 /* PWM on MIDDLE connector */
#define ORIG_PS_ON_PIN     -1
#define KILL_PIN           -1

#define ORIG_HEATER_0_PIN        3 /*DONE PWM on RIGHT connector */
#define ORIG_HEATER_1_PIN       -1
#define ORIG_HEATER_2_PIN       -1
#define ORIG_TEMP_0_PIN          0 // ANALOG INPUT NUMBERING
#define ORIG_TEMP_1_PIN          1 // ANALOG
#define ORIG_TEMP_2_PIN         -1 // 2
#define ORIG_HEATER_BED_PIN      4
#define ORIG_TEMP_BED_PIN        2 // 1,2 or I2C

#define I2C_SCL       16
#define I2C_SDA       17

#endif // OMCA
/****************************************************************************************/



/****************************************************************************************
* 999
* Leapfrog Driver board
****************************************************************************************/
#if MB(LEAPFROG)  // Leapfrog board
#define KNOWN_BOARD 1

#ifndef __AVR_ATmega1280__
 #ifndef __AVR_ATmega2560__
 #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
 #endif
#endif

#define ORIG_X_STEP_PIN         28
#define ORIG_X_DIR_PIN          63
#define ORIG_X_ENABLE_PIN       29
#define ORIG_X_MIN_PIN          47
#define ORIG_X_MAX_PIN          -1   //2 //Max endstops default to disabled "-1", set to commented value to enable.

#define ORIG_Y_STEP_PIN         14 // A6
#define ORIG_Y_DIR_PIN          15 // A0
#define ORIG_Y_ENABLE_PIN       39
#define ORIG_Y_MIN_PIN          48
#define ORIG_Y_MAX_PIN          -1   //15

#define ORIG_Z_STEP_PIN         31 // A2
#define ORIG_Z_DIR_PIN          32 // A6
#define ORIG_Z_ENABLE_PIN       30 // A1
#define ORIG_Z_MIN_PIN          49
#define ORIG_Z_MAX_PIN          -1

#define ORIG_E0_STEP_PIN        34  //34
#define ORIG_E0_DIR_PIN         35 //35
#define ORIG_E0_ENABLE_PIN      33 //33

#define ORIG_E1_STEP_PIN        37 //37
#define ORIG_E1_DIR_PIN         40 //40
#define ORIG_E1_ENABLE_PIN      36 //36

#define Y2_STEP_PIN             37
#define Y2_DIR_PIN              40
#define Y2_ENABLE_PIN           36

#define Z2_STEP_PIN             37
#define Z2_DIR_PIN              40
#define Z2_ENABLE_PIN           36

#define SDPOWER                 -1
#define SDSS                    11
#define SD_DETECT_PIN           -1 // 10 optional also used as mode pin
#define LED_PIN                 13
#define ORIG_FAN_PIN             7
#define ORIG_PS_ON_PIN          -1
#define KILL_PIN                -1
#define SOL1_PIN                16
#define SOL2_PIN                17

#define ORIG_HEATER_0_PIN        9
#define ORIG_HEATER_1_PIN        8 // 12
#define ORIG_HEATER_2_PIN       11 //-1 // 13
#define ORIG_TEMP_0_PIN         13 //D27   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define ORIG_TEMP_1_PIN         15 // 1
#define ORIG_TEMP_2_PIN         -1 // 2
#define ORIG_HEATER_BED_PIN     10 // 14/15
#define ORIG_TEMP_BED_PIN       14 // 1,2 or I2C
/*  Unused (1) (2) (3) 4 5 6 7 8 9 10 11 12 13 (14) (15) (16) 17 (18) (19) (20) (21) (22) (23) 24 (25) (26) (27) 28 (29) (30) (31)  */

#endif // LEAPFROG
/****************************************************************************************/



/****************************************************************************************
* 99
* Custom board
****************************************************************************************/

#if MB(99)
#define KNOWN_BOARD 1

#define ORIG_X_STEP_PIN          2
#define ORIG_X_DIR_PIN           3
#define ORIG_X_ENABLE_PIN       -1
#define X_STOP_PIN         16

#define ORIG_Y_STEP_PIN          5
#define ORIG_Y_DIR_PIN           6
#define ORIG_Y_ENABLE_PIN       -1
#define Y_STOP_PIN         67

#define ORIG_Z_STEP_PIN         62
#define ORIG_Z_DIR_PIN          63
#define ORIG_Z_ENABLE_PIN       -1
#define Z_STOP_PIN         59

#define ORIG_E0_STEP_PIN        65
#define ORIG_E0_DIR_PIN         66
#define ORIG_E0_ENABLE_PIN      -1

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            -1
#define ORIG_FAN_PIN            -1
#define ORIG_PS_ON_PIN           9
#define KILL_PIN           -1

#define ORIG_HEATER_0_PIN       13
#define ORIG_HEATER_1_PIN       -1
#define ORIG_HEATER_2_PIN       -1
#define ORIG_TEMP_0_PIN          6   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define ORIG_TEMP_1_PIN         -1   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define ORIG_TEMP_2_PIN         -1   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define ORIG_HEATER_BED_PIN      4
#define ORIG_TEMP_BED_PIN       10

#endif // 99
/****************************************************************************************/



/****************************************************************************************
******************** Available chip select pins for HW SPI ******************************
*****************************************************************************************/
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define MOSI_PIN            51
  #define MISO_PIN            50
  #define SCK_PIN             52
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284P__)
  #define MOSI_PIN             5
  #define MISO_PIN             6
  #define SCK_PIN              7
#elif defined(__AVR_ATmega32U4__)
  #define MOSI_PIN             2
  #define MISO_PIN             3
  #define SCK_PIN              1
#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
  #define MOSI_PIN            22
  #define MISO_PIN            23
  #define SCK_PIN             21
#elif defined(__AVR_ATmega168__) ||defined(__AVR_ATmega168P__) ||defined(__AVR_ATmega328P__)
  #define MOSI_PIN            11
  #define MISO_PIN            12
  #define SCK_PIN             13
#elif defined(__AVR_ATmega1281__)
  #define MOSI_PIN            11
  #define MISO_PIN            12
  #define SCK_PIN             10
#elif defined (__SAM3X8E__)
  #if (SDSS == 4) || (SDSS == 10) || (SDSS == 52) || (SDSS == 77)
    #if (SDSS == 4)
      #define SPI_PIN         87
      #define SPI_CHAN        1
    #elif (SDSS == 10)
      #define SPI_PIN         77
      #define SPI_CHAN        0
    #elif (SDSS == 52) 
      #define SPI_PIN         86
      #define SPI_CHAN        2
    #else
      #define SPI_PIN         77
      #define SPI_CHAN        0
    #endif
    #define MOSI_PIN          75
    #define MISO_PIN          74
    #define SCK_PIN           76
  #else
    #define DUE_SOFTWARE_SPI
    #define MOSI_PIN		      51
    #define MISO_PIN		      50
    #define SCK_PIN 		      52
  #endif
#endif
/****************************************************************************************/



/****************************************************************************************
********************************* END MOTHERBOARD ***************************************
****************************************************************************************/

#ifndef ORIG_E0_DIR_PIN
  #define ORIG_E0_DIR_PIN     -1
  #define ORIG_E0_ENABLE_PIN  -1
  #define ORIG_E0_STEP_PIN    -1
#endif
#ifndef ORIG_E1_DIR_PIN
  #define ORIG_E1_DIR_PIN     -1
  #define ORIG_E1_ENABLE_PIN  -1
  #define ORIG_E1_STEP_PIN    -1
#endif
#ifndef ORIG_E2_DIR_PIN
  #define ORIG_E2_DIR_PIN     -1
  #define ORIG_E2_ENABLE_PIN  -1
  #define ORIG_E2_STEP_PIN    -1
#endif
#ifndef ORIG_E3_DIR_PIN
  #define ORIG_E3_DIR_PIN     -1
  #define ORIG_E3_ENABLE_PIN  -1
  #define ORIG_E3_STEP_PIN    -1
#endif
#ifndef ORIG_E4_DIR_PIN
  #define ORIG_E4_DIR_PIN     -1
  #define ORIG_E4_ENABLE_PIN  -1
  #define ORIG_E4_STEP_PIN    -1
#endif
#ifndef ORIG_E5_DIR_PIN
  #define ORIG_E5_DIR_PIN     -1
  #define ORIG_E5_ENABLE_PIN  -1
  #define ORIG_E5_STEP_PIN    -1
#endif

#ifndef ORIG_HEATER_1_PIN
  #define ORIG_HEATER_1_PIN   -1
#endif
#ifndef ORIG_TEMP_1_PIN
  #define ORIG_TEMP_1_PIN     -1
#endif
#ifndef ORIG_HEATER_2_PIN
  #define ORIG_HEATER_2_PIN   -1
#endif
#ifndef ORIG_TEMP_2_PIN
  #define ORIG_TEMP_2_PIN     -1
#endif
#ifndef ORIG_HEATER_3_PIN
  #define ORIG_HEATER_3_PIN   -1
#endif
#ifndef ORIG_TEMP_3_PIN
  #define ORIG_TEMP_3_PIN     -1
#endif

#if ENABLED(X_STOP_PIN)
  #if X_HOME_DIR < 0
    #define ORIG_X_MIN_PIN X_STOP_PIN
    #define ORIG_X_MAX_PIN -1
  #else
    #define ORIG_X_MIN_PIN -1
    #define ORIG_X_MAX_PIN X_STOP_PIN
  #endif
#endif

#if ENABLED(Y_STOP_PIN)
  #if Y_HOME_DIR < 0
    #define ORIG_Y_MIN_PIN Y_STOP_PIN
    #define ORIG_Y_MAX_PIN -1
  #else
    #define ORIG_Y_MIN_PIN -1
    #define ORIG_Y_MAX_PIN Y_STOP_PIN
  #endif
#endif

#if ENABLED(Z_STOP_PIN)
  #if Z_HOME_DIR < 0
    #define ORIG_Z_MIN_PIN Z_STOP_PIN
    #define ORIG_Z_MAX_PIN -1
  #else
    #define ORIG_Z_MIN_PIN -1
    #define ORIG_Z_MAX_PIN Z_STOP_PIN
  #endif
#endif

#ifndef X_MS1_PIN
  #define X_MS1_PIN     -1
#endif
#ifndef X_MS2_PIN
  #define X_MS2_PIN     -1
#endif
#ifndef Y_MS1_PIN
  #define Y_MS1_PIN     -1
#endif
#ifndef Y_MS2_PIN
  #define Y_MS2_PIN     -1
#endif
#ifndef Z_MS1_PIN
  #define Z_MS1_PIN     -1
#endif
#ifndef Z_MS2_PIN
  #define Z_MS2_PIN     -1
#endif
#ifndef E0_MS1_PIN
  #define E0_MS1_PIN    -1
#endif
#ifndef E0_MS2_PIN
  #define E0_MS2_PIN    -1
#endif
#ifndef E1_MS1_PIN
  #define E1_MS1_PIN    -1
#endif
#ifndef E1_MS2_PIN
  #define E1_MS2_PIN    -1
#endif
#ifndef DIGIPOTSS_PIN
  #define DIGIPOTSS_PIN -1
#endif
#ifndef LCD_CONTRAST
  #define LCD_CONTRAST  -1
#endif
#ifndef Z2_MIN_PIN
  #define Z2_MIN_PIN    -1
#endif
#ifndef Z2_MAX_PIN
  #define Z2_MAX_PIN    -1
#endif

#ifndef ORIG_FAN_PIN
  #define ORIG_FAN_PIN  -1
#endif
#ifndef ORIG_FAN2_PIN
  #define ORIG_FAN2_PIN -1
#endif

#ifndef ORIG_BEEPER_PIN
  #define ORIG_BEEPER_PIN -1
#endif

/****************************************************************************************/
#include "Configuration_Pins.h"
/****************************************************************************************/


#if X_HOME_DIR > 0    // Home X to MAX
  #undef X_MIN_PIN
  #define X_MIN_PIN -1
#elif X_HOME_DIR < 0  // Home X to MIN
  #undef X_MAX_PIN
  #define X_MAX_PIN -1
#endif //X_HOME_DIR > 0

#if Y_HOME_DIR > 0    // Home Y to MAX
  #undef Y_MIN_PIN
  #define Y_MIN_PIN -1
#elif Y_HOME_DIR < 0  // Home Y to MIN
  #undef Y_MAX_PIN
  #define Y_MAX_PIN -1
#endif //Y_HOME_DIR > 0

#if Z_HOME_DIR > 0    // Home Z to MAX
  #undef Z_MIN_PIN
  #define Z_MIN_PIN -1
#elif Z_HOME_DIR < 0  // Home Z to MIN
  #undef Z_MAX_PIN
  #define Z_MAX_PIN -1
#endif //Z_HOME_DIR > 0

#if DISABLED(Z_PROBE_ENDSTOP) // Allow code to compile regardless of Z_PROBE_ENDSTOP setting.
  #define Z_PROBE_PIN -1
#endif
/****************************************************************************************/

#if ENABLED(DISABLE_XMAX_ENDSTOP)
  #undef X_MAX_PIN
  #define X_MAX_PIN -1
#endif

#if ENABLED(DISABLE_XMIN_ENDSTOP)
  #undef X_MIN_PIN 
  #define X_MIN_PIN -1
#endif

#if ENABLED(DISABLE_YMAX_ENDSTOP)
  #undef Y_MAX_PIN
  #define Y_MAX_PIN -1
#endif

#if ENABLED(DISABLE_YMIN_ENDSTOP)
  #undef Y_MIN_PIN
  #define Y_MIN_PIN -1
#endif

#if ENABLED(DISABLE_ZMAX_ENDSTOP)
  #undef Z_MAX_PIN
  #define Z_MAX_PIN -1
#endif

#if ENABLED(DISABLE_ZMIN_ENDSTOP)
  #undef Z_MIN_PIN 
  #define Z_MIN_PIN -1
#endif
/****************************************************************************************/

// List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, HEATER_0_PIN, analogInputToDigitalPin(TEMP_0_PIN),

#if DRIVER_EXTRUDERS > 1
  #define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, HEATER_1_PIN, analogInputToDigitalPin(TEMP_1_PIN),
#else
  #define _E1_PINS
#endif
#if DRIVER_EXTRUDERS  > 2
  #define _E2_PINS E2_STEP_PIN, E2_DIR_PIN, E2_ENABLE_PIN, HEATER_2_PIN, analogInputToDigitalPin(TEMP_2_PIN),
#else
  #define _E2_PINS
#endif
#if DRIVER_EXTRUDERS > 3
  #define _E3_PINS E3_STEP_PIN, E3_DIR_PIN, E3_ENABLE_PIN, HEATER_3_PIN, analogInputToDigitalPin(TEMP_3_PIN),
#else
  #define _E3_PINS
#endif

#define SENSITIVE_PINS { 0, 1, \
                        X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, \
                        Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, \
                        Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, Z_PROBE_PIN, \
                        PS_ON_PIN, HEATER_BED_PIN, FAN_PIN, \
                        _E0_PINS _E1_PINS _E2_PINS _E3_PINS \
                        analogInputToDigitalPin(TEMP_BED_PIN) \
                       }

#endif //__PINS_H
