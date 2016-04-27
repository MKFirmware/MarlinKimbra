/**
 * MK & MK4due 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Conditionals.h
 * Defines that depend on configuration but are not editable.
 */

#ifndef CONDITIONALS_H
  #define CONDITIONALS_H

  #if ENABLED(MAKRPANEL)
    #define DOGLCD
    #define DEFAULT_LCD_CONTRAST 17
    #define ULTIPANEL
    #define NEWPANEL
  #endif

  #if ENABLED(miniVIKI) || ENABLED(VIKI2) || ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
    #define ULTRA_LCD  //general LCD support, also 16x2
    #define DOGLCD  // Support for SPI LCD 128x64 (Controller ST7565R graphic Display Family)
    #define ULTIMAKERCONTROLLER //as available from the Ultimaker online store.

    #if ENABLED(miniVIKI)
      #define DEFAULT_LCD_CONTRAST 95
    #elif ENABLED(VIKI2)
      #define DEFAULT_LCD_CONTRAST 40
    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
      #define DEFAULT_LCD_CONTRAST 110
      #define U8GLIB_LM6059_AF
    #endif

    #define ENCODER_PULSES_PER_STEP 4
    #define ENCODER_STEPS_PER_MENU_ITEM 1
  #endif

  // Generic support for SSD1306 OLED based LCDs.
  #if ENABLED(U8GLIB_SSD1306)
    #define ULTRA_LCD  //general LCD support, also 16x2
    #define DOGLCD  // Support for I2C LCD 128x64 (Controller SSD1306 graphic Display Family)
  #endif

  #if ENABLED(PANEL_ONE)
    #define ULTIMAKERCONTROLLER
  #endif

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define DOGLCD
    #define U8GLIB_ST7920
    #define REPRAP_DISCOUNT_SMART_CONTROLLER
  #endif

  #if ENABLED(SPARK_FULL_GRAPHICS)
    #define ENCODER_PULSES_PER_STEP 2
    #define ENCODER_STEPS_PER_MENU_ITEM 1

    #define DOGLCD
    #define U8GLIB_ST7920
    #define REPRAP_DISCOUNT_SMART_CONTROLLER
  #endif

  #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
    #if DISABLED(SDSUPPORT)
      #define SDSUPPORT
    #endif
  #endif

  #if ENABLED(ULTIMAKERCONTROLLER) || ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) || ENABLED(G3D_PANEL) || ENABLED(RIGIDBOT_PANEL)
    #define ULTIPANEL
    #define NEWPANEL
  #endif

  #if ENABLED(RADDS_DISPLAY)
    #define ENCODER_PULSES_PER_STEP 2
    #define ENCODER_STEPS_PER_MENU_ITEM 1

    #define ULTIPANEL
    #define NEWPANEL
  #endif

  #if ENABLED(REPRAPWORLD_KEYPAD)
    #define ULTIPANEL
    #define NEWPANEL
  #endif

  #if ENABLED(RA_CONTROL_PANEL)
    #define LCD_I2C_TYPE_PCA8574
    #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
    #define ULTIPANEL
    #define NEWPANEL
  #endif

  #if ENABLED(MINIPANEL)
   #define DOGLCD
   #define ULTIPANEL
   #define NEWPANEL
   #define DEFAULT_LCD_CONTRAST 17
  #endif

  /**
   * I2C PANELS
   */
  #if ENABLED(LCD_I2C_SAINSMART_YWROBOT)
    // This uses the LiquidCrystal_I2C library ( https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home )
    // Make sure it is placed in the Arduino libraries directory.
    #define LCD_I2C_TYPE_PCF8575
    #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
    #define ULTIPANEL
    #define NEWPANEL
  #endif

  // PANELOLU2 LCD with status LEDs, separate encoder and click inputs
  #if ENABLED(LCD_I2C_PANELOLU2)
    #define LCD_I2C_TYPE_MCP23017
    #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
    #define LCD_USE_I2C_BUZZER //comment out to disable buzzer on LCD

    #if DISABLED(ENCODER_PULSES_PER_STEP)
      #define ENCODER_PULSES_PER_STEP 4
    #endif

    #if DISABLED(ENCODER_STEPS_PER_MENU_ITEM)
      #define ENCODER_STEPS_PER_MENU_ITEM 1
    #endif

    #if ENABLED(LCD_USE_I2C_BUZZER)
      #define LCD_FEEDBACK_FREQUENCY_HZ 1000
      #define LCD_FEEDBACK_FREQUENCY_DURATION_MS 100
    #endif

    #define ULTIPANEL
    #define NEWPANEL
  #endif

  // Panucatt VIKI LCD with status LEDs, integrated click & L/R/U/P buttons, separate encoder inputs
  #if ENABLED(LCD_I2C_VIKI)
    // This uses the LiquidTWI2 library v1.2.3 or later ( https://github.com/lincomatic/LiquidTWI2 )
    // Make sure the LiquidTWI2 directory is placed in the Arduino or Sketchbook libraries subdirectory.
    // Note: The pause/stop/resume LCD button pin should be connected to the Arduino
    //       BTN_ENC pin (or set BTN_ENC to -1 if not used)
    #define LCD_I2C_TYPE_MCP23017
    #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
    #define LCD_USE_I2C_BUZZER //comment out to disable buzzer on LCD (requires LiquidTWI2 v1.2.3 or later)
    #define ULTIPANEL
    #define NEWPANEL
  #endif

  // Shift register panels
  // ---------------------
  // 2 wire Non-latching LCD SR from:
  // https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/schematics#!shiftregister-connection

  #if ENABLED(SAV_3DLCD)
     #define SR_LCD_2W_NL    // Non latching 2 wire shiftregister
     #define ULTIPANEL
     #define NEWPANEL
  #endif

  #if ENABLED(DOGLCD) // Change number of lines to match the DOG graphic display
    #if DISABLED(LCD_WIDTH)
      #define LCD_WIDTH 22
    #endif
    #if DISABLED(LCD_HEIGHT)
      #define LCD_HEIGHT 5
    #endif
  #endif

  #if ENABLED(ULTIPANEL)
    #define NEWPANEL  //enable this if you have a click-encoder panel
    #define ULTRA_LCD
    #if DISABLED(LCD_WIDTH)
      #define LCD_WIDTH 20
    #endif
    #if DISABLED(LCD_HEIGHT)
      #define LCD_HEIGHT 4
    #endif
  #else //no panel but just LCD
    #if ENABLED(ULTRA_LCD)
      #if DISABLED(LCD_WIDTH)
        #define LCD_WIDTH 16
      #endif
      #if DISABLED(LCD_HEIGHT)
        #define LCD_HEIGHT 2
      #endif
    #endif
  #endif

  #if ENABLED(DOGLCD)
    /* Custom characters defined in font font_6x10_marlin_symbols */
    // \x00 intentionally skipped to avoid problems in strings
    #define LCD_STR_REFRESH     "\x01"
    #define LCD_STR_FOLDER      "\x02"
    #define LCD_STR_ARROW_RIGHT "\x03"
    #define LCD_STR_UPLEVEL     "\x04"
    #define LCD_STR_CLOCK       "\x05"
    #define LCD_STR_FEEDRATE    "\x06"
    #define LCD_STR_BEDTEMP     "\x07"
    #define LCD_STR_THERMOMETER "\x08"
    #define LCD_STR_DEGREE      "\x09"

    #define LCD_STR_SPECIAL_MAX '\x09'
    // Maximum here is 0x1f because 0x20 is ' ' (space) and the normal charsets begin.
    // Better stay below 0x10 because DISPLAY_CHARSET_HD44780_WESTERN begins here.
  #else
    /* Custom characters defined in the first 8 characters of the LCD */
    #define LCD_STR_BEDTEMP     "\x00"  // this will have 'unexpected' results when used in a string!
    #define LCD_STR_DEGREE      "\x01"
    #define LCD_STR_THERMOMETER "\x02"
    #define LCD_STR_UPLEVEL     "\x03"
    #define LCD_STR_REFRESH     "\x04"
    #define LCD_STR_FOLDER      "\x05"
    #define LCD_STR_FEEDRATE    "\x06"
    #define LCD_STR_CLOCK       "\x07"
    #define LCD_STR_ARROW_RIGHT ">"  /* from the default character set */
  #endif

  /**
   * Default LCD contrast for dogm-like LCD displays
   */
  #if ENABLED(DOGLCD) && DISABLED(DEFAULT_LCD_CONTRAST)
    #define DEFAULT_LCD_CONTRAST 32
  #endif

  #if ENABLED(DOGLCD)
    #define HAS_LCD_CONTRAST
    #if ENABLED(U8GLIB_ST7920)
      #undef HAS_LCD_CONTRAST
    #endif
    #if ENABLED(U8GLIB_SSD1306)
      #undef HAS_LCD_CONTRAST
    #endif
  #endif

  #include "pins.h"

  /**
   * SAM3X8E
   */
  #ifdef __SAM3X8E__
    #ifdef FAST_PWM_FAN
      #undef FAST_PWM_FAN
    #endif
    #ifdef M100_FREE_MEMORY_WATCHER
      #undef M100_FREE_MEMORY_WATCHER
    #endif
  #endif
      
  /**
   * DONDOLO
   */
  #if ENABLED(DONDOLO)
    #undef SINGLENOZZLE
    #undef ADVANCE
    #undef DRIVER_EXTRUDERS
    #define DRIVER_EXTRUDERS 1
  #endif

  /**
   * SINGLENOZZLE
   */
  #if ENABLED(SINGLENOZZLE)
    #undef HOTENDS
    #define HOTENDS 1
    #undef TEMP_SENSOR_1_AS_REDUNDANT
  #else
    #undef HOTENDS
    #define HOTENDS EXTRUDERS
  #endif

  /**
   * DRIVER_EXTRUDERS
   */
  #if DISABLED(MKR4) && DISABLED(NPR2) && DISABLED(DONDOLO) && DISABLED(COLOR_MIXING_EXTRUDER)
    #undef DRIVER_EXTRUDERS
    #define DRIVER_EXTRUDERS EXTRUDERS // This defines the number of Driver extruder
  #endif

  #ifndef __SAM3X8E__
    #ifndef USBCON
      #define HardwareSerial_h // trick to disable the standard HWserial
    #endif
  #endif

  /**
   * ENDSTOPPULLUPS
   */
  #if ENABLED(ENDSTOPPULLUPS)
    #define ENDSTOPPULLUP_XMIN
    #define ENDSTOPPULLUP_YMIN
    #define ENDSTOPPULLUP_ZMIN
    #define ENDSTOPPULLUP_Z2MIN
    #define ENDSTOPPULLUP_XMAX
    #define ENDSTOPPULLUP_YMAX
    #define ENDSTOPPULLUP_ZMAX
    #define ENDSTOPPULLUP_Z2MAX
    #define ENDSTOPPULLUP_ZPROBE
    #define ENDSTOPPULLUP_EMIN
  #endif

  /**
   * ENDSTOP LOGICAL
   */
  #if MB(ALLIGATOR)
    #define X_MIN_ENDSTOP_INVERTING   !X_MIN_ENDSTOP_LOGIC
    #define Y_MIN_ENDSTOP_INVERTING   !Y_MIN_ENDSTOP_LOGIC
    #define Z_MIN_ENDSTOP_INVERTING   !Z_MIN_ENDSTOP_LOGIC
    #define Z2_MIN_ENDSTOP_INVERTING  !Z2_MIN_ENDSTOP_LOGIC
    #define E_MIN_ENDSTOP_INVERTING   !E_MIN_ENDSTOP_LOGIC
    #define X_MAX_ENDSTOP_INVERTING   !X_MAX_ENDSTOP_LOGIC
    #define Y_MAX_ENDSTOP_INVERTING   !Y_MAX_ENDSTOP_LOGIC
    #define Z_MAX_ENDSTOP_INVERTING   !Z_MAX_ENDSTOP_LOGIC
    #define Z2_MAX_ENDSTOP_INVERTING  !Z2_MAX_ENDSTOP_LOGIC
    #define Z_PROBE_ENDSTOP_INVERTING !Z_PROBE_ENDSTOP_LOGIC
    #undef  ENDSTOPPULLUP_XMIN
    #undef  ENDSTOPPULLUP_YMIN
    #undef  ENDSTOPPULLUP_ZMIN
    #undef  ENDSTOPPULLUP_Z2MIN
    #undef  ENDSTOPPULLUP_XMAX
    #undef  ENDSTOPPULLUP_YMAX
    #undef  ENDSTOPPULLUP_ZMAX
    #undef  ENDSTOPPULLUP_Z2MAX
    #undef  ENDSTOPPULLUP_ZPROBE
    #undef  ENDSTOPPULLUP_EMIN
  #else
    #define X_MIN_ENDSTOP_INVERTING   X_MIN_ENDSTOP_LOGIC
    #define Y_MIN_ENDSTOP_INVERTING   Y_MIN_ENDSTOP_LOGIC
    #define Z_MIN_ENDSTOP_INVERTING   Z_MIN_ENDSTOP_LOGIC
    #define Z2_MIN_ENDSTOP_INVERTING  Z2_MIN_ENDSTOP_LOGIC
    #define E_MIN_ENDSTOP_INVERTING   E_MIN_ENDSTOP_LOGIC
    #define X_MAX_ENDSTOP_INVERTING   X_MAX_ENDSTOP_LOGIC
    #define Y_MAX_ENDSTOP_INVERTING   Y_MAX_ENDSTOP_LOGIC
    #define Z_MAX_ENDSTOP_INVERTING   Z_MAX_ENDSTOP_LOGIC
    #define Z2_MAX_ENDSTOP_INVERTING  Z2_MAX_ENDSTOP_LOGIC
    #define Z_PROBE_ENDSTOP_INVERTING Z_PROBE_ENDSTOP_LOGIC
  #endif

  /**
   * Firmware Test
   */
  #if ENABLED(FIRMWARE_TEST)
    #undef BAUDRATE
    #define BAUDRATE 115200  // Baudrate setting to 115200 because serial monitor arduino function at max 115200 baudrate.
  #endif

  /**
   * Axis lengths
   */
  #define X_MAX_LENGTH (X_MAX_POS - (X_MIN_POS))
  #define Y_MAX_LENGTH (Y_MAX_POS - (Y_MIN_POS))
  #define Z_MAX_LENGTH (Z_MAX_POS - (Z_MIN_POS))

  /**
   * SCARA
   */
  #if MECH(SCARA)
    #undef SLOWDOWN
    #define QUICK_HOME //SCARA needs Quickhome
  #endif

  /**
   * DELTA
   */
  #if MECH(DELTA)
    #undef SLOWDOWN //DELTA not needs SLOWDOWN
    #define AUTOLEVEL_GRID_MULTI 1/AUTOLEVEL_GRID
    // DELTA must have same valour for 3 axis endstop hits
    #define X_HOME_BUMP_MM XYZ_HOME_BUMP_MM
    #define Y_HOME_BUMP_MM XYZ_HOME_BUMP_MM
    #define Z_HOME_BUMP_MM XYZ_HOME_BUMP_MM
    #define HOMING_BUMP_DIVISOR {XYZ_BUMP_DIVISOR, XYZ_BUMP_DIVISOR, XYZ_BUMP_DIVISOR}

    // Effective horizontal distance bridged by diagonal push rods.
    #define DEFAULT_DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET - DELTA_EFFECTOR_OFFSET - DELTA_CARRIAGE_OFFSET)
    #define LEFT_PROBE_BED_POSITION -DELTA_PROBABLE_RADIUS
    #define RIGHT_PROBE_BED_POSITION DELTA_PROBABLE_RADIUS
    #define FRONT_PROBE_BED_POSITION -DELTA_PROBABLE_RADIUS
    #define BACK_PROBE_BED_POSITION DELTA_PROBABLE_RADIUS
    
    // Radius for probe
    #define DELTA_PROBABLE_RADIUS DELTA_PRINTABLE_RADIUS - 5
  #endif
    
  /**
   * AUTOSET LOCATIONS OF LIMIT SWITCHES
   */
  #if ENABLED(MANUAL_HOME_POSITIONS)  // Use manual limit switch locations
    #define X_HOME_POS MANUAL_X_HOME_POS
    #define Y_HOME_POS MANUAL_Y_HOME_POS
    #define Z_HOME_POS MANUAL_Z_HOME_POS
  #else //!MANUAL_HOME_POSITIONS – Use home switch positions based on homing direction and travel limits
    #if ENABLED(BED_CENTER_AT_0_0)
      #define X_HOME_POS (X_MAX_LENGTH) * (X_HOME_DIR) * 0.5
      #define Y_HOME_POS (Y_MAX_LENGTH) * (Y_HOME_DIR) * 0.5
    #else
      #define X_HOME_POS (X_HOME_DIR < 0 ? X_MIN_POS : X_MAX_POS)
      #define Y_HOME_POS (Y_HOME_DIR < 0 ? Y_MIN_POS : Y_MAX_POS)
    #endif
    #define Z_HOME_POS (Z_HOME_DIR < 0 ? Z_MIN_POS : Z_MAX_POS)
  #endif //!MANUAL_HOME_POSITIONS

  /**
   * Auto Bed Leveling
   */
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    // Boundaries for probing based on set limits
    #define MIN_PROBE_X (max(X_MIN_POS, X_MIN_POS + X_PROBE_OFFSET_FROM_EXTRUDER))
    #define MAX_PROBE_X (min(X_MAX_POS, X_MAX_POS + X_PROBE_OFFSET_FROM_EXTRUDER))
    #define MIN_PROBE_Y (max(Y_MIN_POS, Y_MIN_POS + Y_PROBE_OFFSET_FROM_EXTRUDER))
    #define MAX_PROBE_Y (min(Y_MAX_POS, Y_MAX_POS + Y_PROBE_OFFSET_FROM_EXTRUDER))

    // Make sure probing points are reachable
    #if LEFT_PROBE_BED_POSITION < MIN_PROBE_X
      #undef LEFT_PROBE_BED_POSITION
      #define LEFT_PROBE_BED_POSITION   MIN_PROBE_X
    #endif
    #if RIGHT_PROBE_BED_POSITION > MAX_PROBE_X
      #undef RIGHT_PROBE_BED_POSITION
      #define RIGHT_PROBE_BED_POSITION  MAX_PROBE_X
    #endif
    #if FRONT_PROBE_BED_POSITION < MIN_PROBE_Y
      #undef FRONT_PROBE_BED_POSITION
      #define FRONT_PROBE_BED_POSITION  MIN_PROBE_Y
    #endif
    #if BACK_PROBE_BED_POSITION > MAX_PROBE_Y
      #undef BACK_PROBE_BED_POSITION
      #define BACK_PROBE_BED_POSITION   MAX_PROBE_Y
    #endif

    // Z_RAISE_AFTER_PROBING is not for all probes. Be sure that it is zero in that cases
    #if DISABLED(ENABLE_SERVOS) && DISABLED(Z_PROBE_SLED)
      #undef Z_RAISE_AFTER_PROBING
      #define Z_RAISE_AFTER_PROBING 0
    #endif
  #endif

  /**
   * Sled Options
   */
  #if ENABLED(Z_PROBE_SLED)
    #define Z_SAFE_HOMING
  #endif

  /**
   * Avoid double-negatives for enabling features
   */
  #if DISABLED(DISABLE_HOST_KEEPALIVE)
    #define HOST_KEEPALIVE_FEATURE
    #define DEFAULT_KEEPALIVE_INTERVAL 2
  #endif

  /**
   * MAX_STEP_FREQUENCY differs for TOSHIBA OR ARDUINO DUE OR ARDUINO MEGA
   */
  #ifdef __SAM3X8E__
    #if ENABLED(CONFIG_STEPPERS_TOSHIBA)
      #define MAX_STEP_FREQUENCY 150000 // Max step frequency for Toshiba Stepper Controllers
      #define DOUBLE_STEP_FREQUENCY MAX_STEP_FREQUENCY
    #else
      #define MAX_STEP_FREQUENCY 320000     // Max step frequency for the Due is approx. 330kHz
      #define DOUBLE_STEP_FREQUENCY 90000  // 96kHz is close to maximum for an Arduino Due
    #endif
  #else
    #if ENABLED(CONFIG_STEPPERS_TOSHIBA)
      #define MAX_STEP_FREQUENCY 10000 // Max step frequency for Toshiba Stepper Controllers
      #define DOUBLE_STEP_FREQUENCY MAX_STEP_FREQUENCY
    #else
      #define MAX_STEP_FREQUENCY 40000 // Max step frequency for Arduino mega
      #define DOUBLE_STEP_FREQUENCY 10000
    #endif
  #endif

  // MS1 MS2 Stepper Driver Microstepping mode table
  #define MICROSTEP1 LOW,LOW
  #define MICROSTEP2 HIGH,LOW
  #define MICROSTEP4 LOW,HIGH
  #define MICROSTEP8 HIGH,HIGH
  #if MB(ALLIGATOR)
    #define MICROSTEP16 LOW,LOW
    #define MICROSTEP32 HIGH,HIGH
  #else
    #define MICROSTEP16 HIGH,HIGH
  #endif

  /**
   * Advance calculated values
   */
  #if ENABLED(ADVANCE)
    #define EXTRUSION_AREA (0.25 * (D_FILAMENT) * (D_FILAMENT) * M_PI)
    #define STEPS_PER_CUBIC_MM_E (axis_steps_per_unit[E_AXIS + active_extruder] / EXTRUSION_AREA)
  #endif

  /**
   * SD DETECT
   *
   */
  #if ENABLED(SD_DISABLED_DETECT)
    #undef SD_DETECT_PIN
    #define SD_DETECT_PIN   -1
  #endif
  #if ENABLED(ULTIPANEL) && DISABLED(ELB_FULL_GRAPHIC_CONTROLLER)
    #undef SD_DETECT_INVERTED
  #endif

  /**
   * Power Signal Control Definitions
   * By default use Normal definition
   */
  #if DISABLED(POWER_SUPPLY)
    #define POWER_SUPPLY 0
  #endif
  #if (POWER_SUPPLY == 1)     // 1 = ATX
    #define PS_ON_AWAKE  LOW
    #define PS_ON_ASLEEP HIGH
  #elif (POWER_SUPPLY == 2)   // 2 = X-Box 360 203W
    #define PS_ON_AWAKE  HIGH
    #define PS_ON_ASLEEP LOW
  #endif
  #define HAS_POWER_SWITCH (POWER_SUPPLY > 0 && PIN_EXISTS(PS_ON))

  /**
   * Temp Sensor defines
   */
  #if TEMP_SENSOR_0 == -2
    #define HEATER_0_USES_MAX6675
  #elif TEMP_SENSOR_0 == -1
    #define HEATER_0_USES_AD595
  #elif TEMP_SENSOR_0 == 0
    #undef HEATER_0_MINTEMP
    #undef HEATER_0_MAXTEMP
  #elif TEMP_SENSOR_0 > 0
    #define THERMISTORHEATER_0 TEMP_SENSOR_0
    #define HEATER_0_USES_THERMISTOR
  #endif

  #if TEMP_SENSOR_1 == -1
    #define HEATER_1_USES_AD595
  #elif TEMP_SENSOR_1 == 0
    #undef HEATER_1_MINTEMP
    #undef HEATER_1_MAXTEMP
  #elif TEMP_SENSOR_1 > 0
    #define THERMISTORHEATER_1 TEMP_SENSOR_1
    #define HEATER_1_USES_THERMISTOR
  #endif

  #if TEMP_SENSOR_2 == -1
    #define HEATER_2_USES_AD595
  #elif TEMP_SENSOR_2 == 0
    #undef HEATER_2_MINTEMP
    #undef HEATER_2_MAXTEMP
  #elif TEMP_SENSOR_2 > 0
    #define THERMISTORHEATER_2 TEMP_SENSOR_2
    #define HEATER_2_USES_THERMISTOR
  #endif

  #if TEMP_SENSOR_3 == -1
    #define HEATER_3_USES_AD595
  #elif TEMP_SENSOR_3 == 0
    #undef HEATER_3_MINTEMP
    #undef HEATER_3_MAXTEMP
  #elif TEMP_SENSOR_3 > 0
    #define THERMISTORHEATER_3 TEMP_SENSOR_3
    #define HEATER_3_USES_THERMISTOR
  #endif

  #if TEMP_SENSOR_BED == -1
    #define BED_USES_AD595
  #elif TEMP_SENSOR_BED == 0
    #undef BED_MINTEMP
    #undef BED_MAXTEMP
  #elif TEMP_SENSOR_BED > 0
    #define THERMISTORBED TEMP_SENSOR_BED
    #define BED_USES_THERMISTOR
  #endif

  #define HEATER_USES_AD595 (ENABLED(HEATER_0_USES_AD595) || ENABLED(HEATER_1_USES_AD595) || ENABLED(HEATER_2_USES_AD595) || ENABLED(HEATER_3_USES_AD595))

  /**
   * ARRAY_BY_EXTRUDERS based on EXTRUDERS
   */
  #if EXTRUDERS > 9
    #define ARRAY_BY_EXTRUDERS(v1) { v1, v1, v1, v1, v1, v1, v1, v1, v1, v1 }
  #elif EXTRUDERS > 8
    #define ARRAY_BY_EXTRUDERS(v1) { v1, v1, v1, v1, v1, v1, v1, v1, v1 }
  #elif EXTRUDERS > 7
    #define ARRAY_BY_EXTRUDERS(v1) { v1, v1, v1, v1, v1, v1, v1, v1 }
  #elif EXTRUDERS > 6
    #define ARRAY_BY_EXTRUDERS(v1) { v1, v1, v1, v1, v1, v1, v1 }
  #elif EXTRUDERS > 5
    #define ARRAY_BY_EXTRUDERS(v1) { v1, v1, v1, v1, v1, v1 }
  #elif EXTRUDERS > 4
    #define ARRAY_BY_EXTRUDERS(v1) { v1, v1, v1, v1, v1 }
  #elif EXTRUDERS > 3
    #define ARRAY_BY_EXTRUDERS(v1) { v1, v1, v1, v1 }
  #elif EXTRUDERS > 2
    #define ARRAY_BY_EXTRUDERS(v1) { v1, v1, v1 }
  #elif EXTRUDERS > 1
    #define ARRAY_BY_EXTRUDERS(v1) { v1, v1 }
  #else
    #define ARRAY_BY_EXTRUDERS(v1) { v1 }
  #endif

  /**
   * ARRAY_BY_HOTENDS based on HOTENDS
   */
  #if HOTENDS > 3
    #define ARRAY_BY_HOTENDS(v1, v2, v3, v4) { v1, v2, v3, v4 }
  #elif HOTENDS > 2
    #define ARRAY_BY_HOTENDS(v1, v2, v3, v4) { v1, v2, v3 }
  #elif HOTENDS > 1
    #define ARRAY_BY_HOTENDS(v1, v2, v3, v4) { v1, v2 }
  #else
    #define ARRAY_BY_HOTENDS(v1, v2, v3, v4) { v1 }
  #endif

  #define ARRAY_BY_HOTENDS1(v1) ARRAY_BY_HOTENDS(v1, v1, v1, v1)

  /**
   * Shorthand for pin tests, used wherever needed
   */
  #define HAS_TEMP_0 (PIN_EXISTS(TEMP_0) && TEMP_SENSOR_0 != 0 && TEMP_SENSOR_0 != -2)
  #define HAS_TEMP_1 (PIN_EXISTS(TEMP_1) && TEMP_SENSOR_1 != 0)
  #define HAS_TEMP_2 (PIN_EXISTS(TEMP_2) && TEMP_SENSOR_2 != 0)
  #define HAS_TEMP_3 (PIN_EXISTS(TEMP_3) && TEMP_SENSOR_3 != 0)
  #define HAS_TEMP_BED (PIN_EXISTS(TEMP_BED) && TEMP_SENSOR_BED != 0)
  #define HAS_HEATER_0 (PIN_EXISTS(HEATER_0))
  #define HAS_HEATER_1 (PIN_EXISTS(HEATER_1))
  #define HAS_HEATER_2 (PIN_EXISTS(HEATER_2))
  #define HAS_HEATER_3 (PIN_EXISTS(HEATER_3))
  #define HAS_HEATER_BED (PIN_EXISTS(HEATER_BED))
  #define HAS_AUTO_FAN_0 (ENABLED(EXTRUDER_AUTO_FAN) && PIN_EXISTS(EXTRUDER_0_AUTO_FAN))
  #define HAS_AUTO_FAN_1 (ENABLED(EXTRUDER_AUTO_FAN) && PIN_EXISTS(EXTRUDER_1_AUTO_FAN))
  #define HAS_AUTO_FAN_2 (ENABLED(EXTRUDER_AUTO_FAN) && PIN_EXISTS(EXTRUDER_2_AUTO_FAN))
  #define HAS_AUTO_FAN_3 (ENABLED(EXTRUDER_AUTO_FAN) && PIN_EXISTS(EXTRUDER_3_AUTO_FAN))
  #define HAS_AUTO_FAN (HAS_AUTO_FAN_0 || HAS_AUTO_FAN_1 || HAS_AUTO_FAN_2 || HAS_AUTO_FAN_3)
  #define HAS_FAN (PIN_EXISTS(FAN))
  #define HAS_CONTROLLERFAN (ENABLED(CONTROLLERFAN) && PIN_EXISTS(CONTROLLERFAN))
  #define HAS_SERVO_0 (PIN_EXISTS(SERVO0))
  #define HAS_SERVO_1 (PIN_EXISTS(SERVO1))
  #define HAS_SERVO_2 (PIN_EXISTS(SERVO2))
  #define HAS_SERVO_3 (PIN_EXISTS(SERVO3))
  #define HAS_SERVOS ((ENABLED(ENABLE_SERVOS) && NUM_SERVOS > 0) && (HAS_SERVO_0 || HAS_SERVO_1 || HAS_SERVO_2 || HAS_SERVO_3))
  #define HAS_FILAMENT_SENSOR (ENABLED(FILAMENT_SENSOR) && PIN_EXISTS(FILWIDTH))
  #define HAS_POWER_CONSUMPTION_SENSOR (ENABLED(POWER_CONSUMPTION) && PIN_EXISTS(POWER_CONSUMPTION))
  #define HAS_Z_PROBE_SLED (ENABLED(Z_PROBE_SLED) && PIN_EXISTS(SLED_PIN))
  #define HAS_FILRUNOUT (ENABLED(FILAMENT_RUNOUT_SENSOR) && PIN_EXISTS(FILRUNOUT))
  #define HAS_HOME (PIN_EXISTS(HOME))
  #define HAS_KILL (PIN_EXISTS(KILL))
  #define HAS_SUICIDE (PIN_EXISTS(SUICIDE))
  #define HAS_CHDK (ENABLED(CHDK) && PIN_EXISTS(CHDK))
  #define HAS_PHOTOGRAPH (ENABLED(PHOTOGRAPH) && PIN_EXISTS(PHOTOGRAPH))
  #define HAS_X_MIN (PIN_EXISTS(X_MIN))
  #define HAS_X_MAX (PIN_EXISTS(X_MAX))
  #define HAS_Y_MIN (PIN_EXISTS(Y_MIN))
  #define HAS_Y_MAX (PIN_EXISTS(Y_MAX))
  #define HAS_Z_MIN (PIN_EXISTS(Z_MIN))
  #define HAS_Z_MAX (PIN_EXISTS(Z_MAX))
  #define HAS_Z2_MIN (PIN_EXISTS(Z2_MIN))
  #define HAS_Z2_MAX (PIN_EXISTS(Z2_MAX))
  #define HAS_Z_PROBE (ENABLED(Z_PROBE_ENDSTOP) && PIN_EXISTS(Z_PROBE))
  #define HAS_E_MIN (PIN_EXISTS(E_MIN))
  #define HAS_SOLENOID_1 (PIN_EXISTS(SOL1))
  #define HAS_SOLENOID_2 (PIN_EXISTS(SOL2))
  #define HAS_SOLENOID_3 (PIN_EXISTS(SOL3))
  #define HAS_MICROSTEPS (ENABLED(USE_MICROSTEPS) && PIN_EXISTS(X_MS1))
  #define HAS_MICROSTEPS_E0 (ENABLED(USE_MICROSTEPS) && PIN_EXISTS(E0_MS1))
  #define HAS_MICROSTEPS_E1 (ENABLED(USE_MICROSTEPS) && PIN_EXISTS(E1_MS1))
  #define HAS_MICROSTEPS_E2 (ENABLED(USE_MICROSTEPS) && PIN_EXISTS(E2_MS1))
  #define HAS_STEPPER_RESET (PIN_EXISTS(STEPPER_RESET))
  #define HAS_X_ENABLE (PIN_EXISTS(X_ENABLE))
  #define HAS_X2_ENABLE (PIN_EXISTS(X2_ENABLE))
  #define HAS_Y_ENABLE (PIN_EXISTS(Y_ENABLE))
  #define HAS_Y2_ENABLE (PIN_EXISTS(Y2_ENABLE))
  #define HAS_Z_ENABLE (PIN_EXISTS(Z_ENABLE))
  #define HAS_Z2_ENABLE (PIN_EXISTS(Z2_ENABLE))
  #define HAS_E0_ENABLE (PIN_EXISTS(E0_ENABLE))
  #define HAS_E1_ENABLE (PIN_EXISTS(E1_ENABLE))
  #define HAS_E2_ENABLE (PIN_EXISTS(E2_ENABLE))
  #define HAS_E3_ENABLE (PIN_EXISTS(E3_ENABLE))
  #define HAS_E4_ENABLE (PIN_EXISTS(E4_ENABLE))
  #define HAS_E5_ENABLE (PIN_EXISTS(E5_ENABLE))
  #define HAS_X_DIR (PIN_EXISTS(X_DIR))
  #define HAS_X2_DIR (PIN_EXISTS(X2_DIR))
  #define HAS_Y_DIR (PIN_EXISTS(Y_DIR))
  #define HAS_Y2_DIR (PIN_EXISTS(Y2_DIR))
  #define HAS_Z_DIR (PIN_EXISTS(Z_DIR))
  #define HAS_Z2_DIR (PIN_EXISTS(Z2_DIR))
  #define HAS_E0_DIR (PIN_EXISTS(E0_DIR))
  #define HAS_E1_DIR (PIN_EXISTS(E1_DIR))
  #define HAS_E2_DIR (PIN_EXISTS(E2_DIR))
  #define HAS_E3_DIR (PIN_EXISTS(E3_DIR))
  #define HAS_E4_DIR (PIN_EXISTS(E4_DIR))
  #define HAS_E5_DIR (PIN_EXISTS(E5_DIR))
  #define HAS_X_STEP (PIN_EXISTS(X_STEP))
  #define HAS_X2_STEP (PIN_EXISTS(X2_STEP))
  #define HAS_Y_STEP (PIN_EXISTS(Y_STEP))
  #define HAS_Y2_STEP (PIN_EXISTS(Y2_STEP))
  #define HAS_Z_STEP (PIN_EXISTS(Z_STEP))
  #define HAS_Z2_STEP (PIN_EXISTS(Z2_STEP))
  #define HAS_E0_STEP (PIN_EXISTS(E0_STEP))
  #define HAS_E1_STEP (PIN_EXISTS(E1_STEP))
  #define HAS_E2_STEP (PIN_EXISTS(E2_STEP))
  #define HAS_E3_STEP (PIN_EXISTS(E3_STEP))
  #define HAS_E4_STEP (PIN_EXISTS(E4_STEP))
  #define HAS_E5_STEP (PIN_EXISTS(E5_STEP))
  #define HAS_E0E1 (PIN_EXISTS(E0E1_CHOICE))
  #define HAS_E0E2 (PIN_EXISTS(E0E2_CHOICE))
  #define HAS_E0E3 (PIN_EXISTS(E0E3_CHOICE))
  #define HAS_E0E4 (PIN_EXISTS(E0E4_CHOICE))
  #define HAS_E0E5 (PIN_EXISTS(E0E5_CHOICE))
  #define HAS_E1E3 (PIN_EXISTS(E1E3_CHOICE))
  #define HAS_BTN_BACK (PIN_EXISTS(BTN_BACK))
  #define HAS_POWER_SWITCH (POWER_SUPPLY > 0 && PIN_EXISTS(PS_ON))
  #define HAS_MOTOR_CURRENT_PWM_XY (PIN_EXISTS(MOTOR_CURRENT_PWM_XY))
  #define HAS_SDSUPPORT (ENABLED(SDSUPPORT))

  #define HAS_DIGIPOTSS (PIN_EXISTS(DIGIPOTSS))

  /**
   * Shorthand for filament sensor and power sensor for ultralcd.cpp, dogm_lcd_implementation.h, ultralcd_implementation_hitachi_HD44780.h
   */
  #define HAS_LCD_FILAMENT_SENSOR (HAS_FILAMENT_SENSOR && ENABLED(FILAMENT_LCD_DISPLAY))
  #define HAS_LCD_POWER_SENSOR (HAS_POWER_CONSUMPTION_SENSOR && ENABLED(POWER_CONSUMPTION_LCD_DISPLAY))

  /**
   * Helper Macros for heaters and extruder fan and rele
   */
  #if ENABLED(INVERTED_HEATER_PINS)
    #define WRITE_HEATER(pin, value) WRITE(pin, !value)
  #else
    #define WRITE_HEATER(pin, value) WRITE(pin, value)
  #endif
  #define WRITE_HEATER_0P(v) WRITE_HEATER(HEATER_0_PIN, v)
  #if HOTENDS > 1 || ENABLED(HEATERS_PARALLEL)
    #define WRITE_HEATER_1(v) WRITE_HEATER(HEATER_1_PIN, v)
    #if HOTENDS > 2
      #define WRITE_HEATER_2(v) WRITE_HEATER(HEATER_2_PIN, v)
      #if HOTENDS > 3
        #define WRITE_HEATER_3(v) WRITE_HEATER(HEATER_3_PIN, v)
      #endif
    #endif
  #endif
  #if ENABLED(HEATERS_PARALLEL)
    #define WRITE_HEATER_0(v) { WRITE_HEATER_0P(v); WRITE_HEATER_1(v); }
  #else
    #define WRITE_HEATER_0(v) WRITE_HEATER_0P(v)
  #endif
  #if HAS(HEATER_BED)
    #if ENABLED(INVERTED_BED_PINS)
      #define WRITE_HEATER_BED(v) WRITE(HEATER_BED_PIN,!v)
    #else
      #define WRITE_HEATER_BED(v) WRITE(HEATER_BED_PIN,v)
    #endif
  #endif
  #if HAS(FAN)
    #if ENABLED(INVERTED_HEATER_PINS)
      #define WRITE_FAN(v) WRITE(FAN_PIN, !v)
    #else
      #define WRITE_FAN(v) WRITE(FAN_PIN, v)
    #endif
  #endif
  #if ENABLED(MKR4)
    #if ENABLED(INVERTED_RELE_PINS)
      #define WRITE_RELE(pin, value) WRITE(pin, !value)
      #define OUT_WRITE_RELE(pin, value) OUT_WRITE(pin, !value)
    #else
      #define WRITE_RELE(pin, value) WRITE(pin, value)
      #define OUT_WRITE_RELE(pin, value) OUT_WRITE(pin, value)
    #endif
  #endif

  /**
   * Buzzer
   */
  #define HAS_BUZZER (PIN_EXISTS(BEEPER) || ENABLED(LCD_USE_I2C_BUZZER))

  /**
   * Servos
   */
  #if HAS(SERVOS)
    #ifndef X_ENDSTOP_SERVO_NR
      #define X_ENDSTOP_SERVO_NR -1
    #endif
    #ifndef Y_ENDSTOP_SERVO_NR
      #define Y_ENDSTOP_SERVO_NR -1
    #endif
    #ifndef Z_ENDSTOP_SERVO_NR
      #define Z_ENDSTOP_SERVO_NR -1
    #endif
    #ifndef SERVO_DEACTIVATION_DELAY
      #define SERVO_DEACTIVATION_DELAY 300
    #endif
    #if X_ENDSTOP_SERVO_NR >= 0 || Y_ENDSTOP_SERVO_NR >= 0 || Z_ENDSTOP_SERVO_NR >= 0
      #define HAS_SERVO_ENDSTOPS true
      #define SERVO_ENDSTOP_IDS { X_ENDSTOP_SERVO_NR, Y_ENDSTOP_SERVO_NR, Z_ENDSTOP_SERVO_NR }
    #endif
  #endif

  /**
   * The axis order in all axis related arrays is X, Y, Z, E
   */
  #define NUM_AXIS 4

#endif //CONDITIONALS_H
