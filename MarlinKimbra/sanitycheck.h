/**
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

#ifndef SANITYCHECK_H
  #define SANITYCHECK_H
  
  #if NOTEXIST(SERIAL_PORT)
    #error DEPENDENCY ERROR: Missing setting SERIAL_PORT
  #endif
  #if NOTEXIST(BAUDRATE)
    #error DEPENDENCY ERROR: Missing setting BAUDRATE
  #endif
  #if NOTEXIST(MACHINE_UUID)
    #error DEPENDENCY ERROR: Missing setting MACHINE_UUID
  #endif

  //board
  #if NOTEXIST(MOTHERBOARD)
    #error DEPENDENCY ERROR: Missing setting MOTHERBOARD
  #endif

  //Mechanism
  #if NOTEXIST(MECHANISM)
    #error DEPENDENCY ERROR: Missing setting MECHANISM
  #endif

  //Power supply
  #if NOTEXIST(POWER_SUPPLY)
    #error DEPENDENCY ERROR: Missing setting POWER_SUPPLY
  #endif

  //Thermistor
  #if NOTEXIST(TEMP_SENSOR_0)
    #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_0
  #endif
  #if NOTEXIST(TEMP_SENSOR_1)
    #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_1
  #endif
  #if NOTEXIST(TEMP_SENSOR_2)
    #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_2
  #endif
  #if NOTEXIST(TEMP_SENSOR_3)
    #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_3
  #endif
  #if NOTEXIST(TEMP_SENSOR_BED)
    #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_BED
  #endif
  #if (THERMISTORHEATER_0 == 998) || (THERMISTORHEATER_1 == 998) || (THERMISTORHEATER_2 == 998) || (THERMISTORHEATER_3 == 998) || (THERMISTORBED == 998) //User EXIST table
    #if NOTEXIST(DUMMY_THERMISTOR_998_VALUE)
      #error DEPENDENCY ERROR: Missing setting DUMMY_THERMISTOR_998_VALUE
    #endif
  #endif

  //Temperature
  /**
   * Temperature defines
   */
  #if EXIST(TEMP_RESIDENCY_TIME)
    #if NOTEXIST(TEMP_HYSTERESIS)
      #error DEPENDENCY ERROR: Missing setting TEMP_HYSTERESIS
    #endif
    #if NOTEXIST(TEMP_WINDOW)
      #error DEPENDENCY ERROR: Missing setting TEMP_WINDOW
    #endif
  #endif
  #if TEMP_SENSOR_0 != 0
    #if NOTEXIST(HEATER_0_MAXTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_0_MAXTEMP
    #endif
    #if NOTEXIST(HEATER_0_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_0_MINTEMP
    #endif
  #endif
  #if TEMP_SENSOR_1 != 0
    #if NOTEXIST(HEATER_1_MAXTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_1_MAXTEMP
    #endif
    #if NOTEXIST(HEATER_0_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_1_MINTEMP
    #endif
  #endif
  #if TEMP_SENSOR_2 != 0
    #if NOTEXIST(HEATER_2_MAXTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_2_MAXTEMP
    #endif
    #if NOTEXIST(HEATER_0_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_2_MINTEMP
    #endif
  #endif
  #if TEMP_SENSOR_3 != 0
    #if NOTEXIST(HEATER_3_MAXTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_3_MAXTEMP
    #endif
    #if NOTEXIST(HEATER_0_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_3_MINTEMP
    #endif
  #endif
  #if TEMP_SENSOR_BED != 0
    #if NOTEXIST(BED_MAXTEMP)
      #error DEPENDENCY ERROR: Missing setting BED_MAXTEMP
    #endif
    #if NOTEXIST(HEATER_0_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting BED_MINTEMP
    #endif
  #endif
  #if NOTEXIST(PLA_PREHEAT_HOTEND_TEMP)
    #error DEPENDENCY ERROR: Missing setting PLA_PREHEAT_HOTEND_TEMP
  #endif
  #if NOTEXIST(PLA_PREHEAT_HPB_TEMP)
    #error DEPENDENCY ERROR: Missing setting PLA_PREHEAT_HPB_TEMP
  #endif
  #if NOTEXIST(PLA_PREHEAT_FAN_SPEED)
    #error DEPENDENCY ERROR: Missing setting PLA_PREHEAT_FAN_SPEED
  #endif
  #if NOTEXIST(ABS_PREHEAT_HOTEND_TEMP)
    #error DEPENDENCY ERROR: Missing setting ABS_PREHEAT_HOTEND_TEMP
  #endif
  #if NOTEXIST(ABS_PREHEAT_HPB_TEMP)
    #error DEPENDENCY ERROR: Missing setting ABS_PREHEAT_HPB_TEMP
  #endif
  #if NOTEXIST(ABS_PREHEAT_FAN_SPEED)
    #error DEPENDENCY ERROR: Missing setting ABS_PREHEAT_FAN_SPEED
  #endif
  #if NOTEXIST(GUM_PREHEAT_HOTEND_TEMP)
    #error DEPENDENCY ERROR: Missing setting GUM_PREHEAT_HOTEND_TEMP
  #endif
  #if NOTEXIST(GUM_PREHEAT_HPB_TEMP)
    #error DEPENDENCY ERROR: Missing setting GUM_PREHEAT_HPB_TEMP
  #endif
  #if NOTEXIST(GUM_PREHEAT_FAN_SPEED)
    #error DEPENDENCY ERROR: Missing setting GUM_PREHEAT_FAN_SPEED
  #endif

  //extruders
  #if NOTEXIST(EXTRUDERS)
    #error DEPENDENCY ERROR: Missing setting EXTRUDERS
  #endif
  #if NOTEXIST(DRIVER_EXTRUDERS)
    #error DEPENDENCY ERROR: Missing setting DRIVER_EXTRUDERS
  #endif

  //Language
  #if NOTEXIST(LANGUAGE_CHOICE)
    #error DEPENDENCY ERROR: Missing setting LANGUAGE_CHOICE
  #endif

  ///FEATURE

  //Temperature
  #if NOTEXIST(PID_MAX)
    #error DEPENDENCY ERROR: Missing setting PID_MAX
  #endif
  #if NOTEXIST(MAX_BED_POWER)
    #error DEPENDENCY ERROR: Missing setting MAX_BED_POWER
  #endif
  #if EXIST(PIDTEMP) || EXIST(PIDTEMPBED)
    #if NOTEXIST(MAX_OVERSHOOT_PID_AUTOTUNE)
      #error DEPENDENCY ERROR: Missing setting MAX_OVERSHOOT_PID_AUTOTUNE
    #endif
  #endif
  #if EXIST(PIDTEMP)
    #if NOTEXIST(PID_OPENLOOP) && NOTEXIST(PID_FUNCTIONAL_RANGE) 
      #error DEPENDENCY ERROR: Missing setting PID_FUNCTIONAL_RANGE
    #endif
    #if NOTEXIST(PID_INTEGRAL_DRIVE_MAX)
      #error DEPENDENCY ERROR: Missing setting PID_INTEGRAL_DRIVE_MAX
    #endif
    #if NOTEXIST(DEFAULT_Kp)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_Kp
    #endif
    #if NOTEXIST(DEFAULT_Ki)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_Ki
    #endif
    #if NOTEXIST(DEFAULT_Kd)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_Kd
    #endif
  #endif
  #if EXIST(PIDTEMPBED)
    #if NOTEXIST(PID_BED_INTEGRAL_DRIVE_MAX)
      #error DEPENDENCY ERROR: Missing setting PID_BED_INTEGRAL_DRIVE_MAX
    #endif
    #if NOTEXIST(DEFAULT_bedKp)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_bedKp
    #endif
    #if NOTEXIST(DEFAULT_bedKi)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_bedKi
    #endif
    #if NOTEXIST(DEFAULT_bedKd)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_bedKd
    #endif
  #endif
  #if EXIST(BED_LIMIT_SWITCHING)
    #if NOTEXIST(BED_HYSTERESIS)
      #error DEPENDENCY ERROR: Missing setting BED_HYSTERESIS
    #endif
    #if NOTEXIST(BED_CHECK_INTERVAL)
      #error DEPENDENCY ERROR: Missing setting BED_CHECK_INTERVAL
    #endif
  #endif
  #if EXIST(THERMAL_PROTECTION_HOTENDS)
    #if NOTEXIST(THERMAL_PROTECTION_PERIOD)
      #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_PERIOD
    #endif
    #if NOTEXIST(THERMAL_PROTECTION_HYSTERESIS)
      #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_HYSTERESIS
    #endif
    #if NOTEXIST(WATCH_TEMP_PERIOD)
      #error DEPENDENCY ERROR: Missing setting WATCH_TEMP_PERIOD
    #endif
    #if NOTEXIST(WATCH_TEMP_INCREASE)
      #error DEPENDENCY ERROR: Missing setting WATCH_TEMP_INCREASE
    #endif
  #endif
  #if EXIST(THERMAL_PROTECTION_BED)
    #if NOTEXIST(THERMAL_PROTECTION_BED_PERIOD)
      #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_BED_PERIOD
    #endif
    #if NOTEXIST(THERMAL_PROTECTION_BED_HYSTERESIS)
      #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_BED_HYSTERESIS
    #endif
  #endif
  //fan
  #if NOTEXIST(SOFT_PWM_SCALE)
    #error DEPENDENCY ERROR: Missing setting SOFT_PWM_SCALE
  #endif

  #if EXIST(CONTROLLERFAN)
    #if NOTEXIST(CONTROLLERFAN_SECS)
      #error DEPENDENCY ERROR: Missing setting CONTROLLERFAN_SECS
    #endif
    #if NOTEXIST(CONTROLLERFAN_SPEED)
      #error DEPENDENCY ERROR: Missing setting CONTROLLERFAN_SPEED
    #endif
    #if NOTEXIST(CONTROLLERFAN_MIN_SPEED)
      #error DEPENDENCY ERROR: Missing setting CONTROLLERFAN_MIN_SPEED
    #endif
  #endif

  #if EXIST(EXTRUDER_AUTO_FAN)
    #if NOTEXIST(EXTRUDER_AUTO_FAN_TEMPERATURE)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_AUTO_FAN_TEMPERATURE
    #endif
    #if NOTEXIST(EXTRUDER_AUTO_FAN_SPEED)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_AUTO_FAN_SPEED
    #endif
    #if NOTEXIST(EXTRUDER_AUTO_FAN_MIN_SPEED)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_AUTO_FAN_MIN_SPEED
    #endif
  #endif

  //extruder
  #if EXIST(PREVENT_DANGEROUS_EXTRUDE)
    #if NOTEXIST(EXTRUDE_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting EXTRUDE_MINTEMP
    #endif
    #if EXIST(PREVENT_LENGTHY_EXTRUDE)
      #if NOTEXIST(EXTRUDE_MAXLENGTH)
        #error DEPENDENCY ERROR: Missing setting EXTRUDE_MAXLENGTH
      #endif
    #endif
  #endif

  #if EXIST(NPR2)
    #if NOTEXIST(COLOR_STEP)
      #error DEPENDENCY ERROR: Missing setting COLOR_STEP
    #endif
    #if NOTEXIST(COLOR_SLOWRATE)
      #error DEPENDENCY ERROR: Missing setting COLOR_SLOWRATE
    #endif
    #if NOTEXIST(COLOR_HOMERATE)
      #error DEPENDENCY ERROR: Missing setting COLOR_HOMERATE
    #endif
    #if NOTEXIST(MOTOR_ANGLE)
      #error DEPENDENCY ERROR: Missing setting MOTOR_ANGLE
    #endif
    #if NOTEXIST(DRIVER_MICROSTEP)
      #error DEPENDENCY ERROR: Missing setting DRIVER_MICROSTEP
    #endif
    #if NOTEXIST(CARTER_MOLTIPLICATOR)
      #error DEPENDENCY ERROR: Missing setting CARTER_MOLTIPLICATOR
    #endif
  #endif

  #if EXIST(IDLE_OOZING_PREVENT)
    #if NOTEXIST(IDLE_OOZING_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_MINTEMP
    #endif
    #if NOTEXIST(IDLE_OOZING_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_FEEDRATE
    #endif
    #if NOTEXIST(IDLE_OOZING_SECONDS)
      #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_SECONDS
    #endif
    #if NOTEXIST(IDLE_OOZING_LENGTH)
      #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_LENGTH
    #endif
    #if NOTEXIST(IDLE_OOZING_RECOVER_LENGTH)
      #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_RECOVER_LENGTH
    #endif
    #if NOTEXIST(IDLE_OOZING_RECOVER_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_RECOVER_FEEDRATE
    #endif
  #endif

  #if EXIST(EXTRUDER_RUNOUT_PREVENT)
    #if NOTEXIST(EXTRUDER_RUNOUT_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_MINTEMP
    #endif
    #if NOTEXIST(EXTRUDER_RUNOUT_SECONDS)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_SECONDS
    #endif
    #if NOTEXIST(EXTRUDER_RUNOUT_ESTEPS)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_ESTEPS
    #endif
    #if NOTEXIST(EXTRUDER_RUNOUT_SPEED)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_SPEED
    #endif
    #if NOTEXIST(EXTRUDER_RUNOUT_EXTRUDE)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_EXTRUDE
    #endif
  #endif

  #if EXIST(EASY_LOAD)
    #if NOTEXIST(BOWDEN_LENGTH)
      #error DEPENDENCY ERROR: Missing setting BOWDEN_LENGTH
    #endif
    #if NOTEXIST(LCD_PURGE_LENGTH)
      #error DEPENDENCY ERROR: Missing setting LCD_PURGE_LENGTH
    #endif
    #if NOTEXIST(LCD_RETRACT_LENGTH)
      #error DEPENDENCY ERROR: Missing setting LCD_RETRACT_LENGTH
    #endif
    #if NOTEXIST(LCD_PURGE_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting LCD_PURGE_FEEDRATE
    #endif
    #if NOTEXIST(LCD_RETRACT_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting LCD_RETRACT_FEEDRATE
    #endif
    #if NOTEXIST(LCD_LOAD_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting LCD_LOAD_FEEDRATE
    #endif
    #if NOTEXIST(LCD_UNLOAD_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting LCD_UNLOAD_FEEDRATE
    #endif
  #endif

  #if EXIST(ADVANCE)
    #if NOTEXIST(EXTRUDER_ADVANCE_K)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_ADVANCE_K
    #endif
    #if NOTEXIST(D_FILAMENT)
      #error DEPENDENCY ERROR: Missing setting D_FILAMENT
    #endif
    #if NOTEXIST(STEPS_MM_E)
      #error DEPENDENCY ERROR: Missing setting STEPS_MM_E
    #endif
  #endif

  #if EXIST(FILAMENTCHANGEENABLE)
    #if NOTEXIST(FILAMENTCHANGE_XPOS)
      #error DEPENDENCY ERROR: Missing setting FILAMENTCHANGE_XPOS
    #endif
    #if NOTEXIST(FILAMENTCHANGE_YPOS)
      #error DEPENDENCY ERROR: Missing setting FILAMENTCHANGE_YPOS
    #endif
    #if NOTEXIST(FILAMENTCHANGE_ZADD)
      #error DEPENDENCY ERROR: Missing setting FILAMENTCHANGE_ZADD
    #endif
    #if NOTEXIST(FILAMENTCHANGE_FIRSTRETRACT)
      #error DEPENDENCY ERROR: Missing setting FILAMENTCHANGE_FIRSTRETRACT
    #endif
    #if NOTEXIST(FILAMENTCHANGE_FINALRETRACT)
      #error DEPENDENCY ERROR: Missing setting FILAMENTCHANGE_FINALRETRACT
    #endif
    #if NOTEXIST(FILAMENTCHANGE_PRINTEROFF)
      #error DEPENDENCY ERROR: Missing setting FILAMENTCHANGE_PRINTEROFF
    #endif
  #endif
    
  //Motion
  #if NOTEXIST(SOFTWARE_MIN_ENDSTOPS)
    #error DEPENDENCY ERROR: Missing setting SOFTWARE_MIN_ENDSTOPS
  #endif
  #if NOTEXIST(SOFTWARE_MAX_ENDSTOPS)
    #error DEPENDENCY ERROR: Missing setting SOFTWARE_MAX_ENDSTOPS
  #endif
  #if EXIST(AUTO_BED_LEVELING_FEATURE)
    #if EXIST(AUTO_BED_LEVELING_GRID)
      #if NOTEXIST(MIN_PROBE_EDGE)
        #error DEPENDENCY ERROR: Missing setting MIN_PROBE_EDGE
      #endif
      #if NOTEXIST(AUTO_BED_LEVELING_GRID_POINTS)
        #error DEPENDENCY ERROR: Missing setting AUTO_BED_LEVELING_GRID_POINTS
      #endif
    #else
      #if NOTEXIST(ABL_PROBE_PT_1_X)
        #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_1_X
      #endif
      #if NOTEXIST(ABL_PROBE_PT_1_Y)
        #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_1_Y
      #endif
      #if NOTEXIST(ABL_PROBE_PT_2_X)
        #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_2_X
      #endif
      #if NOTEXIST(ABL_PROBE_PT_2_Y)
        #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_2_Y
      #endif
      #if NOTEXIST(ABL_PROBE_PT_3_X)
        #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_3_X
      #endif
      #if NOTEXIST(ABL_PROBE_PT_3_Y)
        #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_3_Y
      #endif
    #endif
    #if NOTEXIST(X_PROBE_OFFSET_FROM_EXTRUDER)
      #error DEPENDENCY ERROR: Missing setting X_PROBE_OFFSET_FROM_EXTRUDER
    #endif
    #if NOTEXIST(Y_PROBE_OFFSET_FROM_EXTRUDER)
      #error DEPENDENCY ERROR: Missing setting Y_PROBE_OFFSET_FROM_EXTRUDER
    #endif
    #if NOTEXIST(Z_PROBE_OFFSET_FROM_EXTRUDER)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_OFFSET_FROM_EXTRUDER
    #endif
    #if NOTEXIST(Z_RAISE_BEFORE_HOMING)
      #error DEPENDENCY ERROR: Missing setting Z_RAISE_BEFORE_HOMING
    #endif
    #if NOTEXIST(Z_RAISE_BEFORE_PROBING)
      #error DEPENDENCY ERROR: Missing setting Z_RAISE_BEFORE_PROBING
    #endif
    #if NOTEXIST(Z_RAISE_BETWEEN_PROBINGS)
      #error DEPENDENCY ERROR: Missing setting Z_RAISE_BETWEEN_PROBINGS
    #endif
    #if NOTEXIST(Z_RAISE_AFTER_PROBING)
      #error DEPENDENCY ERROR: Missing setting Z_RAISE_AFTER_PROBING
    #endif
    #if EXIST(Z_PROBE_SLED)
      #if NOTEXIST(SLED_DOCKING_OFFSET)
        #error DEPENDENCY ERROR: Missing setting SLED_DOCKING_OFFSET
      #endif
    #endif
    #if EXIST(Z_SAFE_HOMING)
      #if NOTEXIST(Z_SAFE_HOMING_X_POINT)
        #error DEPENDENCY ERROR: Missing setting Z_SAFE_HOMING_X_POINT
      #endif
      #if NOTEXIST(Z_SAFE_HOMING_Y_POINT)
        #error DEPENDENCY ERROR: Missing setting Z_SAFE_HOMING_Y_POINT
      #endif
    #endif
  #endif
  #if ENABLED(ENABLE_SERVOS)
    #if NOTEXIST(NUM_SERVOS)
      #error DEPENDENCY ERROR: Missing setting NUM_SERVOS
    #endif
    #if NUM_SERVOS > 0
      #if NOTEXIST(X_ENDSTOP_SERVO_NR)
        #error DEPENDENCY ERROR: Missing setting X_ENDSTOP_SERVO_NR
      #endif
      #if NOTEXIST(Y_ENDSTOP_SERVO_NR)
        #error DEPENDENCY ERROR: Missing setting Y_ENDSTOP_SERVO_NR
      #endif
      #if NOTEXIST(Z_ENDSTOP_SERVO_NR)
        #error DEPENDENCY ERROR: Missing setting Z_ENDSTOP_SERVO_NR
      #endif
      #if NOTEXIST(X_ENDSTOP_SERVO_ANGLES)
        #error DEPENDENCY ERROR: Missing setting X_ENDSTOP_SERVO_ANGLES
      #endif
      #if NOTEXIST(Y_ENDSTOP_SERVO_ANGLES)
        #error DEPENDENCY ERROR: Missing setting Y_ENDSTOP_SERVO_ANGLES
      #endif
      #if NOTEXIST(Z_ENDSTOP_SERVO_ANGLES)
        #error DEPENDENCY ERROR: Missing setting Z_ENDSTOP_SERVO_ANGLES
      #endif
      #if NOTEXIST(SERVO_DEACTIVATION_DELAY)
        #error DEPENDENCY ERROR: Missing setting SERVO_DEACTIVATION_DELAY
      #endif
    #endif
  #endif
  #if EXIST(BABYSTEPPING)
    #if NOTEXIST(BABYSTEP_INVERT_Z)
      #error DEPENDENCY ERROR: Missing setting BABYSTEP_INVERT_Z
    #endif
    #if NOTEXIST(BABYSTEP_Z_MULTIPLICATOR)
      #error DEPENDENCY ERROR: Missing setting BABYSTEP_Z_MULTIPLICATOR
    #endif
  #endif
  #if EXIST(FWRETRACT)
    #if NOTEXIST(MIN_RETRACT)
      #error DEPENDENCY ERROR: Missing setting MIN_RETRACT
    #endif
    #if NOTEXIST(RETRACT_LENGTH)
      #error DEPENDENCY ERROR: Missing setting RETRACT_LENGTH
    #endif
    #if NOTEXIST(RETRACT_LENGTH_SWAP)
      #error DEPENDENCY ERROR: Missing setting RETRACT_LENGTH_SWAP
    #endif
    #if NOTEXIST(RETRACT_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting RETRACT_FEEDRATE
    #endif
    #if NOTEXIST(RETRACT_ZLIFT)
      #error DEPENDENCY ERROR: Missing setting RETRACT_ZLIFT
    #endif
    #if NOTEXIST(RETRACT_RECOVER_LENGTH)
      #error DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_LENGTH
    #endif
    #if NOTEXIST(RETRACT_RECOVER_LENGTH_SWAP)
      #error DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_LENGTH_SWAP
    #endif
    #if NOTEXIST(RETRACT_RECOVER_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_FEEDRATE
    #endif
  #endif
  #if EXIST(DUAL_X_CARRIAGE)
    #if NOTEXIST(X2_MIN_POS)
      #error DEPENDENCY ERROR: Missing setting X2_MIN_POS
    #endif
    #if NOTEXIST(X2_MAX_POS)
      #error DEPENDENCY ERROR: Missing setting X2_MAX_POS
    #endif
    #if NOTEXIST(X2_HOME_DIR)
      #error DEPENDENCY ERROR: Missing setting X2_HOME_DIR
    #endif
    #if NOTEXIST(X2_HOME_POS)
      #error DEPENDENCY ERROR: Missing setting X2_HOME_POS
    #endif
    #if NOTEXIST(DEFAULT_DUAL_X_CARRIAGE_MODE)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_DUAL_X_CARRIAGE_MODE
    #endif
    #if NOTEXIST(TOOLCHANGE_PARK_ZLIFT)
      #error DEPENDENCY ERROR: Missing setting TOOLCHANGE_PARK_ZLIFT
    #endif
    #if NOTEXIST(TOOLCHANGE_UNPARK_ZLIFT)
      #error DEPENDENCY ERROR: Missing setting TOOLCHANGE_UNPARK_ZLIFT
    #endif
    #if NOTEXIST(DEFAULT_DUPLICATION_X_OFFSET)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_DUPLICATION_X_OFFSET
    #endif
  #endif
  #if EXIST(Y_DUAL_STEPPER_DRIVERS)
    #if NOTEXIST(INVERT_Y2_VS_Y_DIR)
      #error DEPENDENCY ERROR: Missing setting INVERT_Y2_VS_Y_DIR
    #endif
  #endif

  //sensors
  #if EXIST(FILAMENT_SENSOR)
    #if NOTEXIST(FILAMENT_SENSOR_EXTRUDER_NUM)
      #error DEPENDENCY ERROR: Missing setting FILAMENT_SENSOR_EXTRUDER_NUM
    #endif
    #if NOTEXIST(MEASUREMENT_DELAY_CM)
      #error DEPENDENCY ERROR: Missing setting MEASUREMENT_DELAY_CM
    #endif
    #if NOTEXIST(DEFAULT_NOMINAL_FILAMENT_DIA)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_NOMINAL_FILAMENT_DIA 
    #endif
    #if NOTEXIST(MEASURED_UPPER_LIMIT)
      #error DEPENDENCY ERROR: Missing setting MEASURED_UPPER_LIMIT
    #endif
    #if NOTEXIST(MEASURED_LOWER_LIMIT)
      #error DEPENDENCY ERROR: Missing setting MEASURED_LOWER_LIMIT
    #endif
    #if NOTEXIST(MAX_MEASUREMENT_DELAY)
      #error DEPENDENCY ERROR: Missing setting MAX_MEASUREMENT_DELAY
    #endif
    #if NOTEXIST(DEFAULT_MEASURED_FILAMENT_DIA)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_MEASURED_FILAMENT_DIA
    #endif
  #endif
  #if EXIST(FILAMENT_RUNOUT_SENSOR)
    #if NOTEXIST(FILRUNOUT_PIN_INVERTING)
      #error DEPENDENCY ERROR: Missing setting FILRUNOUT_PIN_INVERTING
    #endif
    #if NOTEXIST(ENDSTOPPULLUP_FIL_RUNOUT)
      #error DEPENDENCY ERROR: Missing setting ENDSTOPPULLUP_FIL_RUNOUT
    #endif
    #if NOTEXIST(FILAMENT_RUNOUT_SCRIPT)
      #error DEPENDENCY ERROR: Missing setting FILAMENT_RUNOUT_SCRIPT 
    #endif
  #endif
  #if EXIST(POWER_CONSUMPTION)
    #if NOTEXIST(POWER_VOLTAGE)
      #error DEPENDENCY ERROR: Missing setting POWER_VOLTAGE
    #endif
    #if NOTEXIST(POWER_SENSITIVITY)
      #error DEPENDENCY ERROR: Missing setting POWER_SENSITIVITY
    #endif
    #if NOTEXIST(POWER_OFFSET)
      #error DEPENDENCY ERROR: Missing setting POWER_OFFSET 
    #endif
    #if NOTEXIST(POWER_ZERO)
      #error DEPENDENCY ERROR: Missing setting POWER_ZERO 
    #endif
    #if NOTEXIST(POWER_ERROR)
      #error DEPENDENCY ERROR: Missing setting POWER_ERROR 
    #endif
    #if NOTEXIST(POWER_EFFICIENCY)
      #error DEPENDENCY ERROR: Missing setting POWER_EFFICIENCY 
    #endif
  #endif

  //addon
  #if EXIST(SDSUPPORT)
    #if NOTEXIST(SD_FINISHED_STEPPERRELEASE)
      #error DEPENDENCY ERROR: Missing setting SD_FINISHED_STEPPERRELEASE
    #endif
    #if NOTEXIST(SD_FINISHED_RELEASECOMMAND)
      #error DEPENDENCY ERROR: Missing setting SD_FINISHED_RELEASECOMMAND
    #endif
    #if EXIST(SD_SETTINGS)
      #if NOTEXIST(SD_CFG_SECONDS)
        #error DEPENDENCY ERROR: Missing setting SD_CFG_SECONDS
      #endif
      #if NOTEXIST(CFG_SD_FILE)
        #error DEPENDENCY ERROR: Missing setting CFG_SD_FILE
      #endif
      #if NOTEXIST(CFG_SD_MAX_KEY_LEN)
        #error DEPENDENCY ERROR: Missing setting CFG_SD_MAX_KEY_LEN
      #endif
      #if NOTEXIST(CFG_SD_MAX_VALUE_LEN)
        #error DEPENDENCY ERROR: Missing setting CFG_SD_MAX_VALUE_LEN
      #endif
    #endif
  #endif
  #if NOTEXIST(DISPLAY_CHARSET_HD44780_JAPAN) && NOTEXIST(DISPLAY_CHARSET_HD44780_WESTERN) && NOTEXIST(DISPLAY_CHARSET_HD44780_CYRILLIC)
    #error DEPENDENCY ERROR: Missing setting DISPLAY_CHARSET_HD44780_JAPAN or DISPLAY_CHARSET_HD44780_WESTERN or DISPLAY_CHARSET_HD44780_CYRILLIC
  #endif
  #if EXIST(SHOW_BOOTSCREEN)
    #if NOTEXIST(STRING_SPLASH_LINE1)
      #error DEPENDENCY ERROR: Missing setting STRING_SPLASH_LINE1
    #endif
    #if NOTEXIST(SPLASH_SCREEN_DURATION)
      #error DEPENDENCY ERROR: Missing setting SPLASH_SCREEN_DURATION
    #endif
  #endif
  #if ENABLED(ULTIPANEL)
    #if EXIST(ENCODER_RATE_MULTIPLIER)
      #if NOTEXIST(ENCODER_10X_STEPS_PER_SEC)
        #error DEPENDENCY ERROR: Missing setting ENCODER_10X_STEPS_PER_SEC
      #endif
      #if NOTEXIST(ENCODER_100X_STEPS_PER_SEC)
        #error DEPENDENCY ERROR: Missing setting ENCODER_100X_STEPS_PER_SEC
      #endif
    #endif
  #endif
  #if MB(ALLIGATOR)
    #if NOTEXIST(UI_VOLTAGE_LEVEL)
      #error DEPENDENCY ERROR: Missing setting UI_VOLTAGE_LEVEL
    #endif
  #endif
  #if EXIST(REPRAPWORLD_KEYPAD)
    #if NOTEXIST(REPRAPWORLD_KEYPAD_MOVE_STEP)
      #error DEPENDENCY ERROR: Missing setting REPRAPWORLD_KEYPAD_MOVE_STEP
    #endif
  #endif
  #if ENABLED(ULTIPANEL)
    #if EXIST(LCD_PROGRESS_BAR)
      #if NOTEXIST(PROGRESS_BAR_BAR_TIME)
        #error DEPENDENCY ERROR: Missing setting PROGRESS_BAR_BAR_TIME
      #endif
      #if NOTEXIST(PROGRESS_BAR_MSG_TIME)
        #error DEPENDENCY ERROR: Missing setting PROGRESS_BAR_MSG_TIME
      #endif
      #if NOTEXIST(PROGRESS_MSG_EXPIRE)
        #error DEPENDENCY ERROR: Missing setting PROGRESS_MSG_EXPIRE
      #endif
    #endif
  #endif
  #if EXIST(CHDK)
    #if NOTEXIST(CHDK_DELAY)
      #error DEPENDENCY ERROR: Missing setting CHDK_DELAY
    #endif
  #endif
  //adv motion
  #if EXIST(USE_MICROSTEPS)
    #if NOTEXIST(MICROSTEP_MODES)
      #error DEPENDENCY ERROR: Missing setting MICROSTEP_MODES
    #endif
  #endif
  #if NOTEXIST(DEFAULT_STEPPER_DEACTIVE_TIME)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_STEPPER_DEACTIVE_TIME
  #endif
  #if ENABLED(STEPPER_HIGH_LOW)
    #if NOTEXIST(STEPPER_HIGH_LOW_DELAY)
      #error DEPENDENCY ERROR: Missing setting STEPPER_HIGH_LOW_DELAY
    #endif
  #endif
  #if EXIST(DIGIPOT_I2C)
    #if NOTEXIST(DIGIPOT_I2C_NUM_CHANNELS)
      #error DEPENDENCY ERROR: Missing setting DIGIPOT_I2C_NUM_CHANNELS
    #endif
    #if NOTEXIST(DIGIPOT_I2C_MOTOR_CURRENTS)
      #error DEPENDENCY ERROR: Missing setting DIGIPOT_I2C_MOTOR_CURRENTS
    #endif
  #endif
  #if EXIST(HAVE_TMCDRIVER)
    #if EXIST(X_IS_TMC)
      #if NOTEXIST(X_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting X_MAX_CURRENT
      #endif
      #if NOTEXIST(X_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting X_SENSE_RESISTOR
      #endif
      #if NOTEXIST(X_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting X_MICROSTEPS
      #endif
    #endif
    #if EXIST(X2_IS_TMC)
      #if NOTEXIST(X2_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting X2_MAX_CURRENT
      #endif
      #if NOTEXIST(X2_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting X2_SENSE_RESISTOR
      #endif
      #if NOTEXIST(X2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting X2_MICROSTEPS
      #endif
    #endif
    #if EXIST(Y_IS_TMC)
      #if NOTEXIST(Y_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting Y_MAX_CURRENT
      #endif
      #if NOTEXIST(Y_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting Y_SENSE_RESISTOR
      #endif
      #if NOTEXIST(Y_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Y_MICROSTEPS
      #endif
    #endif
    #if EXIST(Y2_IS_TMC)
      #if NOTEXIST(Y2_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting Y2_MAX_CURRENT
      #endif
      #if NOTEXIST(Y2_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting Y2_SENSE_RESISTOR
      #endif
      #if NOTEXIST(Y2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Y2_MICROSTEPS
      #endif
    #endif
    #if EXIST(Z_IS_TMC)
      #if NOTEXIST(Z_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting Z_MAX_CURRENT
      #endif
      #if NOTEXIST(Z_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting Z_SENSE_RESISTOR
      #endif
      #if NOTEXIST(Z_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Z_MICROSTEPS
      #endif
    #endif
    #if EXIST(Z2_IS_TMC)
      #if NOTEXIST(Z2_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting Z2_MAX_CURRENT
      #endif
      #if NOTEXIST(Z2_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting Z2_SENSE_RESISTOR
      #endif
      #if NOTEXIST(Z2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Z2_MICROSTEPS
      #endif
    #endif
    #if EXIST(E0_IS_TMC)
      #if NOTEXIST(E0_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting E0_MAX_CURRENT
      #endif
      #if NOTEXIST(E0_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting E0_SENSE_RESISTOR
      #endif
      #if NOTEXIST(E0_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E0_MICROSTEPS
      #endif
    #endif
    #if EXIST(E1_IS_TMC)
      #if NOTEXIST(E1_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting E1_MAX_CURRENT
      #endif
      #if NOTEXIST(E1_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting E1_SENSE_RESISTOR
      #endif
      #if NOTEXIST(E1_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E1_MICROSTEPS
      #endif
    #endif
    #if EXIST(E2_IS_TMC)
      #if NOTEXIST(E2_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting E2_MAX_CURRENT
      #endif
      #if NOTEXIST(E2_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting E2_SENSE_RESISTOR
      #endif
      #if NOTEXIST(E2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E2_MICROSTEPS
      #endif
    #endif
    #if EXIST(E3_IS_TMC)
      #if NOTEXIST(E3_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting E3_MAX_CURRENT
      #endif
      #if NOTEXIST(E3_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting E3_SENSE_RESISTOR
      #endif
      #if NOTEXIST(E3_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E3_MICROSTEPS
      #endif
    #endif
  #endif
  #if EXIST(HAVE_L6470DRIVER)
    #if EXIST(X_IS_L6470)
      #if NOTEXIST(X_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting X_MICROSTEPS
      #endif
      #if NOTEXIST(X_K_VAL)
        #error DEPENDENCY ERROR: Missing setting X_K_VAL
      #endif
      #if NOTEXIST(X_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting X_OVERCURRENT
      #endif
      #if NOTEXIST(X_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting X_STALLCURRENT
      #endif
    #endif
    #if EXIST(X2_IS_L6470)
      #if NOTEXIST(X2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting X2_MICROSTEPS
      #endif
      #if NOTEXIST(X2_K_VAL)
        #error DEPENDENCY ERROR: Missing setting X2_K_VAL
      #endif
      #if NOTEXIST(X2_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting X2_OVERCURRENT
      #endif
      #if NOTEXIST(X2_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting X2_STALLCURRENT
      #endif
    #endif
    #if EXIST(Y_IS_L6470)
      #if NOTEXIST(Y_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Y_MICROSTEPS
      #endif
      #if NOTEXIST(Y_K_VAL)
        #error DEPENDENCY ERROR: Missing setting Y_K_VAL
      #endif
      #if NOTEXIST(Y_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting Y_OVERCURRENT
      #endif
      #if NOTEXIST(Y_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting Y_STALLCURRENT
      #endif
    #endif
    #if EXIST(Y2_IS_L6470)
      #if NOTEXIST(Y2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Y2_MICROSTEPS
      #endif
      #if NOTEXIST(Y2_K_VAL)
        #error DEPENDENCY ERROR: Missing setting Y2_K_VAL
      #endif
      #if NOTEXIST(Y2_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting Y2_OVERCURRENT
      #endif
      #if NOTEXIST(Y2_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting Y2_STALLCURRENT
      #endif
    #endif
    #if EXIST(Z_IS_L6470)
      #if NOTEXIST(Z_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Z_MICROSTEPS
      #endif
      #if NOTEXIST(Z_K_VAL)
        #error DEPENDENCY ERROR: Missing setting Z_K_VAL
      #endif
      #if NOTEXIST(Z_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting Z_OVERCURRENT
      #endif
      #if NOTEXIST(Z_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting Z_STALLCURRENT
      #endif
    #endif
    #if EXIST(Z2_IS_L6470)
      #if NOTEXIST(Z2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Z2_MICROSTEPS
      #endif
      #if NOTEXIST(Z2_K_VAL)
        #error DEPENDENCY ERROR: Missing setting Z2_K_VAL
      #endif
      #if NOTEXIST(Z2_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting Z2_OVERCURRENT
      #endif
      #if NOTEXIST(Z2_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting Z2_STALLCURRENT
      #endif
    #endif
    #if EXIST(E0_IS_L6470)
      #if NOTEXIST(E0_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E0_MICROSTEPS
      #endif
      #if NOTEXIST(E0_K_VAL)
        #error DEPENDENCY ERROR: Missing setting E0_K_VAL
      #endif
      #if NOTEXIST(E0_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting E0_OVERCURRENT
      #endif
      #if NOTEXIST(E0_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting E0_STALLCURRENT
      #endif
    #endif
    #if EXIST(E1_IS_L6470)
      #if NOTEXIST(E1_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E1_MICROSTEPS
      #endif
      #if NOTEXIST(E1_K_VAL)
        #error DEPENDENCY ERROR: Missing setting E1_K_VAL
      #endif
      #if NOTEXIST(E1_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting E1_OVERCURRENT
      #endif
      #if NOTEXIST(E1_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting E1_STALLCURRENT
      #endif
    #endif
    #if EXIST(E2_IS_L6470)
      #if NOTEXIST(E2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E2_MICROSTEPS
      #endif
      #if NOTEXIST(E2_K_VAL)
        #error DEPENDENCY ERROR: Missing setting E2_K_VAL
      #endif
      #if NOTEXIST(E2_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting E2_OVERCURRENT
      #endif
      #if NOTEXIST(E2_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting E2_STALLCURRENT
      #endif
    #endif
    #if EXIST(E3_IS_L6470)
      #if NOTEXIST(E3_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E3_MICROSTEPS
      #endif
      #if NOTEXIST(E3_K_VAL)
        #error DEPENDENCY ERROR: Missing setting E3_K_VAL
      #endif
      #if NOTEXIST(E3_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting E3_OVERCURRENT
      #endif
      #if NOTEXIST(E3_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting E3_STALLCURRENT
      #endif
    #endif
  #endif
  //buffer
  #if NOTEXIST(BLOCK_BUFFER_SIZE)
    #error DEPENDENCY ERROR: Missing setting BLOCK_BUFFER_SIZE
  #endif
  #if NOTEXIST(MAX_CMD_SIZE)
    #error DEPENDENCY ERROR: Missing setting MAX_CMD_SIZE
  #endif
  #if NOTEXIST(BUFSIZE)
    #error DEPENDENCY ERROR: Missing setting BUFSIZE
  #endif
  #if NOTEXIST(NUM_POSITON_SLOTS)
    #error DEPENDENCY ERROR: Missing setting NUM_POSITON_SLOTS
  #endif
  #if NOTEXIST(DROP_SEGMENTS)
    #error DEPENDENCY ERROR: Missing setting DROP_SEGMENTS
  #endif
  #if NOTEXIST(DROP_SEGMENTS)
    #error DEPENDENCY ERROR: Missing setting DROP_SEGMENTS
  #endif
  #if NOTEXIST(DEFAULT_MINSEGMENTTIME)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_MINSEGMENTTIME
  #endif
  #if NOTEXIST(MM_PER_ARC_SEGMENT)
    #error DEPENDENCY ERROR: Missing setting MM_PER_ARC_SEGMENT
  #endif
  #if NOTEXIST(N_ARC_CORRECTION)
    #error DEPENDENCY ERROR: Missing setting N_ARC_CORRECTION
  #endif

  //Machines
  #if NOTEXIST(X_MIN_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting X_MIN_ENDSTOP_LOGIC
  #endif
  #if NOTEXIST(Y_MIN_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting Y_MIN_ENDSTOP_LOGIC
  #endif
  #if NOTEXIST(Z_MIN_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting Z_MIN_ENDSTOP_LOGIC
  #endif
  #if NOTEXIST(Z2_MIN_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting Z2_MIN_ENDSTOP_LOGIC
  #endif
  #if NOTEXIST(X_MAX_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting X_MAX_ENDSTOP_LOGIC
  #endif
  #if NOTEXIST(Y_MAX_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting Y_MAX_ENDSTOP_LOGIC
  #endif
  #if NOTEXIST(Z_MAX_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting Z_MAX_ENDSTOP_LOGIC
  #endif
  #if NOTEXIST(Z2_MAX_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting Z2_MAX_ENDSTOP_LOGIC
  #endif
  #if NOTEXIST(Z_PROBE_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting Z_PROBE_ENDSTOP_LOGIC
  #endif
  #if NOTEXIST(E_MIN_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting E_MIN_ENDSTOP_LOGIC
  #endif
  #if NOTEXIST(X_HOME_DIR)
    #error DEPENDENCY ERROR: Missing setting X_HOME_DIR
  #endif
  #if NOTEXIST(Y_HOME_DIR)
    #error DEPENDENCY ERROR: Missing setting Y_HOME_DIR
  #endif
  #if NOTEXIST(Z_HOME_DIR)
    #error DEPENDENCY ERROR: Missing setting Z_HOME_DIR
  #endif
  #if NOTEXIST(E_HOME_DIR)
    #error DEPENDENCY ERROR: Missing setting E_HOME_DIR
  #endif
  #if NOTEXIST(X_ENABLE_ON)
    #error DEPENDENCY ERROR: Missing setting X_ENABLE_ON
  #endif
  #if NOTEXIST(Y_ENABLE_ON)
    #error DEPENDENCY ERROR: Missing setting Y_ENABLE_ON
  #endif
  #if NOTEXIST(Z_ENABLE_ON)
    #error DEPENDENCY ERROR: Missing setting Z_ENABLE_ON
  #endif
  #if NOTEXIST(E_ENABLE_ON)
    #error DEPENDENCY ERROR: Missing setting E_ENABLE_ON
  #endif
  #if NOTEXIST(INVERT_X_STEP_PIN)
    #error DEPENDENCY ERROR: Missing setting INVERT_X_STEP_PIN
  #endif
  #if NOTEXIST(INVERT_Y_STEP_PIN)
    #error DEPENDENCY ERROR: Missing setting INVERT_Y_STEP_PIN
  #endif
  #if NOTEXIST(INVERT_Z_STEP_PIN)
    #error DEPENDENCY ERROR: Missing setting INVERT_Z_STEP_PIN
  #endif
  #if NOTEXIST(INVERT_E_STEP_PIN)
    #error DEPENDENCY ERROR: Missing setting INVERT_E_STEP_PIN
  #endif
  #if NOTEXIST(INVERT_X_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_X_DIR
  #endif
  #if NOTEXIST(INVERT_Y_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_Y_DIR
  #endif
  #if NOTEXIST(INVERT_Z_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_Z_DIR
  #endif
  #if NOTEXIST(INVERT_E0_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_E0_DIR
  #endif
  #if NOTEXIST(INVERT_E1_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_E1_DIR
  #endif
  #if NOTEXIST(INVERT_E2_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_E2_DIR
  #endif
  #if NOTEXIST(INVERT_E3_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_E3_DIR
  #endif
  #if NOTEXIST(DISABLE_X)
    #error DEPENDENCY ERROR: Missing setting DISABLE_X
  #endif
  #if NOTEXIST(DISABLE_Y)
    #error DEPENDENCY ERROR: Missing setting DISABLE_Y
  #endif
  #if NOTEXIST(DISABLE_Z)
    #error DEPENDENCY ERROR: Missing setting DISABLE_Z
  #endif
  #if NOTEXIST(DISABLE_E)
    #error DEPENDENCY ERROR: Missing setting DISABLE_E
  #endif
  #if NOTEXIST(DISABLE_INACTIVE_EXTRUDER)
    #error DEPENDENCY ERROR: Missing setting DISABLE_INACTIVE_EXTRUDER
  #endif
  #if NOTEXIST(X_MAX_POS)
    #error DEPENDENCY ERROR: Missing setting X_MAX_POS
  #endif
  #if NOTEXIST(X_MIN_POS)
    #error DEPENDENCY ERROR: Missing setting X_MIN_POS
  #endif
  #if NOTEXIST(Y_MAX_POS)
    #error DEPENDENCY ERROR: Missing setting Y_MAX_POS
  #endif
  #if NOTEXIST(Y_MIN_POS)
    #error DEPENDENCY ERROR: Missing setting Y_MIN_POS
  #endif
  #if NOTEXIST(Z_MAX_POS)
    #error DEPENDENCY ERROR: Missing setting Z_MAX_POS
  #endif
  #if NOTEXIST(Z_MIN_POS)
    #error DEPENDENCY ERROR: Missing setting Z_MIN_POS
  #endif
  #if NOTEXIST(E_MIN_POS)
    #error DEPENDENCY ERROR: Missing setting E_MIN_POS
  #endif
  #if NOTEXIST(AXIS_RELATIVE_MODES)
    #error DEPENDENCY ERROR: Missing setting AXIS_RELATIVE_MODES
  #endif
  #if NOTEXIST(DEFAULT_AXIS_STEPS_PER_UNIT)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_AXIS_STEPS_PER_UNIT
  #endif
  #if ENABLED(ULTIPANEL) && NOTEXIST(MANUAL_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting MANUAL_FEEDRATE
  #endif
  #if NOTEXIST(DEFAULT_MINTRAVELFEEDRATE)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_MINTRAVELFEEDRATE
  #endif
  #if NOTEXIST(MINIMUM_PLANNER_SPEED)
    #error DEPENDENCY ERROR: Missing setting MINIMUM_PLANNER_SPEED
  #endif
  #if NOTEXIST(DEFAULT_MAX_ACCELERATION)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_MAX_ACCELERATION
  #endif
  #if NOTEXIST(DEFAULT_RETRACT_ACCELERATION)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_RETRACT_ACCELERATION
  #endif
  #if NOTEXIST(DEFAULT_ACCELERATION)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_ACCELERATION
  #endif
  #if NOTEXIST(DEFAULT_TRAVEL_ACCELERATION)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_TRAVEL_ACCELERATION
  #endif
  #if NOTEXIST(DEFAULT_XYJERK)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_XYJERK
  #endif
  #if NOTEXIST(DEFAULT_ZJERK)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_ZJERK
  #endif
  #if NOTEXIST(HOMING_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting HOMING_FEEDRATE
  #endif
  #if NOTEXIST(X_HOME_BUMP_MM)
    #error DEPENDENCY ERROR: Missing setting X_HOME_BUMP_MM
  #endif
  #if NOTEXIST(Y_HOME_BUMP_MM)
    #error DEPENDENCY ERROR: Missing setting Y_HOME_BUMP_MM
  #endif
  #if NOTEXIST(Z_HOME_BUMP_MM)
    #error DEPENDENCY ERROR: Missing setting Z_HOME_BUMP_MM
  #endif
  #if NOTEXIST(HOMING_BUMP_DIVISOR)
    #error DEPENDENCY ERROR: Missing setting HOMING_BUMP_DIVISOR
  #endif
  #if NOTEXIST(LEFT_PROBE_BED_POSITION)
    #error DEPENDENCY ERROR: Missing setting LEFT_PROBE_BED_POSITION
  #endif
  #if NOTEXIST(RIGHT_PROBE_BED_POSITION)
    #error DEPENDENCY ERROR: Missing setting RIGHT_PROBE_BED_POSITION
  #endif
  #if NOTEXIST(FRONT_PROBE_BED_POSITION)
    #error DEPENDENCY ERROR: Missing setting FRONT_PROBE_BED_POSITION
  #endif
  #if !MECH(DELTA)
    #if NOTEXIST(XY_TRAVEL_SPEED)
      #error DEPENDENCY ERROR: Missing setting XY_TRAVEL_SPEED
    #endif
  #endif
  #if ENABLED(MANUAL_HOME_POSITIONS)
    #if NOTEXIST(MANUAL_X_HOME_POS)
      #error DEPENDENCY ERROR: Missing setting MANUAL_X_HOME_POS
    #endif
    #if NOTEXIST(MANUAL_Y_HOME_POS)
      #error DEPENDENCY ERROR: Missing setting MANUAL_Y_HOME_POS
    #endif
    #if NOTEXIST(MANUAL_Z_HOME_POS)
      #error DEPENDENCY ERROR: Missing setting MANUAL_Z_HOME_POS
    #endif
  #endif
  #if MECH(COREXY) || MECH(COREXZ)
    #if NOTEXIST(COREX_YZ_FACTOR)
      #error DEPENDENCY ERROR: Missing setting COREX_YZ_FACTOR
    #endif
  #endif
  #if MECH(SCARA)
    #if NOTEXIST(LINKAGE_1)
      #error DEPENDENCY ERROR: Missing setting LINKAGE_1
    #endif
    #if NOTEXIST(LINKAGE_2)
      #error DEPENDENCY ERROR: Missing setting LINKAGE_2
    #endif
    #if NOTEXIST(SCARA_OFFSET_X)
      #error DEPENDENCY ERROR: Missing setting SCARA_OFFSET_X
    #endif
    #if NOTEXIST(SCARA_OFFSET_Y)
      #error DEPENDENCY ERROR: Missing setting SCARA_OFFSET_Y
    #endif
    #if NOTEXIST(SCARA_RAD2DEG)
      #error DEPENDENCY ERROR: Missing setting SCARA_RAD2DEG
    #endif
    #if NOTEXIST(THETA_HOMING_OFFSET)
      #error DEPENDENCY ERROR: Missing setting THETA_HOMING_OFFSET
    #endif
    #if NOTEXIST(PSI_HOMING_OFFSET)
      #error DEPENDENCY ERROR: Missing setting PSI_HOMING_OFFSET
    #endif
  #endif

  #if MECH(DELTA)
    #if NOTEXIST(DEFAULT_DELTA_DIAGONAL_ROD)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_DELTA_DIAGONAL_ROD
    #endif
    #if NOTEXIST(DELTA_SMOOTH_ROD_OFFSET)
      #error DEPENDENCY ERROR: Missing setting DELTA_SMOOTH_ROD_OFFSET
    #endif
    #if NOTEXIST(DELTA_CARRIAGE_OFFSET)
      #error DEPENDENCY ERROR: Missing setting DELTA_CARRIAGE_OFFSET
    #endif
    #if NOTEXIST(PRINTER_RADIUS)
      #error DEPENDENCY ERROR: Missing setting PRINTER_RADIUS
    #endif
    #if NOTEXIST(DEFAULT_DELTA_RADIUS)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_DELTA_RADIUS
    #endif
    #if NOTEXIST(AUTOCAL_TRAVELRATE)
      #error DEPENDENCY ERROR: Missing setting AUTOCAL_TRAVELRATE
    #endif
    #if NOTEXIST(AUTOCAL_PROBERATE)
      #error DEPENDENCY ERROR: Missing setting AUTOCAL_PROBERATE
    #endif
    #if NOTEXIST(AUTOCALIBRATION_PRECISION)
      #error DEPENDENCY ERROR: Missing setting AUTOCALIBRATION_PRECISION
    #endif
    #if NOTEXIST(TOWER_A_ENDSTOP_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_A_ENDSTOP_ADJ
    #endif
    #if NOTEXIST(TOWER_B_ENDSTOP_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_B_ENDSTOP_ADJ
    #endif
    #if NOTEXIST(TOWER_C_ENDSTOP_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_C_ENDSTOP_ADJ
    #endif
    #if NOTEXIST(TOWER_A_POSITION_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_A_POSITION_ADJ
    #endif
    #if NOTEXIST(TOWER_B_POSITION_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_B_POSITION_ADJ
    #endif
    #if NOTEXIST(TOWER_C_POSITION_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_C_POSITION_ADJ
    #endif
    #if NOTEXIST(TOWER_A_RADIUS_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_A_RADIUS_ADJ
    #endif
    #if NOTEXIST(TOWER_B_RADIUS_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_B_RADIUS_ADJ
    #endif
    #if NOTEXIST(TOWER_C_RADIUS_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_C_RADIUS_ADJ
    #endif
    #if NOTEXIST(TOWER_A_DIAGROD_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_A_DIAGROD_ADJ
    #endif
    #if NOTEXIST(TOWER_B_DIAGROD_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_B_DIAGROD_ADJ
    #endif
    #if NOTEXIST(TOWER_C_DIAGROD_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_C_DIAGROD_ADJ
    #endif
    #if NOTEXIST(Z_PROBE_OFFSET)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_OFFSET
    #endif
    #if NOTEXIST(Z_PROBE_DEPLOY_START_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_DEPLOY_START_LOCATION
    #endif
    #if NOTEXIST(Z_PROBE_DEPLOY_END_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_DEPLOY_END_LOCATION
    #endif
    #if NOTEXIST(Z_PROBE_RETRACT_START_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_RETRACT_START_LOCATION
    #endif
    #if NOTEXIST(Z_PROBE_RETRACT_END_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_RETRACT_END_LOCATION
    #endif
    #if NOTEXIST(Z_RAISE_BETWEEN_PROBINGS)
      #error DEPENDENCY ERROR: Missing setting Z_RAISE_BETWEEN_PROBINGS
    #endif
    #if NOTEXIST(AUTO_BED_LEVELING_GRID_POINTS)
      #error DEPENDENCY ERROR: Missing setting AUTO_BED_LEVELING_GRID_POINTS
    #endif
  #endif
  
  /**
   * Board
   */
  #if NOTEXIST(KNOWN_BOARD)
    #error DEPENDENCY ERROR: You have to set a valid MOTHERBOARD.
  #endif
  
  /**
   * Mechanics
   */
  #if NOTEXIST(KNOWN_MECH)
    #error DEPENDENCY ERROR: You have to set a valid MECHANICS.
  #endif

  /**
   * Dual Stepper Drivers
   */
  #if ENABLED(Z_DUAL_STEPPER_DRIVERS) && ENABLED(Y_DUAL_STEPPER_DRIVERS)
    #error CONFLICT ERROR: You cannot have dual stepper drivers for both Y and Z.
  #endif

  /**
   * Progress Bar
   */
  #if ENABLED(LCD_PROGRESS_BAR)
    #if DISABLED(SDSUPPORT)
      #error DEPENDENCY ERROR: LCD_PROGRESS_BAR requires SDSUPPORT.
    #endif
    #if ENABLED(DOGLCD)
      #error CONFLICT ERROR: LCD_PROGRESS_BAR does not apply to graphical displays.
    #endif
    #if ENABLED(FILAMENT_LCD_DISPLAY)
      #error CONFLICT ERROR: LCD_PROGRESS_BAR and FILAMENT_LCD_DISPLAY are not fully compatible.
    #endif
    #if ENABLED(POWER_CONSUMPTION_LCD_DISPLAY)
      #error CONFLICT ERROR: LCD_PROGRESS_BAR and POWER_CONSUMPTION_LCD_DISPLAY are not fully compatible.
    #endif
  #endif

  /**
   * Babystepping
   */
  #if ENABLED(BABYSTEPPING)
    #if MECH(COREXY) && ENABLED(BABYSTEP_XY)
      #error CONFLICT ERROR: BABYSTEPPING only implemented for Z axis on CoreXY.
    #endif
    #if MECH(SCARA)
      #error CONFLICT ERROR: BABYSTEPPING is not implemented for SCARA yet.
    #endif
    #if MECH(DELTA) && ENABLED(BABYSTEP_XY)
      #error CONFLICT ERROR: BABYSTEPPING only implemented for Z axis on deltabots.
    #endif
  #endif

  /**
   * Extruder Runout Prevention
   */
  #if DISABLED(PREVENT_DANGEROUS_EXTRUDE) && ENABLED(EXTRUDER_RUNOUT_PREVENT)
    #error DEPENDENCY ERROR: EXTRUDER_RUNOUT_PREVENT needs PREVENT_DANGEROUS_EXTRUDE
  #endif
  #if ENABLED(EXTRUDER_RUNOUT_PREVENT) && EXTRUDER_RUNOUT_MINTEMP < EXTRUDE_MINTEMP
    #error CONFLICT ERROR: EXTRUDER_RUNOUT_MINTEMP have to be greater than EXTRUDE_MINTEMP
  #endif

  /**
   * Idle oozing prevent with Extruder Runout Prevention
   */
  #if ENABLED(EXTRUDER_RUNOUT_PREVENT) && ENABLED(IDLE_OOZING_PREVENT)
    #error CONFLICT ERROR: EXTRUDER_RUNOUT_PREVENT and IDLE_OOZING_PREVENT are incopatible. Please comment one of them.
  #endif

  /**
   * Idle oozing prevent
   */
  #if DISABLED(PREVENT_DANGEROUS_EXTRUDE) && ENABLED(IDLE_OOZING_PREVENT)
    #error DEPENDENCY ERROR: IDLE_OOZING_MINTEMP needs PREVENT_DANGEROUS_EXTRUDE
  #endif
  #if ENABLED(IDLE_OOZING_PREVENT) && IDLE_OOZING_MINTEMP < EXTRUDE_MINTEMP
    #error CONFLICT ERROR: IDLE_OOZING_MINTEMP have to be greater than EXTRUDE_MINTEMP
  #endif

  /**
   * Options only for EXTRUDERS == 1
   */
  #if EXTRUDERS > 1

    #if ENABLED(TEMP_SENSOR_1_AS_REDUNDANT)
      #error CONFLICT ERROR: EXTRUDERS must be 1 with TEMP_SENSOR_1_AS_REDUNDANT.
    #endif

    #if ENABLED(HEATERS_PARALLEL)
      #error CONFLICT ERROR: EXTRUDERS must be 1 with HEATERS_PARALLEL.
    #endif

  #endif // EXTRUDERS > 1

  /**
   * Limited number of servos
   */
  #if NUM_SERVOS > 4
    #error CONFLICT ERROR: The maximum number of SERVOS in Marlin is 4.
  #endif
  #if ENABLED(ENABLE_SERVOS)
    #if NUM_SERVOS < 1
      #error CONFLICT ERROR: NUM_SERVOS has to be at least one if you enable ENABLE_SERVOS
    #endif
    #if X_ENDSTOP_SERVO_NR >= 0 || Y_ENDSTOP_SERVO_NR >= 0 || Z_ENDSTOP_SERVO_NR >= 0
      #if X_ENDSTOP_SERVO_NR >= NUM_SERVOS
        #error CONFLICT ERROR: X_ENDSTOP_SERVO_NR must be smaller than NUM_SERVOS.
      #elif Y_ENDSTOP_SERVO_NR >= NUM_SERVOS
        #error CONFLICT ERROR: Y_ENDSTOP_SERVO_NR must be smaller than NUM_SERVOS.
      #elif Z_ENDSTOP_SERVO_NR >= NUM_SERVOS
        #error CONFLICT ERROR: Z_ENDSTOP_SERVO_NR must be smaller than NUM_SERVOS.
      #endif
    #endif
  #endif

  /**
   * Servo deactivation depends on servo endstops
   */
  #if ENABLED(DEACTIVATE_SERVOS_AFTER_MOVE) && HASNT(SERVO_ENDSTOPS)
    #error DEPENDENCY ERROR: At least one of the ?_ENDSTOP_SERVO_NR is required for DEACTIVATE_SERVOS_AFTER_MOVE.
  #endif

  /**
   * Required LCD language
   */
  #if DISABLED(DOGLCD) && ENABLED(ULTRA_LCD) && DISABLED(DISPLAY_CHARSET_HD44780_JAPAN) && DISABLED(DISPLAY_CHARSET_HD44780_WESTERN) && DISABLED(DISPLAY_CHARSET_HD44780_CYRILLIC)
    #error DEPENDENCY ERROR: You must enable either DISPLAY_CHARSET_HD44780_JAPAN or DISPLAY_CHARSET_HD44780_WESTERN  or DISPLAY_CHARSET_HD44780_CYRILLIC for your LCD controller.
  #endif

  /**
   * Required LCD for FILAMENTCHANGEENABLE
   */
  #if ENABLED(FILAMENTCHANGEENABLE) && DISABLED(ULTRA_LCD)
    #error DEPENDENCY ERROR: You must have LCD in order to use FILAMENTCHANGEENABLE
  #endif

  /**
   * Auto Bed Leveling
   */
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)

    /**
     * Require a Z Min pin
     */
    #if Z_MIN_PIN == -1
      #if Z_PROBE_PIN == -1 || DISABLED(Z_PROBE_ENDSTOP) // It's possible for someone to set a pin for the Z Probe, but not enable it.
        #if ENABLED(Z_PROBE_REPEATABILITY_TEST)
          #error DEPENDENCY ERROR: You must have a Z_MIN or Z_PROBE endstop to enable Z_PROBE_REPEATABILITY_TEST.
        #else
          #error DEPENDENCY ERROR: AUTO_BED_LEVELING_FEATURE requires a Z_MIN or Z_PROBE endstop. Z_MIN_PIN or Z_PROBE_PIN must point to a valid hardware pin.
        #endif
      #endif
    #endif

    /**
     * Require a Z Probe Pin if Z_PROBE_ENDSTOP is enabled.
     */
    #if ENABLED(Z_PROBE_ENDSTOP)
      #if !PIN_EXISTS(Z_PROBE)
        #error DEPENDENCY ERROR: You must set Z_PROBE_PIN to a valid pin if you enable Z_PROBE_ENDSTOP
      #endif
      #if DISABLED(ENABLE_SERVOS)
        #error DEPENDENCY ERROR: You must enable ENABLE_SERVOS and must have NUM_SERVOS EXIST and there must be at least 1 configured to use Z_PROBE_ENDSTOP.
      #endif
      #if NUM_SERVOS < 1
        #error DEPENDENCY ERROR: You must have at least 1 servo EXIST for NUM_SERVOS to use Z_PROBE_ENDSTOP.
      #endif
      #if Z_ENDSTOP_SERVO_NR < 0
        #error DEPENDENCY ERROR: You must have Z_ENDSTOP_SERVO_NR set to at least 0 or above to use Z_PROBE_ENDSTOP.
      #endif
      #if NOTEXIST(SERVO_ENDSTOP_ANGLES)
        #error DEPENDENCY ERROR: You must have SERVO_ENDSTOP_ANGLES EXIST for Z Extend and Retract to use Z_PROBE_ENDSTOP.
      #endif
    #endif
    /**
     * Check if Probe_Offset * Grid Points is greater than Probing Range
     */
    #if ENABLED(AUTO_BED_LEVELING_GRID)
      // Be sure points are in the right order
      #if LEFT_PROBE_BED_POSITION > RIGHT_PROBE_BED_POSITION
        #error CONFLICT ERROR: LEFT_PROBE_BED_POSITION must be less than RIGHT_PROBE_BED_POSITION.
      #elif FRONT_PROBE_BED_POSITION > BACK_PROBE_BED_POSITION
        #error CONFLICT ERROR: BACK_PROBE_BED_POSITION must be less than FRONT_PROBE_BED_POSITION.
      #endif
      // Make sure probing points are reachable
      #if LEFT_PROBE_BED_POSITION < MIN_PROBE_X
        #error CONFLICT ERROR: "The given LEFT_PROBE_BED_POSITION can't be reached by the probe."
      #elif RIGHT_PROBE_BED_POSITION > MAX_PROBE_X
        #error CONFLICT ERROR: "The given RIGHT_PROBE_BED_POSITION can't be reached by the probe."
      #elif FRONT_PROBE_BED_POSITION < MIN_PROBE_Y
        #error CONFLICT ERROR: "The given FRONT_PROBE_BED_POSITION can't be reached by the probe."
      #elif BACK_PROBE_BED_POSITION > MAX_PROBE_Y
        #error CONFLICT ERROR: "The given BACK_PROBE_BED_POSITION can't be reached by the probe."
      #endif

    #else // !AUTO_BED_LEVELING_GRID

      // Check the triangulation points
      #if ABL_PROBE_PT_1_X < MIN_PROBE_X || ABL_PROBE_PT_1_X > MAX_PROBE_X
        #error CONFLICT ERROR: "The given ABL_PROBE_PT_1_X can't be reached by the probe."
      #elif ABL_PROBE_PT_2_X < MIN_PROBE_X || ABL_PROBE_PT_2_X > MAX_PROBE_X
        #error CONFLICT ERROR: "The given ABL_PROBE_PT_2_X can't be reached by the probe."
      #elif ABL_PROBE_PT_3_X < MIN_PROBE_X || ABL_PROBE_PT_3_X > MAX_PROBE_X
        #error CONFLICT ERROR: "The given ABL_PROBE_PT_3_X can't be reached by the probe."
      #elif ABL_PROBE_PT_1_Y < MIN_PROBE_Y || ABL_PROBE_PT_1_Y > MAX_PROBE_Y
        #error CONFLICT ERROR: "The given ABL_PROBE_PT_1_Y can't be reached by the probe."
      #elif ABL_PROBE_PT_2_Y < MIN_PROBE_Y || ABL_PROBE_PT_2_Y > MAX_PROBE_Y
        #error CONFLICT ERROR: "The given ABL_PROBE_PT_2_Y can't be reached by the probe."
      #elif ABL_PROBE_PT_3_Y < MIN_PROBE_Y || ABL_PROBE_PT_3_Y > MAX_PROBE_Y
        #error CONFLICT ERROR: "The given ABL_PROBE_PT_3_Y can't be reached by the probe."
      #endif

    #endif // !AUTO_BED_LEVELING_GRID

  #endif // AUTO_BED_LEVELING_FEATURE

  /**
   * ULTIPANEL encoder
   */
  #if ENABLED(ULTIPANEL) && DISABLED(NEWPANEL) && DISABLED(SR_LCD_2W_NL) && DISABLED(SHIFT_CLK)
    #error DEPENDENCY ERROR: ULTIPANEL requires some kind of encoder.
  #endif

  /**
   * Delta & Z_PROBE_ENDSTOP
   */
  #if MECH(DELTA) && ENABLED(Z_PROBE_ENDSTOP)
    #if PIN_EXISTS(Z_PROBE)
      #error DEPENDENCY ERROR: You must set Z_PROBE_PIN to a valid pin if you enable Z_PROBE_ENDSTOP
    #endif
  #endif

  /**
   * Dual X Carriage requirements
   */
  #if ENABLED(DUAL_X_CARRIAGE)
    #if EXTRUDERS == 1 || MECH(COREXY) \
        || HASNT(X2_ENABLE) || HASNT(X2_STEP) || HASNT(X2_DIR) \
        || NOTEXIST(X2_HOME_POS) || NOTEXIST(X2_MIN_POS) || NOTEXIST(X2_MAX_POS) \
        || HASNT(X_MAX)
      #error DEPENDENCY ERROR: Missing or invalid definitions for DUAL_X_CARRIAGE mode.
    #endif
    #if X_HOME_DIR != -1 || X2_HOME_DIR != 1
      #error CONFLICT ERROR: Please use canonical x-carriage assignment.
    #endif
  #endif // DUAL_X_CARRIAGE

  /**
   * Make sure auto fan pins don't conflict with the fan pin
   */
  #if HAS(AUTO_FAN) && HAS(FAN)
    #if EXTRUDER_0_AUTO_FAN_PIN == FAN_PIN
      #error CONFLICT ERROR: You cannot set EXTRUDER_0_AUTO_FAN_PIN equal to FAN_PIN.
    #elif EXTRUDER_1_AUTO_FAN_PIN == FAN_PIN
      #error CONFLICT ERROR: You cannot set EXTRUDER_1_AUTO_FAN_PIN equal to FAN_PIN.
    #elif EXTRUDER_2_AUTO_FAN_PIN == FAN_PIN
      #error CONFLICT ERROR: You cannot set EXTRUDER_2_AUTO_FAN_PIN equal to FAN_PIN.
    #elif EXTRUDER_3_AUTO_FAN_PIN == FAN_PIN
      #error CONFLICT ERROR: You cannot set EXTRUDER_3_AUTO_FAN_PIN equal to FAN_PIN.
    #endif
  #endif

  #if HAS(FAN) && CONTROLLERFAN_PIN == FAN_PIN
    #error CONFLICT ERROR: You cannot set CONTROLLERFAN_PIN equal to FAN_PIN.
  #endif

  /**
   * Test required HEATER defines
   */
  #if HOTENDS > 3
    #if HASNT(HEATER_3)
      #error DEPENDENCY ERROR: HEATER_3_PIN not EXIST for this board
    #endif
  #elif HOTENDS > 2
    #if HASNT(HEATER_2)
      #error DEPENDENCY ERROR: HEATER_2_PIN not EXIST for this board
    #endif
  #elif HOTENDS > 1 || ENABLED(HEATERS_PARALLEL)
    #if HASNT(HEATER_1)
      #error DEPENDENCY ERROR: HEATER_1_PIN not EXIST for this board
    #endif
  #elif HOTENDS > 0
    #if HASNT(HEATER_0)
      #error DEPENDENCY ERROR: HEATER_0_PIN not EXIST for this board
    #endif
  #endif
  
  #if DISABLED(SDSUPPORT) && ENABLED(SD_SETTINGS)
    #error DEPENDENCY ERROR: You have to enable SDSUPPORT to use SD_SETTINGS
  #endif

  #if MECH(COREXZ) && ENABLED(Z_LATE_ENABLE)
    #error CONFLICT ERROR: "Z_LATE_ENABLE can't be used with COREXZ."
  #endif

  #if ENABLED(POWER_CONSUMPTION) && !PIN_EXISTS(POWER_CONSUMPTION)
    #error DEPENDENCY ERROR: You have to set POWER_CONSUMPTION_PIN to a valid pin if you enable POWER_CONSUMPTION
  #endif
  
  #if ENABLED(CHDK) || ENABLED(PHOTOGRAPH)
    #error CONFLICT ERROR: CHDK and PHOTOGRAPH are incompatible.
  #endif
  
  
  #if ENABLED(MKR4)
    #if (EXTRUDERS == 2) && (DRIVER_EXTRUDERS == 1) && !PIN_EXISTS(E0E1_CHOICE)
      #error DEPENDENCY ERROR: You have to set E0E1_CHOICE_PIN to a valid pin if you enable MKR4 with 2 extruder and 1 driver
    #elif (EXTRUDERS == 3) && (DRIVER_EXTRUDERS == 1) && (!PIN_EXISTS(E0E1_CHOICE) || !PIN_EXISTS(E0E2_CHOICE))
      #error DEPENDENCY ERROR: You have to set E0E1_CHOICE_PIN and E0E2_CHOICE_PIN to a valid pin if you enable MKR4 with 3 extruder and 1 driver
    #elif (EXTRUDERS == 4) && (DRIVER_EXTRUDERS == 1) && (!PIN_EXISTS(E0E1_CHOICE) || !PIN_EXISTS(E0E2_CHOICE) || !PIN_EXISTS(E0E3_CHOICE))
      #error DEPENDENCY ERROR: You have to set E0E1_CHOICE_PIN, E0E2_CHOICE_PIN and E0E3_CHOICE_PIN to a valid pin if you enable MKR4 with 4 extruder and 1 driver
    #elif (EXTRUDERS == 3) && (DRIVER_EXTRUDERS == 2) && !PIN_EXISTS(E0E2_CHOICE)
      #error DEPENDENCY ERROR: You have to set E0E2_CHOICE_PIN to a valid pin if you enable MKR4 with 3 extruder and 2 driver
    #elif (EXTRUDERS == 4) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(E0E1_CHOICE) || !PIN_EXISTS(E1E3_CHOICE))
      #error DEPENDENCY ERROR: You have to set E0E2_CHOICE_PIN and E1E3_CHOICE_PIN to a valid pin if you enable MKR4 with 4 extruder and 2 driver
    #endif 
  #endif

  #if ENABLED(NPR2) && !PIN_EXISTS(E_MIN)
    #error DEPENDENCY ERROR: You have to set E_MIN_PIN to a valid pin if you enable NPR2
  #endif

  #if ENABLED(LASERBEAM) && (!PIN_EXISTS(LASER_PWR) ||  !PIN_EXISTS(LASER_TTL)) 
    #error DEPENDENCY ERROR: You have to set LASER_PWR_PIN and LASER_TTL_PIN to a valid pin if you enable LASERBEAM
  #endif

  #if ENABLED(FILAMENT_RUNOUT_SENSOR) && !PIN_EXISTS(FILRUNOUT)
    #error DEPENDENCY ERROR: You have to set FILRUNOUT_PIN to a valid pin if you enable FILAMENT_RUNOUT_SENSOR
  #endif

  #if ENABLED(FILAMENT_SENSOR) && !PIN_EXISTS(FILWIDTH)
    #error DEPENDENCY ERROR: You have to set FILWIDTH_PIN to a valid pin if you enable FILAMENT_SENSOR
  #endif

  #if ENABLED(POWER_CONSUMPTION) && !PIN_EXISTS(POWER_CONSUMPTION)
    #error DEPENDENCY ERROR: You have to set POWER_CONSUMPTION_PIN to a valid pin if you enable POWER_CONSUMPTION
  #endif

  #if ENABLED(PHOTOGRAPH) && !PIN_EXISTS(PHOTOGRAPH)
    #error DEPENDENCY ERROR: You have to set PHOTOGRAPH_PIN to a valid pin if you enable PHOTOGRAPH
  #endif

  #if ENABLED(CHDK) && !PIN_EXISTS(CHDK)
    #error DEPENDENCY ERROR: You have to set CHDK_PIN to a valid pin if you enable CHDK
  #endif

  #if ENABLED(CONTROLLERFAN) && !PIN_EXISTS(CONTROLLERFAN)
    #error DEPENDENCY ERROR: You have to set CONTROLLERFAN_PIN to a valid pin if you enable CONTROLLERFAN
  #endif

  #if ENABLED(EXTRUDER_AUTO_FAN) && !PIN_EXISTS(EXTRUDER_0_AUTO_FAN) && !PIN_EXISTS(EXTRUDER_1_AUTO_FAN) && !PIN_EXISTS(EXTRUDER_2_AUTO_FAN) && !PIN_EXISTS(EXTRUDER_3_AUTO_FAN)
    #error DEPENDENCY ERROR: You have to set at least one EXTRUDER_?_AUTO_FAN_PIN to a valid pin if you enable EXTRUDER_AUTO_FAN
  #endif

  #if ENABLED(X2_IS_TMC) && (!PIN_EXISTS(X2_ENABLE) || !PIN_EXISTS(X2_STEP) || !PIN_EXISTS(X2_DIR))
    #error DEPENDENCY ERROR: You have to set X2_ENABLE_PIN, X2_STEP_PIN and X2_DIR_PIN to a valid pin if you enable X2_IS_TMC
  #endif
    
  #if ((ENABLED(ENABLE_SERVOS) && NUM_SERVOS > 0) && !(HAS_SERVO_0 || HAS_SERVO_1 || HAS_SERVO_2 || HAS_SERVO_3))
    #error DEPENDENCY ERROR: You have to set at least one SERVO?_PIN to a valid pin if you enable ENABLE_SERVOS
  #endif
  
  #if ENABLED(Z_PROBE_SLED) && !PIN_EXISTS(SLED)
    #error DEPENDENCY ERROR: You have to set SLED_PIN to a valid pin if you enable Z_PROBE_SLED
  #endif

#endif //SANITYCHECK_H
