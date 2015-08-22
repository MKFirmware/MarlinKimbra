#ifndef DEPENDENCY_H
#define DEPENDENCY_H

/*
 * This file check if some essential options aren't been defined.
 * This file is usefult to see dependency between features while developing.
 */
 
// Serial comunication
#if !defined(SERIAL_PORT)
  #error DEPENDENCY ERROR: Missing setting SERIAL_PORT
#endif
#if !defined(BAUDRATE)
  #error DEPENDENCY ERROR: Missing setting BAUDRATE
#endif
#if !defined(MACHINE_UUID)
	#error DEPENDENCY ERROR: Missing setting MACHINE_UUID
#endif

//board
#if !defined(MOTHERBOARD)
  #error DEPENDENCY ERROR: Missing setting MOTHERBOARD
#endif

//Mechanism
#if !defined(MEACHANISM)
  #error DEPENDENCY ERROR: Missing setting MECHANISM
#endif

//Power supply
#if !defined(POWER_SUPPLY)
  #error DEPENDENCY ERROR: Missing setting POWER_SUPPLY
#endif

//Thermistor
#if !defined(TEMP_SENSOR_0)
  #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_0
#endif
#if !defined(TEMP_SENSOR_1)
  #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_1
#endif
#if !defined(TEMP_SENSOR_2)
  #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_2
#endif
#if !defined(TEMP_SENSOR_3)
  #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_3
#endif
#if !defined(TEMP_SENSOR_BED)
  #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_BED
#endif
#if (THERMISTORHEATER_0 == 998) || (THERMISTORHEATER_1 == 998) || (THERMISTORHEATER_2 == 998) || (THERMISTORHEATER_3 == 998) || (THERMISTORBED == 998) //User defined table
  #if !defined(DUMMY_THERMISTOR_998_VALUE)
    #error DEPENDENCY ERROR: Missing setting DUMMY_THERMISTOR_998_VALUE
  #endif
#endif

//Temperature
/**
 * Temperature defines
 */
#if defined(TEMP_RESIDENCY_TIME)
  #if !defined(TEMP_HYSTERESIS)
    #error DEPENDENCY ERROR: Missing setting TEMP_HYSTERESIS
  #endif
  #if !defined(TEMP_WINDOW)
    #error DEPENDENCY ERROR: Missing setting TEMP_WINDOW
  #endif
#endif
#if TEMP_SENSOR_0 != 0
  #if !defined(HEATER_0_MAXTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_0_MAXTEMP
  #endif
  #if !defined(HEATER_0_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_0_MINTEMP
  #endif
#endif
#if TEMP_SENSOR_1 != 0
  #if !defined(HEATER_1_MAXTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_1_MAXTEMP
  #endif
  #if !defined(HEATER_0_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_1_MINTEMP
  #endif
#endif
#if TEMP_SENSOR_2 != 0
  #if !defined(HEATER_2_MAXTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_2_MAXTEMP
  #endif
  #if !defined(HEATER_0_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_2_MINTEMP
  #endif
#endif
#if TEMP_SENSOR_3 != 0
  #if !defined(HEATER_3_MAXTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_3_MAXTEMP
  #endif
  #if !defined(HEATER_0_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting HEATER_3_MINTEMP
  #endif
#endif
#if TEMP_SENSOR_BED != 0
  #if !defined(BED_MAXTEMP)
    #error DEPENDENCY ERROR: Missing setting BED_MAXTEMP
  #endif
  #if !defined(HEATER_0_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting BED_MINTEMP
  #endif
#endif
#if !defined(PLA_PREHEAT_HOTEND_TEMP)
  #error DEPENDENCY ERROR: Missing setting PLA_PREHEAT_HOTEND_TEMP
#endif
#if !defined(PLA_PREHEAT_HPB_TEMP)
  #error DEPENDENCY ERROR: Missing setting PLA_PREHEAT_HPB_TEMP
#endif
#if !defined(PLA_PREHEAT_FAN_SPEED)
  #error DEPENDENCY ERROR: Missing setting PLA_PREHEAT_FAN_SPEED
#endif
#if !defined(ABS_PREHEAT_HOTEND_TEMP)
  #error DEPENDENCY ERROR: Missing setting ABS_PREHEAT_HOTEND_TEMP
#endif
#if !defined(ABS_PREHEAT_HPB_TEMP)
  #error DEPENDENCY ERROR: Missing setting ABS_PREHEAT_HPB_TEMP
#endif
#if !defined(ABS_PREHEAT_FAN_SPEED)
  #error DEPENDENCY ERROR: Missing setting ABS_PREHEAT_FAN_SPEED
#endif
#if !defined(GUM_PREHEAT_HOTEND_TEMP)
  #error DEPENDENCY ERROR: Missing setting GUM_PREHEAT_HOTEND_TEMP
#endif
#if !defined(GUM_PREHEAT_HPB_TEMP)
  #error DEPENDENCY ERROR: Missing setting GUM_PREHEAT_HPB_TEMP
#endif
#if !defined(GUM_PREHEAT_FAN_SPEED)
  #error DEPENDENCY ERROR: Missing setting GUM_PREHEAT_FAN_SPEED
#endif

//extruders
#if !defined(EXTRUDERS)
  #error DEPENDENCY ERROR: Missing setting EXTRUDERS
#endif
#if !defined(DRIVER_EXTRUDERS)
  #error DEPENDENCY ERROR: Missing setting DRIVER_EXTRUDERS
#endif

//Language
#if !defined(LANGUAGE_CHOICE)
  #error DEPENDENCY ERROR: Missing setting LANGUAGE_CHOICE
#endif

///FEATURE

//Temperature
#if !defined(PID_MAX)
  #error DEPENDENCY ERROR: Missing setting PID_MAX
#endif
#if !defined(MAX_BED_POWER)
  #error DEPENDENCY ERROR: Missing setting MAX_BED_POWER
#endif
#if defined(PIDTEMP) || defined(PIDTEMPBED)
  #if !defined(MAX_OVERSHOOT_PID_AUTOTUNE)
    #error DEPENDENCY ERROR: Missing setting MAX_OVERSHOOT_PID_AUTOTUNE
  #endif
#endif
#if defined(PIDTEMP)
  #if !defined(PID_OPENLOOP)
    #error DEPENDENCY ERROR: Missing setting PID_FUNCTIONAL_RANGE
  #endif
  #if !defined(PID_INTEGRAL_DRIVE_MAX)
    #error DEPENDENCY ERROR: Missing setting PID_INTEGRAL_DRIVE_MAX
  #endif
  #if !defined(DEFAULT_Kp)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_Kp
  #endif
  #if !defined(DEFAULT_Ki)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_Ki
  #endif
  #if !defined(DEFAULT_Kd)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_Kd
  #endif
#endif
#if defined(PIDTEMPBED)
  #if !defined(PID_BED_INTEGRAL_DRIVE_MAX)
    #error DEPENDENCY ERROR: Missing setting PID_BED_INTEGRAL_DRIVE_MAX
  #endif
  #if !defined(DEFAULT_bedKp)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_bedKp
  #endif
  #if !defined(DEFAULT_bedKi)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_bedKi
  #endif
  #if !defined(DEFAULT_bedKd)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_bedKd
  #endif
#endif
#if defined(BED_LIMIT_SWITCHING)
  #if !defined(BED_HYSTERESIS)
    #error DEPENDENCY ERROR: Missing setting BED_HYSTERESIS
  #endif
  #if !defined(BED_CHECK_INTERVAL)
    #error DEPENDENCY ERROR: Missing setting BED_CHECK_INTERVAL
  #endif
#endif
#if defined(THERMAL_PROTECTION_HOTENDS)
  #if !defined(THERMAL_PROTECTION_PERIOD)
    #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_PERIOD
  #endif
  #if !defined(THERMAL_PROTECTION_HYSTERESIS)
    #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_HYSTERESIS
  #endif
  #if !defined(WATCH_TEMP_PERIOD)
    #error DEPENDENCY ERROR: Missing setting WATCH_TEMP_PERIOD
  #endif
  #if !defined(WATCH_TEMP_INCREASE)
    #error DEPENDENCY ERROR: Missing setting WATCH_TEMP_INCREASE
  #endif
#endif
#if defined(THERMAL_PROTECTION_BED)
  #if !defined(THERMAL_PROTECTION_BED_PERIOD)
    #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_BED_PERIOD
  #endif
  #if !defined(THERMAL_PROTECTION_BED_HYSTERESIS)
    #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_BED_HYSTERESIS
  #endif
#endif
//fan
#if !defined(SOFT_PWM_SCALE)
  #error DEPENDENCY ERROR: Missing setting SOFT_PWM_SCALE
#endif

#if defined(CONTROLLERFAN)
  #if !defined(CONTROLLERFAN_SECS)
    #error DEPENDENCY ERROR: Missing setting CONTROLLERFAN_SECS
  #endif
  #if !defined(CONTROLLERFAN_SPEED)
    #error DEPENDENCY ERROR: Missing setting CONTROLLERFAN_SPEED
  #endif
  #if !defined(CONTROLLERFAN_MIN_SPEED)
    #error DEPENDENCY ERROR: Missing setting CONTROLLERFAN_MIN_SPEED
  #endif
#endif

#if defined(EXTRUDER_AUTO_FAN)
  #if !defined(EXTRUDER_AUTO_FAN_TEMPERATURE)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_AUTO_FAN_TEMPERATURE
  #endif
  #if !defined(EXTRUDER_AUTO_FAN_SPEED)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_AUTO_FAN_SPEED
  #endif
  #if !defined(EXTRUDER_AUTO_FAN_MIN_SPEED)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_AUTO_FAN_MIN_SPEED
  #endif
#endif

//extruder
#if defined(PREVENT_DANGEROUS_EXTRUDE)
  #if !defined(EXTRUDE_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting EXTRUDE_MINTEMP
  #endif
  #if defined(PREVENT_LENGTHY_EXTRUDE)
    #if !defined(EXTRUDE_MAXLENGTH)
      #error DEPENDENCY ERROR: Missing setting EXTRUDE_MAXLENGTH
    #endif
  #endif
#endif

#if defined(NPR2)
  #if !defined(COLOR_STEP)
    #error DEPENDENCY ERROR: Missing setting COLOR_STEP
  #endif
  #if !defined(COLOR_SLOWRATE)
    #error DEPENDENCY ERROR: Missing setting COLOR_SLOWRATE
  #endif
  #if !defined(COLOR_HOMERATE)
    #error DEPENDENCY ERROR: Missing setting COLOR_HOMERATE
  #endif
  #if !defined(MOTOR_ANGLE)
    #error DEPENDENCY ERROR: Missing setting MOTOR_ANGLE
  #endif
  #if !defined(DRIVER_MICROSTEP)
    #error DEPENDENCY ERROR: Missing setting DRIVER_MICROSTEP
  #endif
  #if !defined(CARTER_MOLTIPLICATOR)
    #error DEPENDENCY ERROR: Missing setting CARTER_MOLTIPLICATOR
  #endif
#endif

#if defined(IDLE_OOZING_PREVENT)
  #if !defined(IDLE_OOZING_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_MINTEMP
  #endif
  #if !defined(IDLE_OOZING_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_FEEDRATE
  #endif
  #if !defined(IDLE_OOZING_SECONDS)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_SECONDS
  #endif
  #if !defined(IDLE_OOZING_LENGTH)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_LENGTH
  #endif
  #if !defined(IDLE_OOZING_RECOVER_LENGTH)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_RECOVER_LENGTH
  #endif
  #if !defined(IDLE_OOZING_RECOVER_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_RECOVER_FEEDRATE
  #endif
#endif

#if defined(EXTRUDER_RUNOUT_PREVENT)
  #if !defined(EXTRUDER_RUNOUT_MINTEMP)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_MINTEMP
  #endif
  #if !defined(EXTRUDER_RUNOUT_SECONDS)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_SECONDS
  #endif
  #if !defined(EXTRUDER_RUNOUT_ESTEPS)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_ESTEPS
  #endif
  #if !defined(EXTRUDER_RUNOUT_SPEED)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_SPEED
  #endif
  #if !defined(EXTRUDER_RUNOUT_EXTRUDE)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_EXTRUDE
  #endif
#endif

#if defined(EASY_LOAD)
  #if !defined(BOWDEN_LENGTH)
    #error DEPENDENCY ERROR: Missing setting BOWDEN_LENGTH
  #endif
  #if !defined(LCD_PURGE_LENGTH)
    #error DEPENDENCY ERROR: Missing setting LCD_PURGE_LENGTH
  #endif
  #if !defined(LCD_RETRACT_LENGTH)
    #error DEPENDENCY ERROR: Missing setting LCD_RETRACT_LENGTH
  #endif
  #if !defined(LCD_PURGE_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting LCD_PURGE_FEEDRATE
  #endif
  #if !defined(LCD_RETRACT_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting LCD_RETRACT_FEEDRATE
  #endif
  #if !defined(LCD_LOAD_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting LCD_LOAD_FEEDRATE
  #endif
  #if !defined(LCD_UNLOAD_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting LCD_UNLOAD_FEEDRATE
  #endif
#endif

#if defined(ADVANCE)
  #if !defined(EXTRUDER_ADVANCE_K)
    #error DEPENDENCY ERROR: Missing setting EXTRUDER_ADVANCE_K
  #endif
  #if !defined(D_FILAMENT)
    #error DEPENDENCY ERROR: Missing setting D_FILAMENT
  #endif
  #if !defined(STEPS_MM_E)
    #error DEPENDENCY ERROR: Missing setting STEPS_MM_E
  #endif
#endif

#if defined(FILAMENTCHANGEENABLE)
  #if !defined(FILAMENTCHANGE_XPOS)
    #error DEPENDENCY ERROR: Missing setting FILAMENTCHANGE_XPOS
  #endif
  #if !defined(FILAMENTCHANGE_YPOS)
    #error DEPENDENCY ERROR: Missing setting FILAMENTCHANGE_YPOS
  #endif
  #if !defined(FILAMENTCHANGE_ZADD)
    #error DEPENDENCY ERROR: Missing setting FILAMENTCHANGE_ZADD
  #endif
  #if !defined(FILAMENTCHANGE_FIRSTRETRACT)
    #error DEPENDENCY ERROR: Missing setting FILAMENTCHANGE_FIRSTRETRACT
  #endif
  #if !defined(FILAMENTCHANGE_FINALRETRACT)
    #error DEPENDENCY ERROR: Missing setting FILAMENTCHANGE_FINALRETRACT
  #endif
  #if !defined(FILAMENTCHANGE_PRINTEROFF)
    #error DEPENDENCY ERROR: Missing setting FILAMENTCHANGE_PRINTEROFF
  #endif
#endif
  
//Motion
#if !defined(SOFTWARE_MIN_ENDSTOPS)
  #error DEPENDENCY ERROR: Missing setting SOFTWARE_MIN_ENDSTOPS
#endif
#if !defined(SOFTWARE_MAX_ENDSTOPS)
  #error DEPENDENCY ERROR: Missing setting SOFTWARE_MAX_ENDSTOPS
#endif
#if defined(AUTO_BED_LEVELING_FEATURE)
  #if defined(AUTO_BED_LEVELING_GRID)
    #if !defined(MIN_PROBE_EDGE)
      #error DEPENDENCY ERROR: Missing setting MIN_PROBE_EDGE
    #endif
    #if !defined(AUTO_BED_LEVELING_GRID_POINTS)
      #error DEPENDENCY ERROR: Missing setting AUTO_BED_LEVELING_GRID_POINTS
    #endif
  #else
    #if !defined(ABL_PROBE_PT_1_X)
      #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_1_X
    #endif
    #if !defined(ABL_PROBE_PT_1_Y)
      #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_1_Y
    #endif
    #if !defined(ABL_PROBE_PT_2_X)
      #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_2_X
    #endif
    #if !defined(ABL_PROBE_PT_2_Y)
      #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_2_Y
    #endif
    #if !defined(ABL_PROBE_PT_3_X)
      #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_3_X
    #endif
    #if !defined(ABL_PROBE_PT_3_Y)
      #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_3_Y
    #endif
  #endif
  #if !defined(X_PROBE_OFFSET_FROM_EXTRUDER)
    #error DEPENDENCY ERROR: Missing setting X_PROBE_OFFSET_FROM_EXTRUDER
  #endif
  #if !defined(Y_PROBE_OFFSET_FROM_EXTRUDER)
    #error DEPENDENCY ERROR: Missing setting Y_PROBE_OFFSET_FROM_EXTRUDER
  #endif
  #if !defined(Z_PROBE_OFFSET_FROM_EXTRUDER)
    #error DEPENDENCY ERROR: Missing setting Z_PROBE_OFFSET_FROM_EXTRUDER
  #endif
  #if !defined(Z_RAISE_BEFORE_HOMING)
    #error DEPENDENCY ERROR: Missing setting Z_RAISE_BEFORE_HOMING
  #endif
  #if !defined(Z_RAISE_BEFORE_PROBING)
    #error DEPENDENCY ERROR: Missing setting Z_RAISE_BEFORE_PROBING
  #endif
  #if !defined(Z_RAISE_BETWEEN_PROBINGS)
    #error DEPENDENCY ERROR: Missing setting Z_RAISE_BETWEEN_PROBINGS
  #endif
  #if !defined(Z_RAISE_AFTER_PROBING)
    #error DEPENDENCY ERROR: Missing setting Z_RAISE_AFTER_PROBING
  #endif
  #if defined(Z_PROBE_SLED)
    #if !defined(SLED_DOCKING_OFFSET)
      #error DEPENDENCY ERROR: Missing setting SLED_DOCKING_OFFSET
    #endif
  #endif
  #if defined(Z_SAFE_HOMING)
    #if !defined(Z_SAFE_HOMING_X_POINT)
      #error DEPENDENCY ERROR: Missing setting Z_SAFE_HOMING_X_POINT
    #endif
    #if !defined(Z_SAFE_HOMING_Y_POINT)
      #error DEPENDENCY ERROR: Missing setting Z_SAFE_HOMING_Y_POINT
    #endif
  #endif
#endif

#if !defined(NUM_SERVOS)
  #error DEPENDENCY ERROR: Missing setting NUM_SERVOS 0
#endif
#if NUM_SERVOS > 0
  #if !defined(X_ENDSTOP_SERVO_NR)
    #error DEPENDENCY ERROR: Missing setting X_ENDSTOP_SERVO_NR
  #endif
  #if !defined(Y_ENDSTOP_SERVO_NR)
    #error DEPENDENCY ERROR: Missing setting Y_ENDSTOP_SERVO_NR
  #endif
  #if !defined(Z_ENDSTOP_SERVO_NR)
    #error DEPENDENCY ERROR: Missing setting Z_ENDSTOP_SERVO_NR
  #endif
  #if !defined(X_ENDSTOP_SERVO_ANGLES)
    #error DEPENDENCY ERROR: Missing setting X_ENDSTOP_SERVO_ANGLES
  #endif
  #if !defined(Y_ENDSTOP_SERVO_ANGLES)
    #error DEPENDENCY ERROR: Missing setting Y_ENDSTOP_SERVO_ANGLES
  #endif
  #if !defined(Z_ENDSTOP_SERVO_ANGLES)
    #error DEPENDENCY ERROR: Missing setting Z_ENDSTOP_SERVO_ANGLES
  #endif
  #if !defined(SERVO_DEACTIVATION_DELAY)
    #error DEPENDENCY ERROR: Missing setting SERVO_DEACTIVATION_DELAY
  #endif
#endif
#if defined(BABYSTEPPING)
  #if !defined(BABYSTEP_INVERT_Z)
    #error DEPENDENCY ERROR: Missing setting BABYSTEP_INVERT_Z
  #endif
  #if !defined(BABYSTEP_Z_MULTIPLICATOR)
    #error DEPENDENCY ERROR: Missing setting BABYSTEP_Z_MULTIPLICATOR
  #endif
#endif
#if defined(FWRETRACT)
  #if !defined(MIN_RETRACT)
    #error DEPENDENCY ERROR: Missing setting MIN_RETRACT
  #endif
  #if !defined(RETRACT_LENGTH)
    #error DEPENDENCY ERROR: Missing setting RETRACT_LENGTH
  #endif
  #if !defined(RETRACT_LENGTH_SWAP)
    #error DEPENDENCY ERROR: Missing setting RETRACT_LENGTH_SWAP
  #endif
  #if !defined(RETRACT_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting RETRACT_FEEDRATE
  #endif
  #if !defined(RETRACT_ZLIFT)
    #error DEPENDENCY ERROR: Missing setting RETRACT_ZLIFT
  #endif
  #if !defined(RETRACT_RECOVER_LENGTH)
    #error DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_LENGTH
  #endif
  #if !defined(RETRACT_RECOVER_LENGTH_SWAP)
    #error DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_LENGTH_SWAP
  #endif
  #if !defined(RETRACT_RECOVER_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_FEEDRATE
  #endif
#endif
#if defined(DUAL_X_CARRIAGE)
  #if !defined(X2_MIN_POS)
    #error DEPENDENCY ERROR: Missing setting X2_MIN_POS
  #endif
  #if !defined(X2_MAX_POS)
    #error DEPENDENCY ERROR: Missing setting X2_MAX_POS
  #endif
  #if !defined(X2_HOME_DIR)
    #error DEPENDENCY ERROR: Missing setting X2_HOME_DIR
  #endif
  #if !defined(X2_HOME_POS)
    #error DEPENDENCY ERROR: Missing setting X2_HOME_POS
  #endif
  #if !defined(DEFAULT_DUAL_X_CARRIAGE_MODE)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_DUAL_X_CARRIAGE_MODE
  #endif
  #if !defined(TOOLCHANGE_PARK_ZLIFT)
    #error DEPENDENCY ERROR: Missing setting TOOLCHANGE_PARK_ZLIFT
  #endif
  #if !defined(TOOLCHANGE_UNPARK_ZLIFT)
    #error DEPENDENCY ERROR: Missing setting TOOLCHANGE_UNPARK_ZLIFT
  #endif
  #if !defined(DEFAULT_DUPLICATION_X_OFFSET)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_DUPLICATION_X_OFFSET
  #endif
#endif
#if defined(Y_DUAL_STEPPER_DRIVERS)
  #if !defined(INVERT_Y2_VS_Y_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_Y2_VS_Y_DIR
  #endif
#endif

//sensors
#if defined(FILAMENT_SENSOR)
  #if !defined(FILAMENT_SENSOR_EXTRUDER_NUM)
    #error DEPENDENCY ERROR: Missing setting FILAMENT_SENSOR_EXTRUDER_NUM
  #endif
  #if !defined(MEASUREMENT_DELAY_CM)
    #error DEPENDENCY ERROR: Missing setting MEASUREMENT_DELAY_CM
  #endif
  #if !defined(DEFAULT_NOMINAL_FILAMENT_DIA)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_NOMINAL_FILAMENT_DIA 
  #endif
  #if !defined(MEASURED_UPPER_LIMIT)
    #error DEPENDENCY ERROR: Missing setting MEASURED_UPPER_LIMIT
  #endif
  #if !defined(MEASURED_LOWER_LIMIT)
    #error DEPENDENCY ERROR: Missing setting MEASURED_LOWER_LIMIT
  #endif
  #if !defined(MAX_MEASUREMENT_DELAY)
    #error DEPENDENCY ERROR: Missing setting MAX_MEASUREMENT_DELAY
  #endif
  #if !defined(DEFAULT_MEASURED_FILAMENT_DIA)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_MEASURED_FILAMENT_DIA
  #endif
#endif
#if defined(FILAMENT_RUNOUT_SENSOR)
  #if !defined(FILRUNOUT_PIN_INVERTING)
    #error DEPENDENCY ERROR: Missing setting FILRUNOUT_PIN_INVERTING
  #endif
  #if !defined(ENDSTOPPULLUP_FIL_RUNOUT)
    #error DEPENDENCY ERROR: Missing setting ENDSTOPPULLUP_FIL_RUNOUT
  #endif
  #if !defined(FILAMENT_RUNOUT_SCRIPT)
    #error DEPENDENCY ERROR: Missing setting FILAMENT_RUNOUT_SCRIPT 
  #endif
#endif
#if defined(POWER_CONSUMPTION)
  #if !defined(POWER_VOLTAGE)
    #error DEPENDENCY ERROR: Missing setting POWER_VOLTAGE
  #endif
  #if !defined(POWER_SENSITIVITY)
    #error DEPENDENCY ERROR: Missing setting POWER_SENSITIVITY
  #endif
  #if !defined(POWER_OFFSET)
    #error DEPENDENCY ERROR: Missing setting POWER_OFFSET 
  #endif
  #if !defined(POWER_ZERO)
    #error DEPENDENCY ERROR: Missing setting POWER_ZERO 
  #endif
  #if !defined(POWER_ERROR)
    #error DEPENDENCY ERROR: Missing setting POWER_ERROR 
  #endif
  #if !defined(POWER_EFFICIENCY)
    #error DEPENDENCY ERROR: Missing setting POWER_EFFICIENCY 
  #endif
#endif

//addon
#if defined(SDSUPPORT)
  #if !defined(SD_FINISHED_STEPPERRELEASE)
    #error DEPENDENCY ERROR: Missing setting SD_FINISHED_STEPPERRELEASE
  #endif
  #if !defined(SD_FINISHED_RELEASECOMMAND)
    #error DEPENDENCY ERROR: Missing setting SD_FINISHED_RELEASECOMMAND
  #endif
  #if defined(SD_SETTINGS)
    #if !defined(SD_CFG_SECONDS)
      #error DEPENDENCY ERROR: Missing setting SD_CFG_SECONDS
    #endif
    #if !defined(CFG_SD_FILE)
      #error DEPENDENCY ERROR: Missing setting CFG_SD_FILE
    #endif
    #if !defined(CFG_SD_MAX_KEY_LEN)
      #error DEPENDENCY ERROR: Missing setting CFG_SD_MAX_KEY_LEN
    #endif
    #if !defined(CFG_SD_MAX_VALUE_LEN)
      #error DEPENDENCY ERROR: Missing setting CFG_SD_MAX_VALUE_LEN
    #endif
  #endif
#endif
#if !defined(DISPLAY_CHARSET_HD44780_JAPAN) && !defined(DISPLAY_CHARSET_HD44780_WESTERN) && !defined(DISPLAY_CHARSET_HD44780_CYRILLIC)
  #error DEPENDENCY ERROR: Missing setting DISPLAY_CHARSET_HD44780_JAPAN or DISPLAY_CHARSET_HD44780_WESTERN or DISPLAY_CHARSET_HD44780_CYRILLIC
#endif
#if defined(SHOW_BOOTSCREEN)
  #if !defined(STRING_SPLASH_LINE1)
    #error DEPENDENCY ERROR: Missing setting STRING_SPLASH_LINE1
  #endif
  #if !defined(SPLASH_SCREEN_DURATION)
    #error DEPENDENCY ERROR: Missing setting SPLASH_SCREEN_DURATION
  #endif
#endif
#if ENABLED(ULTIPANEL)
  #if defined(ENCODER_RATE_MULTIPLIER)
    #if !defined(ENCODER_10X_STEPS_PER_SEC)
      #error DEPENDENCY ERROR: Missing setting ENCODER_10X_STEPS_PER_SEC
    #endif
    #if !defined(ENCODER_100X_STEPS_PER_SEC)
      #error DEPENDENCY ERROR: Missing setting ENCODER_100X_STEPS_PER_SEC
    #endif
  #endif
#endif
#if MB(ALLIGATOR)
  #if !defined(UI_VOLTAGE_LEVEL)
    #error DEPENDENCY ERROR: Missing setting UI_VOLTAGE_LEVEL
  #endif
#endif
#if defined(REPRAPWORLD_KEYPAD)
  #if !defined(REPRAPWORLD_KEYPAD_MOVE_STEP)
    #error DEPENDENCY ERROR: Missing setting REPRAPWORLD_KEYPAD_MOVE_STEP
  #endif
#endif
#if ENABLED(ULTIPANEL)
  #if defined(LCD_PROGRESS_BAR)
    #if !defined(PROGRESS_BAR_BAR_TIME)
      #error DEPENDENCY ERROR: Missing setting PROGRESS_BAR_BAR_TIME
    #endif
    #if !defined(PROGRESS_BAR_MSG_TIME)
      #error DEPENDENCY ERROR: Missing setting PROGRESS_BAR_MSG_TIME
    #endif
    #if !defined(PROGRESS_MSG_EXPIRE)
      #error DEPENDENCY ERROR: Missing setting PROGRESS_MSG_EXPIRE
    #endif
  #endif
#endif
#if defined(CHDK)
  #if !defined(CHDK_DELAY)
    #error DEPENDENCY ERROR: Missing setting CHDK_DELAY
  #endif
#endif
//adv motion
#if defined(DIGIPOT_I2C)
  #if !defined(DIGIPOT_I2C_NUM_CHANNELS)
    #error DEPENDENCY ERROR: Missing setting DIGIPOT_I2C_NUM_CHANNELS
  #endif
  #if !defined(DIGIPOT_I2C_MOTOR_CURRENTS)
    #error DEPENDENCY ERROR: Missing setting DIGIPOT_I2C_MOTOR_CURRENTS
  #endif
#endif
#if defined(HAVE_TMCDRIVER)
  #if defined(X_IS_TMC)
    #if !defined(X_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting X_MAX_CURRENT
    #endif
    #if !defined(X_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting X_SENSE_RESISTOR
    #endif
    #if !defined(X_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting X_MICROSTEPS
    #endif
  #endif
  #if defined(X2_IS_TMC)
    #if !defined(X2_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting X2_MAX_CURRENT
    #endif
    #if !defined(X2_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting X2_SENSE_RESISTOR
    #endif
    #if !defined(X2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting X2_MICROSTEPS
    #endif
  #endif
  #if defined(Y_IS_TMC)
    #if !defined(Y_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting Y_MAX_CURRENT
    #endif
    #if !defined(Y_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting Y_SENSE_RESISTOR
    #endif
    #if !defined(Y_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Y_MICROSTEPS
    #endif
  #endif
  #if defined(Y2_IS_TMC)
    #if !defined(Y2_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting Y2_MAX_CURRENT
    #endif
    #if !defined(Y2_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting Y2_SENSE_RESISTOR
    #endif
    #if !defined(Y2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Y2_MICROSTEPS
    #endif
  #endif
  #if defined(Z_IS_TMC)
    #if !defined(Z_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting Z_MAX_CURRENT
    #endif
    #if !defined(Z_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting Z_SENSE_RESISTOR
    #endif
    #if !defined(Z_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Z_MICROSTEPS
    #endif
  #endif
  #if defined(Z2_IS_TMC)
    #if !defined(Z2_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting Z2_MAX_CURRENT
    #endif
    #if !defined(Z2_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting Z2_SENSE_RESISTOR
    #endif
    #if !defined(Z2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Z2_MICROSTEPS
    #endif
  #endif
  #if defined(E0_IS_TMC)
    #if !defined(E0_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting E0_MAX_CURRENT
    #endif
    #if !defined(E0_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting E0_SENSE_RESISTOR
    #endif
    #if !defined(E0_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E0_MICROSTEPS
    #endif
  #endif
  #if defined(E1_IS_TMC)
    #if !defined(E1_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting E1_MAX_CURRENT
    #endif
    #if !defined(E1_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting E1_SENSE_RESISTOR
    #endif
    #if !defined(E1_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E1_MICROSTEPS
    #endif
  #endif
  #if defined(E2_IS_TMC)
    #if !defined(E2_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting E2_MAX_CURRENT
    #endif
    #if !defined(E2_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting E2_SENSE_RESISTOR
    #endif
    #if !defined(E2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E2_MICROSTEPS
    #endif
  #endif
  #if defined(E3_IS_TMC)
    #if !defined(E3_MAX_CURRENT)
      #error DEPENDENCY ERROR: Missing setting E3_MAX_CURRENT
    #endif
    #if !defined(E3_SENSE_RESISTOR)
      #error DEPENDENCY ERROR: Missing setting E3_SENSE_RESISTOR
    #endif
    #if !defined(E3_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E3_MICROSTEPS
    #endif
  #endif
#endif
#if defined(HAVE_L6470DRIVER)
  #if defined(X_IS_L6470)
    #if !defined(X_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting X_MICROSTEPS
    #endif
    #if !defined(X_K_VAL)
      #error DEPENDENCY ERROR: Missing setting X_K_VAL
    #endif
    #if !defined(X_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting X_OVERCURRENT
    #endif
    #if !defined(X_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting X_STALLCURRENT
    #endif
  #endif
  #if defined(X2_IS_L6470)
    #if !defined(X2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting X2_MICROSTEPS
    #endif
    #if !defined(X2_K_VAL)
      #error DEPENDENCY ERROR: Missing setting X2_K_VAL
    #endif
    #if !defined(X2_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting X2_OVERCURRENT
    #endif
    #if !defined(X2_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting X2_STALLCURRENT
    #endif
  #endif
  #if defined(Y_IS_L6470)
    #if !defined(Y_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Y_MICROSTEPS
    #endif
    #if !defined(Y_K_VAL)
      #error DEPENDENCY ERROR: Missing setting Y_K_VAL
    #endif
    #if !defined(Y_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting Y_OVERCURRENT
    #endif
    #if !defined(Y_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting Y_STALLCURRENT
    #endif
  #endif
  #if defined(Y2_IS_L6470)
    #if !defined(Y2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Y2_MICROSTEPS
    #endif
    #if !defined(Y2_K_VAL)
      #error DEPENDENCY ERROR: Missing setting Y2_K_VAL
    #endif
    #if !defined(Y2_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting Y2_OVERCURRENT
    #endif
    #if !defined(Y2_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting Y2_STALLCURRENT
    #endif
  #endif
  #if defined(Z_IS_L6470)
    #if !defined(Z_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Z_MICROSTEPS
    #endif
    #if !defined(Z_K_VAL)
      #error DEPENDENCY ERROR: Missing setting Z_K_VAL
    #endif
    #if !defined(Z_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting Z_OVERCURRENT
    #endif
    #if !defined(Z_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting Z_STALLCURRENT
    #endif
  #endif
  #if defined(Z2_IS_L6470)
    #if !defined(Z2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting Z2_MICROSTEPS
    #endif
    #if !defined(Z2_K_VAL)
      #error DEPENDENCY ERROR: Missing setting Z2_K_VAL
    #endif
    #if !defined(Z2_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting Z2_OVERCURRENT
    #endif
    #if !defined(Z2_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting Z2_STALLCURRENT
    #endif
  #endif
  #if defined(E0_IS_L6470)
    #if !defined(E0_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E0_MICROSTEPS
    #endif
    #if !defined(E0_K_VAL)
      #error DEPENDENCY ERROR: Missing setting E0_K_VAL
    #endif
    #if !defined(E0_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting E0_OVERCURRENT
    #endif
    #if !defined(E0_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting E0_STALLCURRENT
    #endif
  #endif
  #if defined(E1_IS_L6470)
    #if !defined(E1_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E1_MICROSTEPS
    #endif
    #if !defined(E1_K_VAL)
      #error DEPENDENCY ERROR: Missing setting E1_K_VAL
    #endif
    #if !defined(E1_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting E1_OVERCURRENT
    #endif
    #if !defined(E1_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting E1_STALLCURRENT
    #endif
  #endif
  #if defined(E2_IS_L6470)
    #if !defined(E2_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E2_MICROSTEPS
    #endif
    #if !defined(E2_K_VAL)
      #error DEPENDENCY ERROR: Missing setting E2_K_VAL
    #endif
    #if !defined(E2_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting E2_OVERCURRENT
    #endif
    #if !defined(E2_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting E2_STALLCURRENT
    #endif
  #endif
  #if defined(E3_IS_L6470)
    #if !defined(E3_MICROSTEPS)
      #error DEPENDENCY ERROR: Missing setting E3_MICROSTEPS
    #endif
    #if !defined(E3_K_VAL)
      #error DEPENDENCY ERROR: Missing setting E3_K_VAL
    #endif
    #if !defined(E3_OVERCURRENT)
      #error DEPENDENCY ERROR: Missing setting E3_OVERCURRENT
    #endif
    #if !defined(E3_STALLCURRENT)
      #error DEPENDENCY ERROR: Missing setting E3_STALLCURRENT
    #endif
  #endif
#endif
//buffer
#if !defined(BLOCK_BUFFER_SIZE)
  #error DEPENDENCY ERROR: Missing setting BLOCK_BUFFER_SIZE
#endif
#if !defined(MAX_CMD_SIZE)
  #error DEPENDENCY ERROR: Missing setting MAX_CMD_SIZE
#endif
#if !defined(BUFSIZE)
  #error DEPENDENCY ERROR: Missing setting BUFSIZE
#endif
#if !defined(NUM_POSITON_SLOTS)
  #error DEPENDENCY ERROR: Missing setting NUM_POSITON_SLOTS
#endif
#if !defined(DROP_SEGMENTS)
  #error DEPENDENCY ERROR: Missing setting DROP_SEGMENTS
#endif
#if !defined(DROP_SEGMENTS)
  #error DEPENDENCY ERROR: Missing setting DROP_SEGMENTS
#endif
#if !defined(DEFAULT_MINSEGMENTTIME)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_MINSEGMENTTIME
#endif
#if !defined(MM_PER_ARC_SEGMENT)
  #error DEPENDENCY ERROR: Missing setting MM_PER_ARC_SEGMENT
#endif
#if !defined(N_ARC_CORRECTION)
  #error DEPENDENCY ERROR: Missing setting N_ARC_CORRECTION
#endif

//Machines
#if !defined(X_MIN_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting X_MIN_ENDSTOP_LOGIC
#endif
#if !defined(Y_MIN_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Y_MIN_ENDSTOP_LOGIC
#endif
#if !defined(Z_MIN_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Z_MIN_ENDSTOP_LOGIC
#endif
#if !defined(Z2_MIN_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Z2_MIN_ENDSTOP_LOGIC
#endif
#if !defined(X_MAX_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting X_MAX_ENDSTOP_LOGIC
#endif
#if !defined(Y_MAX_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Y_MAX_ENDSTOP_LOGIC
#endif
#if !defined(Z_MAX_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Z_MAX_ENDSTOP_LOGIC
#endif
#if !defined(Z2_MAX_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Z2_MAX_ENDSTOP_LOGIC
#endif
#if !defined(Z_PROBE_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting Z_PROBE_ENDSTOP_LOGIC
#endif
#if !defined(E_MIN_ENDSTOP_LOGIC)
  #error DEPENDENCY ERROR: Missing setting E_MIN_ENDSTOP_LOGIC
#endif
#if !defined(X_HOME_DIR)
  #error DEPENDENCY ERROR: Missing setting X_HOME_DIR
#endif
#if !defined(Y_HOME_DIR)
  #error DEPENDENCY ERROR: Missing setting Y_HOME_DIR
#endif
#if !defined(Z_HOME_DIR)
  #error DEPENDENCY ERROR: Missing setting Z_HOME_DIR
#endif
#if !defined(E_HOME_DIR)
  #error DEPENDENCY ERROR: Missing setting E_HOME_DIR
#endif
#if !defined(X_ENABLE_ON)
  #error DEPENDENCY ERROR: Missing setting X_ENABLE_ON
#endif
#if !defined(Y_ENABLE_ON)
  #error DEPENDENCY ERROR: Missing setting Y_ENABLE_ON
#endif
#if !defined(Z_ENABLE_ON)
  #error DEPENDENCY ERROR: Missing setting Z_ENABLE_ON
#endif
#if !defined(E_ENABLE_ON)
  #error DEPENDENCY ERROR: Missing setting E_ENABLE_ON
#endif
#if !defined(INVERT_X_STEP_PIN)
  #error DEPENDENCY ERROR: Missing setting INVERT_X_STEP_PIN
#endif
#if !defined(INVERT_Y_STEP_PIN)
  #error DEPENDENCY ERROR: Missing setting INVERT_Y_STEP_PIN
#endif
#if !defined(INVERT_Z_STEP_PIN)
  #error DEPENDENCY ERROR: Missing setting INVERT_Z_STEP_PIN
#endif
#if !defined(INVERT_E_STEP_PIN)
  #error DEPENDENCY ERROR: Missing setting INVERT_E_STEP_PIN
#endif
#if !defined(INVERT_X_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_X_DIR
#endif
#if !defined(INVERT_Y_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_Y_DIR
#endif
#if !defined(INVERT_Z_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_Z_DIR
#endif
#if !defined(INVERT_E0_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_E0_DIR
#endif
#if !defined(INVERT_E1_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_E1_DIR
#endif
#if !defined(INVERT_E2_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_E2_DIR
#endif
#if !defined(INVERT_E3_DIR)
  #error DEPENDENCY ERROR: Missing setting INVERT_E3_DIR
#endif
#if !defined(DISABLE_X)
  #error DEPENDENCY ERROR: Missing setting DISABLE_X
#endif
#if !defined(DISABLE_Y)
  #error DEPENDENCY ERROR: Missing setting DISABLE_Y
#endif
#if !defined(DISABLE_Z)
  #error DEPENDENCY ERROR: Missing setting DISABLE_Z
#endif
#if !defined(DISABLE_E)
  #error DEPENDENCY ERROR: Missing setting DISABLE_E
#endif
#if !defined(DISABLE_INACTIVE_EXTRUDER)
  #error DEPENDENCY ERROR: Missing setting DISABLE_INACTIVE_EXTRUDER
#endif
#if !defined(X_MAX_POS)
  #error DEPENDENCY ERROR: Missing setting X_MAX_POS
#endif
#if !defined(X_MIN_POS)
  #error DEPENDENCY ERROR: Missing setting X_MIN_POS
#endif
#if !defined(Y_MAX_POS)
  #error DEPENDENCY ERROR: Missing setting Y_MAX_POS
#endif
#if !defined(Y_MIN_POS)
  #error DEPENDENCY ERROR: Missing setting Y_MIN_POS
#endif
#if !defined(Z_MAX_POS)
  #error DEPENDENCY ERROR: Missing setting Z_MAX_POS
#endif
#if !defined(Z_MIN_POS)
  #error DEPENDENCY ERROR: Missing setting Z_MIN_POS
#endif
#if !defined(E_MIN_POS)
  #error DEPENDENCY ERROR: Missing setting E_MIN_POS
#endif
#if !defined(AXIS_RELATIVE_MODES)
  #error DEPENDENCY ERROR: Missing setting AXIS_RELATIVE_MODES
#endif
#if !defined(DEFAULT_AXIS_STEPS_PER_UNIT)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_AXIS_STEPS_PER_UNIT
#endif
#if ENABLED(ULTIPANEL) && !defined(MANUAL_FEEDRATE)
  #error DEPENDENCY ERROR: Missing setting MANUAL_FEEDRATE
#endif
#if !defined(DEFAULT_MINTRAVELFEEDRATE)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_MINTRAVELFEEDRATE
#endif
#if !defined(MINIMUM_PLANNER_SPEED)
  #error DEPENDENCY ERROR: Missing setting MINIMUM_PLANNER_SPEED
#endif
#if !defined(DEFAULT_MAX_ACCELERATION)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_MAX_ACCELERATION
#endif
#if !defined(DEFAULT_RETRACT_ACCELERATION)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_RETRACT_ACCELERATION
#endif
#if !defined(DEFAULT_ACCELERATION)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_ACCELERATION
#endif
#if !defined(DEFAULT_TRAVEL_ACCELERATION)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_TRAVEL_ACCELERATION
#endif
#if !defined(DEFAULT_XYJERK)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_XYJERK
#endif
#if !defined(DEFAULT_ZJERK)
  #error DEPENDENCY ERROR: Missing setting DEFAULT_ZJERK
#endif
#if !defined(HOMING_FEEDRATE)
  #error DEPENDENCY ERROR: Missing setting HOMING_FEEDRATE
#endif
#if !defined(X_HOME_BUMP_MM)
  #error DEPENDENCY ERROR: Missing setting X_HOME_BUMP_MM
#endif
#if !defined(Y_HOME_BUMP_MM)
  #error DEPENDENCY ERROR: Missing setting Y_HOME_BUMP_MM
#endif
#if !defined(Z_HOME_BUMP_MM)
  #error DEPENDENCY ERROR: Missing setting Z_HOME_BUMP_MM
#endif
#if !defined(HOMING_BUMP_DIVISOR)
  #error DEPENDENCY ERROR: Missing setting HOMING_BUMP_DIVISOR
#endif
#if !defined(LEFT_PROBE_BED_POSITION)
  #error DEPENDENCY ERROR: Missing setting LEFT_PROBE_BED_POSITION
#endif
#if !defined(RIGHT_PROBE_BED_POSITION)
  #error DEPENDENCY ERROR: Missing setting RIGHT_PROBE_BED_POSITION
#endif
#if !defined(FRONT_PROBE_BED_POSITION)
  #error DEPENDENCY ERROR: Missing setting FRONT_PROBE_BED_POSITION
#endif
#if !MECH(DELTA)
  #if !defined(XY_TRAVEL_SPEED)
    #error DEPENDENCY ERROR: Missing setting XY_TRAVEL_SPEED
  #endif
#endif
#if ENABLED(MANUAL_HOME_POSITIONS)
  #if !defined(MANUAL_X_HOME_POS)
    #error DEPENDENCY ERROR: Missing setting MANUAL_X_HOME_POS
  #endif
  #if !defined(MANUAL_Y_HOME_POS)
    #error DEPENDENCY ERROR: Missing setting MANUAL_Y_HOME_POS
  #endif
  #if !defined(MANUAL_Z_HOME_POS)
    #error DEPENDENCY ERROR: Missing setting MANUAL_Z_HOME_POS
  #endif
#endif
#if MECH(COREXY) || MECH(COREXZ)
  #if !defined(COREX_YZ_FACTOR)
    #error DEPENDENCY ERROR: Missing setting COREX_YZ_FACTOR
  #endif
#endif
#if MECH(SCARA)
  #if !defined(LINKAGE_1)
    #error DEPENDENCY ERROR: Missing setting LINKAGE_1
  #endif
  #if !defined(LINKAGE_2)
    #error DEPENDENCY ERROR: Missing setting LINKAGE_2
  #endif
  #if !defined(SCARA_OFFSET_X)
    #error DEPENDENCY ERROR: Missing setting SCARA_OFFSET_X
  #endif
  #if !defined(SCARA_OFFSET_Y)
    #error DEPENDENCY ERROR: Missing setting SCARA_OFFSET_Y
  #endif
  #if !defined(SCARA_RAD2DEG)
    #error DEPENDENCY ERROR: Missing setting SCARA_RAD2DEG
  #endif
  #if !defined(THETA_HOMING_OFFSET)
    #error DEPENDENCY ERROR: Missing setting THETA_HOMING_OFFSET
  #endif
  #if !defined(PSI_HOMING_OFFSET)
    #error DEPENDENCY ERROR: Missing setting PSI_HOMING_OFFSET
  #endif
#endif

#if MECH(DELTA)
  #if !defined(DEFAULT_DELTA_DIAGONAL_ROD)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_DELTA_DIAGONAL_ROD
  #endif
  #if !defined(DELTA_SMOOTH_ROD_OFFSET)
    #error DEPENDENCY ERROR: Missing setting DELTA_SMOOTH_ROD_OFFSET
  #endif
  #if !defined(DELTA_CARRIAGE_OFFSET)
    #error DEPENDENCY ERROR: Missing setting DELTA_CARRIAGE_OFFSET
  #endif
  #if !defined(PRINTER_RADIUS)
    #error DEPENDENCY ERROR: Missing setting PRINTER_RADIUS
  #endif
  #if !defined(DEFAULT_DELTA_RADIUS)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_DELTA_RADIUS
  #endif
  #if !defined(AUTOCAL_TRAVELRATE)
    #error DEPENDENCY ERROR: Missing setting AUTOCAL_TRAVELRATE
  #endif
  #if !defined(AUTOCAL_PROBERATE)
    #error DEPENDENCY ERROR: Missing setting AUTOCAL_PROBERATE
  #endif
  #if !defined(AUTOCALIBRATION_PRECISION)
    #error DEPENDENCY ERROR: Missing setting AUTOCALIBRATION_PRECISION
  #endif
  #if !defined(TOWER_A_ENDSTOP_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_A_ENDSTOP_ADJ
  #endif
  #if !defined(TOWER_B_ENDSTOP_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_B_ENDSTOP_ADJ
  #endif
  #if !defined(TOWER_C_ENDSTOP_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_C_ENDSTOP_ADJ
  #endif
  #if !defined(TOWER_A_POSITION_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_A_POSITION_ADJ
  #endif
  #if !defined(TOWER_B_POSITION_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_B_POSITION_ADJ
  #endif
  #if !defined(TOWER_C_POSITION_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_C_POSITION_ADJ
  #endif
  #if !defined(TOWER_A_RADIUS_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_A_RADIUS_ADJ
  #endif
  #if !defined(TOWER_B_RADIUS_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_B_RADIUS_ADJ
  #endif
  #if !defined(TOWER_C_RADIUS_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_C_RADIUS_ADJ
  #endif
  #if !defined(TOWER_A_DIAGROD_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_A_DIAGROD_ADJ
  #endif
  #if !defined(TOWER_B_DIAGROD_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_B_DIAGROD_ADJ
  #endif
  #if !defined(TOWER_C_DIAGROD_ADJ)
    #error DEPENDENCY ERROR: Missing setting TOWER_C_DIAGROD_ADJ
  #endif
  #if !defined(Z_PROBE_OFFSET)
    #error DEPENDENCY ERROR: Missing setting Z_PROBE_OFFSET
  #endif
  #if !defined(Z_PROBE_DEPLOY_START_LOCATION)
    #error DEPENDENCY ERROR: Missing setting Z_PROBE_DEPLOY_START_LOCATION
  #endif
  #if !defined(Z_PROBE_DEPLOY_END_LOCATION)
    #error DEPENDENCY ERROR: Missing setting Z_PROBE_DEPLOY_END_LOCATION
  #endif
  #if !defined(Z_PROBE_RETRACT_START_LOCATION)
    #error DEPENDENCY ERROR: Missing setting Z_PROBE_RETRACT_START_LOCATION
  #endif
  #if !defined(Z_PROBE_RETRACT_END_LOCATION)
    #error DEPENDENCY ERROR: Missing setting Z_PROBE_RETRACT_END_LOCATION
  #endif
  #if !defined(Z_RAISE_BETWEEN_PROBINGS)
    #error DEPENDENCY ERROR: Missing setting Z_RAISE_BETWEEN_PROBINGS
  #endif
  #if !defined(AUTO_BED_LEVELING_GRID_POINTS)
    #error DEPENDENCY ERROR: Missing setting AUTO_BED_LEVELING_GRID_POINTS
  #endif
#endif

#endif