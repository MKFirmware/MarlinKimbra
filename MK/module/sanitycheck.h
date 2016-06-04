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
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

#ifndef SANITYCHECK_H
  #define SANITYCHECK_H

  // Start check
  #if DISABLED(SERIAL_PORT)
    #error DEPENDENCY ERROR: Missing setting SERIAL_PORT
  #endif
  #if DISABLED(BAUDRATE)
    #error DEPENDENCY ERROR: Missing setting BAUDRATE
  #endif
  #if DISABLED(STRING_CONFIG_H_AUTHOR)
    #define STRING_CONFIG_H_AUTHOR "(none, default config)"
  #endif
  #if DISABLED(MACHINE_UUID)
    #error DEPENDENCY ERROR: Missing setting MACHINE_UUID
  #endif

  // Board
  #if DISABLED(MOTHERBOARD)
    #error DEPENDENCY ERROR: Missing setting MOTHERBOARD
  #endif

  // Mechanism
  #if DISABLED(MECHANISM)
    #error DEPENDENCY ERROR: Missing setting MECHANISM
  #endif

  // Power supply
  #if DISABLED(POWER_SUPPLY)
    #define POWER_SUPPLY 0
  #endif

  // Extruders
  #if DISABLED(EXTRUDERS)
    #error DEPENDENCY ERROR: Missing setting EXTRUDERS
  #endif
  #if DISABLED(DRIVER_EXTRUDERS)
    #error DEPENDENCY ERROR: Missing setting DRIVER_EXTRUDERS
  #endif

  // Thermistor
  #if DISABLED(TEMP_SENSOR_0)
    #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_0
  #endif
  #if DISABLED(TEMP_SENSOR_1)
    #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_1
  #endif
  #if DISABLED(TEMP_SENSOR_2)
    #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_2
  #endif
  #if DISABLED(TEMP_SENSOR_3)
    #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_3
  #endif
  #if DISABLED(TEMP_SENSOR_BED)
    #error DEPENDENCY ERROR: Missing setting TEMP_SENSOR_BED
  #endif
  #if DISABLED(TEMP_SENSOR_CHAMBER)
    #error DEPENDENCY_ERROR: Missing setting TEMP_SENSOR_CHAMBER
  #endif
  #if DISABLED(TEMP_SENSOR_COOLER)
    #error DEPENDENCY_ERROR: Missing setting TEMP_SENSOR_COOLER
  #endif
  #if (THERMISTORHEATER_0 == 998) || (THERMISTORHEATER_1 == 998) || (THERMISTORHEATER_2 == 998) || (THERMISTORHEATER_3 == 998) || (THERMISTORBED == 998) || (THERMISTORCHAMBER == 998) || (THERMISTORCOOLER == 998) // User EXIST table
    #if DISABLED(DUMMY_THERMISTOR_998_VALUE)
      #define DUMMY_THERMISTOR_998_VALUE 25
    #endif
  #endif
  #if (THERMISTORHEATER_0 == 999) || (THERMISTORHEATER_1 == 999) || (THERMISTORHEATER_2 == 999) || (THERMISTORHEATER_3 == 999) || (THERMISTORBED == 999) || (THERMISTORCHAMBER == 999) || (THERMISTORCOOLER == 999)// User EXIST table
    #if DISABLED(DUMMY_THERMISTOR_999_VALUE)
      #define DUMMY_THERMISTOR_999_VALUE 25
    #endif
  #endif

  // Temperature
  /**
   * Temperature defines
   */
  #if ENABLED(TEMP_RESIDENCY_TIME)
    #if DISABLED(TEMP_HYSTERESIS)
      #error DEPENDENCY ERROR: Missing setting TEMP_HYSTERESIS
    #endif
    #if DISABLED(TEMP_WINDOW)
      #error DEPENDENCY ERROR: Missing setting TEMP_WINDOW
    #endif
  #endif
  #if TEMP_SENSOR_0 != 0
    #if DISABLED(HEATER_0_MAXTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_0_MAXTEMP
    #endif
    #if DISABLED(HEATER_0_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_0_MINTEMP
    #endif
  #endif
  #if TEMP_SENSOR_1 != 0
    #if DISABLED(HEATER_1_MAXTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_1_MAXTEMP
    #endif
    #if DISABLED(HEATER_1_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_1_MINTEMP
    #endif
  #endif
  #if TEMP_SENSOR_2 != 0
    #if DISABLED(HEATER_2_MAXTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_2_MAXTEMP
    #endif
    #if DISABLED(HEATER_2_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_2_MINTEMP
    #endif
  #endif
  #if TEMP_SENSOR_3 != 0
    #if DISABLED(HEATER_3_MAXTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_3_MAXTEMP
    #endif
    #if DISABLED(HEATER_3_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting HEATER_3_MINTEMP
    #endif
  #endif
  #if TEMP_SENSOR_BED != 0
    #if DISABLED(BED_MAXTEMP)
      #error DEPENDENCY ERROR: Missing setting BED_MAXTEMP
    #endif
    #if DISABLED(BED_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting BED_MINTEMP
    #endif
  #endif
  #if TEMP_SENSOR_CHAMBER != 0
    #if DISABLED(CHAMBER_MAXTEMP)
      #error DEPENDENCY ERROR: Missing setting CHAMBER_MAXTEMP
    #endif
    #if DISABLED(CHAMBER_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting CHAMBER_MINTEMP
    #endif
    #if HASNT(HEATER_CHAMBER)
      #error DEPENDENCY ERROR: Cannot enable TEMP_SENSOR_CHAMBER and not HEATER_CHAMBER_PIN
    #endif
  #endif
  #if TEMP_SENSOR_COOLER != 0
    #if DISABLED(COOLER_MAXTEMP)
      #error DEPENDENCY ERROR: Missing setting COOLER_MAXTEMP
    #endif
    #if DISABLED(COOLER_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting COOLER_MINTEMP
    #endif
    #if HASNT(COOLER)
      #error DEPENDENCY ERROR: Cannot enable TEMP_SENSOR_COOLER and not COOLER_PIN
    #endif
  #endif
  #if DISABLED(PLA_PREHEAT_HOTEND_TEMP)
    #error DEPENDENCY ERROR: Missing setting PLA_PREHEAT_HOTEND_TEMP
  #endif
  #if DISABLED(PLA_PREHEAT_HPB_TEMP)
    #error DEPENDENCY ERROR: Missing setting PLA_PREHEAT_HPB_TEMP
  #endif
  #if DISABLED(PLA_PREHEAT_FAN_SPEED)
    #error DEPENDENCY ERROR: Missing setting PLA_PREHEAT_FAN_SPEED
  #endif
  #if DISABLED(ABS_PREHEAT_HOTEND_TEMP)
    #error DEPENDENCY ERROR: Missing setting ABS_PREHEAT_HOTEND_TEMP
  #endif
  #if DISABLED(ABS_PREHEAT_HPB_TEMP)
    #error DEPENDENCY ERROR: Missing setting ABS_PREHEAT_HPB_TEMP
  #endif
  #if DISABLED(ABS_PREHEAT_FAN_SPEED)
    #error DEPENDENCY ERROR: Missing setting ABS_PREHEAT_FAN_SPEED
  #endif
  #if DISABLED(GUM_PREHEAT_HOTEND_TEMP)
    #error DEPENDENCY ERROR: Missing setting GUM_PREHEAT_HOTEND_TEMP
  #endif
  #if DISABLED(GUM_PREHEAT_HPB_TEMP)
    #error DEPENDENCY ERROR: Missing setting GUM_PREHEAT_HPB_TEMP
  #endif
  #if DISABLED(GUM_PREHEAT_FAN_SPEED)
    #error DEPENDENCY ERROR: Missing setting GUM_PREHEAT_FAN_SPEED
  #endif

  // Language
  #if DISABLED(LCD_LANGUAGE)
    #error DEPENDENCY ERROR: Missing setting LCD_LANGUAGE
  #endif

  /// FEATURE

  // Temperature
  #if DISABLED(PID_MAX)
    #error DEPENDENCY ERROR: Missing setting PID_MAX
  #endif
  #if DISABLED(MAX_BED_POWER)
    #error DEPENDENCY ERROR: Missing setting MAX_BED_POWER
  #endif
  #if DISABLED(MAX_CHAMBER_POWER)
    #error DEPENDENCY ERROR: Missing setting MAX_CHAMBER_POWER
  #endif
  #if DISABLED(MAX_COOLER_POWER)
    #error DEPENDENCY ERROR: Missing setting MAX_COOLER_POWER
  #endif
  #if ENABLED(PIDTEMP) || ENABLED(PIDTEMPBED) || ENABLED(PIDTEMPCHAMBER) || ENABLED(PIDTEMPCOOLER)
    #if DISABLED(MAX_OVERSHOOT_PID_AUTOTUNE)
      #error DEPENDENCY ERROR: Missing setting MAX_OVERSHOOT_PID_AUTOTUNE
    #endif
  #endif
  #if ENABLED(PIDTEMP)
    #if DISABLED(PID_OPENLOOP) && DISABLED(PID_FUNCTIONAL_RANGE) 
      #error DEPENDENCY ERROR: Missing setting PID_FUNCTIONAL_RANGE
    #endif
    #if DISABLED(PID_INTEGRAL_DRIVE_MAX)
      #error DEPENDENCY ERROR: Missing setting PID_INTEGRAL_DRIVE_MAX
    #endif
    #if DISABLED(DEFAULT_Kp)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_Kp
    #endif
    #if DISABLED(DEFAULT_Ki)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_Ki
    #endif
    #if DISABLED(DEFAULT_Kd)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_Kd
    #endif
  #endif
  #if ENABLED(PIDTEMPBED)
    #if DISABLED(PID_BED_INTEGRAL_DRIVE_MAX)
      #error DEPENDENCY ERROR: Missing setting PID_BED_INTEGRAL_DRIVE_MAX
    #endif
    #if DISABLED(DEFAULT_bedKp)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_bedKp
    #endif
    #if DISABLED(DEFAULT_bedKi)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_bedKi
    #endif
    #if DISABLED(DEFAULT_bedKd)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_bedKd
    #endif
  #endif
  #if ENABLED(PIDTEMPCHAMBER)
    #if DISABLED(PID_CHAMBER_INTEGRAL_DRIVE_MAX)
       #error DEPENDENCY ERROR: Missing setting PID_CHAMBER_INTEGRAL_DRIVE_MAX
    #endif
    #if DISABLED(DEFAULT_chamberKp)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_chamberKp
    #endif
    #if DISABLED(DEFAULT_chamberKi)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_chamberKi
    #endif
    #if DISABLED(DEFAULT_chamberKd)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_chamberKd
    #endif

  #endif
  #if ENABLED(PIDTEMPCOOLER)
    #if DISABLED(PID_COOLER_INTEGRAL_DRIVE_MAX)
       #error DEPENDENCY ERROR: Missing setting PID_COOLER_INTEGRAL_DRIVE_MAX
    #endif
    #if DISABLED(DEFAULT_coolerKp)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_coolerKp
    #endif
    #if DISABLED(DEFAULT_coolerKi)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_coolerKi
    #endif
    #if DISABLED(DEFAULT_coolerKd)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_coolerKd
    #endif
  #endif
  #if ENABLED(BED_LIMIT_SWITCHING)
    #if DISABLED(BED_HYSTERESIS)
      #error DEPENDENCY ERROR: Missing setting BED_HYSTERESIS
    #endif
    #if DISABLED(BED_CHECK_INTERVAL)
      #error DEPENDENCY ERROR: Missing setting BED_CHECK_INTERVAL
    #endif
  #endif
  #if ENABLED(CHAMBER_LIMIT_SWITCHING)
    #if DISABLED(CHAMBER_HYSTERESIS)
      #error DEPENDENCY ERROR: Missing setting CHAMBER_HYSTERESIS
    #endif
    #if DISABLED(CHAMBER_CHECK_INTERVAL)
      #error DEPENDENCY ERROR: Missing setting CHAMBER_CHECK_INTERVAL
    #endif
  #endif
  #if ENABLED(COOLER_LIMIT_SWITCHING)
    #if DISABLED(COOLER_HYSTERESIS)
      #error DEPENDENCY ERROR: Missing setting COOLER_HYSTERESIS
    #endif
    #if DISABLED(COOLER_CHECK_INTERVAL)
      #error DEPENDENCY ERROR: Missing setting COOLER_CHECK_INTERVAL
    #endif
  #endif
  #if ENABLED(THERMAL_PROTECTION_HOTENDS)
    #if DISABLED(THERMAL_PROTECTION_PERIOD)
      #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_PERIOD
    #endif
    #if DISABLED(THERMAL_PROTECTION_HYSTERESIS)
      #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_HYSTERESIS
    #endif
    #if DISABLED(WATCH_TEMP_PERIOD)
      #error DEPENDENCY ERROR: Missing setting WATCH_TEMP_PERIOD
    #endif
    #if DISABLED(WATCH_TEMP_INCREASE)
      #error DEPENDENCY ERROR: Missing setting WATCH_TEMP_INCREASE
    #endif
  #endif
  #if ENABLED(THERMAL_PROTECTION_BED)
    #if DISABLED(THERMAL_PROTECTION_BED_PERIOD)
      #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_BED_PERIOD
    #endif
    #if DISABLED(THERMAL_PROTECTION_BED_HYSTERESIS)
      #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_BED_HYSTERESIS
    #endif
  #endif
  #if ENABLED(THERMAL_PROTECTION_COOLER)
    #if DISANLED(THERMAL_PROTECTION_COOLER_PERIOD)
      #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_COOLER_PERIOD
    #endif
    #if DISABLED(THERMAL_PROTECTION_COOLER_HYSTERESIS)
      #error DEPENDENCY ERROR: Missing setting THERMAL_PROTECTION_COOLER_HYSTERESIS
    #endif
    #if ENABLED(THERMAL_PROTECTION_COOLER_WATCHDOG)
       #if DISABLED(WATCH_TEMP_COOLER_PERIOD)
         #error DEPENDENCY ERROR: Missing setting WATCH_TEMP_COOLER_PERIOD
       #endif
       #if DISABLED(WATCH_TEMP_COOLER_DECREASE)
         #error DEPENDENCY ERROR: Missing setting WATCH_TEMP_COOLER_DECREASE
       #endif
    #endif
  #endif


  // Fan
  #if DISABLED(SOFT_PWM_SCALE)
    #error DEPENDENCY ERROR: Missing setting SOFT_PWM_SCALE
  #endif

  #if ENABLED(CONTROLLERFAN)
    #if DISABLED(CONTROLLERFAN_SECS)
      #error DEPENDENCY ERROR: Missing setting CONTROLLERFAN_SECS
    #endif
    #if DISABLED(CONTROLLERFAN_SPEED)
      #error DEPENDENCY ERROR: Missing setting CONTROLLERFAN_SPEED
    #endif
    #if DISABLED(CONTROLLERFAN_MIN_SPEED)
      #error DEPENDENCY ERROR: Missing setting CONTROLLERFAN_MIN_SPEED
    #endif
  #endif

  #if ENABLED(EXTRUDER_AUTO_FAN)
    #if DISABLED(EXTRUDER_AUTO_FAN_TEMPERATURE)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_AUTO_FAN_TEMPERATURE
    #endif
    #if DISABLED(EXTRUDER_AUTO_FAN_SPEED)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_AUTO_FAN_SPEED
    #endif
    #if DISABLED(EXTRUDER_AUTO_FAN_MIN_SPEED)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_AUTO_FAN_MIN_SPEED
    #endif
  #endif

  // Extruder
  #if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
    #if DISABLED(EXTRUDE_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting EXTRUDE_MINTEMP
    #endif
    #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
      #if DISABLED(EXTRUDE_MAXLENGTH)
        #error DEPENDENCY ERROR: Missing setting EXTRUDE_MAXLENGTH
      #endif
    #endif
  #endif

  #if ENABLED(COLOR_MIXING_EXTRUDER)
    #if EXTRUDERS > 1
      #error COLOR_MIXING_EXTRUDER supports plus one extruder.
    #endif
    #if DRIVER_EXTRUDERS < 2
      #error You must set DRIVER_EXTRUDERS >= 2 for a mixing extruder.
    #endif
    #if ENABLED(FILAMENT_SENSOR)
      #error COLOR_MIXING_EXTRUDER is incompatible with FILAMENT_SENSOR. Comment out this line to use it anyway.
    #endif
  #endif

  #if ENABLED(NPR2)
    #if DISABLED(COLOR_STEP)
      #error DEPENDENCY ERROR: Missing setting COLOR_STEP
    #endif
    #if DISABLED(COLOR_SLOWRATE)
      #error DEPENDENCY ERROR: Missing setting COLOR_SLOWRATE
    #endif
    #if DISABLED(COLOR_HOMERATE)
      #error DEPENDENCY ERROR: Missing setting COLOR_HOMERATE
    #endif
    #if DISABLED(MOTOR_ANGLE)
      #error DEPENDENCY ERROR: Missing setting MOTOR_ANGLE
    #endif
    #if DISABLED(DRIVER_MICROSTEP)
      #error DEPENDENCY ERROR: Missing setting DRIVER_MICROSTEP
    #endif
    #if DISABLED(CARTER_MOLTIPLICATOR)
      #error DEPENDENCY ERROR: Missing setting CARTER_MOLTIPLICATOR
    #endif
  #endif

  #if ENABLED(IDLE_OOZING_PREVENT)
    #if DISABLED(IDLE_OOZING_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_MINTEMP
    #endif
    #if DISABLED(IDLE_OOZING_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_FEEDRATE
    #endif
    #if DISABLED(IDLE_OOZING_SECONDS)
      #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_SECONDS
    #endif
    #if DISABLED(IDLE_OOZING_LENGTH)
      #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_LENGTH
    #endif
    #if DISABLED(IDLE_OOZING_RECOVER_LENGTH)
      #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_RECOVER_LENGTH
    #endif
    #if DISABLED(IDLE_OOZING_RECOVER_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting IDLE_OOZING_RECOVER_FEEDRATE
    #endif
  #endif

  #if ENABLED(EXTRUDER_RUNOUT_PREVENT)
    #if DISABLED(EXTRUDER_RUNOUT_MINTEMP)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_MINTEMP
    #endif
    #if DISABLED(EXTRUDER_RUNOUT_SECONDS)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_SECONDS
    #endif
    #if DISABLED(EXTRUDER_RUNOUT_ESTEPS)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_ESTEPS
    #endif
    #if DISABLED(EXTRUDER_RUNOUT_SPEED)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_SPEED
    #endif
    #if DISABLED(EXTRUDER_RUNOUT_EXTRUDE)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_RUNOUT_EXTRUDE
    #endif
  #endif

  #if ENABLED(EASY_LOAD)
    #if DISABLED(BOWDEN_LENGTH)
      #error DEPENDENCY ERROR: Missing setting BOWDEN_LENGTH
    #endif
    #if DISABLED(LCD_PURGE_LENGTH)
      #error DEPENDENCY ERROR: Missing setting LCD_PURGE_LENGTH
    #endif
    #if DISABLED(LCD_RETRACT_LENGTH)
      #error DEPENDENCY ERROR: Missing setting LCD_RETRACT_LENGTH
    #endif
    #if DISABLED(LCD_PURGE_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting LCD_PURGE_FEEDRATE
    #endif
    #if DISABLED(LCD_RETRACT_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting LCD_RETRACT_FEEDRATE
    #endif
    #if DISABLED(LCD_LOAD_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting LCD_LOAD_FEEDRATE
    #endif
    #if DISABLED(LCD_UNLOAD_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting LCD_UNLOAD_FEEDRATE
    #endif
  #endif

  /**
   * Advance Extrusion
   */
  #if ENABLED(ADVANCE) && ENABLED(ADVANCE_LPC)
    #error You can enable ADVANCE or ADVANCE_LPC, but not both.
  #endif
  #if ENABLED(ADVANCE)
    #if DISABLED(EXTRUDER_ADVANCE_K)
      #error DEPENDENCY ERROR: Missing setting EXTRUDER_ADVANCE_K
    #endif
    #if DISABLED(D_FILAMENT)
      #error DEPENDENCY ERROR: Missing setting D_FILAMENT
    #endif
  #endif

  #if ENABLED(FILAMENT_CHANGE_FEATURE)
    #if DISABLED(FILAMENT_CHANGE_X_POS)
      #error DEPENDENCY ERROR: Missing setting FILAMENT_CHANGE_X_POS
    #endif
    #if DISABLED(FILAMENT_CHANGE_Y_POS)
      #error DEPENDENCY ERROR: Missing setting FILAMENT_CHANGE_Y_POS
    #endif
    #if DISABLED(FILAMENT_CHANGE_Z_ADD)
      #error DEPENDENCY ERROR: Missing setting FILAMENT_CHANGE_Z_ADD
    #endif
    #if DISABLED(FILAMENT_CHANGE_RETRACT_LENGTH)
      #error DEPENDENCY ERROR: Missing setting FILAMENT_CHANGE_RETRACT_LENGTH
    #endif
    #if DISABLED(FILAMENT_CHANGE_UNLOAD_LENGTH)
      #error DEPENDENCY ERROR: Missing setting FILAMENT_CHANGE_UNLOAD_LENGTH
    #endif
    #if DISABLED(FILAMENT_CHANGE_PRINTER_OFF)
      #error DEPENDENCY ERROR: Missing setting FILAMENT_CHANGE_PRINTER_OFF
    #endif
  #endif
    
  //Motion
  #if DISABLED(SOFTWARE_MIN_ENDSTOPS)
    #error DEPENDENCY ERROR: Missing setting SOFTWARE_MIN_ENDSTOPS
  #endif
  #if DISABLED(SOFTWARE_MAX_ENDSTOPS)
    #error DEPENDENCY ERROR: Missing setting SOFTWARE_MAX_ENDSTOPS
  #endif

  /**
   * Mesh Bed Leveling
   */
  #if ENABLED(MESH_BED_LEVELING)
    #if MECH(DELTA)
      #error "MESH_BED_LEVELING does not yet support DELTA printers."
    #endif
    #if ENABLED(AUTO_BED_LEVELING_FEATURE)
      #error "Select AUTO_BED_LEVELING_FEATURE or MESH_BED_LEVELING, not both."
    #endif
    #if MESH_NUM_X_POINTS > 7 || MESH_NUM_Y_POINTS > 7
      #error "MESH_NUM_X_POINTS and MESH_NUM_Y_POINTS need to be less than 8."
    #endif
  #elif ENABLED(MANUAL_BED_LEVELING)
    #error "MESH_BED_LEVELING is required for MANUAL_BED_LEVELING."
  #endif

  /**
   * Auto Bed Leveling
   */
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    #if ENABLED(AUTO_BED_LEVELING_GRID)
      #if DISABLED(MIN_PROBE_EDGE)
        #error DEPENDENCY ERROR: Missing setting MIN_PROBE_EDGE
      #endif
      #if DISABLED(AUTO_BED_LEVELING_GRID_POINTS)
        #error DEPENDENCY ERROR: Missing setting AUTO_BED_LEVELING_GRID_POINTS
      #endif
    #else
      #if DISABLED(ABL_PROBE_PT_1_X)
        #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_1_X
      #endif
      #if DISABLED(ABL_PROBE_PT_1_Y)
        #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_1_Y
      #endif
      #if DISABLED(ABL_PROBE_PT_2_X)
        #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_2_X
      #endif
      #if DISABLED(ABL_PROBE_PT_2_Y)
        #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_2_Y
      #endif
      #if DISABLED(ABL_PROBE_PT_3_X)
        #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_3_X
      #endif
      #if DISABLED(ABL_PROBE_PT_3_Y)
        #error DEPENDENCY ERROR: Missing setting ABL_PROBE_PT_3_Y
      #endif
    #endif
    #if DISABLED(X_PROBE_OFFSET_FROM_EXTRUDER)
      #error DEPENDENCY ERROR: Missing setting X_PROBE_OFFSET_FROM_EXTRUDER
    #endif
    #if DISABLED(Y_PROBE_OFFSET_FROM_EXTRUDER)
      #error DEPENDENCY ERROR: Missing setting Y_PROBE_OFFSET_FROM_EXTRUDER
    #endif
    #if DISABLED(Z_PROBE_OFFSET_FROM_EXTRUDER)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_OFFSET_FROM_EXTRUDER
    #endif
    #if DISABLED(Z_RAISE_BEFORE_PROBING)
      #error DEPENDENCY ERROR: Missing setting Z_RAISE_BEFORE_PROBING
    #endif
    #if DISABLED(Z_RAISE_BETWEEN_PROBINGS)
      #error DEPENDENCY ERROR: Missing setting Z_RAISE_BETWEEN_PROBINGS
    #endif
    #if DISABLED(Z_RAISE_AFTER_PROBING)
      #error DEPENDENCY ERROR: Missing setting Z_RAISE_AFTER_PROBING
    #endif
    #if ENABLED(Z_PROBE_SLED)
      #if DISABLED(SLED_DOCKING_OFFSET)
        #error DEPENDENCY ERROR: Missing setting SLED_DOCKING_OFFSET
      #endif
    #endif
    #if ENABLED(Z_SAFE_HOMING)
      #if DISABLED(Z_SAFE_HOMING_X_POINT)
        #error DEPENDENCY ERROR: Missing setting Z_SAFE_HOMING_X_POINT
      #endif
      #if DISABLED(Z_SAFE_HOMING_Y_POINT)
        #error DEPENDENCY ERROR: Missing setting Z_SAFE_HOMING_Y_POINT
      #endif
    #endif
  #endif
  #if ENABLED(ENABLE_SERVOS)
    #if DISABLED(NUM_SERVOS)
      #error DEPENDENCY ERROR: Missing setting NUM_SERVOS
    #endif
    #if NUM_SERVOS > 0
      #if DISABLED(X_ENDSTOP_SERVO_NR)
        #error DEPENDENCY ERROR: Missing setting X_ENDSTOP_SERVO_NR
      #endif
      #if DISABLED(Y_ENDSTOP_SERVO_NR)
        #error DEPENDENCY ERROR: Missing setting Y_ENDSTOP_SERVO_NR
      #endif
      #if DISABLED(Z_ENDSTOP_SERVO_NR)
        #error DEPENDENCY ERROR: Missing setting Z_ENDSTOP_SERVO_NR
      #endif
      #if ENABLED(X_ENDSTOP_SERVO_NR) && X_ENDSTOP_SERVO_NR > -1
        #if DISABLED(X_ENDSTOP_SERVO_ANGLES)
          #error DEPENDENCY ERROR: Missing setting X_ENDSTOP_SERVO_ANGLES
        #endif
      #endif
      #if ENABLED(Y_ENDSTOP_SERVO_NR) && Y_ENDSTOP_SERVO_NR > -1
        #if DISABLED(Y_ENDSTOP_SERVO_ANGLES)
          #error DEPENDENCY ERROR: Missing setting Y_ENDSTOP_SERVO_ANGLES
        #endif
      #endif
      #if ENABLED(Z_ENDSTOP_SERVO_NR) && Z_ENDSTOP_SERVO_NR > -1
        #if DISABLED(Z_ENDSTOP_SERVO_ANGLES)
          #error DEPENDENCY ERROR: Missing setting Z_ENDSTOP_SERVO_ANGLES
        #endif
      #endif
      #if DISABLED(SERVO_DEACTIVATION_DELAY)
        #error DEPENDENCY ERROR: Missing setting SERVO_DEACTIVATION_DELAY
      #endif
    #endif
  #endif
  #if ENABLED(BABYSTEPPING)
    #if DISABLED(BABYSTEP_INVERT_Z)
      #error DEPENDENCY ERROR: Missing setting BABYSTEP_INVERT_Z
    #endif
  #endif
  #if ENABLED(FWRETRACT)
    #if DISABLED(MIN_RETRACT)
      #error DEPENDENCY ERROR: Missing setting MIN_RETRACT
    #endif
    #if DISABLED(RETRACT_LENGTH)
      #error DEPENDENCY ERROR: Missing setting RETRACT_LENGTH
    #endif
    #if DISABLED(RETRACT_LENGTH_SWAP)
      #error DEPENDENCY ERROR: Missing setting RETRACT_LENGTH_SWAP
    #endif
    #if DISABLED(RETRACT_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting RETRACT_FEEDRATE
    #endif
    #if DISABLED(RETRACT_ZLIFT)
      #error DEPENDENCY ERROR: Missing setting RETRACT_ZLIFT
    #endif
    #if DISABLED(RETRACT_RECOVER_LENGTH)
      #error DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_LENGTH
    #endif
    #if DISABLED(RETRACT_RECOVER_LENGTH_SWAP)
      #error DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_LENGTH_SWAP
    #endif
    #if DISABLED(RETRACT_RECOVER_FEEDRATE)
      #error DEPENDENCY ERROR: Missing setting RETRACT_RECOVER_FEEDRATE
    #endif
  #endif
  #if ENABLED(DUAL_X_CARRIAGE)
    #if DISABLED(X2_MIN_POS)
      #error DEPENDENCY ERROR: Missing setting X2_MIN_POS
    #endif
    #if DISABLED(X2_MAX_POS)
      #error DEPENDENCY ERROR: Missing setting X2_MAX_POS
    #endif
    #if DISABLED(X2_HOME_DIR)
      #error DEPENDENCY ERROR: Missing setting X2_HOME_DIR
    #endif
    #if DISABLED(X2_HOME_POS)
      #error DEPENDENCY ERROR: Missing setting X2_HOME_POS
    #endif
    #if DISABLED(DEFAULT_DUAL_X_CARRIAGE_MODE)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_DUAL_X_CARRIAGE_MODE
    #endif
    #if DISABLED(TOOLCHANGE_PARK_ZLIFT)
      #error DEPENDENCY ERROR: Missing setting TOOLCHANGE_PARK_ZLIFT
    #endif
    #if DISABLED(TOOLCHANGE_UNPARK_ZLIFT)
      #error DEPENDENCY ERROR: Missing setting TOOLCHANGE_UNPARK_ZLIFT
    #endif
    #if DISABLED(DEFAULT_DUPLICATION_X_OFFSET)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_DUPLICATION_X_OFFSET
    #endif
  #endif
  #if ENABLED(Y_DUAL_STEPPER_DRIVERS)
    #if DISABLED(INVERT_Y2_VS_Y_DIR)
      #error DEPENDENCY ERROR: Missing setting INVERT_Y2_VS_Y_DIR
    #endif
  #endif

  //sensors
  #if ENABLED(FILAMENT_SENSOR)
    #if DISABLED(FILAMENT_SENSOR_EXTRUDER_NUM)
      #error DEPENDENCY ERROR: Missing setting FILAMENT_SENSOR_EXTRUDER_NUM
    #endif
    #if DISABLED(MEASUREMENT_DELAY_CM)
      #error DEPENDENCY ERROR: Missing setting MEASUREMENT_DELAY_CM
    #endif
    #if DISABLED(DEFAULT_NOMINAL_FILAMENT_DIA)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_NOMINAL_FILAMENT_DIA 
    #endif
    #if DISABLED(MEASURED_UPPER_LIMIT)
      #error DEPENDENCY ERROR: Missing setting MEASURED_UPPER_LIMIT
    #endif
    #if DISABLED(MEASURED_LOWER_LIMIT)
      #error DEPENDENCY ERROR: Missing setting MEASURED_LOWER_LIMIT
    #endif
    #if DISABLED(MAX_MEASUREMENT_DELAY)
      #error DEPENDENCY ERROR: Missing setting MAX_MEASUREMENT_DELAY
    #endif
    #if DISABLED(DEFAULT_MEASURED_FILAMENT_DIA)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_MEASURED_FILAMENT_DIA
    #endif
  #endif
  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    #if DISABLED(FILRUNOUT_PIN_INVERTING)
      #error DEPENDENCY ERROR: Missing setting FILRUNOUT_PIN_INVERTING
    #endif
    #if DISABLED(ENDSTOPPULLUP_FIL_RUNOUT)
      #error DEPENDENCY ERROR: Missing setting ENDSTOPPULLUP_FIL_RUNOUT
    #endif
    #if DISABLED(FILAMENT_RUNOUT_SCRIPT)
      #error DEPENDENCY ERROR: Missing setting FILAMENT_RUNOUT_SCRIPT 
    #endif
  #endif
  #if ENABLED(POWER_CONSUMPTION)
    #if DISABLED(POWER_VOLTAGE)
      #error DEPENDENCY ERROR: Missing setting POWER_VOLTAGE
    #endif
    #if DISABLED(POWER_SENSITIVITY)
      #error DEPENDENCY ERROR: Missing setting POWER_SENSITIVITY
    #endif
    #if DISABLED(POWER_OFFSET)
      #error DEPENDENCY ERROR: Missing setting POWER_OFFSET 
    #endif
    #if DISABLED(POWER_ZERO)
      #error DEPENDENCY ERROR: Missing setting POWER_ZERO 
    #endif
    #if DISABLED(POWER_ERROR)
      #error DEPENDENCY ERROR: Missing setting POWER_ERROR 
    #endif
    #if DISABLED(POWER_EFFICIENCY)
      #error DEPENDENCY ERROR: Missing setting POWER_EFFICIENCY 
    #endif
  #endif

  //addon
  #if ENABLED(SDSUPPORT)
    #if DISABLED(SD_FINISHED_STEPPERRELEASE)
      #error DEPENDENCY ERROR: Missing setting SD_FINISHED_STEPPERRELEASE
    #endif
    #if DISABLED(SD_FINISHED_RELEASECOMMAND)
      #error DEPENDENCY ERROR: Missing setting SD_FINISHED_RELEASECOMMAND
    #endif
    #if ENABLED(SD_SETTINGS)
      #if DISABLED(SD_CFG_SECONDS)
        #error DEPENDENCY ERROR: Missing setting SD_CFG_SECONDS
      #endif
      #if DISABLED(CFG_SD_FILE)
        #error DEPENDENCY ERROR: Missing setting CFG_SD_FILE
      #endif
    #endif
  #endif
  #if DISABLED(DISPLAY_CHARSET_HD44780_JAPAN) && DISABLED(DISPLAY_CHARSET_HD44780_WESTERN) && DISABLED(DISPLAY_CHARSET_HD44780_CYRILLIC)
    #error DEPENDENCY ERROR: Missing setting DISPLAY_CHARSET_HD44780_JAPAN or DISPLAY_CHARSET_HD44780_WESTERN or DISPLAY_CHARSET_HD44780_CYRILLIC
  #endif
  #if ENABLED(SHOW_BOOTSCREEN)
    #if DISABLED(STRING_SPLASH_LINE1)
      #error DEPENDENCY ERROR: Missing setting STRING_SPLASH_LINE1
    #endif
    #if DISABLED(SPLASH_SCREEN_DURATION)
      #error DEPENDENCY ERROR: Missing setting SPLASH_SCREEN_DURATION
    #endif
  #endif
  #if ENABLED(ULTIPANEL)
    #if ENABLED(ENCODER_RATE_MULTIPLIER)
      #if DISABLED(ENCODER_10X_STEPS_PER_SEC)
        #error DEPENDENCY ERROR: Missing setting ENCODER_10X_STEPS_PER_SEC
      #endif
      #if DISABLED(ENCODER_100X_STEPS_PER_SEC)
        #error DEPENDENCY ERROR: Missing setting ENCODER_100X_STEPS_PER_SEC
      #endif
    #endif
  #endif
  #if MB(ALLIGATOR)
    #if DISABLED(UI_VOLTAGE_LEVEL)
      #error DEPENDENCY ERROR: Missing setting UI_VOLTAGE_LEVEL
    #endif
  #endif
  #if ENABLED(REPRAPWORLD_KEYPAD)
    #if DISABLED(REPRAPWORLD_KEYPAD_MOVE_STEP)
      #error DEPENDENCY ERROR: Missing setting REPRAPWORLD_KEYPAD_MOVE_STEP
    #endif
  #endif
  #if ENABLED(ULTIPANEL)
    #if ENABLED(LCD_PROGRESS_BAR)
      #if DISABLED(PROGRESS_BAR_BAR_TIME)
        #error DEPENDENCY ERROR: Missing setting PROGRESS_BAR_BAR_TIME
      #endif
      #if DISABLED(PROGRESS_BAR_MSG_TIME)
        #error DEPENDENCY ERROR: Missing setting PROGRESS_BAR_MSG_TIME
      #endif
      #if DISABLED(PROGRESS_MSG_EXPIRE)
        #error DEPENDENCY ERROR: Missing setting PROGRESS_MSG_EXPIRE
      #endif
    #endif
  #endif
  #if ENABLED(CHDK)
    #if DISABLED(CHDK_DELAY)
      #error DEPENDENCY ERROR: Missing setting CHDK_DELAY
    #endif
  #endif
  //adv motion
  #if ENABLED(USE_MICROSTEPS)
    #if DISABLED(MICROSTEP_MODES)
      #error DEPENDENCY ERROR: Missing setting MICROSTEP_MODES
    #endif
  #endif
  #if DISABLED(DEFAULT_STEPPER_DEACTIVE_TIME)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_STEPPER_DEACTIVE_TIME
  #endif
  #if ENABLED(STEPPER_HIGH_LOW)
    #if DISABLED(STEPPER_HIGH_LOW_DELAY)
      #error DEPENDENCY ERROR: Missing setting STEPPER_HIGH_LOW_DELAY
    #endif
  #endif
  #if ENABLED(DIGIPOT_I2C)
    #if DISABLED(DIGIPOT_I2C_NUM_CHANNELS)
      #error DEPENDENCY ERROR: Missing setting DIGIPOT_I2C_NUM_CHANNELS
    #endif
    #if DISABLED(DIGIPOT_I2C_MOTOR_CURRENTS)
      #error DEPENDENCY ERROR: Missing setting DIGIPOT_I2C_MOTOR_CURRENTS
    #endif
  #endif
  #if ENABLED(HAVE_TMCDRIVER)
    #if ENABLED(X_IS_TMC)
      #if DISABLED(X_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting X_MAX_CURRENT
      #endif
      #if DISABLED(X_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting X_SENSE_RESISTOR
      #endif
      #if DISABLED(X_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting X_MICROSTEPS
      #endif
    #endif
    #if ENABLED(X2_IS_TMC)
      #if DISABLED(X2_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting X2_MAX_CURRENT
      #endif
      #if DISABLED(X2_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting X2_SENSE_RESISTOR
      #endif
      #if DISABLED(X2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting X2_MICROSTEPS
      #endif
    #endif
    #if ENABLED(Y_IS_TMC)
      #if DISABLED(Y_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting Y_MAX_CURRENT
      #endif
      #if DISABLED(Y_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting Y_SENSE_RESISTOR
      #endif
      #if DISABLED(Y_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Y_MICROSTEPS
      #endif
    #endif
    #if ENABLED(Y2_IS_TMC)
      #if DISABLED(Y2_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting Y2_MAX_CURRENT
      #endif
      #if DISABLED(Y2_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting Y2_SENSE_RESISTOR
      #endif
      #if DISABLED(Y2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Y2_MICROSTEPS
      #endif
    #endif
    #if ENABLED(Z_IS_TMC)
      #if DISABLED(Z_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting Z_MAX_CURRENT
      #endif
      #if DISABLED(Z_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting Z_SENSE_RESISTOR
      #endif
      #if DISABLED(Z_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Z_MICROSTEPS
      #endif
    #endif
    #if ENABLED(Z2_IS_TMC)
      #if DISABLED(Z2_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting Z2_MAX_CURRENT
      #endif
      #if DISABLED(Z2_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting Z2_SENSE_RESISTOR
      #endif
      #if DISABLED(Z2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Z2_MICROSTEPS
      #endif
    #endif
    #if ENABLED(E0_IS_TMC)
      #if DISABLED(E0_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting E0_MAX_CURRENT
      #endif
      #if DISABLED(E0_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting E0_SENSE_RESISTOR
      #endif
      #if DISABLED(E0_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E0_MICROSTEPS
      #endif
    #endif
    #if ENABLED(E1_IS_TMC)
      #if DISABLED(E1_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting E1_MAX_CURRENT
      #endif
      #if DISABLED(E1_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting E1_SENSE_RESISTOR
      #endif
      #if DISABLED(E1_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E1_MICROSTEPS
      #endif
    #endif
    #if ENABLED(E2_IS_TMC)
      #if DISABLED(E2_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting E2_MAX_CURRENT
      #endif
      #if DISABLED(E2_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting E2_SENSE_RESISTOR
      #endif
      #if DISABLED(E2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E2_MICROSTEPS
      #endif
    #endif
    #if ENABLED(E3_IS_TMC)
      #if DISABLED(E3_MAX_CURRENT)
        #error DEPENDENCY ERROR: Missing setting E3_MAX_CURRENT
      #endif
      #if DISABLED(E3_SENSE_RESISTOR)
        #error DEPENDENCY ERROR: Missing setting E3_SENSE_RESISTOR
      #endif
      #if DISABLED(E3_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E3_MICROSTEPS
      #endif
    #endif
  #endif
  #if ENABLED(HAVE_L6470DRIVER)
    #if ENABLED(X_IS_L6470)
      #if DISABLED(X_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting X_MICROSTEPS
      #endif
      #if DISABLED(X_K_VAL)
        #error DEPENDENCY ERROR: Missing setting X_K_VAL
      #endif
      #if DISABLED(X_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting X_OVERCURRENT
      #endif
      #if DISABLED(X_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting X_STALLCURRENT
      #endif
    #endif
    #if ENABLED(X2_IS_L6470)
      #if DISABLED(X2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting X2_MICROSTEPS
      #endif
      #if DISABLED(X2_K_VAL)
        #error DEPENDENCY ERROR: Missing setting X2_K_VAL
      #endif
      #if DISABLED(X2_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting X2_OVERCURRENT
      #endif
      #if DISABLED(X2_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting X2_STALLCURRENT
      #endif
    #endif
    #if ENABLED(Y_IS_L6470)
      #if DISABLED(Y_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Y_MICROSTEPS
      #endif
      #if DISABLED(Y_K_VAL)
        #error DEPENDENCY ERROR: Missing setting Y_K_VAL
      #endif
      #if DISABLED(Y_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting Y_OVERCURRENT
      #endif
      #if DISABLED(Y_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting Y_STALLCURRENT
      #endif
    #endif
    #if ENABLED(Y2_IS_L6470)
      #if DISABLED(Y2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Y2_MICROSTEPS
      #endif
      #if DISABLED(Y2_K_VAL)
        #error DEPENDENCY ERROR: Missing setting Y2_K_VAL
      #endif
      #if DISABLED(Y2_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting Y2_OVERCURRENT
      #endif
      #if DISABLED(Y2_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting Y2_STALLCURRENT
      #endif
    #endif
    #if ENABLED(Z_IS_L6470)
      #if DISABLED(Z_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Z_MICROSTEPS
      #endif
      #if DISABLED(Z_K_VAL)
        #error DEPENDENCY ERROR: Missing setting Z_K_VAL
      #endif
      #if DISABLED(Z_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting Z_OVERCURRENT
      #endif
      #if DISABLED(Z_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting Z_STALLCURRENT
      #endif
    #endif
    #if ENABLED(Z2_IS_L6470)
      #if DISABLED(Z2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting Z2_MICROSTEPS
      #endif
      #if DISABLED(Z2_K_VAL)
        #error DEPENDENCY ERROR: Missing setting Z2_K_VAL
      #endif
      #if DISABLED(Z2_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting Z2_OVERCURRENT
      #endif
      #if DISABLED(Z2_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting Z2_STALLCURRENT
      #endif
    #endif
    #if ENABLED(E0_IS_L6470)
      #if DISABLED(E0_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E0_MICROSTEPS
      #endif
      #if DISABLED(E0_K_VAL)
        #error DEPENDENCY ERROR: Missing setting E0_K_VAL
      #endif
      #if DISABLED(E0_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting E0_OVERCURRENT
      #endif
      #if DISABLED(E0_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting E0_STALLCURRENT
      #endif
    #endif
    #if ENABLED(E1_IS_L6470)
      #if DISABLED(E1_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E1_MICROSTEPS
      #endif
      #if DISABLED(E1_K_VAL)
        #error DEPENDENCY ERROR: Missing setting E1_K_VAL
      #endif
      #if DISABLED(E1_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting E1_OVERCURRENT
      #endif
      #if DISABLED(E1_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting E1_STALLCURRENT
      #endif
    #endif
    #if ENABLED(E2_IS_L6470)
      #if DISABLED(E2_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E2_MICROSTEPS
      #endif
      #if DISABLED(E2_K_VAL)
        #error DEPENDENCY ERROR: Missing setting E2_K_VAL
      #endif
      #if DISABLED(E2_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting E2_OVERCURRENT
      #endif
      #if DISABLED(E2_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting E2_STALLCURRENT
      #endif
    #endif
    #if ENABLED(E3_IS_L6470)
      #if DISABLED(E3_MICROSTEPS)
        #error DEPENDENCY ERROR: Missing setting E3_MICROSTEPS
      #endif
      #if DISABLED(E3_K_VAL)
        #error DEPENDENCY ERROR: Missing setting E3_K_VAL
      #endif
      #if DISABLED(E3_OVERCURRENT)
        #error DEPENDENCY ERROR: Missing setting E3_OVERCURRENT
      #endif
      #if DISABLED(E3_STALLCURRENT)
        #error DEPENDENCY ERROR: Missing setting E3_STALLCURRENT
      #endif
    #endif
  #endif
  //buffer
  #if DISABLED(BLOCK_BUFFER_SIZE)
    #error DEPENDENCY ERROR: Missing setting BLOCK_BUFFER_SIZE
  #endif
  #if DISABLED(MAX_CMD_SIZE)
    #error DEPENDENCY ERROR: Missing setting MAX_CMD_SIZE
  #endif
  #if DISABLED(BUFSIZE)
    #error DEPENDENCY ERROR: Missing setting BUFSIZE
  #endif
  #if DISABLED(NUM_POSITON_SLOTS)
    #error DEPENDENCY ERROR: Missing setting NUM_POSITON_SLOTS
  #endif
  #if DISABLED(DROP_SEGMENTS)
    #error DEPENDENCY ERROR: Missing setting DROP_SEGMENTS
  #endif
  #if DISABLED(DROP_SEGMENTS)
    #error DEPENDENCY ERROR: Missing setting DROP_SEGMENTS
  #endif
  #if DISABLED(DEFAULT_MINSEGMENTTIME)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_MINSEGMENTTIME
  #endif
  #if DISABLED(MM_PER_ARC_SEGMENT)
    #error DEPENDENCY ERROR: Missing setting MM_PER_ARC_SEGMENT
  #endif
  #if DISABLED(N_ARC_CORRECTION)
    #error DEPENDENCY ERROR: Missing setting N_ARC_CORRECTION
  #endif

  //Machines
  #if DISABLED(X_MIN_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting X_MIN_ENDSTOP_LOGIC
  #endif
  #if DISABLED(Y_MIN_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting Y_MIN_ENDSTOP_LOGIC
  #endif
  #if DISABLED(Z_MIN_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting Z_MIN_ENDSTOP_LOGIC
  #endif
  #if DISABLED(Z2_MIN_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting Z2_MIN_ENDSTOP_LOGIC
  #endif
  #if DISABLED(X_MAX_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting X_MAX_ENDSTOP_LOGIC
  #endif
  #if DISABLED(Y_MAX_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting Y_MAX_ENDSTOP_LOGIC
  #endif
  #if DISABLED(Z_MAX_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting Z_MAX_ENDSTOP_LOGIC
  #endif
  #if DISABLED(Z2_MAX_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting Z2_MAX_ENDSTOP_LOGIC
  #endif
  #if DISABLED(Z_PROBE_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting Z_PROBE_ENDSTOP_LOGIC
  #endif
  #if DISABLED(E_MIN_ENDSTOP_LOGIC)
    #error DEPENDENCY ERROR: Missing setting E_MIN_ENDSTOP_LOGIC
  #endif
  #if DISABLED(X_HOME_DIR)
    #error DEPENDENCY ERROR: Missing setting X_HOME_DIR
  #endif
  #if DISABLED(Y_HOME_DIR)
    #error DEPENDENCY ERROR: Missing setting Y_HOME_DIR
  #endif
  #if DISABLED(Z_HOME_DIR)
    #error DEPENDENCY ERROR: Missing setting Z_HOME_DIR
  #endif
  #if DISABLED(E_HOME_DIR)
    #error DEPENDENCY ERROR: Missing setting E_HOME_DIR
  #endif
  #if DISABLED(X_ENABLE_ON)
    #error DEPENDENCY ERROR: Missing setting X_ENABLE_ON
  #endif
  #if DISABLED(Y_ENABLE_ON)
    #error DEPENDENCY ERROR: Missing setting Y_ENABLE_ON
  #endif
  #if DISABLED(Z_ENABLE_ON)
    #error DEPENDENCY ERROR: Missing setting Z_ENABLE_ON
  #endif
  #if DISABLED(E_ENABLE_ON)
    #error DEPENDENCY ERROR: Missing setting E_ENABLE_ON
  #endif
  #if DISABLED(INVERT_X_STEP_PIN)
    #error DEPENDENCY ERROR: Missing setting INVERT_X_STEP_PIN
  #endif
  #if DISABLED(INVERT_Y_STEP_PIN)
    #error DEPENDENCY ERROR: Missing setting INVERT_Y_STEP_PIN
  #endif
  #if DISABLED(INVERT_Z_STEP_PIN)
    #error DEPENDENCY ERROR: Missing setting INVERT_Z_STEP_PIN
  #endif
  #if DISABLED(INVERT_E_STEP_PIN)
    #error DEPENDENCY ERROR: Missing setting INVERT_E_STEP_PIN
  #endif
  #if DISABLED(INVERT_X_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_X_DIR
  #endif
  #if DISABLED(INVERT_Y_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_Y_DIR
  #endif
  #if DISABLED(INVERT_Z_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_Z_DIR
  #endif
  #if DISABLED(INVERT_E0_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_E0_DIR
  #endif
  #if DISABLED(INVERT_E1_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_E1_DIR
  #endif
  #if DISABLED(INVERT_E2_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_E2_DIR
  #endif
  #if DISABLED(INVERT_E3_DIR)
    #error DEPENDENCY ERROR: Missing setting INVERT_E3_DIR
  #endif
  #if DISABLED(DISABLE_X)
    #error DEPENDENCY ERROR: Missing setting DISABLE_X
  #endif
  #if DISABLED(DISABLE_Y)
    #error DEPENDENCY ERROR: Missing setting DISABLE_Y
  #endif
  #if DISABLED(DISABLE_Z)
    #error DEPENDENCY ERROR: Missing setting DISABLE_Z
  #endif
  #if DISABLED(DISABLE_E)
    #error DEPENDENCY ERROR: Missing setting DISABLE_E
  #endif
  #if DISABLED(DISABLE_INACTIVE_EXTRUDER)
    #error DEPENDENCY ERROR: Missing setting DISABLE_INACTIVE_EXTRUDER
  #endif
  #if DISABLED(X_MAX_POS)
    #error DEPENDENCY ERROR: Missing setting X_MAX_POS
  #endif
  #if DISABLED(X_MIN_POS)
    #error DEPENDENCY ERROR: Missing setting X_MIN_POS
  #endif
  #if DISABLED(Y_MAX_POS)
    #error DEPENDENCY ERROR: Missing setting Y_MAX_POS
  #endif
  #if DISABLED(Y_MIN_POS)
    #error DEPENDENCY ERROR: Missing setting Y_MIN_POS
  #endif
  #if DISABLED(Z_MAX_POS)
    #error DEPENDENCY ERROR: Missing setting Z_MAX_POS
  #endif
  #if DISABLED(Z_MIN_POS)
    #error DEPENDENCY ERROR: Missing setting Z_MIN_POS
  #endif
  #if DISABLED(E_MIN_POS)
    #error DEPENDENCY ERROR: Missing setting E_MIN_POS
  #endif
  #if DISABLED(AXIS_RELATIVE_MODES)
    #error DEPENDENCY ERROR: Missing setting AXIS_RELATIVE_MODES
  #endif
  #if DISABLED(DEFAULT_AXIS_STEPS_PER_UNIT)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_AXIS_STEPS_PER_UNIT
  #endif
  #if ENABLED(ULTIPANEL) && DISABLED(MANUAL_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting MANUAL_FEEDRATE
  #endif
  #if DISABLED(DEFAULT_MINTRAVELFEEDRATE)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_MINTRAVELFEEDRATE
  #endif
  #if DISABLED(MINIMUM_PLANNER_SPEED)
    #error DEPENDENCY ERROR: Missing setting MINIMUM_PLANNER_SPEED
  #endif
  #if DISABLED(DEFAULT_MAX_ACCELERATION)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_MAX_ACCELERATION
  #endif
  #if DISABLED(DEFAULT_RETRACT_ACCELERATION)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_RETRACT_ACCELERATION
  #endif
  #if DISABLED(DEFAULT_ACCELERATION)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_ACCELERATION
  #endif
  #if DISABLED(DEFAULT_TRAVEL_ACCELERATION)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_TRAVEL_ACCELERATION
  #endif
  #if DISABLED(DEFAULT_XYJERK)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_XYJERK
  #endif
  #if DISABLED(DEFAULT_ZJERK)
    #error DEPENDENCY ERROR: Missing setting DEFAULT_ZJERK
  #endif
  #if DISABLED(HOMING_FEEDRATE)
    #error DEPENDENCY ERROR: Missing setting HOMING_FEEDRATE
  #endif
  #if DISABLED(X_HOME_BUMP_MM)
    #error DEPENDENCY ERROR: Missing setting X_HOME_BUMP_MM
  #endif
  #if DISABLED(Y_HOME_BUMP_MM)
    #error DEPENDENCY ERROR: Missing setting Y_HOME_BUMP_MM
  #endif
  #if DISABLED(Z_HOME_BUMP_MM)
    #error DEPENDENCY ERROR: Missing setting Z_HOME_BUMP_MM
  #endif
  #if DISABLED(HOMING_BUMP_DIVISOR)
    #error DEPENDENCY ERROR: Missing setting HOMING_BUMP_DIVISOR
  #endif
  #if DISABLED(LEFT_PROBE_BED_POSITION)
    #error DEPENDENCY ERROR: Missing setting LEFT_PROBE_BED_POSITION
  #endif
  #if DISABLED(RIGHT_PROBE_BED_POSITION)
    #error DEPENDENCY ERROR: Missing setting RIGHT_PROBE_BED_POSITION
  #endif
  #if DISABLED(FRONT_PROBE_BED_POSITION)
    #error DEPENDENCY ERROR: Missing setting FRONT_PROBE_BED_POSITION
  #endif
  #if !MECH(DELTA)
    #if DISABLED(XY_TRAVEL_SPEED)
      #error DEPENDENCY ERROR: Missing setting XY_TRAVEL_SPEED
    #endif
  #endif
  #if ENABLED(MANUAL_HOME_POSITIONS)
    #if DISABLED(MANUAL_X_HOME_POS)
      #error DEPENDENCY ERROR: Missing setting MANUAL_X_HOME_POS
    #endif
    #if DISABLED(MANUAL_Y_HOME_POS)
      #error DEPENDENCY ERROR: Missing setting MANUAL_Y_HOME_POS
    #endif
    #if DISABLED(MANUAL_Z_HOME_POS)
      #error DEPENDENCY ERROR: Missing setting MANUAL_Z_HOME_POS
    #endif
  #endif
  #if MECH(COREXY) || MECH(COREYX) || MECH(COREXZ) || MECH(COREZX)
    #if DISABLED(COREX_YZ_FACTOR)
      #error DEPENDENCY ERROR: Missing setting COREX_YZ_FACTOR
    #endif
  #endif
  #if MECH(SCARA)
    #if DISABLED(LINKAGE_1)
      #error DEPENDENCY ERROR: Missing setting LINKAGE_1
    #endif
    #if DISABLED(LINKAGE_2)
      #error DEPENDENCY ERROR: Missing setting LINKAGE_2
    #endif
    #if DISABLED(SCARA_OFFSET_X)
      #error DEPENDENCY ERROR: Missing setting SCARA_OFFSET_X
    #endif
    #if DISABLED(SCARA_OFFSET_Y)
      #error DEPENDENCY ERROR: Missing setting SCARA_OFFSET_Y
    #endif
    #if DISABLED(SCARA_RAD2DEG)
      #error DEPENDENCY ERROR: Missing setting SCARA_RAD2DEG
    #endif
    #if DISABLED(THETA_HOMING_OFFSET)
      #error DEPENDENCY ERROR: Missing setting THETA_HOMING_OFFSET
    #endif
    #if DISABLED(PSI_HOMING_OFFSET)
      #error DEPENDENCY ERROR: Missing setting PSI_HOMING_OFFSET
    #endif
  #endif

  #if MECH(DELTA)
    #if DISABLED(DELTA_DIAGONAL_ROD)
      #error DEPENDENCY ERROR: Missing setting DELTA_DIAGONAL_ROD
    #endif
    #if DISABLED(DELTA_SMOOTH_ROD_OFFSET)
      #error DEPENDENCY ERROR: Missing setting DELTA_SMOOTH_ROD_OFFSET
    #endif
    #if DISABLED(DELTA_CARRIAGE_OFFSET)
      #error DEPENDENCY ERROR: Missing setting DELTA_CARRIAGE_OFFSET
    #endif
    #if DISABLED(DELTA_PRINTABLE_RADIUS)
      #error DEPENDENCY ERROR: Missing setting DELTA_PRINTABLE_RADIUS
    #endif
    #if DISABLED(DEFAULT_DELTA_RADIUS)
      #error DEPENDENCY ERROR: Missing setting DEFAULT_DELTA_RADIUS
    #endif
    #if DISABLED(AUTOCAL_TRAVELRATE)
      #error DEPENDENCY ERROR: Missing setting AUTOCAL_TRAVELRATE
    #endif
    #if DISABLED(AUTOCAL_PROBERATE)
      #error DEPENDENCY ERROR: Missing setting AUTOCAL_PROBERATE
    #endif
    #if DISABLED(AUTOCALIBRATION_PRECISION)
      #error DEPENDENCY ERROR: Missing setting AUTOCALIBRATION_PRECISION
    #endif
    #if DISABLED(TOWER_A_ENDSTOP_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_A_ENDSTOP_ADJ
    #endif
    #if DISABLED(TOWER_B_ENDSTOP_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_B_ENDSTOP_ADJ
    #endif
    #if DISABLED(TOWER_C_ENDSTOP_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_C_ENDSTOP_ADJ
    #endif
    #if DISABLED(TOWER_A_POSITION_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_A_POSITION_ADJ
    #endif
    #if DISABLED(TOWER_B_POSITION_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_B_POSITION_ADJ
    #endif
    #if DISABLED(TOWER_C_POSITION_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_C_POSITION_ADJ
    #endif
    #if DISABLED(TOWER_A_RADIUS_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_A_RADIUS_ADJ
    #endif
    #if DISABLED(TOWER_B_RADIUS_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_B_RADIUS_ADJ
    #endif
    #if DISABLED(TOWER_C_RADIUS_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_C_RADIUS_ADJ
    #endif
    #if DISABLED(TOWER_A_DIAGROD_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_A_DIAGROD_ADJ
    #endif
    #if DISABLED(TOWER_B_DIAGROD_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_B_DIAGROD_ADJ
    #endif
    #if DISABLED(TOWER_C_DIAGROD_ADJ)
      #error DEPENDENCY ERROR: Missing setting TOWER_C_DIAGROD_ADJ
    #endif
    #if DISABLED(X_PROBE_OFFSET_FROM_EXTRUDER)
      #error DEPENDENCY ERROR: Missing setting X_PROBE_OFFSET_FROM_EXTRUDER
    #endif
    #if DISABLED(Y_PROBE_OFFSET_FROM_EXTRUDER)
      #error DEPENDENCY ERROR: Missing setting Y_PROBE_OFFSET_FROM_EXTRUDER
    #endif
    #if DISABLED(Z_PROBE_OFFSET_FROM_EXTRUDER)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_OFFSET_FROM_EXTRUDER
    #endif
    #if DISABLED(Z_PROBE_DEPLOY_START_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_DEPLOY_START_LOCATION
    #endif
    #if DISABLED(Z_PROBE_DEPLOY_END_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_DEPLOY_END_LOCATION
    #endif
    #if DISABLED(Z_PROBE_RETRACT_START_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_RETRACT_START_LOCATION
    #endif
    #if DISABLED(Z_PROBE_RETRACT_END_LOCATION)
      #error DEPENDENCY ERROR: Missing setting Z_PROBE_RETRACT_END_LOCATION
    #endif
    #if DISABLED(Z_RAISE_BETWEEN_PROBINGS)
      #error DEPENDENCY ERROR: Missing setting Z_RAISE_BETWEEN_PROBINGS
    #endif
    #if DISABLED(AUTO_BED_LEVELING_GRID_POINTS)
      #error DEPENDENCY ERROR: Missing setting AUTO_BED_LEVELING_GRID_POINTS
    #endif
  #endif
  
  /**
   * Board
   */
  #if DISABLED(KNOWN_BOARD)
    #error DEPENDENCY ERROR: You have to set a valid MOTHERBOARD.
  #endif
  
  /**
   * Mechanics
   */
  #if DISABLED(KNOWN_MECH)
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
    #if (MECH(COREXY) || MECH(COREYX)) && ENABLED(BABYSTEP_XY)
      #error CONFLICT ERROR: BABYSTEPPING only implemented for Z axis on CoreXY.
    #endif
    #if (MECH(COREXZ) || MECH(COREZX))
      #error CONFLICT ERROR: BABYSTEPPING not implemented for CoreXZ or CoreZX.
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
   * Required LCD for FILAMENT_CHANGE_FEATURE
   */
  #if ENABLED(FILAMENT_CHANGE_FEATURE) && DISABLED(ULTRA_LCD)
    #error DEPENDENCY ERROR: You must have LCD in order to use FILAMENT_CHANGE_FEATURE
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
      #if ENABLED(ENABLE_SERVOS)
        #if NUM_SERVOS < 1
          #error DEPENDENCY ERROR: You must have at least 1 servo EXIST for NUM_SERVOS to use Z_PROBE_ENDSTOP.
        #endif
        #if Z_ENDSTOP_SERVO_NR < 0
          #error DEPENDENCY ERROR: You must have Z_ENDSTOP_SERVO_NR set to at least 0 or above to use Z_PROBE_ENDSTOP.
        #endif
         #if DISABLED(Z_ENDSTOP_SERVO_ANGLES)
         #error DEPENDENCY ERROR: You must have Z_ENDSTOP_SERVO_ANGLES EXIST for Z Extend and Retract to use Z_PROBE_ENDSTOP.
        #endif
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
        #error CONFLICT ERROR: FRONT_PROBE_BED_POSITION must be less than BACK_PROBE_BED_POSITION.
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
    #if !PIN_EXISTS(Z_PROBE)
      #error DEPENDENCY ERROR: You must set Z_PROBE_PIN to a valid pin if you enable Z_PROBE_ENDSTOP
    #endif
  #endif

  /**
   * Dual X Carriage requirements
   */
  #if ENABLED(DUAL_X_CARRIAGE)
    #if EXTRUDERS == 1 || MECH(COREXY) \
        || HASNT(X2_ENABLE) || HASNT(X2_STEP) || HASNT(X2_DIR) \
        || DISABLED(X2_HOME_POS) || DISABLED(X2_MIN_POS) || DISABLED(X2_MAX_POS) \
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
  
  #if !PIN_EXISTS(X_STEP)
    #error DEPENDENCY ERROR: X_STEP_PIN is not defined for your board. You have to define it yourself.
  #endif
  #if !PIN_EXISTS(X_DIR)
    #error DEPENDENCY ERROR: X_DIR_PIN is not defined for your board. You have to define it yourself.
  #endif
  #if !PIN_EXISTS(X_ENABLE)
    #error DEPENDENCY ERROR: X_ENABLE_PIN is not defined for your board. You have to define it yourself.
  #endif
  #if !PIN_EXISTS(Y_STEP)
    #error DEPENDENCY ERROR: Y_STEP_PIN is not defined for your board. You have to define it yourself.
  #endif
  #if !PIN_EXISTS(Y_DIR)
    #error DEPENDENCY ERROR: Y_DIR_PIN is not defined for your board. You have to define it yourself.
  #endif
  #if !PIN_EXISTS(Y_ENABLE)
    #error DEPENDENCY ERROR: Y_ENABLE_PIN is not defined for your board. You have to define it yourself.
  #endif
  #if !PIN_EXISTS(Z_STEP)
    #error DEPENDENCY ERROR: Z_STEP_PIN is not defined for your board. You have to define it yourself.
  #endif
  #if !PIN_EXISTS(Z_DIR)
    #error DEPENDENCY ERROR: Z_DIR_PIN is not defined for your board. You have to define it yourself.
  #endif
  #if !PIN_EXISTS(Z_ENABLE)
    #error DEPENDENCY ERROR: Z_ENABLE_PIN is not defined for your board. You have to define it yourself.
  #endif
  
  #if DRIVER_EXTRUDERS > 0
    #if !PIN_EXISTS(E0_STEP)
      #error DEPENDENCY ERROR: E0_STEP_PIN is not defined for your board. You have to define it yourself.
    #endif
    #if !PIN_EXISTS(E0_DIR)
      #error DEPENDENCY ERROR: E0_DIR_PIN is not defined for your board. You have to define it yourself.
    #endif
    #if !PIN_EXISTS(E0_ENABLE)
      #error DEPENDENCY ERROR: E0_ENABLE_PIN is not defined for your board. You have to define it yourself.
    #endif
    #if DRIVER_EXTRUDERS > 1
      #if !PIN_EXISTS(E1_STEP)
        #error DEPENDENCY ERROR: E1_STEP_PIN is not defined for your board. You have to define it yourself.
      #endif
      #if !PIN_EXISTS(E1_DIR)
        #error DEPENDENCY ERROR: E1_DIR_PIN is not defined for your board. You have to define it yourself.
      #endif
      #if !PIN_EXISTS(E1_ENABLE)
        #error DEPENDENCY ERROR: E1_ENABLE_PIN is not defined for your board. You have to define it yourself.
      #endif
      #if DRIVER_EXTRUDERS > 2
        #if !PIN_EXISTS(E2_STEP)
          #error DEPENDENCY ERROR: E2_STEP_PIN is not defined for your board. You have to define it yourself.
        #endif
        #if !PIN_EXISTS(E2_DIR)
          #error DEPENDENCY ERROR: E2_DIR_PIN is not defined for your board. You have to define it yourself.
        #endif
        #if !PIN_EXISTS(E2_ENABLE)
          #error DEPENDENCY ERROR: E2_ENABLE_PIN is not defined for your board. You have to define it yourself.
        #endif
        #if DRIVER_EXTRUDERS > 3
          #if !PIN_EXISTS(E3_STEP)
            #error DEPENDENCY ERROR: E3_STEP_PIN is not defined for your board. You have to define it yourself.
          #endif
          #if !PIN_EXISTS(E3_DIR)
            #error DEPENDENCY ERROR: E3_DIR_PIN is not defined for your board. You have to define it yourself.
          #endif
          #if !PIN_EXISTS(E3_ENABLE)
            #error DEPENDENCY ERROR: E3_ENABLE_PIN is not defined for your board. You have to define it yourself.
          #endif
          #if DRIVER_EXTRUDERS > 4
            #if !PIN_EXISTS(E4_STEP)
              #error DEPENDENCY ERROR: E4_STEP_PIN is not defined for your board. You have to define it yourself.
            #endif
            #if !PIN_EXISTS(E4_DIR)
              #error DEPENDENCY ERROR: E4_DIR_PIN is not defined for your board. You have to define it yourself.
            #endif
            #if !PIN_EXISTS(E4_ENABLE)
              #error DEPENDENCY ERROR: E4_ENABLE_PIN is not defined for your board. You have to define it yourself.
            #endif
            #if DRIVER_EXTRUDERS > 5
              #if !PIN_EXISTS(E5_STEP)
                #error DEPENDENCY ERROR: E5_STEP_PIN is not defined for your board. You have to define it yourself.
              #endif
              #if !PIN_EXISTS(E5_DIR)
                #error DEPENDENCY ERROR: E5_DIR_PIN is not defined for your board. You have to define it yourself.
              #endif
              #if !PIN_EXISTS(E5_ENABLE)
                #error DEPENDENCY ERROR: E5_ENABLE_PIN is not defined for your board. You have to define it yourself.
              #endif
            #endif
          #endif
        #endif
      #endif
    #endif
  #endif
  
  #if ENABLED(MKR4)
    #if (EXTRUDERS == 2) && (DRIVER_EXTRUDERS == 1) && !PIN_EXISTS(E0E1_CHOICE)
      #error DEPENDENCY ERROR: You have to set E0E1_CHOICE_PIN to a valid pin if you enable MKR4 with 2 extruder and 1 driver
    #elif (EXTRUDERS == 3) && (DRIVER_EXTRUDERS == 1) && (!PIN_EXISTS(E0E1_CHOICE) || !PIN_EXISTS(E0E2_CHOICE))
      #error DEPENDENCY ERROR: You have to set E0E1_CHOICE_PIN and E0E2_CHOICE_PIN to a valid pin if you enable MKR4 with 3 extruder and 1 driver
    #elif (EXTRUDERS == 4) && (DRIVER_EXTRUDERS == 1) && (!PIN_EXISTS(E0E1_CHOICE) || !PIN_EXISTS(E0E2_CHOICE) || !PIN_EXISTS(E0E3_CHOICE))
      #error DEPENDENCY ERROR: You have to set E0E1_CHOICE_PIN, E0E2_CHOICE_PIN and E0E3_CHOICE_PIN to a valid pin if you enable MKR4 with 4 extruder and 1 driver
    #elif (EXTRUDERS == 5) && (DRIVER_EXTRUDERS == 1) && (!PIN_EXISTS(E0E1_CHOICE) || !PIN_EXISTS(E0E2_CHOICE) || !PIN_EXISTS(E0E3_CHOICE) || !PIN_EXISTS(E0E4_CHOICE))
      #error DEPENDENCY ERROR: You have to set E0E1_CHOICE_PIN, E0E2_CHOICE_PIN and E0E3_CHOICE_PIN to a valid pin if you enable MKR4 with 4 extruder and 1 driver
    #elif (EXTRUDERS == 6) && (DRIVER_EXTRUDERS == 1) && (!PIN_EXISTS(E0E1_CHOICE) || !PIN_EXISTS(E0E2_CHOICE) || !PIN_EXISTS(E0E3_CHOICE) || !PIN_EXISTS(E0E4_CHOICE) || !PIN_EXISTS(E0E5_CHOICE))
      #error DEPENDENCY ERROR: You have to set E0E1_CHOICE_PIN, E0E2_CHOICE_PIN and E0E3_CHOICE_PIN to a valid pin if you enable MKR4 with 4 extruder and 1 driver
    #elif (EXTRUDERS == 3) && (DRIVER_EXTRUDERS == 2) && !PIN_EXISTS(E0E2_CHOICE)
      #error DEPENDENCY ERROR: You have to set E0E2_CHOICE_PIN to a valid pin if you enable MKR4 with 3 extruder and 2 driver
    #elif (EXTRUDERS == 4) && (DRIVER_EXTRUDERS == 2) && (!PIN_EXISTS(E0E2_CHOICE) || !PIN_EXISTS(E1E3_CHOICE))
      #error DEPENDENCY ERROR: You have to set E0E2_CHOICE_PIN and E1E3_CHOICE_PIN to a valid pin if you enable MKR4 with 4 extruder and 2 driver
    #endif 
  #endif

  #if ENABLED(NPR2) && !PIN_EXISTS(E_MIN)
    #error DEPENDENCY ERROR: You have to set E_MIN_PIN to a valid pin if you enable NPR2
  #endif

  #if (ENABLED(DONDOLO_SINGLE_MOTOR) || ENABLED(DONDOLO_DUAL_MOTOR)) && HASNT(SERVOS)
    #error DEPENDENCY ERROR: You must enabled ENABLE_SERVOS and set NUM_SERVOS > 0 for DONDOLO MULTI EXTRUDER
  #endif

  #if ENABLED(DONDOLO_SINGLE_MOTOR) && ENABLED(DONDOLO_DUAL_MOTOR)
    #error DEPENDENCY ERROR: You must enabled only one for DONDOLO_SINGLE_MOTOR and DONDOLO_DUAL_MOTOR
  #endif

  #if (ENABLED(DONDOLO_SINGLE_MOTOR) || ENABLED(DONDOLO_DUAL_MOTOR)) && EXTRUDERS != 2
    #error DEPENDENCY ERROR: You must set EXTRUDERS = 2 for DONDOLO
  #endif

  #if ENABLED(LASERBEAM) 
    #if (!ENABLED(LASER_REMAP_INTENSITY) && ENABLED(LASER_RASTER))
      #error DEPENDENCY ERROR: You have to set LASER_REMAP_INTENSITY with LASER_RASTER enabled
    #endif
    #if (!ENABLED(LASER_CONTROL) || ((LASER_CONTROL != 1) && (LASER_CONTROL != 2)))
       #error DEPENDENCY ERROR: You have to set LASER_CONTROL to 1 or 2
    #else
      #if(LASER_CONTROL == 1)
        #if( !PIN_EXISTS(LASER_PWR))
          #error DEPENDENCY ERROR: You have to set LASER_PWR_PIN
        #endif
      #else
        #if( !PIN_EXISTS(LASER_PWR) || !PIN_EXISTS(LASER_TTL))
          #error DEPENDENCY ERROR: You have to set LASER_PWR_PIN and LASER_TTL_PIN to a valid pin if you enable LASER
        #endif
      #endif
    #endif
    #if DISABLED(LASER_HAS_FOCUS)
      #error DEPENDENCY ERROR: Missing LASER_HAS_FOCUS setting
    #endif
  #endif

  #if ENABLED(FILAMENT_RUNOUT_SENSOR) && !PIN_EXISTS(FILRUNOUT)
    #error DEPENDENCY ERROR: You have to set FILRUNOUT_PIN to a valid pin if you enable FILAMENT_RUNOUT_SENSOR
  #endif

  #if ENABLED(FILAMENT_SENSOR) && !PIN_EXISTS(FILWIDTH)
    #error DEPENDENCY ERROR: You have to set FILWIDTH_PIN to a valid pin if you enable FILAMENT_SENSOR
  #endif

  #if ENABLED(FILAMENT_SENSOR) && !PIN_EXISTS(FLOWMETER)
    #error DEPENDENCY ERROR: You have to set FLOWMETER_PIN to a valid pin if you enable FLOWMETER_SENSOR
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

  /**
   * Warnings for old configurations
   */
  #if WATCH_TEMP_PERIOD > 500
    #error "WATCH_TEMP_PERIOD now uses seconds instead of milliseconds."
  #elif defined(Z_RAISE_BEFORE_HOMING)
    #error "Z_RAISE_BEFORE_HOMING is deprecated. Use MIN_Z_HEIGHT_FOR_HOMING instead."
  #endif
#endif //SANITYCHECK_H
