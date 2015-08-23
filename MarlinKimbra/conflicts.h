/**
 * SanityCheck.h
 *
 * Test configuration values for errors at compile-time.
 */
#ifndef SANITYCHECK_H
  #define SANITYCHECK_H

  /**
   * Dual Stepper Drivers
   */
  #if ENABLED(Z_DUAL_STEPPER_DRIVERS) && ENABLED(Y_DUAL_STEPPER_DRIVERS)
    #error CONFLICT ERROR: CONFLICT ERROR: You cannot have dual stepper drivers for both Y and Z.
  #endif

  /**
   * Progress Bar
   */
  #if ENABLED(LCD_PROGRESS_BAR)
    #if DISABLED(SDSUPPORT)
      #error CONFLICT ERROR: LCD_PROGRESS_BAR requires SDSUPPORT.
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
  #if NUM_SERVOS > 0
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
    #error CONFLICT ERROR: At least one of the ?_ENDSTOP_SERVO_NR is required for DEACTIVATE_SERVOS_AFTER_MOVE.
  #endif

  /**
   * Required LCD language
   */
  #if DISABLED(DOGLCD) && ENABLED(ULTRA_LCD) && DISABLED(DISPLAY_CHARSET_HD44780_JAPAN) && DISABLED(DISPLAY_CHARSET_HD44780_WESTERN) && DISABLED(DISPLAY_CHARSET_HD44780_CYRILLIC)
    #error CONFLICT ERROR: You must enable either DISPLAY_CHARSET_HD44780_JAPAN or DISPLAY_CHARSET_HD44780_WESTERN  or DISPLAY_CHARSET_HD44780_CYRILLIC for your LCD controller.
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
          #error CONFLICT ERROR: You must have a Z_MIN or Z_PROBE endstop to enable Z_PROBE_REPEATABILITY_TEST.
        #else
          #error CONFLICT ERROR: AUTO_BED_LEVELING_FEATURE requires a Z_MIN or Z_PROBE endstop. Z_MIN_PIN or Z_PROBE_PIN must point to a valid hardware pin.
        #endif
      #endif
    #endif

    /**
     * Require a Z Probe Pin if Z_PROBE_ENDSTOP is enabled.
     */
    #if ENABLED(Z_PROBE_ENDSTOP)
      #if NOTEXIST(Z_PROBE_PIN)
        #error CONFLICT ERROR: You must have a Z_PROBE_PIN defined in pins2tool.h file if you enable Z_PROBE_ENDSTOP.
        #erro sistema pins2tool.h
      #endif
      #if Z_PROBE_PIN == -1
        #error CONFLICT ERROR: You must set Z_PROBE_PIN to a valid pin if you enable Z_PROBE_ENDSTOP.
      #endif

        #if DISABLED(ENABLE_SERVOS)
          #error CONFLICT ERROR: You must enable ENABLE_SERVOS and must have NUM_SERVOS defined and there must be at least 1 configured to use Z_PROBE_ENDSTOP.
        #endif
        #if NUM_SERVOS < 1
          #error CONFLICT ERROR: You must have at least 1 servo defined for NUM_SERVOS to use Z_PROBE_ENDSTOP.
        #endif
        #if Z_ENDSTOP_SERVO_NR < 0
          #error CONFLICT ERROR: You must have Z_ENDSTOP_SERVO_NR set to at least 0 or above to use Z_PROBE_ENDSTOP.
        #endif
        #if NOTEXIST(SERVO_ENDSTOP_ANGLES)
          #error CONFLICT ERROR: You must have SERVO_ENDSTOP_ANGLES defined for Z Extend and Retract to use Z_PROBE_ENDSTOP.
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
    #error CONFLICT ERROR: ULTIPANEL requires some kind of encoder.
  #endif

  /**
   * Delta & Z_PROBE_ENDSTOP
   */
  #if MECH(DELTA) && ENABLED(Z_PROBE_ENDSTOP)
    #if NOTEXIST(Z_PROBE_PIN)
      #error CONFLICT ERROR: You must have a Z_PROBE_PIN defined in your pins2tool.h file if you enable Z_PROBE_ENDSTOP
    #endif
    #if Z_PROBE_PIN == -1
      #error CONFLICT ERROR: You must set Z_PROBE_PIN to a valid pin if you enable Z_PROBE_ENDSTOP
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
      #error CONFLICT ERROR: Missing or invalid definitions for DUAL_X_CARRIAGE mode.
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
      #error CONFLICT ERROR: HEATER_3_PIN not defined for this board
    #endif
  #elif HOTENDS > 2
    #if HASNT(HEATER_2)
      #error CONFLICT ERROR: HEATER_2_PIN not defined for this board
    #endif
  #elif HOTENDS > 1 || ENABLED(HEATERS_PARALLEL)
    #if HASNT(HEATER_1)
      #error CONFLICT ERROR: HEATER_1_PIN not defined for this board
    #endif
  #elif HOTENDS > 0
    #if HASNT(HEATER_0)
      #error CONFLICT ERROR: HEATER_0_PIN not defined for this board
    #endif
  #endif
  
  #if DISABLED(SDSUPPORT) && ENABLED(SD_SETTINGS)
    #error CONFLICT ERROR: You have to enable SDSUPPORT to use SD_SETTINGS
  #endif

  /**
   * Warnings for old configurations
   */
  #if ENABLED(X_HOME_RETRACT_MM)
    #error CONFLICT ERROR: [XYZ]_HOME_RETRACT_MM settings have been renamed [XYZ]_HOME_BUMP_MM.
  #endif

  #if WATCH_TEMP_PERIOD > 500
    #error CONFLICT ERROR: WATCH_TEMP_PERIOD now uses seconds instead of milliseconds.
  #endif

  #if DISABLED(THERMAL_PROTECTION_HOTENDS) && (ENABLED(WATCH_TEMP_PERIOD) || ENABLED(THERMAL_PROTECTION_PERIOD))
    #error CONFLICT ERROR: Thermal Runaway Protection for hotends must now be enabled with THERMAL_PROTECTION_HOTENDS.
  #endif

  #if DISABLED(THERMAL_PROTECTION_BED) && ENABLED(THERMAL_PROTECTION_BED_PERIOD)
    #error CONFLICT ERROR: Thermal Runaway Protection for the bed must now be enabled with THERMAL_PROTECTION_BED.
  #endif

  #if ENABLED(PROBE_SERVO_DEACTIVATION_DELAY)
    #error CONFLICT ERROR: PROBE_SERVO_DEACTIVATION_DELAY has been replaced with DEACTIVATE_SERVOS_AFTER_MOVE and SERVO_DEACTIVATION_DELAY.
  #endif

  #if MECH(COREXZ) && ENABLED(Z_LATE_ENABLE)
    #error CONFLICT ERROR: "Z_LATE_ENABLE can't be used with COREXZ."
  #endif

  #if ENABLED(BEEPER)
    #error CONFLICT ERROR: BEEPER has been replaced with BEEPER_PIN. Please update your pins definitions.
  #endif

  #if ENABLED(SDCARDDETECT)
    #error CONFLICT ERROR: SDCARDDETECT is now SD_DETECT_PIN. Please update your pins definitions.
  #endif

  #if ENABLED(SDCARDDETECTINVERTED)
    #error CONFLICT ERROR: SDCARDDETECTINVERTED is now SD_DETECT_INVERTED. Please update your configuration.
  #endif

  #if ENABLED(POWER_CONSUMPTION) && !PIN_EXISTS(POWER_CONSUMPTION)
    #error CONFLICT ERROR: You have to set a valid POWER_CONSUMPTION_PIN in pins.h in order to use this feature. 
  #endif

#endif //SANITYCHECK_H
