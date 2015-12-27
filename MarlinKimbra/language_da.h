/**
 * Danish
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_DA_H
#define LANGUAGE_DA_H

#define MAPPER_C2C3
#define DISPLAY_CHARSET_ISO10646_1







#define WELCOME_MSG                         MACHINE_NAME " er klar"

#define MSG_SD_INSERTED                     "Kort isat"
#define MSG_SD_REMOVED                      "Kort fjernet"
#define MSG_MAIN                            "Main"
#define MSG_AUTOSTART                       "Autostart"
#define MSG_DISABLE_STEPPERS                "Disable steppers"
#define MSG_AUTO_HOME                       "Home" // G28
#define MSG_MBL_SETTING                     "Manual Bed Leveling"
#define MSG_MBL_BUTTON                      " Press the button   "
#define MSG_MBL_INTRO                       " Leveling bed...    "
#define MSG_MBL_1                           " Adjust first point "
#define MSG_MBL_2                           " Adjust second point"
#define MSG_MBL_3                           " Adjust third point "
#define MSG_MBL_4                           " Adjust fourth point"
#define MSG_MBL_5                           "    Is it ok?       "
#define MSG_MBL_6                           " BED leveled!       "
#define MSG_SET_HOME_OFFSETS                "Set home offsets"
#define MSG_SET_ORIGIN                      "Set origin"
#define MSG_ONFOR                           "On x:"
#define MSG_PWRCONSUMED                     "P.er:"
#define MSG_DISABLE_STEPPERS                "Slå stepper fra"
#define MSG_SET_HOME_OFFSETS                "Sæt home offsets"
#define MSG_SET_ORIGIN                      "Sæt origin"
#define MSG_SWITCH_PS_ON                    "Slå strøm til"
#define MSG_SWITCH_PS_OFF                   "Slå strøm fra"


#define MSG_PREHEAT_PLA                     "Forvarm PLA"
#define MSG_PREHEAT_PLA_N                   "Forvarm PLA "
#define MSG_PREHEAT_PLA_ALL                 "Forvarm PLA Alle"
#define MSG_PREHEAT_PLA_BEDONLY             "Forvarm PLA Bed"
#define MSG_PREHEAT_PLA_SETTINGS            "Forvarm PLA conf"
#define MSG_PREHEAT_ABS                     "Forvarm ABS"
#define MSG_PREHEAT_ABS_N                   "Forvarm ABS "
#define MSG_PREHEAT_ABS_ALL                 "Forvarm ABS Alle"
#define MSG_PREHEAT_ABS_BEDONLY             "Forvarm ABS Bed"
#define MSG_PREHEAT_ABS_SETTINGS            "Forvarm ABS conf"
#define MSG_EXTRUDE                         "Extruder"
#define MSG_COOLDOWN                        "Afkøl"
#define MSG_RETRACT                         "Retract"
#define MSG_PURGE                           "Purge"
#define MSG_LEVEL_BED                       "Level bed"
#define MSG_SPEED                           "Hastighed"
#define MSG_NOZZLE                          "Dyse"
#define MSG_BED                             "Plade"
#define MSG_FAN_SPEED                       "Blæser hastighed"
#define MSG_FLOW                            "Flow"
#define MSG_CONTROL                         "Kontrol"
#define MSG_STATS                           "Statistics"
#define MSG_FIX_LOSE_STEPS                  "Fix axis steps"
#define MSG_MIN                             LCD_STR_THERMOMETER " Min"
#define MSG_MAX                             LCD_STR_THERMOMETER " Max"
#define MSG_FACTOR                          LCD_STR_THERMOMETER " Fact"
#define MSG_IDLEOOZING                      "Anti oozing"
#define MSG_AUTOTEMP                        "Autotemp"
#define MSG_ON                              "On "
#define MSG_OFF                             "Off"
#define MSG_PID_P                           "PID-P"
#define MSG_PID_I                           "PID-I"
#define MSG_PID_D                           "PID-D"
#define MSG_H1                              " H1"
#define MSG_H2                              " H2"
#define MSG_H3                              " H3"
#define MSG_ACC                             "Accel"
#define MSG_VXY_JERK                        "Vxy-jerk"
#define MSG_VZ_JERK                         "Vz-jerk"
#define MSG_VE_JERK                         "Ve-jerk"
#define MSG_VMAX                            "Vmax "
#define MSG_X                               "X"
#define MSG_Y                               "Y"
#define MSG_Z                               "Z"
#define MSG_E                               "E"

#define MSG_MOVE_AXIS                       "Flyt akser"
#define MSG_MOVE_X                          "Flyt X"
#define MSG_MOVE_Y                          "Flyt Y"
#define MSG_MOVE_Z                          "Flyt Z"
#define MSG_MOVE_01MM                       "Flyt 0.1mm"
#define MSG_MOVE_1MM                        "Flyt 1mm"
#define MSG_MOVE_10MM                       "Flyt 10mm"
#define MSG_MOVE_E                          "Extruder"
#define MSG_VMIN                            "Vmin"
#define MSG_VTRAV_MIN                       "VTrav min"
#define MSG_AMAX                            "Amax "
#define MSG_A_RETRACT                       "A-retract"
#define MSG_A_TRAVEL                        "A-travel"
#define MSG_XSTEPS                          "X steps/mm"
#define MSG_YSTEPS                          "Y steps/mm"
#define MSG_ZSTEPS                          "Z steps/mm"
#define MSG_E0STEPS                         "E0 steps/mm"
#define MSG_E1STEPS                         "E1 steps/mm"
#define MSG_E2STEPS                         "E2 steps/mm"
#define MSG_E3STEPS                         "E3 steps/mm"
#define MSG_TEMPERATURE                     "Temperatur"
#define MSG_MOTION                          "Motion"
#define MSG_VOLUMETRIC                      "Filament"
#define MSG_VOLUMETRIC_ENABLED              "E in mm" STR_h3
#define MSG_FILAMENT_SIZE_EXTRUDER          "Fil. Dia."
#define MSG_CONTRAST                        "LCD kontrast"
#define MSG_STORE_EPROM                     "Gem i EEPROM"
#define MSG_LOAD_EPROM                      "Hent fra EEPROM"
#define MSG_RESTORE_FAILSAFE                "Gendan failsafe"
#define MSG_REFRESH                         "Genopfrisk"
#define MSG_WATCH                           "Info skærm"
#define MSG_PREPARE                         "Forbered"
#define MSG_TUNE                            "Tune"
#define MSG_PAUSE_PRINT                     "Pause printet"
#define MSG_RESUME_PRINT                    "Forsæt printet"
#define MSG_STOP_PRINT                      "Stop printet"
#define MSG_CARD_MENU                       "Print fra SD"
#define MSG_NO_CARD                         "Intet SD kort"
#define MSG_DWELL                           "Dvale..."
#define MSG_USERWAIT                        "Venter på bruger..."
#define MSG_RESUMING                        "Forsætter printet"
#define MSG_PRINT_ABORTED                   "Print annuleret"
#define MSG_NO_MOVE                         "No move."
#define MSG_KILLED                          "KILLED. "
#define MSG_STOPPED                         "STOPPED. "
#define MSG_CONTROL_RETRACT                 "Tilbagetraek mm"
#define MSG_CONTROL_RETRACT_SWAP            "Skift Re.mm"
#define MSG_CONTROL_RETRACTF                "Tilbagetræk V"
#define MSG_CONTROL_RETRACT_ZLIFT           "Hop mm"
#define MSG_CONTROL_RETRACT_RECOVER         "UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "S UnRet+mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "UnRet  V"
#define MSG_AUTORETRACT                     "AutoRetr."
#define MSG_FILAMENTCHANGE                  "Skift filament"
#define MSG_INIT_SDCARD                     "Init. SD card"
#define MSG_CNG_SDCARD                      "Skift SD kort"
#define MSG_ZPROBE_OUT                      "Probe udenfor plade"
#define MSG_POSITION_UNKNOWN                "Home X/Y før Z"
#define MSG_ZPROBE_ZOFFSET                  "Z Offset"
#define MSG_BABYSTEP                        "Babystep"
#define MSG_BABYSTEP_X                      MSG_BABYSTEP " " MSG_X
#define MSG_BABYSTEP_Y                      MSG_BABYSTEP " " MSG_Y
#define MSG_BABYSTEP_Z                      MSG_BABYSTEP " " MSG_Z
#define MSG_ENDSTOP_ABORT                   "Endstop abort"
#define MSG_HEATING_FAILED_LCD              "Heating failed"
#define MSG_ERR_REDUNDANT_TEMP              "Err: REDUNDANT TEMP ERROR"
#define MSG_THERMAL_RUNAWAY                 "THERMAL RUNAWAY"
#define MSG_HOTEND_AD595                    "HOTEND AD595 Offset & Gain"
#define MSG_ERR_MAXTEMP                     "Err: MAXTEMP"
#define MSG_ERR_MINTEMP                     "Err: MINTEMP"
#define MSG_ERR_MAXTEMP_BED                 "Err: MAXTEMP BED"
#define MSG_ERR_MINTEMP_BED                 "Err: MINTEMP BED"
#define MSG_END_DAY                         "days"
#define MSG_END_HOUR                        "timer"
#define MSG_END_MINUTE                      "minutter"

#define MSG_ENDSTOPS_HIT                    "endstops hit: "
#define MSG_BABYSTEPPING                    "Babystepping"
#define MSG_BABYSTEPPING_X                  MSG_BABYSTEPPING " " MSG_X
#define MSG_BABYSTEPPING_Y                  MSG_BABYSTEPPING " " MSG_Y
#define MSG_BABYSTEPPING_Z                  MSG_BABYSTEPPING " " MSG_Z

#define MSG_ENDSTOP_XS                      "X"
#define MSG_ENDSTOP_YS                      "Y"
#define MSG_ENDSTOP_ZS                      "Z"
#define MSG_ENDSTOP_ZPS                     "ZP"
#define MSG_ENDSTOP_ES                      "E"

// Debug
#define MSG_DEBUG_ECHO                      "DEBUG ECHO ENABLED"
#define MSG_DEBUG_INFO                      "DEBUG INFO ENABLED"
#define MSG_DEBUG_ERRORS                    "DEBUG ERRORS ENABLED"
#define MSG_DEBUG_DRYRUN                    "DEBUG DRYRUN ENABLED"
#define MSG_DEBUG                           "DEBUG ENABLED"

// Calibrate Delta
#if MECH(DELTA)
  #define MSG_DELTA_CALIBRATE               "Delta Kalibrering"
  #define MSG_DELTA_CALIBRATE               "Calibrate"
  #define MSG_DELTA_CALIBRATE_X             MSG_DELTA_CALIBRATE " " MSG_X
  #define MSG_DELTA_CALIBRATE_Y             MSG_DELTA_CALIBRATE " " MSG_Y
  #define MSG_DELTA_CALIBRATE_Z             MSG_DELTA_CALIBRATE " " MSG_Z
  #define MSG_DELTA_CALIBRATE_CENTER        MSG_DELTA_CALIBRATE " Center"
#endif // DELTA

// Scara
#if MECH(SCARA)
  #define MSG_SCALE                         "Scale"
  #define MSG_XSCALE                        MSG_X " " MSG_SCALE
  #define MSG_YSCALE                        MSG_Y " " MSG_SCALE
#endif

#define MSG_HEATING                         "Opvarmer..."
#define MSG_HEATING_COMPLETE                "Opvarmet"
#define MSG_BED_HEATING                     "Opvarmer plade"
#define MSG_BED_DONE                        "Plade opvarmet"

// Extra
#define MSG_LASER                           "Laser Preset"
#define MSG_CONFIG                          "Configuration"
#define MSG_E_BOWDEN_LENGTH                 MSG_EXTRUDE " " STRINGIFY(BOWDEN_LENGTH) "mm"
#define MSG_R_BOWDEN_LENGTH                 MSG_RETRACT " " STRINGIFY(BOWDEN_LENGTH) "mm"
#define MSG_PURGE_XMM                       MSG_PURGE " " STRINGIFY(LCD_PURGE_LENGTH) "mm"
#define MSG_RETRACT_XMM                     MSG_RETRACT " " STRINGIFY(LCD_RETRACT_LENGTH) "mm"
#define MSG_SAVED_POS                       "Saved position"
#define MSG_RESTORING_POS                   "Restoring position"
#define MSG_INVALID_POS_SLOT                "Invalid slot, total slots: "

// Firmware Test
#if ENABLED(FIRMWARE_TEST)
  #define MSG_FWTEST_YES                    "Put the Y command to go next"
  #define MSG_FWTEST_NO                     "Put the N command to go next"
  #define MSG_FWTEST_YES_NO                 "Put the Y or N command to go next"
  #define MSG_FWTEST_ENDSTOP_ERR            "ENDSTOP ERROR! Check wire and connection"
  #define MSG_FWTEST_PRESS                  "Press and hold the endstop "
  #define MSG_FWTEST_INVERT                 "Reverse value of "
  #define MSG_FWTEST_XAXIS                  "Has the nozzle moved to the right?"
  #define MSG_FWTEST_YAXIS                  "Has the nozzle moved forward?"
  #define MSG_FWTEST_ZAXIS                  "Has the nozzle moved up?"
  #define MSG_FWTEST_01                     "Manually move the axes X, Y and Z away from the endstop"
  #define MSG_FWTEST_02                     "Do you want check ENDSTOP?"
  #define MSG_FWTEST_03                     "Start check ENDSTOP"
  #define MSG_FWTEST_04                     "Start check MOTOR"
  #define MSG_FWTEST_ATTENTION              "ATTENTION! Check that the three axes are more than 5 mm from the endstop!"
  #define MSG_FWTEST_END                    "Finish Test. Disable FIRMWARE_TEST and recompile."
  #define MSG_FWTEST_INTO                   "into "
  #define MSG_FWTEST_ERROR                  "ERROR"
  #define MSG_FWTEST_OK                     "OK"
  #define MSG_FWTEST_NDEF                   "not defined"
#endif // FIRMWARE_TEST

#endif // LANGUAGE_DA_H
