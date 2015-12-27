/**
 * Japanese (Kana)
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
 
#ifndef LANGUAGE_KANA_H
#define LANGUAGE_KANA_H

#define MAPPER_NON
#define SIMULATE_ROMFONT
#define DISPLAY_CHARSET_ISO10646_KANA




// 片仮名表示定義
#define WELCOME_MSG                         MACHINE_NAME " ready."

#define MSG_SD_INSERTED                     "\xb6\xb0\xc4\xde\x20\xbf\xb3\xc6\xad\xb3\xbb\xda\xcf\xbc\xc0" // "Card inserted"
#define MSG_SD_REMOVED                      "\xb6\xb0\xc4\xde\xb6xde\xb1\xd8\xcf\xbe\xdd"                  // "Card removed"
#define MSG_MAIN                            "\xd2\xb2\xdd"                                                 // "Main"
#define MSG_AUTOSTART                       "\xbc\xde\xc4\xde\xb3\xb6\xb2\xbc"                             // "Autostart"
#define MSG_DISABLE_STEPPERS                "\xd3\xb0\xc0\xb0\xc3\xde\xdd\xb9\xde\xdd\x20\xb5\xcc"         // "Disable steppers"
#define MSG_AUTO_HOME                       "\xb9\xde\xdd\xc3\xdd\xc6\xb2\xc4\xde\xb3"                     // "Auto home"
#define MSG_MBL_SETTING                     "Manual Bed Leveling"
#define MSG_MBL_BUTTON                      " Press the button   "
#define MSG_MBL_INTRO                       " Leveling bed...    "
#define MSG_MBL_1                           " Adjust first point "
#define MSG_MBL_2                           " Adjust second point"
#define MSG_MBL_3                           " Adjust third point "
#define MSG_MBL_4                           " Adjust fourth point"
#define MSG_MBL_5                           "    Is it ok?       "
#define MSG_MBL_6                           " BED leveled!       "
#define MSG_SET_HOME_OFFSETS                "\xb7\xbc\xde\xad\xdd\xb5\xcc\xbe\xaf\xc4\xbe\xaf\xc3\xb2"     // "Set home offsets"
#define MSG_SET_ORIGIN                      "\xb7\xbc\xde\xad\xdd\xbe\xaf\xc4"                             // "Set origin"
#define MSG_ONFOR                           "On x:"
#define MSG_PWRCONSUMED                     "P.er:"
#define MSG_PREHEAT                         "Preheat"
#define MSG_CONGIG                          "conf."
#define MSG_PREHEAT_PLA                     "PLA \xd6\xc8\xc2"
#define MSG_PREHEAT_PLA_ALL                 MSG_PREHEAT_PLA " \xbd\xcd\xde\xc3"                            // " All"
#define MSG_PREHEAT_PLA_BEDONLY             MSG_PREHEAT_PLA " \xcd\xde\xaf\xc4\xde"                        // "Bed"
#define MSG_PREHEAT_PLA_SETTINGS            MSG_PREHEAT_PLA " \xbe\xaf\xc3\xb2"                            // "conf"
#define MSG_PREHEAT_ABS                     "ABS \xd6\xc8\xc2" 
#define MSG_PREHEAT_ABS_ALL                 MSG_PREHEAT_ABS " \xbd\xcd\xde\xc3"                            // " All"
#define MSG_PREHEAT_ABS_BEDONLY             MSG_PREHEAT_ABS " \xcd\xde\xaf\xc4\xde"                        // "Bed"
#define MSG_PREHEAT_ABS_SETTINGS            MSG_PREHEAT_ABS " \xbe\xaf\xc3\xb2"                            // "conf"
#define MSG_PREHEAT_GUM                     "Preheat GUM"
#define MSG_PREHEAT_GUM_ALL                 "Preheat GUM All"
#define MSG_PREHEAT_GUM_BEDONLY             "Preheat GUM Bed"
#define MSG_PREHEAT_GUM_SETTINGS            "Preheat GUM conf"
#define MSG_TOO_COLD_FOR_FILAMENTCHANGE     "Hotend too cold to change filament"
#define MSG_COOLDOWN                        "\xb6\xc8\xc2\xc3\xb2\xbc"                                     // "Cooldown"
#define MSG_SWITCH_PS_ON                    "\xc3\xde\xdd\xb9\xdd\xde\x20\xb5\xdd"                         // "Switch power on"
#define MSG_SWITCH_PS_OFF                   "\xc3\xde\xdd\xb9\xdd\xde\x20\xb5\xcc"                         // "Switch power off"
#define MSG_EXTRUDE                         "\xb5\xbc\xc0\xde\xbc"                                         // "Extrude"
#define MSG_RETRACT                         "\xd8\xc4\xd7\xb8\xc4"                                         // "Retract"
#define MSG_PURGE                           "Purge"
#define MSG_LEVEL_BED                       "Level bed"
#define MSG_SPEED                           "\xbd\xcb\xdf\xb0\xc4\xde"                                     // "Speed"
#define MSG_NOZZLE                          "\xc9\xbd\xde\xd9"                                             // "Nozzle"
#define MSG_BED                             "\xcd\xde\xaf\xc4\xde"                                         // "Bed"
#define MSG_FAN_SPEED                       "\xcc\xa7\xdd\xbf\xb8\xc4\xde"                                 // "Fan speed"
#define MSG_FLOW                            "\xb5\xb8\xd8\xd8\xae\xb3"                                     // "Flow"
#define MSG_CONTROL                         "\xba\xdd\xc4\xdb\xb0\xd9"                                     // "Control"
#define MSG_STATS                           "Statistics"
#define MSG_FIX_LOSE_STEPS                  "Fix axis steps"
#define MSG_MIN                             LCD_STR_THERMOMETER " Min"
#define MSG_MAX                             LCD_STR_THERMOMETER " Max"
#define MSG_FACTOR                          LCD_STR_THERMOMETER " Fact"
#define MSG_IDLEOOZING                      "Anti oozing"
#define MSG_AUTOTEMP                        "\xbc\xde\xc4\xde\xb3\xb5\xdd\xc4\xde"                         // "Autotemp"
#define MSG_ON                              "On "
#define MSG_OFF                             "Off"
#define MSG_PID_P                           "PID-P"
#define MSG_PID_I                           "PID-I"
#define MSG_PID_D                           "PID-D"
#define MSG_H1                              " H1"
#define MSG_H2                              " H2"
#define MSG_H3                              " H3"
#define MSG_ACC                             "\xb6\xbf\xb8\xc4\xde"                                         // "Accel"
#define MSG_VXY_JERK                        "Vxy-jerk"
#define MSG_VZ_JERK                         "Vz-jerk"
#define MSG_VE_JERK                         "Ve-jerk"
#define MSG_VMAX                            "Vmax "
#define MSG_X                               "X"
#define MSG_Y                               "Y"
#define MSG_Z                               "Z"
#define MSG_E                               "E"

#define MSG_MOVE_AXIS                       "\xbc\xde\xb8\xb2\xc4\xde\xb3"                                 // "Move axis"
#define MSG_MOVE_X                          "X\xbc\xde\xb8\x20\xb2\xc4\xde\xb3"                            // "Move X"
#define MSG_MOVE_Y                          "Y\xbc\xde\xb8\x20\xb2\xc4\xde\xb3"                            // "Move Y"
#define MSG_MOVE_Z                          "Z\xbc\xde\xb8\x20\xb2\xc4\xde\xb3"                            // "Move Z"
#define MSG_MOVE_01MM                       "0.1mm \xb2\xc4\xde\xb3"                                       // "Move 0.1mm"
#define MSG_MOVE_1MM                        "  1mm \xb2\xc4\xde\xb3"                                       // "Move 1mm"
#define MSG_MOVE_10MM                       " 10mm \xb2\xc4\xde\xb3"                                       // "Move 10mm"
#define MSG_MOVE_E                          "\xb4\xb8\xbd\xc4\xd9\xb0\xc0\xde\xb0"                         // "Extruder"
#define MSG_VMIN                            "Vmin"
#define MSG_VTRAV_MIN                       "VTrav min"
#define MSG_AMAX                            "Amax "
#define MSG_A_RETRACT                       "A-retract"
#define MSG_A_TRAVEL                        "A-travel"
#define MSG_XSTEPS                          "X steps/mm"
#define MSG_YSTEPS                          "Y steps/mm"
#define MSG_ZSTEPS                          "Z steps/mm"
#define MSG_E0STEPS                         "E 0steps/mm"
#define MSG_E1STEPS                         "E1 steps/mm"
#define MSG_E2STEPS                         "E2 steps/mm"
#define MSG_E3STEPS                         "E3 steps/mm"
#define MSG_TEMPERATURE                     "\xb5\xdd\xc4\xde"                                             // "Temperature"
#define MSG_MOTION                          "\xb3\xba\xde\xb7\xbe\xaf\xc3\xb2"                             // "Motion"
#define MSG_FILAMENT                        "\xcc\xa8\xd7\xd2\xdd\xc4"                                     // "Filament"
#define MSG_VOLUMETRIC_ENABLED              "E in mm" STR_h3
#define MSG_FILAMENT_SIZE_EXTRUDER          "Fil. Dia."
#define MSG_CONTRAST                        "LCD\xba\xdd\xc4\xd7\xbd\xc4"                                  // "LCD contrast"
#define MSG_STORE_EPROM                     "\xd2\xd3\xd8\xcd\xb6\xb8\xc9\xb3"                             // "Store memory"
#define MSG_LOAD_EPROM                      "\xd2\xd3\xd8\xb6\xd7\xd6\xd0\ba\xd0"                          // "Load memory"
#define MSG_RESTORE_FAILSAFE                "\xbe\xaf\xc3\xb2\xd8\xbe\xaf\xc4"                             // "Restore failsafe"
#define MSG_REFRESH                         "\xd8\xcc\xda\xaf\xbc\xad"                                     // "Refresh"
#define MSG_WATCH                           "\xb2\xdd\xcc\xab"                                             // "Info screen"
#define MSG_PREPARE                         "\xbc\xde\xad\xdd\xcb\xde\xbe\xaf\xc3\xb2"                     // "Prepare"
#define MSG_TUNE                            "\xc1\xae\xb3\xbe\xb2"                                         // "Tune"
#define MSG_PAUSE_PRINT                     "\xb2\xc1\xbc\xde\xc3\xb2\xbc"                                 // "Pause print"
#define MSG_RESUME_PRINT                    "\xcc\xdf\xd8\xdd\xc4\xbb\xb2\xb6\xb2"                         // "Resume print"
#define MSG_STOP_PRINT                      "\xcc\xdf\xd8\xdd\xc4\xc3\xb2\xbc"                             // "Stop print"
#define MSG_CARD_MENU                       "SD\xb6\xb0\xc4\xde\xb6\xd7\xcc\xdf\xd8\xdd\xc4"               // "Print from SD"
#define MSG_NO_CARD                         "SD\xb6\xb0\xc4\xde\xb6\xde\xb1\xd8\xcf\xbe\xdd"               // "No SD card"
#define MSG_DWELL                           "\xbd\xd8\xb0\xcc\xdf"                                         // "Sleep..."
#define MSG_USERWAIT                        "\xbc\xca\xde\xd7\xb9\xb5\xcf\xc1\xb8\xc0\xde\xbb\xb2"         // "Wait for user..."
#define MSG_RESUMING                        "\xcc\xdf\xd8\xdd\xc4\xbb\xb2\xb6\xb2"                         // "Resuming print"
#define MSG_PRINT_ABORTED                   "\xcc\xdf\xd8\xdd\xc4\xc1\xad\xb3\xbc\xbb\xda\xcf\xbc\xc0"     // "Print aborted"
#define MSG_NO_MOVE                         "\xb3\xba\xde\xb7\xcf\xbe\xdd"                                 // "No move."
#define MSG_KILLED                          "\xbc\xae\xb3\xb7\xae"                                         // "KILLED. "
#define MSG_STOPPED                         "\xc3\xb2\xbc\xbc\xcf\xbc\xc0"                                 // "STOPPED. "
#define MSG_CONTROL_RETRACT                 "Retract mm"
#define MSG_CONTROL_RETRACT_SWAP            "Swap Re.mm"
#define MSG_CONTROL_RETRACTF                "Retract  F"
#define MSG_CONTROL_RETRACT_ZLIFT           "Hop mm"
#define MSG_CONTROL_RETRACT_RECOVER         "UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "S UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "UnRet  F"
#define MSG_AUTORETRACT                     "AutoRetr."
#define MSG_FILAMENTCHANGE                  "\xcc\xa8\xd7\xd2\xdd\xc4\xba\xb3\xb6\xdd"                     // "Change filament"
#define MSG_INIT_SDCARD                     "SD\xb6\xb0\xc4\xde\xbb\xb2\xd6\xd0\xba\xd0"                   // "Init. SD card"
#define MSG_CNG_SDCARD                      "SD\xb6\xb0\xc4\xde\xba\xb3\xb6\xdd"                           // "Change SD card"
#define MSG_ZPROBE_OUT                      "Z\xcc\xdf\xdb\xb0\xcc\xde \xcd\xde\xaf\xc4\xee\xb6\xde\xb2"   // "Z probe out. bed"
#define MSG_POSITION_UNKNOWN                "\xb9\xde\xdd\xc3\xdd\xcaXY\xb2\xc4\xde\xb3\xba\xdeZ"          // "Home X/Y before Z"
#define MSG_ZPROBE_ZOFFSET                  "Z\xb5\xcc\xbe\xaf\xc4"                                        // "Z Offset"
#define MSG_BABYSTEP                        ""\xcb\xde\xc4\xde\xb3"
#define MSG_BABYSTEP_X                      MSG_BABYSTEP " "  MSG_X                                        // "Babystep X"
#define MSG_BABYSTEP_Y                      MSG_BABYSTEP " "  MSG_Y                                        // "Babystep Y"
#define MSG_BABYSTEP_Z                      MSG_BABYSTEP " "  MSG_Z                                        // "Babystep Z"
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
#define MSG_END_HOUR                        "hours"
#define MSG_END_MINUTE                      "minutes"

#define MSG_ENDSTOPS_HIT                    "endstops hit: "
#define MSG_BABYSTEPPING                    "Babystepping"
#define MSG_BABYSTEPPING_X                  MSG_BABYSTEPPING " " MSG_X
#define MSG_BABYSTEPPING_Y                  MSG_BABYSTEPPING " " MSG_Y
#define MSG_BABYSTEPPING_Z                  MSG_BABYSTEPPING " " MSG_Z

#define MSG_ENDSTOP_XS                      MSG_X
#define MSG_ENDSTOP_YS                      MSG_Y
#define MSG_ENDSTOP_ZS                      MSG_Z
#define MSG_ENDSTOP_ZPS                     MSG_Z "P"
#define MSG_ENDSTOP_ES                      MSG_E

// Debug
#define MSG_DEBUG_ECHO                      "DEBUG ECHO ENABLED"
#define MSG_DEBUG_INFO                      "DEBUG INFO ENABLED"
#define MSG_DEBUG_ERRORS                    "DEBUG ERRORS ENABLED"
#define MSG_DEBUG_DRYRUN                    "DEBUG DRYRUN ENABLED"
#define MSG_DEBUG                           "DEBUG ENABLED"

// Calibrate Delta
#if MECH(DELTA)
  #define MSG_DELTA_CALIBRATE               "Delta Calibration"
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

#define MSG_HEATING                         "Heating..."
#define MSG_HEATING_COMPLETE                "Heating done."
#define MSG_BED_HEATING                     "Bed Heating."
#define MSG_BED_DONE                        "Bed done."

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

#endif // LANGUAGE_KANA_H
