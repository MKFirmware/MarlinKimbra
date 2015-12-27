/**
 * Basque-Euskera
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_EU_H
#define LANGUAGE_EU_H

#define MAPPER_NON
#define DISPLAY_CHARSET_ISO10646_1







#define WELCOME_MSG                         MACHINE_NAME " prest."

#define MSG_SD_INSERTED                     "Txartela sartuta"
#define MSG_SD_REMOVED                      "Txartela kenduta"
#define MSG_MAIN                            "Menu nagusia"
#define MSG_AUTOSTART                       "Auto hasiera"
#define MSG_DISABLE_STEPPERS                "Itzali motoreak"
#define MSG_AUTO_HOME                       "Hasierara joan"
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
#define MSG_SET_ORIGIN                      "Hasiera ipini"
#define MSG_ONFOR                           "On x:"
#define MSG_PWRCONSUMED                     "P.er:"


#define MSG_PREHEAT_PLA                     "Aurreberotu PLA"
#define MSG_PREHEAT_PLA_ALL                 "Berotu PLA Guztia"
#define MSG_PREHEAT_PLA_BEDONLY             "Berotu PLA Ohea"
#define MSG_PREHEAT_PLA_SETTINGS            "Berotu PLA Konfig"
#define MSG_PREHEAT_ABS                     "Aurreberotu ABS"
#define MSG_PREHEAT_ABS_ALL                 "Berotu ABS Guztia"
#define MSG_PREHEAT_ABS_BEDONLY             "Berotu ABS Ohea"
#define MSG_PREHEAT_ABS_SETTINGS            "Berotu ABS Konfig"
#define MSG_PREHEAT_GUM                     "Preheat GUM"
#define MSG_PREHEAT_GUM_ALL                 "Preheat GUM All"
#define MSG_PREHEAT_GUM_BEDONLY             "Preheat GUM Bed"
#define MSG_PREHEAT_GUM_SETTINGS            "Preheat GUM conf"
#define MSG_TOO_COLD_FOR_FILAMENTCHANGE     "Hotend too cold to change filament"
#define MSG_COOLDOWN                        "Hoztu"
#define MSG_SWITCH_PS_ON                    "Energia piztu"
#define MSG_SWITCH_PS_OFF                   "Energia itzali"
#define MSG_EXTRUDE                         "Estruitu"
#define MSG_RETRACT                         "Atzera eragin"
#define MSG_PURGE                           "Purge"
#define MSG_LEVEL_BED                       "Level bed"
#define MSG_SPEED                           "Abiadura"
#define MSG_NOZZLE                          "Pita"
#define MSG_BED                             "Ohea"
#define MSG_FAN_SPEED                       "Haizagailua"
#define MSG_FLOW                            "Fluxua"
#define MSG_CONTROL                         "Kontrola"
#define MSG_STATS                           "Statistics"
#define MSG_FIX_LOSE_STEPS                  "Fix axis steps"
#define MSG_MIN                             LCD_STR_THERMOMETER " Min"
#define MSG_MAX                             LCD_STR_THERMOMETER " Max"
#define MSG_FACTOR                          LCD_STR_THERMOMETER " Fact"
#define MSG_IDLEOOZING                      "Anti oozing"
#define MSG_AUTOTEMP                        "Auto tenperatura"
#define MSG_ON                              "On "
#define MSG_OFF                             "Off"
#define MSG_PID_P                           "PID-P"
#define MSG_PID_I                           "PID-I"
#define MSG_PID_D                           "PID-D"
#define MSG_H1                              " H1"
#define MSG_H2                              " H2"
#define MSG_H3                              " H3"
#define MSG_ACC                             "Azelerazioa"
#define MSG_VXY_JERK                        "Vxy-astindua"
#define MSG_VZ_JERK                         "Vz-astindua"
#define MSG_VE_JERK                         "Ve-astindua"
#define MSG_VMAX                            "Vmax "
#define MSG_X                               "X"
#define MSG_Y                               "Y"
#define MSG_Z                               "Z"
#define MSG_E                               "E"

#define MSG_MOVE_AXIS                       "Ardatzak mugitu"
#define MSG_MOVE_X                          "Mugitu X"
#define MSG_MOVE_Y                          "Mugitu Y"
#define MSG_MOVE_Z                          "Mugitu Z"
#define MSG_MOVE_01MM                       "Mugitu 0.1mm"
#define MSG_MOVE_1MM                        "Mugitu 1mm"
#define MSG_MOVE_10MM                       "Mugitu 10mm"
#define MSG_MOVE_E                          "Estrusorea"
#define MSG_VMIN                            "Vmin"
#define MSG_VTRAV_MIN                       "VTrav min"
#define MSG_AMAX                            "Amax "
#define MSG_A_RETRACT                       "A-retrakt"
#define MSG_A_TRAVEL                        "A-travel"
#define MSG_XSTEPS                          "X pausoak/mm"
#define MSG_YSTEPS                          "Y pausoak/mm"
#define MSG_ZSTEPS                          "Z pausoak/mm"
#define MSG_E0STEPS                         "E pausoak/mm"
#define MSG_E1STEPS                         "E1 pausoak/mm"
#define MSG_E2STEPS                         "E2 pausoak/mm"
#define MSG_E3STEPS                         "E3 pausoak/mm"
#define MSG_TEMPERATURE                     "Tenperatura"
#define MSG_MOTION                          "Mugimendua"
#define MSG_VOLUMETRIC                      "Filament"
#define MSG_VOLUMETRIC_ENABLED              "E in mm" STR_h3
#define MSG_FILAMENT_SIZE_EXTRUDER          "Fil. Dia."
#define MSG_CONTRAST                        "LCD kontrastea"
#define MSG_STORE_EPROM                     "Gorde memoria"
#define MSG_LOAD_EPROM                      "Kargatu memoria"
#define MSG_RESTORE_FAILSAFE                "Larri. berriz."
#define MSG_REFRESH                         "Berriz kargatu"
#define MSG_WATCH                           "Pantaila info"
#define MSG_PREPARE                         "Prestatu"
#define MSG_TUNE                            "Doitu"
#define MSG_PAUSE_PRINT                     "Pausatu inprimak."
#define MSG_RESUME_PRINT                    "Jarraitu inprima."
#define MSG_STOP_PRINT                      "Gelditu inprima."
#define MSG_CARD_MENU                       "SD-tik inprimatu"
#define MSG_NO_CARD                         "Ez dago txartelik"
#define MSG_DWELL                           "Lo egin..."
#define MSG_USERWAIT                        "Aginduak zain..."
#define MSG_RESUMING                        "Jarraitzen inpri."
#define MSG_PRINT_ABORTED                   "Print aborted"
#define MSG_NO_MOVE                         "Mugimendu gabe"
#define MSG_KILLED                          "LARRIALDI GELDIA"
#define MSG_STOPPED                         "GELDITUTA. "
#define MSG_CONTROL_RETRACT                 "Atzera egin mm"
#define MSG_CONTROL_RETRACT_SWAP            "Swap Atzera egin mm"
#define MSG_CONTROL_RETRACTF                "Atzera egin V"
#define MSG_CONTROL_RETRACT_ZLIFT           "Igo mm"
#define MSG_CONTROL_RETRACT_RECOVER         "Atzera egin +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "Swap Atzera egin +mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "Atzera egin V"
#define MSG_AUTORETRACT                     "Atzera egin"
#define MSG_FILAMENTCHANGE                  "Aldatu filament."
#define MSG_INIT_SDCARD                     "Hasieratu txartela"
#define MSG_CNG_SDCARD                      "Aldatu txartela"
#define MSG_ZPROBE_OUT                      "Z ohe hasiera"
#define MSG_POSITION_UNKNOWN                "Posizio ezezaguna"
#define MSG_ZPROBE_ZOFFSET                  "Z konpentsatu"
#define MSG_BABYSTEP                        "Babystep"
#define MSG_BABYSTEP_X                      MSG_BABYSTEP " " MSG_X
#define MSG_BABYSTEP_Y                      MSG_BABYSTEP " " MSG_Y
#define MSG_BABYSTEP_Z                      MSG_BABYSTEP " " MSG_Z
#define MSG_ENDSTOP_ABORT                   "Endstop deuseztat"
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
#define MSG_E_BOWDEN_LENGTH                 "Extrude " STRINGIFY(BOWDEN_LENGTH) "mm"
#define MSG_R_BOWDEN_LENGTH                 "Retract " STRINGIFY(BOWDEN_LENGTH) "mm"
#define MSG_PURGE_XMM                       "Purge " STRINGIFY(LCD_PURGE_LENGTH) "mm"
#define MSG_RETRACT_XMM                     "Retract " STRINGIFY(LCD_RETRACT_LENGTH) "mm"
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

#endif // LANGUAGE_EU_H
