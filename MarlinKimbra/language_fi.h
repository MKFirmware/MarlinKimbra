/**
 * Finnish
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_FI_H
#define LANGUAGE_FI_H

#define MAPPER_C2C3
#define DISPLAY_CHARSET_ISO10646_1







#define WELCOME_MSG                         MACHINE_NAME " valmis."

#define MSG_SD_INSERTED                     "Kortti asetettu"
#define MSG_SD_REMOVED                      "Kortti poistettu"
#define MSG_MAIN                            "Palaa"
#define MSG_AUTOSTART                       "Automaatti"
#define MSG_DISABLE_STEPPERS                "Vapauta moottorit"
#define MSG_AUTO_HOME                       "Aja referenssiin"
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
#define MSG_SET_ORIGIN                      "Aseta origo"
#define MSG_ONFOR                           "On x:"
#define MSG_PWRCONSUMED                     "P.er:"


#define MSG_PREHEAT_PLA                     "Esil" STR_ae "mmit" STR_ae " PLA"
#define MSG_PREHEAT_PLA_ALL                 "Esil" STR_ae ". PLA Kaikki"
#define MSG_PREHEAT_PLA_BEDONLY             "Esil" STR_ae ". PLA Alusta"
#define MSG_PREHEAT_PLA_SETTINGS            "Esil" STR_ae "mm. PLA konf"
#define MSG_PREHEAT_ABS                     "Esil" STR_ae "mmit" STR_ae " ABS"
#define MSG_PREHEAT_ABS_ALL                 "Esil" STR_ae ". ABS Kaikki"
#define MSG_PREHEAT_ABS_BEDONLY             "Esil" STR_ae ". ABS Alusta"
#define MSG_PREHEAT_ABS_SETTINGS            "Esil" STR_ae "mm. ABS konf"
#define MSG_PREHEAT_GUM                     "Esil" STR_ae "mmit" STR_ae " GUM"
#define MSG_PREHEAT_GUM_ALL                 "Esil" STR_ae ". GUM Kaikki"
#define MSG_PREHEAT_GUM_BEDONLY             "Esil" STR_ae ". GUM Alusta"
#define MSG_PREHEAT_GUM_SETTINGS            "Esil" STR_ae "mm. GUM konf"
#define MSG_TOO_COLD_FOR_FILAMENTCHANGE     "Hotend too cold to change filament"
#define MSG_COOLDOWN                        "J" STR_ae "" STR_ae "hdyt" STR_ae ""
#define MSG_SWITCH_PS_ON                    "Virta p" STR_ae "" STR_ae "lle"
#define MSG_SWITCH_PS_OFF                   "Virta pois"
#define MSG_EXTRUDE                         "Pursota"
#define MSG_RETRACT                         "Ved" STR_ae " takaisin"
#define MSG_LEVEL_BED                       "Level bed"
#define MSG_PURGE                           "Purge"
#define MSG_SPEED                           "Nopeus"
#define MSG_NOZZLE                          "Suutin"
#define MSG_BED                             "Alusta"
#define MSG_FAN_SPEED                       "Tuul. nopeus"
#define MSG_FLOW                            "Virtaus"
#define MSG_CONTROL                         "Kontrolli"
#define MSG_STATS                           "Statistics"
#define MSG_FIX_LOSE_STEPS                  "Fix axis steps"
#define MSG_MIN                             LCD_STR_THERMOMETER " Min"
#define MSG_MAX                             LCD_STR_THERMOMETER " Max"
#define MSG_FACTOR                          LCD_STR_THERMOMETER " Kerr"
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
#define MSG_ACC                             "Kiihtyv"
#define MSG_VXY_JERK                        "Vxy-jerk"
#define MSG_VZ_JERK                         "Vz-jerk"
#define MSG_VE_JERK                         "Ve-jerk"
#define MSG_VMAX                            "Vmax "
#define MSG_X                               "X"
#define MSG_Y                               "Y"
#define MSG_Z                               "Z"
#define MSG_E                               "E"

#define MSG_MOVE_AXIS                       "Liikuta akseleita"
#define MSG_MOVE_X                          "Liikuta X"
#define MSG_MOVE_Y                          "Liikuta Y"
#define MSG_MOVE_Z                          "Liikuta Z"
#define MSG_MOVE_01MM                       "Liikuta 0.1mm"
#define MSG_MOVE_1MM                        "Liikuta 1mm"
#define MSG_MOVE_10MM                       "Liikuta 10mm"
#define MSG_MOVE_E                          "Extruder"
#define MSG_VMIN                            "Vmin"
#define MSG_VTRAV_MIN                       "VLiike min"
#define MSG_AMAX                            "Amax "
#define MSG_A_RETRACT                       "A-peruuta"
#define MSG_A_TRAVEL                        "A-travel"
#define MSG_XSTEPS                          "X steps/mm"
#define MSG_YSTEPS                          "Y steps/mm"
#define MSG_ZSTEPS                          "Z steps/mm"
#define MSG_E0STEPS                         "E0 steps/mm"
#define MSG_E1STEPS                         "E1 steps/mm"
#define MSG_E2STEPS                         "E2 steps/mm"
#define MSG_E3STEPS                         "E3 steps/mm"
#define MSG_TEMPERATURE                     "L" STR_ae "mp" STR_oe "tila"
#define MSG_MOTION                          "Liike"
#define MSG_VOLUMETRIC                      "Filament"
#define MSG_VOLUMETRIC_ENABLED              "E in mm" STR_h3
#define MSG_FILAMENT_SIZE_EXTRUDER          "Fil. Dia."
#define MSG_CONTRAST                        "LCD kontrasti"
#define MSG_STORE_EPROM                     "Tallenna muistiin"
#define MSG_LOAD_EPROM                      "Lataa muistista"
#define MSG_RESTORE_FAILSAFE                "Palauta oletus"
#define MSG_REFRESH                         "P" STR_ae "ivit" STR_ae ""
#define MSG_WATCH                           "Seuraa"
#define MSG_PREPARE                         "Valmistele"
#define MSG_TUNE                            "S" STR_ae "" STR_ae "d" STR_ae ""
#define MSG_PAUSE_PRINT                     "Keskeyt" STR_ae " tulostus"
#define MSG_RESUME_PRINT                    "Jatka tulostusta"
#define MSG_STOP_PRINT                      "Pys" STR_ae "yt" STR_ae " tulostus"
#define MSG_CARD_MENU                       "Korttivalikko"
#define MSG_NO_CARD                         "Ei korttia"
#define MSG_DWELL                           "Nukkumassa..."
#define MSG_USERWAIT                        "Odotet. valintaa"
#define MSG_RESUMING                        "Jatke. tulostusta"
#define MSG_PRINT_ABORTED                   "Print aborted"
#define MSG_NO_MOVE                         "Ei liiketta."
#define MSG_KILLED                          "KILLED. "
#define MSG_STOPPED                         "STOPPED. "
#define MSG_CONTROL_RETRACT                 "Ved" STR_ae " mm"
#define MSG_CONTROL_RETRACT_SWAP            "Va. Ved" STR_ae " mm"
#define MSG_CONTROL_RETRACTF                "Ved" STR_ae " V"
#define MSG_CONTROL_RETRACT_ZLIFT           "Z mm"
#define MSG_CONTROL_RETRACT_RECOVER         "UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "Va. UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "UnRet  V"
#define MSG_AUTORETRACT                     "AutoVeto."
#define MSG_FILAMENTCHANGE                  "Change filament"
#define MSG_INIT_SDCARD                     "Init. SD-Card"
#define MSG_CNG_SDCARD                      "Change SD-Card"
#define MSG_ZPROBE_OUT                      "Z probe out. bed"
#define MSG_POSITION_UNKNOWN                "Home X/Y before Z"
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
  #define MSG_DELTA_CALIBRATE               "Delta Kalibrointi"
  #define MSG_DELTA_CALIBRATE_X             "Kalibroi X"
  #define MSG_DELTA_CALIBRATE_Y             "Kalibroi Y"
  #define MSG_DELTA_CALIBRATE_Z             "Kalibroi Z"
  #define MSG_DELTA_CALIBRATE_CENTER        "Kalibroi Center"
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

#endif // LANGUAGE_FI_H
