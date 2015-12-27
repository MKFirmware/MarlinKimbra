/**
 * French
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_FR_H
#define LANGUAGE_FR_H

#define MAPPER_NON
#define DISPLAY_CHARSET_ISO10646_1







#define WELCOME_MSG                         MACHINE_NAME " prete."

#define MSG_SD_INSERTED                     "Carte inseree"
#define MSG_SD_REMOVED                      "Carte retiree"
#define MSG_MAIN                            "Menu principal"
#define MSG_AUTOSTART                       "Demarrage auto"
#define MSG_DISABLE_STEPPERS                "Arreter moteurs"
#define MSG_AUTO_HOME                       "Home auto."
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
#define MSG_SET_ORIGIN                      "Regler origine"
#define MSG_ONFOR                           "On x:"
#define MSG_PWRCONSUMED                     "P.er:"


#define MSG_PREHEAT_PLA                     "Prechauffage PLA"
#define MSG_PREHEAT_PLA_ALL                 "Prech. PLA Tout"
#define MSG_PREHEAT_PLA_BEDONLY             "Prech. PLA Plateau"
#define MSG_PREHEAT_PLA_SETTINGS            "Regl. prech. PLA"
#define MSG_PREHEAT_ABS                     "Prechauffage ABS"
#define MSG_PREHEAT_ABS_ALL                 "Prech. ABS Tout"
#define MSG_PREHEAT_ABS_BEDONLY             "Prech. ABS Plateau"
#define MSG_PREHEAT_ABS_SETTINGS            "Regl. prech. ABS"
#define MSG_PREHEAT_GUM                     "Preheat GUM"
#define MSG_PREHEAT_GUM_ALL                 "Preheat GUM All"
#define MSG_PREHEAT_GUM_BEDONLY             "Preheat GUM Bed"
#define MSG_PREHEAT_GUM_SETTINGS            "Preheat GUM conf"
#define MSG_TOO_COLD_FOR_FILAMENTCHANGE     "Hotend too cold to change filament"
#define MSG_COOLDOWN                        "Refroidir"
#define MSG_SWITCH_PS_ON                    "Allumer alim."
#define MSG_SWITCH_PS_OFF                   "Eteindre alim."
#define MSG_EXTRUDE                         "Extrusion"
#define MSG_RETRACT                         "Retraction"
#define MSG_LEVEL_BED                       "Level bed"
#define MSG_PURGE                           "Purge"
#define MSG_SPEED                           "Vitesse"
#define MSG_NOZZLE                          "Buse"
#define MSG_BED                             "Plateau"
#define MSG_FAN_SPEED                       "Vite. ventilateur"
#define MSG_FLOW                            "Flux"
#define MSG_CONTROL                         "Controler"
#define MSG_STATS                           "Statistics"
#define MSG_FIX_LOSE_STEPS                  "Fix axis steps"
#define MSG_MIN                             LCD_STR_THERMOMETER " Min"
#define MSG_MAX                             LCD_STR_THERMOMETER " Max"
#define MSG_FACTOR                          LCD_STR_THERMOMETER " Facteur"
#define MSG_IDLEOOZING                      "Anti oozing"
#define MSG_AUTOTEMP                        "Temp. Auto."
#define MSG_ON                              "Marche "
#define MSG_OFF                             "Arret"
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

#define MSG_MOVE_AXIS                       "Deplacer un axe"
#define MSG_MOVE_X                          "Depl. X"
#define MSG_MOVE_Y                          "Depl. Y"
#define MSG_MOVE_Z                          "Depl. Z"
#define MSG_MOVE_01MM                       "Depl. 0.1mm"
#define MSG_MOVE_1MM                        "Depl. 1mm"
#define MSG_MOVE_10MM                       "Depl. 10mm"
#define MSG_MOVE_E                          "Extruder"
#define MSG_VMIN                            "Vmin"
#define MSG_VTRAV_MIN                       "Vdepl min"
#define MSG_AMAX                            "Amax "
#define MSG_A_RETRACT                       "A-retract"
#define MSG_A_TRAVEL                        "A-travel"
#define MSG_XSTEPS                          "X pas/mm"
#define MSG_YSTEPS                          "Y pas/mm"
#define MSG_ZSTEPS                          "Z pas/mm"
#define MSG_E0STEPS                         "E0 pas/mm"
#define MSG_E1STEPS                         "E1 pas/mm"
#define MSG_E2STEPS                         "E2 pas/mm"
#define MSG_E3STEPS                         "E3 pas/mm"
#define MSG_TEMPERATURE                     "Temperature"
#define MSG_MOTION                          "Mouvement"
#define MSG_VOLUMETRIC                      "Filament"
#define MSG_VOLUMETRIC_ENABLED              "E in mm" STR_h3
#define MSG_FILAMENT_SIZE_EXTRUDER          "Fil. Dia."
#define MSG_CONTRAST                        "Contraste LCD"
#define MSG_STORE_EPROM                     "Sauver config"
#define MSG_LOAD_EPROM                      "Lire config"
#define MSG_RESTORE_FAILSAFE                "Restaurer defauts"
#define MSG_REFRESH                         "Actualiser"
#define MSG_WATCH                           "Surveiller"
#define MSG_PREPARE                         "Preparer"
#define MSG_TUNE                            "Regler"
#define MSG_PAUSE_PRINT                     "Interrompre impr."
#define MSG_RESUME_PRINT                    "Reprendre impr."
#define MSG_STOP_PRINT                      "Arreter impr."
#define MSG_CARD_MENU                       "Impr. depuis SD"
#define MSG_NO_CARD                         "Pas de carte"
#define MSG_DWELL                           "Repos..."
#define MSG_USERWAIT                        "Atten. de l'util."
#define MSG_RESUMING                        "Repri. de l'impr."
#define MSG_PRINT_ABORTED                   "Impr. Annulee"
#define MSG_NO_MOVE                         "Aucun mouvement."
#define MSG_KILLED                          "MORT."
#define MSG_STOPPED                         "STOPPE."
#define MSG_CONTROL_RETRACT                 "Retraction mm"
#define MSG_CONTROL_RETRACT_SWAP            "Ech. Retr. mm"
#define MSG_CONTROL_RETRACTF                "Retraction V"
#define MSG_CONTROL_RETRACT_ZLIFT           "Hop mm"
#define MSG_CONTROL_RETRACT_RECOVER         "UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "Ech. UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "UnRet V"
#define MSG_AUTORETRACT                     "Retract. Auto."
#define MSG_FILAMENTCHANGE                  "Changer filament"
#define MSG_INIT_SDCARD                     "Init. la carte SD"
#define MSG_CNG_SDCARD                      "Changer de carte"
#define MSG_ZPROBE_OUT                      "Z sonde exte. lit"
#define MSG_POSITION_UNKNOWN                "Rev. dans XY av.Z"
#define MSG_ZPROBE_ZOFFSET                  "Offset Z"
#define MSG_BABYSTEP                        "Babystep"
#define MSG_BABYSTEP_X                      MSG_BABYSTEP " " MSG_X
#define MSG_BABYSTEP_Y                      MSG_BABYSTEP " " MSG_Y
#define MSG_BABYSTEP_Z                      MSG_BABYSTEP " " MSG_Z
#define MSG_ENDSTOP_ABORT                   "Butee abandon"
#define MSG_HEATING_FAILED_LCD              "Heating failed"
#define MSG_ERR_REDUNDANT_TEMP              "Err: REDUNDANT TEMP ERROR"
#define MSG_THERMAL_RUNAWAY                 "THERMAL RUNAWAY"
#define MSG_HOTEND_AD595                    "HOTEND AD595 Offset & Gain"
#define MSG_ERR_MAXTEMP                     "Err: MAXTEMP"
#define MSG_ERR_MINTEMP                     "Err: MINTEMP"
#define MSG_ERR_MAXTEMP_BED                 "Err: MAXTEMP BED"
#define MSG_ERR_MINTEMP_BED                 "Err: MINTEMP BED"
#define MSG_END_DAY                         "days"
#define MSG_END_HOUR                        "heures"
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

#endif // LANGUAGE_FR_H
