/**
 * Japanese (Kana UTF8 version)
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_KANA_UTF_H
#define LANGUAGE_KANA_UTF_H

#define MAPPER_E382E383
#define DISPLAY_CHARSET_ISO10646_KANA
// This is very crude replacement of the codes used in language_kana.h from somebody who really does not know what he is doing.
// Just to show the potential benefit of unicode. 
// This translation can be improved by using the full charset of unicode codeblock U+30A0 to U+30FF.
// 片仮名表示定義



#define WELCOME_MSG                         MACHINE_NAME " ready."
#define MSG_SD                              "SD"
#define MSG_SD_INSERTED                     "セード ンウニユウアレマシタ"          // "Card inserted"
#define MSG_SD_REMOVED                      "セードゼアリマセン"               // "Card removed"
#define MSG_MAIN                            "ナイン"                        // "Main"
#define MSG_AUTOSTART                       "ヅドウセイシ"                   // "Autostart"
#define MSG_DISABLE_STEPPERS                "モーターデンゲン オフ"             // "Disable steppers"
#define MSG_AUTO_HOME                       "ゲンテンニイドウ"                // "Auto home"
#define MSG_MBL_SETTING                     "Manual Bed Leveling"
#define MSG_MBL_BUTTON                      " Press the button   "
#define MSG_MBL_INTRO                       " Leveling bed...    "
#define MSG_MBL_1                           " Adjust first point "
#define MSG_MBL_2                           " Adjust second point"
#define MSG_MBL_3                           " Adjust third point "
#define MSG_MBL_4                           " Adjust fourth point"
#define MSG_MBL_5                           "    Is it ok?       "
#define MSG_MBL_6                           " BED leveled!       "
#define MSG_SET_HOME_OFFSETS                "キヅユンオフセツトセツテイ"         // "Set home offsets"
#define MSG_SET_ORIGIN                      "キヅユンセツト"                 // "Set origin"
#define MSG_ONFOR                           "On x:"
#define MSG_PWRCONSUMED                     "P.er:"
#define MSG_PREHEAT                         "Preheat"
#define MSG_PREHEAT_PLA                     "PLA ヨネシ"                    // "Preheat PLA"
#define MSG_PREHEAT_PLA_ALL                 MSG_PREHEAT_PLA " スベテ"      // " All"
#define MSG_PREHEAT_PLA_BEDONLY             MSG_PREHEAT_PLA " ベツド"    // "Bed"
#define MSG_PREHEAT_PLA_SETTINGS            MSG_PREHEAT_PLA " セツテイ"     // "conf"
#define MSG_PREHEAT_ABS                     "ABS ヨネシ"                    // "Preheat ABS"
#define MSG_PREHEAT_ABS_ALL                 MSG_PREHEAT_ABS " スベテ"      // " All"
#define MSG_PREHEAT_ABS_BEDONLY             MSG_PREHEAT_ABS " ベツド"    // "Bed"
#define MSG_PREHEAT_ABS_SETTINGS            MSG_PREHEAT_ABS " セツテイ"    // "conf"
#define MSG_PREHEAT_GUM                     MSG_PREHEAT " GUM"
#define MSG_PREHEAT_GUM_ALL                 MSG_PREHEAT_GUM " All"
#define MSG_PREHEAT_GUM_BEDONLY             MSG_PREHEAT_GUM " Bed"
#define MSG_PREHEAT_GUM_SETTINGS            "GUM conf."
#define MSG_TOO_COLD_FOR_FILAMENTCHANGE     "Hotend too cold to change filament"
#define MSG_COOLDOWN                        "セネシテイシ"                    // "Cooldown"
#define MSG_SWITCH_PS_ON                    "デンケゾ オン"                 // "Switch power on"
#define MSG_SWITCH_PS_OFF                   "デンケゾ オフ"                 // "Switch power off"
#define MSG_EXTRUDE                         "オシダシ"                     // "Extrude"
#define MSG_RETRACT                         "リトラケト"                     // "Retract"
#define MSG_PURGE                           "Purge"
#define MSG_LEVEL_BED                       "Level bed"
#define MSG_SPEED                           "スヒ゜ード"                     // "Speed"
#define MSG_NOZZLE                          "ノズル"                       // "Nozzle"
#define MSG_BED                             "ベツド"                     // "Bed"
#define MSG_FAN_SPEED                       "ファンンケド"                    // "Fan speed"
#define MSG_FLOW                            "オケリリョウ"                     // "Flow"
#define MSG_CONTROL                         "コントロール"                    // "Control"
#define MSG_STATS                           "Statistics"
#define MSG_FIX_LOSE_STEPS                  "Fix axis steps"
#define MSG_MIN                             LCD_STR_THERMOMETER " Min"
#define MSG_MAX                             LCD_STR_THERMOMETER " Max"
#define MSG_FACTOR                          LCD_STR_THERMOMETER " Fact"
#define MSG_IDLEOOZING                      "Anti oozing"
#define MSG_AUTOTEMP                        "ヅドウオンド"                  // "Autotemp"
#define MSG_ON                              "ON "
#define MSG_OFF                             "OFF"
#define MSG_PID_P                           "PID-P"
#define MSG_PID_I                           "PID-I"
#define MSG_PID_D                           "PID-D"
#define MSG_H1                              " H1"
#define MSG_H2                              " H2"
#define MSG_H3                              " H3"
#define MSG_ACC                             "センケド"                     // "Accel"
#define MSG_VXY_JERK                        "Vxy-jerk"
#define MSG_VZ_JERK                         "Vz-jerk"
#define MSG_VE_JERK                         "Ve-jerk"
#define MSG_VMAX                            "Vmax "
#define MSG_X                               "X"
#define MSG_Y                               "Y"
#define MSG_Z                               "Z"
#define MSG_E                               "E"
#define MSG_MOVE                            "Move"
#define MSG_MOVE_AXIS                       "ヅケイドウ"                   // "Move axis"
#define MSG_MOVE_X                          "Xヅケ イドウ"                 // "Move X"
#define MSG_MOVE_Y                          "Yヅケ イドウ"                 // "Move Y"
#define MSG_MOVE_Z                          "Zヅケ イドウ"                 // "Move Z"
#define MSG_MOVE_01MM                       "0.1mm イドウ"                 // "Move 0.1mm"
#define MSG_MOVE_1MM                        "  1mm イドウ"                 // "Move 1mm"
#define MSG_MOVE_10MM                       " 10mm イドウ"                 // "Move 10mm"
#define MSG_MOVE_E                          "エケストルーダー"                // "Extruder"
#define MSG_VMIN                            "Vmin"
#define MSG_VTRAV_MIN                       "VTrav min"
#define MSG_AMAX                            "Amax "
#define MSG_A_RETRACT                       "A-retract"
#define MSG_A_TRAVEL                        "A-travel"
#define MSG_XSTEPS                          "Xsteps/mm"
#define MSG_YSTEPS                          "Ysteps/mm"
#define MSG_ZSTEPS                          "Zsteps/mm"
#define MSG_E0STEPS                         "E0steps/mm"
#define MSG_E1STEPS                         "E1steps/mm"
#define MSG_E2STEPS                         "E2steps/mm"
#define MSG_E3STEPS                         "E3steps/mm"
#define MSG_TEMPERATURE                     "オンド"                      // "Temperature"
#define MSG_MOTION                          "ウゴキセツテイ"                // "Motion"
#define MSG_FILAMENT                        "フィラナント"                    // "Filament"
#define MSG_VOLUMETRIC_ENABLED              MSG_E " in mm3"
#define MSG_FILAMENT_SIZE_EXTRUDER          "Fil. Dia."
#define MSG_CONTRAST                        "LCDコントラスト"                 // "LCD contrast"
#define MSG_STORE_EPROM                     "ナモリヘセケノウ"                 // "Store memory"
#define MSG_LOAD_EPROM                      "ナモリセラヨミbaミ"               // "Load memory"
#define MSG_RESTORE_FAILSAFE                "セツテイリセツト"               // "Restore failsafe"
#define MSG_REFRESH                         "リフレツシユ"                  // "Refresh"
#define MSG_WATCH                           "インフォ"                     // "Info screen"
#define MSG_PREPARE                         "ヅユンゼセツテイ"             //"Prepare"
#define MSG_TUNE                            "チョウセイ"                    // "Tune"
#define MSG_PAUSE_PRINT                     "イチヅテイシ"                  // "Pause print"
#define MSG_RESUME_PRINT                    "プリントアイセイ"                // "Resume print"
#define MSG_STOP_PRINT                      "プリントテイシ"                 // "Stop print"
#define MSG_CARD_MENU                       "SDセードセラプリント"            // "Print from SD"
#define MSG_NO_CARD                         "SDセードゼアリマセン"            // "No SD card"
#define MSG_DWELL                           "スリープ"                     // "Sleep..."
#define MSG_USERWAIT                        "シバラケオマチケダアイ"           // "Wait for user..."
#define MSG_RESUMING                        "プリントアイセイ"                // "Resuming print"
#define MSG_PRINT_ABORTED                   "プリントチユウシアレマシタ"          // "Print aborted"
#define MSG_NO_MOVE                         "ウゴキマセン"                  // "No move."
#define MSG_KILLED                          "ショウキョ"                     // "KILLED. "
#define MSG_STOPPED                         "テイシシマシタ"                  // "STOPPED. "
#define MSG_CONTROL_RETRACT                 "Retract mm"
#define MSG_CONTROL_RETRACT_SWAP            "Swap Re.mm"
#define MSG_CONTROL_RETRACTF                "Retract  F"
#define MSG_CONTROL_RETRACT_ZLIFT           "Hop mm"
#define MSG_CONTROL_RETRACT_RECOVER         "UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "S UnRet+mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "UnRet  F"
#define MSG_AUTORETRACT                     "AutoRetr."
#define MSG_FILAMENTCHANGE                  "フィラナントコウセン"               // "Change filament"
#define MSG_INIT_SDCARD                     "SDセードアイヨミコミ"              // "Init. SD card"
#define MSG_CNG_SDCARD                      "SDセードコウセン"                // "Change SD card"
#define MSG_ZPROBE_OUT                      "Zプローブ ベツトnゼイ"         // "Z probe out. bed"
#define MSG_POSITION_UNKNOWN                "ゲンテンハXYイドウゴZ"           // "Home X/Y before Z"
#define MSG_ZPROBE_ZOFFSET                  "Zオフセツト"                   // "Z Offset"
#define MSG_BABYSTEP                        "Babystep"
#define MSG_BABYSTEP_X                      "ゼドウ X"                    // "Babystep X"
#define MSG_BABYSTEP_Y                      "ゼドウ Y"                    // "Babystep Y"
#define MSG_BABYSTEP_Z                      "ゼドウ Z"                    // "Babystep Z"
#define MSG_ENDSTOP_ABORT                   "Endstop abort"
#define MSG_HEATING_FAILED_LCD              "Heating failed"
#define MSG_ERR_REDUNDANT_TEMP              "REDUNDANT TEMP ERROR"
#define MSG_THERMAL_RUNAWAY                 "THERMAL RUNAWAY"
#define MSG_HOTEND_AD595                    "HOTEND AD595 Offset & Gain"
#define MSG_ERR_MAXTEMP                     "MAXTEMP ERROR"
#define MSG_ERR_MINTEMP                     "MINTEMP ERROR"
#define MSG_ERR_MAXTEMP_BED                 "MAXTEMP BED ERROR"
#define MSG_ERR_MINTEMP_BED                 "MINTEMP BED ERROR"
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

// Calibrate Delta
#if MECH(DELTA)
  #define MSG_DELTA_CALIBRATE               "Delta Calibration"
  #define MSG_DELTA_CALIBRATE_X             "Calibrate " MSG_X
  #define MSG_DELTA_CALIBRATE_Y             "Calibrate " MSG_Y
  #define MSG_DELTA_CALIBRATE_Z             "Calibrate " MSG_Z
  #define MSG_DELTA_CALIBRATE_CENTER        "Calibrate Center"
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

#endif // LANGUAGE_KANA_UTF_H
