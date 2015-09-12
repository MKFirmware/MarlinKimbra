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
// Define SIMULATE_ROMFONT to see what is seen on the character based display defined in Configuration.h
//#define SIMULATE_ROMFONT
#define DISPLAY_CHARSET_ISO10646_KANA

// This is very crude replacement of the codes used in language_kana.h from somebody who really does not know what he is doing.
// Just to show the potential benefit of unicode. 
// This translation can be improved by using the full charset of unicode codeblock U+30A0 to U+30FF.

// 片仮名表示定義
#define WELCOME_MSG                         MACHINE_NAME " ready."
#define MSG_SD_INSERTED                     "セード ンウニユウアレマシタ"          // "Card inserted"
#define MSG_SD_REMOVED                      "セードゼアリマセン"               // "Card removed"
#define MSG_MAIN                            "ナイン"                        // "Main"
#define MSG_AUTOSTART                       "ヅドウセイシ"                   // "Autostart"
#define MSG_DISABLE_STEPPERS                "モーターデンゲン オフ"             // "Disable steppers"
#define MSG_AUTO_HOME                       "ゲンテンニイドウ"                // "Auto home"
#define MSG_SET_HOME_OFFSETS                "キヅユンオフセツトセツテイ"         // "Set home offsets"
#define MSG_SET_ORIGIN                      "キヅユンセツト"                 // "Set origin"
#define MSG_PREHEAT_PLA                     "PLA ヨネシ"                    // "Preheat PLA"
#define MSG_PREHEAT_PLA_N                   MSG_PREHEAT_PLA " "
#define MSG_PREHEAT_PLA_ALL                 MSG_PREHEAT_PLA " スベテ"      // " All"
#define MSG_PREHEAT_PLA_BEDONLY             MSG_PREHEAT_PLA " ベツド"    // "Bed"
#define MSG_PREHEAT_PLA_SETTINGS            MSG_PREHEAT_PLA " セツテイ"     // "conf"
#define MSG_PREHEAT_ABS                     "ABS ヨネシ"                    // "Preheat ABS"
#define MSG_PREHEAT_ABS_N                   MSG_PREHEAT_ABS " "
#define MSG_PREHEAT_ABS_ALL                 MSG_PREHEAT_ABS " スベテ"      // " All"
#define MSG_PREHEAT_ABS_BEDONLY             MSG_PREHEAT_ABS " ベツド"    // "Bed"
#define MSG_PREHEAT_ABS_SETTINGS            MSG_PREHEAT_ABS " セツテイ"    // "conf"
#define MSG_COOLDOWN                        "セネシテイシ"                    // "Cooldown"
#define MSG_SWITCH_PS_ON                    "デンケゾ オン"                 // "Switch power on"
#define MSG_SWITCH_PS_OFF                   "デンケゾ オフ"                 // "Switch power off"
#define MSG_EXTRUDE                         "オシダシ"                     // "Extrude"
#define MSG_RETRACT                         "リトラケト"                     // "Retract"
#define MSG_MOVE_AXIS                       "ヅケイドウ"                   // "Move axis"
#define MSG_MOVE_X                          "Xヅケ イドウ"                 // "Move X"
#define MSG_MOVE_Y                          "Yヅケ イドウ"                 // "Move Y"
#define MSG_MOVE_Z                          "Zヅケ イドウ"                 // "Move Z"
#define MSG_MOVE_E                          "エケストルーダー"                // "Extruder"
#define MSG_MOVE_01MM                       "0.1mm イドウ"                 // "Move 0.1mm"
#define MSG_MOVE_1MM                        "  1mm イドウ"                 // "Move 1mm"
#define MSG_MOVE_10MM                       " 10mm イドウ"                 // "Move 10mm"
#define MSG_SPEED                           "スヒ゜ード"                     // "Speed"
#define MSG_NOZZLE                          "ノズル"                       // "Nozzle"
#define MSG_BED                             "ベツド"                     // "Bed"
#define MSG_FAN_SPEED                       "ファンンケド"                    // "Fan speed"
#define MSG_FLOW                            "オケリリョウ"                     // "Flow"
#define MSG_CONTROL                         "コントロール"                    // "Control"
#define MSG_MIN                             LCD_STR_THERMOMETER " Min"
#define MSG_MAX                             LCD_STR_THERMOMETER " Max"
#define MSG_FACTOR                          LCD_STR_THERMOMETER " Fact"
#define MSG_IDLEOOZING                      "Anti oozing"
#define MSG_AUTOTEMP                        "ヅドウオンド"                  // "Autotemp"
#define MSG_ON                              "On "
#define MSG_OFF                             "Off"
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
#define MSG_VOLUMETRIC                      "フィラナント"                    // "Filament"
#define MSG_VOLUMETRIC_ENABLED              "E in mm3"
#define MSG_FILAMENT_SIZE_EXTRUDER_0        "Fil. Dia. 1"
#define MSG_FILAMENT_SIZE_EXTRUDER_1        "Fil. Dia. 2"
#define MSG_FILAMENT_SIZE_EXTRUDER_2        "Fil. Dia. 3"
#define MSG_FILAMENT_SIZE_EXTRUDER_3        "Fil. Dia. 4"
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
#define MSG_CONTROL_RETRACTF                "Retract  V"
#define MSG_CONTROL_RETRACT_ZLIFT           "Hop mm"
#define MSG_CONTROL_RETRACT_RECOVER         "UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "S UnRet+mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "UnRet  V"
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
#define MSG_ERR_REDUNDANT_TEMP              "Err: REDUNDANT TEMP ERROR"
#define MSG_THERMAL_RUNAWAY                 "THERMAL RUNAWAY"
#define MSG_ERR_MAXTEMP                     "Err: MAXTEMP"
#define MSG_ERR_MINTEMP                     "Err: MINTEMP"
#define MSG_ERR_MAXTEMP_BED                 "Err: MAXTEMP BED"
#define MSG_ERR_MINTEMP_BED                 "Err: MINTEMP BED"
#define MSG_END_DAY                         "days"
#define MSG_END_HOUR                        "hours"
#define MSG_END_MINUTE                      "minutes"

// Debug
#define MSG_DEBUG_ECHO                      "DEBUG ECHO ENABLED"
#define MSG_DEBUG_INFO                      "DEBUG INFO ENABLED"
#define MSG_DEBUG_ERRORS                    "DEBUG ERRORS ENABLED"
#define MSG_DEBUG_DRYRUN                    "DEBUG DRYRUN ENABLED"

// Calibrate Delta
#if MECH(DELTA)
  #define MSG_DELTA_CALIBRATE               "Delta Calibration"
  #define MSG_DELTA_CALIBRATE_X             "Calibrate X"
  #define MSG_DELTA_CALIBRATE_Y             "Calibrate Y"
  #define MSG_DELTA_CALIBRATE_Z             "Calibrate Z"
  #define MSG_DELTA_CALIBRATE_CENTER        "Calibrate Center"
#endif // DELTA

// Scara
#if MECH(SCARA)
  #define MSG_XSCALE                        "X Scale"
  #define MSG_YSCALE                        "Y Scale"
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
  #define MSG_FWTEST_INTO                    "into "
  #define MSG_FWTEST_ERROR                   "ERROR"
  #define MSG_FWTEST_OK                      "OK"
  #define MSG_FWTEST_NDEF                    "not defined"
#endif // FIRMWARE_TEST

#endif // LANGUAGE_KANA_UTF_H
