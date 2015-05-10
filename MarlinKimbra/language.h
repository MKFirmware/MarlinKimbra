#ifndef LANGUAGE_H
#define LANGUAGE_H

#include "Configuration.h"

// NOTE: IF YOU CHANGE LANGUAGE FILES OR MERGE A FILE WITH CHANGES
//
//   ==> ALWAYS TRY TO COMPILE MARLIN WITH/WITHOUT "ULTIPANEL" / "ULTRALCD" / "SDSUPPORT" #define IN "Configuration.h"
//   ==> ALSO TRY ALL AVAILABLE "LANGUAGE_CHOICE" OPTIONS
// See also documentation/LCDLanguageFont.md

// Languages
// 1  English  // Language base
// 2  Polish
// 3  French
// 4  German
// 5  Spanish
// 6  Russian
// 7  Italian
// 8  Portuguese
// 9  Finnish
// 10 Aragonese
// 11 Dutch
// 12 Catalan
// 13 Basque-Euskera
// 14 Portuguese (Brazil)

#ifndef LANGUAGE_CHOICE
  #define LANGUAGE_CHOICE 7  // Pick your language from the list above
#endif

#define PROTOCOL_VERSION "1.0"

#if MB(ULTIMAKER)|| MB(ULTIMAKER_OLD)|| MB(ULTIMAIN_2)
  #define MACHINE_NAME "Ultimaker"
  #define FIRMWARE_URL "http://firmware.ultimaker.com"
#elif MB(RUMBA)
  #define MACHINE_NAME "Rumba"
#elif MB(3DRAG)
  #define MACHINE_NAME "3Drag"
  #define FIRMWARE_URL "http://3dprint.elettronicain.it/"
#elif MB(K8200)
  #define MACHINE_NAME "K8200"
#elif MB(5DPRINT)
  #define MACHINE_NAME "Makibox"
#elif MB(SAV_MKI)
  #define MACHINE_NAME "SAV MkI"
  #define FIRMWARE_URL "https://github.com/fmalpartida/Marlin/tree/SAV-MkI-config"
#else // Default firmware set to Mendel
  #define MACHINE_NAME "Mendel"
#endif

#ifdef CUSTOM_MENDEL_NAME
  #error CUSTOM_MENDEL_NAME deprecated - use CUSTOM_MACHINE_NAME
  #define CUSTOM_MACHINE_NAME CUSTOM_MENDEL_NAME
#endif

#ifdef CUSTOM_MACHINE_NAME
  #undef MACHINE_NAME
  #define MACHINE_NAME CUSTOM_MACHINE_NAME
#endif

#ifndef FIRMWARE_URL
  #define FIRMWARE_URL "https://github.com/MagoKimbra/MarlinKimbra"
#endif

#ifndef BUILD_VERSION
  #define BUILD_VERSION "V4;"
#endif

#ifndef MACHINE_UUID
  #define MACHINE_UUID "00000000-0000-0000-0000-000000000000"
#endif


#define STRINGIFY_(n) #n
#define STRINGIFY(n) STRINGIFY_(n)


// Common LCD messages

  /* nothing here yet */

// Common serial messages
#define MSG_MARLIN "MarlinKimbra"

// Serial Console Messages (do not translate those!)

#define MSG_ENQUEUEING                      "enqueueing \""
#define MSG_POWERUP                         "PowerUp"
#define MSG_EXTERNAL_RESET                  " External Reset"
#define MSG_BROWNOUT_RESET                  " Brown out Reset"
#define MSG_WATCHDOG_RESET                  " Watchdog Reset"
#define MSG_SOFTWARE_RESET                  " Software Reset"
#define MSG_AUTHOR                          " | Author: "
#define MSG_CONFIGURATION_VER               " Last Updated: "
#define MSG_FREE_MEMORY                     " Free Memory: "
#define MSG_PLANNER_BUFFER_BYTES            " PlannerBufferBytes: "
#define MSG_OK                              "ok"
#define MSG_WAIT                            "wait"
#define MSG_FILE_SAVED                      "Done saving file."
#define MSG_ERR_LINE_NO                     "Line Number is not Last Line Number+1, Last Line: "
#define MSG_ERR_CHECKSUM_MISMATCH           "checksum mismatch, Last Line: "
#define MSG_ERR_NO_CHECKSUM                 "No Checksum with line number, Last Line: "
#define MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM "No Line Number with checksum, Last Line: "
#define MSG_FILE_PRINTED                    "Done printing file"
#define MSG_BEGIN_FILE_LIST                 "Begin file list"
#define MSG_END_FILE_LIST                   "End file list"
#define MSG_INVALID_EXTRUDER                "Invalid extruder"
#define MSG_INVALID_SOLENOID                "Invalid solenoid"
#define MSG_M104_INVALID_EXTRUDER           "M104 " MSG_INVALID_EXTRUDER " "
#define MSG_M105_INVALID_EXTRUDER           "M105 " MSG_INVALID_EXTRUDER " "
#define MSG_M109_INVALID_EXTRUDER           "M109 " MSG_INVALID_EXTRUDER " "
#define MSG_M200_INVALID_EXTRUDER           "M200 " MSG_INVALID_EXTRUDER " "
#define MSG_M218_INVALID_EXTRUDER           "M218 " MSG_INVALID_EXTRUDER " "
#define MSG_M221_INVALID_EXTRUDER           "M221 " MSG_INVALID_EXTRUDER " "
#define MSG_ERR_NO_THERMISTORS              "No thermistors - no temperature"
#define MSG_HEATING                         "Heating..."
#define MSG_HEATING_COMPLETE                "Heating done."
#define MSG_BED_HEATING                     "Bed Heating."
#define MSG_BED_DONE                        "Bed done."
#define MSG_M115_REPORT                     "FIRMWARE_NAME:MarlinKimbra " BUILD_VERSION " FIRMWARE_URL:" FIRMWARE_URL " PROTOCOL_VERSION:" PROTOCOL_VERSION " MACHINE_TYPE:" MACHINE_NAME " EXTRUDER_COUNT:" STRINGIFY(EXTRUDERS) " UUID:" MACHINE_UUID "\n"
#define MSG_COUNT_X                         " Count X: "
#define MSG_ERR_KILLED                      "Printer halted. kill() called!"
#define MSG_ERR_STOPPED                     "Printer stopped due to errors. Fix the error and use M999 to restart. (Temperature is reset. Set it after restarting)"
#define MSG_UNKNOWN_COMMAND                 "Unknown command: \""
#define MSG_ACTIVE_DRIVER                   "Active Driver: "
#define MSG_ACTIVE_EXTRUDER                 "Active Extruder: "
#define MSG_ACTIVE_COLOR                    "Active Color: "
#define MSG_X_MIN                           "x_min: "
#define MSG_X_MAX                           "x_max: "
#define MSG_Y_MIN                           "y_min: "
#define MSG_Y_MAX                           "y_max: "
#define MSG_Z_MIN                           "z_min: "
#define MSG_Z_MAX                           "z_max: "
#define MSG_Z2_MAX                          "z2_max: "
#define MSG_Z_PROBE                         "z_probe: "
#define MSG_E_MIN                           "e_min: "
#define MSG_M119_REPORT                     "Reporting endstop status"
#define MSG_ENDSTOP_HIT                     "TRIGGERED"
#define MSG_ENDSTOP_OPEN                    "NOT TRIGGERED"
#define MSG_HOTEND_OFFSET                   "Hotend offsets:"
#define MSG_EMPTY_PLANE                     "Autolevel can only be execute on an actual plane, make sure width and height are not 0!"
#define MSG_ERR_MATERIAL_INDEX              "M145 S<index> out of range (0-2)"
#define MSG_FILRUNOUT_PIN                   "filament_runout_pin: "
#define MSG_ERR_M428_TOO_FAR                "Too far from reference point"

#define MSG_SD_CANT_OPEN_SUBDIR             "Cannot open subdir"
#define MSG_SD_INIT_FAIL                    "SD init fail"
#define MSG_SD_VOL_INIT_FAIL                "volume.init failed"
#define MSG_SD_OPENROOT_FAIL                "openRoot failed"
#define MSG_SD_CARD_OK                      "SD card ok"
#define MSG_SD_WORKDIR_FAIL                 "workDir open failed"
#define MSG_SD_OPEN_FILE_FAIL               "open failed, File: "
#define MSG_SD_FILE_OPENED                  "File opened: "
#define MSG_SD_SIZE                         " Size: "
#define MSG_SD_FILE_SELECTED                "File selected"
#define MSG_SD_WRITE_TO_FILE                "Writing to file: "
#define MSG_SD_PRINTING_BYTE                "SD printing byte "
#define MSG_SD_NOT_PRINTING                 "Not SD printing"
#define MSG_SD_ERR_WRITE_TO_FILE            "error writing to file"
#define MSG_SD_CANT_ENTER_SUBDIR            "Cannot enter subdir: "
#define MSG_SD_FILE_DELETED                 "File deleted:"
#define MSG_SD_SLASH                        "/"
#define MSG_SD_FILE_DELETION_ERR            "Deletion failed, File: "
#define MSG_SD_MAX_DEPTH                    "trying to call sub-gcode files with too many levels. MAX level is:"

#define MSG_STEPPER_TOO_HIGH                "Steprate too high: "
#define MSG_ENDSTOPS_HIT                    "endstops hit: "
#define MSG_ERR_COLD_EXTRUDE_STOP           " cold extrusion prevented"
#define MSG_ERR_LONG_EXTRUDE_STOP           " too long extrusion prevented"
#define MSG_BABYSTEPPING_X                  "Babystepping X"
#define MSG_BABYSTEPPING_Y                  "Babystepping Y"
#define MSG_BABYSTEPPING_Z                  "Babystepping Z"
#define MSG_SERIAL_ERROR_MENU_STRUCTURE     "Error in menu structure"
#define MSG_MICROSTEP_MS1_MS2                "MS1,MS2 Pins"
#define MSG_MICROSTEP_X                      "X:"
#define MSG_MICROSTEP_Y                      "Y:"
#define MSG_MICROSTEP_Z                      "Z:"
#define MSG_MICROSTEP_E0                     "E0:"
#define MSG_MICROSTEP_E1                     "E1:"
#define MSG_ENDSTOP_X                        " X:"
#define MSG_ENDSTOP_Y                        " Y:"
#define MSG_ENDSTOP_Z                        " Z:"
#define MSG_ENDSTOP_E                        " E:"
#define MSG_ENDSTOP_ZP                       " ZP:"

#define MSG_ERR_EEPROM_WRITE                "Error writing to EEPROM!"

// temperature.cpp strings
#define MSG_PID_AUTOTUNE                    "PID Autotune"
#define MSG_PID_AUTOTUNE_START              MSG_PID_AUTOTUNE " start"
#define MSG_PID_AUTOTUNE_FAILED             MSG_PID_AUTOTUNE " failed!"
#define MSG_PID_BAD_EXTRUDER_NUM            MSG_PID_AUTOTUNE_FAILED " Bad extruder number"
#define MSG_PID_TEMP_TOO_HIGH               MSG_PID_AUTOTUNE_FAILED " Temperature too high"
#define MSG_PID_TIMEOUT                     MSG_PID_AUTOTUNE_FAILED " timeout"
#define MSG_BIAS                            " bias: "
#define MSG_D                               " d: "
#define MSG_T_MIN                           " min: "
#define MSG_T_MAX                           " max: "
#define MSG_KU                              " Ku: "
#define MSG_TU                              " Tu: "
#define MSG_CLASSIC_PID                     " Classic PID "
#define MSG_KP                              " Kp: "
#define MSG_KI                              " Ki: "
#define MSG_KD                              " Kd: "
#define MSG_B                               " B:"
#define MSG_T                               " T:"
#define MSG_AT                              " @:"
#define MSG_PID_AUTOTUNE_FINISHED           MSG_PID_AUTOTUNE " finished! Put the last Kp, Ki and Kd constants from above into Configuration.h or send command M500 for save in EEPROM the new value!"

#define MSG_HEATING_FAILED                  "Heating failed"
#define MSG_THERMAL_RUNAWAY_STOP            "Thermal Runaway, system stopped! Heater_ID: "
#define MSG_THERMAL_RUNAWAY_BED             "bed"
#define MSG_TEMP_READ_ERROR                 "Temp measurement error!"
#define MSG_TEMP_BED                        "bed"
#define MSG_EXTRUDER_SWITCHED_OFF           "Extruder switched off. Temperature difference between temp sensors is too high !"

#define MSG_INVALID_EXTRUDER_NUM            " - Invalid extruder number !"
#define MSG_SWITCHED_OFF_MAX                " switched off. MAXTEMP triggered !!"
#define MSG_MINTEMP_EXTRUDER_OFF            ": Extruder switched off. MINTEMP triggered !"
#define MSG_MAXTEMP_EXTRUDER_OFF            ": Extruder" MSG_SWITCHED_OFF_MAX
#define MSG_MAXTEMP_BED_OFF                 "Heated bed" MSG_SWITCHED_OFF_MAX

#define MSG_ENDSTOP_XS                      "X"
#define MSG_ENDSTOP_YS                      "Y"
#define MSG_ENDSTOP_ZS                      "Z"
#define MSG_ENDSTOP_ZPS                     "ZP"
#define MSG_ENDSTOP_ES                      "E"

//watchdog.cpp
#define MSG_WATCHDOG_RESET                  "Something is wrong, please turn off the printer."

//other
#define MSG_COMPILED                        "Compiled: "
#define MSG_ERR_HOMING_DIV                  "The Homing Bump Feedrate Divisor cannot be less than 1"
#define MSG_BED_LEVELLING_BED               "Bed"
#define MSG_BED_LEVELLING_X                 " X: "
#define MSG_BED_LEVELLING_Y                 " Y: "
#define MSG_BED_LEVELLING_Z                 " Z: "

// LCD Menu Messages

#if !(defined( DISPLAY_CHARSET_HD44780_JAPAN ) || defined( DISPLAY_CHARSET_HD44780_WESTERN ) || defined( DISPLAY_CHARSET_HD44780_CYRILLIC ))
  #define DISPLAY_CHARSET_HD44780_JAPAN
#endif

#include "language_en.h" // English

#if LANGUAGE_CHOICE == 2 // Polish
  #include "language_pl.h"
#elif LANGUAGE_CHOICE == 3 // French
  #include "language_fr.h"
#elif LANGUAGE_CHOICE == 4 // German
  #include "language_de.h"
#elif LANGUAGE_CHOICE == 5 // Spanish
  #include "language_es.h"
#elif LANGUAGE_CHOICE == 6 // Russian
  #include "language_ru.h"
#elif LANGUAGE_CHOICE == 7 // Italian
  #include "language_it.h"
#elif LANGUAGE_CHOICE == 8 // Portuguese
  #include "language_pt.h"
#elif LANGUAGE_CHOICE == 9 // Finnish
  #include "language_fi.h"
#elif LANGUAGE_CHOICE == 10 // Aragonese
  #include "language_an.h"
#elif LANGUAGE_CHOICE == 11 // Dutch
  #include "language_nl.h"
#elif LANGUAGE_CHOICE == 12 // Catalan
  #include "language_ca.h"
#elif LANGUAGE_CHOICE == 13 // Basque-Euskera
  #include "language_eu.h"
#elif LANGUAGE_CHOICE == 14 // Portuguese - Brasil
  #include "language_pt-br.h"
#endif

#endif //__LANGUAGE_H
