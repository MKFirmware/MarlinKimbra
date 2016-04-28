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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef LANGUAGE_H
#define LANGUAGE_H

// NOTE: IF YOU CHANGE LANGUAGE FILES OR MERGE A FILE WITH CHANGES
//
//   ==> ALWAYS TRY TO COMPILE MARLIN WITH/WITHOUT "ULTIPANEL" / "ULTRALCD" / "SDSUPPORT" #define IN "Configuration_Basic.h"
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
// 12 Danish
// 13 Catalan
// 14 Basque-Euskera
// 15 Portuguese (Brazil)
// 16 Bulgarian
// 17 Japanese
// 18 Japanese utf
// 19 Chinese

#define STRINGIFY_(n) #n
#define STRINGIFY(n) STRINGIFY_(n)

#define PROTOCOL_VERSION "2.0"

#if MB(ULTIMAKER)|| MB(ULTIMAKER_OLD)|| MB(ULTIMAIN_2)
  #define MACHINE_NAME "Ultimaker"
#elif MB(RUMBA)
  #define MACHINE_NAME "Rumba"
#elif MB(3DRAG)
  #define MACHINE_NAME "3Drag"
#elif MB(K8200)
  #define MACHINE_NAME "K8200"
#elif MB(5DPRINT)
  #define MACHINE_NAME "Makibox"
#elif MB(SAV_MKI)
  #define MACHINE_NAME "SAV MkI"
#elif DISABLED(MACHINE_NAME)
  #define MACHINE_NAME "3D Printer"
#endif

#if ENABLED(CUSTOM_MACHINE_NAME)
  #undef MACHINE_NAME
  #define MACHINE_NAME CUSTOM_MACHINE_NAME
#endif

// LCD Menu Messages
#if !(ENABLED( DISPLAY_CHARSET_HD44780_JAPAN ) || ENABLED( DISPLAY_CHARSET_HD44780_WESTERN ) || ENABLED( DISPLAY_CHARSET_HD44780_CYRILLIC ))
  #define DISPLAY_CHARSET_HD44780_JAPAN
#endif

// Serial Console Messages (do not translate those!)
#if MECH(CARTESIAN)
  #define SERIAL_M115_REPORT                    "FIRMWARE_NAME:" BUILD_VERSION " FIRMWARE_URL:" FIRMWARE_URL " PROTOCOL_VERSION:" PROTOCOL_VERSION " MACHINE_TYPE:Cartesian EXTRUDER_COUNT:" STRINGIFY(EXTRUDERS) " UUID:" MACHINE_UUID "\n"
#elif MECH(COREXY)
  #define SERIAL_M115_REPORT                    "FIRMWARE_NAME:" BUILD_VERSION " FIRMWARE_URL:" FIRMWARE_URL " PROTOCOL_VERSION:" PROTOCOL_VERSION " MACHINE_TYPE:CoreXY EXTRUDER_COUNT:" STRINGIFY(EXTRUDERS) " UUID:" MACHINE_UUID "\n"
#elif MECH(COREYX)
  #define SERIAL_M115_REPORT                    "FIRMWARE_NAME:" BUILD_VERSION " FIRMWARE_URL:" FIRMWARE_URL " PROTOCOL_VERSION:" PROTOCOL_VERSION " MACHINE_TYPE:CoreYX EXTRUDER_COUNT:" STRINGIFY(EXTRUDERS) " UUID:" MACHINE_UUID "\n"
#elif MECH(COREXZ)
  #define SERIAL_M115_REPORT                    "FIRMWARE_NAME:" BUILD_VERSION " FIRMWARE_URL:" FIRMWARE_URL " PROTOCOL_VERSION:" PROTOCOL_VERSION " MACHINE_TYPE:CoreXZ EXTRUDER_COUNT:" STRINGIFY(EXTRUDERS) " UUID:" MACHINE_UUID "\n"
#elif MECH(COREZX)
  #define SERIAL_M115_REPORT                    "FIRMWARE_NAME:" BUILD_VERSION " FIRMWARE_URL:" FIRMWARE_URL " PROTOCOL_VERSION:" PROTOCOL_VERSION " MACHINE_TYPE:CoreZX EXTRUDER_COUNT:" STRINGIFY(EXTRUDERS) " UUID:" MACHINE_UUID "\n"
#elif MECH(DELTA)
  #define SERIAL_M115_REPORT                    "FIRMWARE_NAME:" BUILD_VERSION " FIRMWARE_URL:" FIRMWARE_URL " PROTOCOL_VERSION:" PROTOCOL_VERSION " MACHINE_TYPE:Delta EXTRUDER_COUNT:" STRINGIFY(EXTRUDERS) " UUID:" MACHINE_UUID "\n"
#elif MECH(SCARA)
  #define SERIAL_M115_REPORT                    "FIRMWARE_NAME:" BUILD_VERSION " FIRMWARE_URL:" FIRMWARE_URL " PROTOCOL_VERSION:" PROTOCOL_VERSION " MACHINE_TYPE:Scara EXTRUDER_COUNT:" STRINGIFY(EXTRUDERS) " UUID:" MACHINE_UUID "\n"
#endif  

#define SERIAL_ENQUEUEING                       "enqueueing \""
#define SERIAL_POWERUP                          "PowerUp"
#define SERIAL_EXTERNAL_RESET                   "External Reset"
#define SERIAL_BROWNOUT_RESET                   "Brown out Reset"
#define SERIAL_WATCHDOG_RESET                   "Watchdog Reset"
#define SERIAL_SOFTWARE_RESET                   "Software Reset"
#define SERIAL_AUTHOR                           " | Author: "
#define SERIAL_CONFIGURATION_VER                "Last Updated: "
#define SERIAL_COMPILED                         "Compiled: "
#define SERIAL_FREE_MEMORY                      "Free Memory: "
#define SERIAL_PLANNER_BUFFER_BYTES             " PlannerBufferBytes: "
#define SERIAL_ERR_LINE_NO                      "Line Number is not Last Line Number+1, Last Line: "
#define SERIAL_ERR_CHECKSUM_MISMATCH            "checksum mismatch, Last Line: "
#define SERIAL_ERR_NO_CHECKSUM                  "No Checksum with line number, Last Line: "
#define SERIAL_ERR_NO_LINENUMBER_WITH_CHECKSUM  "No Line Number with checksum, Last Line: "
#define SERIAL_FILE_PRINTED                     "Done printing file"
#define SERIAL_BEGIN_FILE_LIST                  "Begin file list"
#define SERIAL_END_FILE_LIST                    "End file list"
#define SERIAL_INVALID_EXTRUDER                 "Invalid extruder"
#define SERIAL_INVALID_HOTEND                   "Invalid hotend"
#define SERIAL_INVALID_SOLENOID                 "Invalid solenoid"
#define SERIAL_ERR_NO_THERMISTORS               "No thermistors - no temperature"
#define SERIAL_COUNT_X                          " Count X: "
#define SERIAL_ERR_KILLED                       "Printer halted. kill() called!"
#define SERIAL_ERR_STOPPED                      "Printer stopped due to errors. Fix the error and use M999 to restart. (Temperature is reset. Set it after restarting)"
#define SERIAL_BUSY_PROCESSING                  "processing"
#define SERIAL_BUSY_PAUSED_FOR_USER             "paused for user"
#define SERIAL_BUSY_PAUSED_FOR_INPUT            "paused for input"
#define SERIAL_UNKNOWN_COMMAND                  "Unknown command: \""
#define SERIAL_ACTIVE_DRIVER                    "Active Driver: "
#define SERIAL_ACTIVE_EXTRUDER                  "Active Extruder: "
#define SERIAL_ACTIVE_COLOR                     "Active Color: "
#define MSG_COUNT_X                             " Count X:"
#define MSG_COUNT_A                             " Count A:"
#define MSG_COUNT_ALPHA                         " Count Alpha:"
#define SERIAL_X_MIN                            "x_min: "
#define SERIAL_X_MAX                            "x_max: "
#define SERIAL_Y_MIN                            "y_min: "
#define SERIAL_Y_MAX                            "y_max: "
#define SERIAL_Z_MIN                            "z_min: "
#define SERIAL_Z_MAX                            "z_max: "
#define SERIAL_Z2_MAX                           "z2_max: "
#define SERIAL_Z_PROBE                          "z_probe: "
#define SERIAL_E_MIN                            "e_min: "
#define SERIAL_ERR_MATERIAL_INDEX               "M145 S<index> out of range (0-2)"
#define SERIAL_ERR_M428_TOO_FAR                 "Too far from reference point"
#define SERIAL_M119_REPORT                      "Reporting endstop status"
#define SERIAL_ENDSTOP_HIT                      "TRIGGERED"
#define SERIAL_ENDSTOP_OPEN                     "NOT TRIGGERED"
#define SERIAL_HOTEND_OFFSET                    "Hotend offsets:"
#define SERIAL_EMPTY_PLANE                      "Autolevel can only be execute on an actual plane, make sure width and height are not 0!"
#define SERIAL_FILRUNOUT_PIN                    "filament_runout_pin: "

// SD Card
#define SERIAL_SD_ERRORCODE                     "SD errorCode:"
#define SERIAL_SD_CANT_OPEN_SUBDIR              "Cannot open subdir"
#define SERIAL_SD_INIT_FAIL                     "SD init fail"
#define SERIAL_SD_VOL_INIT_FAIL                 "volume.init failed"
#define SERIAL_SD_OPENROOT_FAIL                 "openRoot failed"
#define SERIAL_SD_CARD_OK                       "SD card ok"
#define SERIAL_SD_WORKDIR_FAIL                  "workDir open failed"
#define SERIAL_SD_OPEN_FILE_FAIL                "open failed, File: "
#define SERIAL_SD_FILE_OPENED                   "File opened: "
#define SERIAL_SD_SIZE                          " Size: "
#define SERIAL_SD_FILE_SELECTED                 "File selected"
#define SERIAL_SD_WRITE_TO_FILE                 "Writing to file: "
#define SERIAL_SD_FILE_SAVED                    "Done saving file."
#define SERIAL_SD_PRINTING_BYTE                 "SD printing byte "
#define SERIAL_SD_NOT_PRINTING                  "Not SD printing"
#define SERIAL_SD_ERR_WRITE_TO_FILE             "error writing to file"
#define SERIAL_SD_CANT_ENTER_SUBDIR             "Cannot enter subdir: "
#define SERIAL_SD_FILE_DELETED                  "File deleted"
#define SERIAL_SD_FILE_DELETION_ERR             "Deletion failed"
#define SERIAL_SD_DIRECTORY_CREATED             "Directory created"
#define SERIAL_SD_CREATION_FAILED               "Creation failed"
#define SERIAL_SD_SLASH                         "/"
#define SERIAL_SD_MAX_DEPTH                     "trying to call sub-gcode files with too many levels. MAX level is:"

#define SERIAL_STEPPER_TOO_HIGH                 "Steprate too high: "
#define SERIAL_ENDSTOPS_HIT                     "endstops hit: "
#define SERIAL_ERR_COLD_EXTRUDE_STOP            "cold extrusion prevented"
#define SERIAL_ERR_LONG_EXTRUDE_STOP            "too long extrusion prevented"
#define SERIAL_MICROSTEP_MS1_MS2                "MS1,MS2 Pins"
#define SERIAL_MICROSTEP_X                      "X:"
#define SERIAL_MICROSTEP_Y                      "Y:"
#define SERIAL_MICROSTEP_Z                      "Z:"
#define SERIAL_MICROSTEP_E0                     "E0:"
#define SERIAL_MICROSTEP_E1                     "E1:"

#define SERIAL_ERR_EEPROM_WRITE                 "Error writing to EEPROM!"

// temperature.cpp strings
#define SERIAL_PID_AUTOTUNE                     "PID Autotune"
#define SERIAL_PID_AUTOTUNE_START               SERIAL_PID_AUTOTUNE " start"
#define SERIAL_PID_AUTOTUNE_FAILED              SERIAL_PID_AUTOTUNE " failed!"
#define SERIAL_PID_BAD_EXTRUDER_NUM             SERIAL_PID_AUTOTUNE_FAILED " Bad extruder number"
#define SERIAL_PID_TEMP_TOO_HIGH                SERIAL_PID_AUTOTUNE_FAILED " Temperature too high"
#define SERIAL_PID_TIMEOUT                      SERIAL_PID_AUTOTUNE_FAILED " timeout"
#define SERIAL_BIAS                             " bias: "
#define SERIAL_D                                " d: "
#define SERIAL_T_MIN                            " min: "
#define SERIAL_T_MAX                            " max: "
#define SERIAL_KU                               " Ku: "
#define SERIAL_TU                               " Tu: "
#define SERIAL_CLASSIC_PID                      " Classic PID "
#define SERIAL_KP                               " Kp: "
#define SERIAL_KI                               " Ki: "
#define SERIAL_KD                               " Kd: "
#define SERIAL_T                                " T:"
#define SERIAL_B                                " B:"
#define SERIAL_AT                               " @"
#define SERIAL_BAT                              " B@:"
#define SERIAL_W                                " W:"
#define SERIAL_PID_AUTOTUNE_FINISHED            SERIAL_PID_AUTOTUNE " finished! Put the last Kp, Ki and Kd constants from above into Configuration.h or send command M500 for save in EEPROM the new value!"
#define SERIAL_PID_DEBUG                        " PID_DEBUG "
#define SERIAL_PID_DEBUG_INPUT                  ": Input "
#define SERIAL_PID_DEBUG_OUTPUT                 " Output "
#define SERIAL_PID_DEBUG_PTERM                  " pTerm "
#define SERIAL_PID_DEBUG_ITERM                  " iTerm "
#define SERIAL_PID_DEBUG_DTERM                  " dTerm "
#define SERIAL_PID_DEBUG_CTERM                  " cTerm "
#define SERIAL_INVALID_EXTRUDER_NUM             " - Invalid extruder number !"

#define SERIAL_HEATER_BED                       "bed"
#define SERIAL_STOPPED_HEATER                   ", system stopped! Heater_ID: "
//#define SERIAL_REDUNDANCY                     "Heater switched off. Temperature difference between temp sensors is too high !"
#define SERIAL_T_HEATING_FAILED                 "Heating failed"
#define SERIAL_T_THERMAL_RUNAWAY                "Thermal Runaway"
#define SERIAL_T_MAXTEMP                        "MAXTEMP triggered"
#define SERIAL_T_MINTEMP                        "MINTEMP triggered"

// Endstop
#define SERIAL_ENDSTOP_X                        " X="
#define SERIAL_ENDSTOP_Y                        " Y="
#define SERIAL_ENDSTOP_Z                        " Z="
#define SERIAL_ENDSTOP_E                        " E="
#define SERIAL_ENDSTOP_PROBE                    " PROBE="

// Debug
#define SERIAL_DEBUG_PREFIX                     "DEBUG:"
#define SERIAL_DEBUG_OFF                        "off"
#define SERIAL_DEBUG_ECHO                       "ECHO"
#define SERIAL_DEBUG_INFO                       "INFO"
#define SERIAL_DEBUG_ERRORS                     "ERRORS"
#define SERIAL_DEBUG_DRYRUN                     "DRYRUN"
#define SERIAL_DEBUG_COMMUNICATION              "COMMUNICATION"
#define SERIAL_DEBUG_DEBUG                      "DEBUG"

//other
#define SERIAL_ERR_HOMING_DIV                   "The Homing Bump Feedrate Divisor cannot be less than 1"
#define SERIAL_BED_LEVELLING_BED                "Bed"
#define SERIAL_BED_LEVELLING_X                  " X: "
#define SERIAL_BED_LEVELLING_Y                  " Y: "
#define SERIAL_BED_LEVELLING_Z                  " Z: "

#if   LANGUAGE_CHOICE == 1  // English
  #include "language_en.h"
#elif LANGUAGE_CHOICE == 2  // Polish
  #include "language_pl.h"
#elif LANGUAGE_CHOICE == 3  // French
  #include "language_fr.h"
#elif LANGUAGE_CHOICE == 4  // German
  #include "language_de.h"
#elif LANGUAGE_CHOICE == 5  // Spanish
  #include "language_es.h"
#elif LANGUAGE_CHOICE == 6  // Russian
	#define MAPPER_D0D1       // For Cyrillic
  #include "language_ru.h"
#elif LANGUAGE_CHOICE == 7  // Italian
  #include "language_it.h"
#elif LANGUAGE_CHOICE == 8  // Portuguese
  #include "language_pt.h"
#elif LANGUAGE_CHOICE == 9  // Finnish
  #include "language_fi.h"
#elif LANGUAGE_CHOICE == 10 // Aragonese
  #include "language_an.h"
#elif LANGUAGE_CHOICE == 11 // Dutch
  #include "language_nl.h"
#elif LANGUAGE_CHOICE == 12 // Danish
  #include "language_da.h"
#elif LANGUAGE_CHOICE == 13 // Catalan
  #include "language_ca.h"
#elif LANGUAGE_CHOICE == 14 // Basque-Euskera
  #include "language_eu.h"
#elif LANGUAGE_CHOICE == 15 // Portuguese - Brasil
  #include "language_pt-br.h"
#elif LANGUAGE_CHOICE == 16 // Bulgarian
  #include "language_bg.h"
#elif LANGUAGE_CHOICE == 17 // Japanese
  #include "language_kana.h"
#elif LANGUAGE_CHOICE == 18 // Japanese utf
  #include "language_kana_utf8.h"
#elif LANGUAGE_CHOICE == 19 // Chinese
  #include "language_cn.h"
#endif

#endif //__LANGUAGE_H
