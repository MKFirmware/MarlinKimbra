/**
 * Italian
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_IT_H
#define LANGUAGE_IT_H

#define MAPPER_NON                  // For direct asci codes
#define DISPLAY_CHARSET_ISO10646_1  // use the better font on full graphic displays.


#define WELCOME_MSG                         MACHINE_NAME " pronta."
#define MSG_SD                              "SD"
#define MSG_SD_INSERTED                     MSG_SD " inserita"
#define MSG_SD_REMOVED                      MSG_SD " rimossa"
#define MSG_MAIN                            "Menu principale"
#define MSG_AUTOSTART                       "Autostart"
#define MSG_DISABLE_STEPPERS                "Disabilita Motori"
#define MSG_AUTO_HOME                       "Auto Home"
#define MSG_MBL_SETTING                     "Liv. piatto manuale "
#define MSG_MBL_BUTTON                      " Premi il tasto   "
#define MSG_MBL_INTRO                       " Liv. piatto...    "
#define MSG_MBL_1                           " Calibra il primo punto"
#define MSG_MBL_2                           " Calibra il secondo punto"
#define MSG_MBL_3                           " Calibra il terzo punto"
#define MSG_MBL_4                           " Calibra il quarto punto"
#define MSG_MBL_5                           "    Va bene?       "
#define MSG_MBL_6                           " Piatto livellato!       "
#define MSG_SET_HOME_OFFSETS                "Setta offset home"
#define MSG_SET_ORIGIN                      "Imposta Origine"
#define MSG_ONFOR                           "On x:"
#define MSG_PWRCONSUMED                     "P.za:"
#define MSG_FILCONSUMED                     "F:"
#define MSG_PREHEAT                         "Preriscalda"
#define MSG_PREHEAT_PLA                     "Preriscalda PLA"
#define MSG_PREHEAT_PLA_ALL                 "Prer. PLA Tutto"
#define MSG_PREHEAT_PLA_BEDONLY             "Prer. PLA Piatto"
#define MSG_PREHEAT_PLA_SETTINGS            "Config. prer. PLA"
#define MSG_PREHEAT_ABS                     "Preriscalda ABS"
#define MSG_PREHEAT_ABS_ALL                 "Prer. ABS Tutto"
#define MSG_PREHEAT_ABS_BEDONLY             "Prer. ABS Piatto"
#define MSG_PREHEAT_ABS_SETTINGS            "Config. prer. ABS"
#define MSG_PREHEAT_GUM                     "Preriscalda GOMMA"
#define MSG_PREHEAT_GUM_ALL                 "Preri. GOMMA Tutto"
#define MSG_PREHEAT_GUM_BEDONLY             "Preri. GOMMA Piatto"
#define MSG_PREHEAT_GUM_SETTINGS            "Config. prer. GOMMA"
#define MSG_TOO_COLD_FOR_FILAMENTCHANGE     "Hotend troppo freddo per il cambio filo"
#define MSG_COOLDOWN                        "Raffredda"
#define MSG_SWITCH_PS_ON                    "Accendi aliment."
#define MSG_SWITCH_PS_OFF                   "Spegni aliment."
#define MSG_EXTRUDE                         "Estrudi"
#define MSG_RETRACT                         "Ritrai"
#define MSG_PURGE                           "Purge"
#define MSG_LEVEL_BED                       "Liv. piatto"
#define MSG_SPEED                           "Velocita"
#define MSG_NOZZLE                          "Ugello"
#define MSG_BED                             "Piatto"
#define MSG_FAN_SPEED                       "Ventola"
#define MSG_FLOW                            "Flusso"
#define MSG_CONTROL                         "Controllo"
#define MSG_STATS                           "Statistiche"
#define MSG_FIX_LOSE_STEPS                  "Fix axis steps"
#define MSG_MIN                             LCD_STR_THERMOMETER " Min"
#define MSG_MAX                             LCD_STR_THERMOMETER " Max"
#define MSG_FACTOR                          LCD_STR_THERMOMETER " Fact"
#define MSG_IDLEOOZING                      "Anti oozing"
#define MSG_AUTOTEMP                        "Autotemp"
#define MSG_ON                              "ON "
#define MSG_OFF                             "OFF"
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
#define MSG_MOVE                            "Muovi"
#define MSG_MOVE_AXIS                       MSG_MOVE " asse"
#define MSG_MOVE_X                          MSG_MOVE " " MSG_X
#define MSG_MOVE_Y                          MSG_MOVE " " MSG_Y
#define MSG_MOVE_Z                          MSG_MOVE " " MSG_Z
#define MSG_MOVE_01MM                       MSG_MOVE " 0.1mm"
#define MSG_MOVE_1MM                        MSG_MOVE " 1mm"
#define MSG_MOVE_10MM                       MSG_MOVE " 10mm"
#define MSG_MOVE_E                          "Estrusore"
#define MSG_VMIN                            "Vmin"
#define MSG_VTRAV_MIN                       "VTrav min"
#define MSG_AMAX                            "Amax "
#define MSG_A_RETRACT                       "A-retract"
#define MSG_A_TRAVEL                        "A-travel"
#define MSG_XSTEPS                          MSG_X " steps/mm"
#define MSG_YSTEPS                          MSG_Y " steps/mm"
#define MSG_ZSTEPS                          MSG_Z " steps/mm"
#define MSG_E0STEPS                         MSG_E "0 steps/mm"
#define MSG_E1STEPS                         MSG_E "1 steps/mm"
#define MSG_E2STEPS                         MSG_E "2 steps/mm"
#define MSG_E3STEPS                         MSG_E "3 steps/mm"
#define MSG_TEMPERATURE                     "Temperatura"
#define MSG_MOTION                          "Movimento"
#define MSG_FILAMENT                        "Filamento"
#define MSG_VOLUMETRIC_ENABLED              MSG_E " in mm3"
#define MSG_FILAMENT_SIZE_EXTRUDER          "Diam. filo"
#define MSG_CONTRAST                        "Contrasto LCD"
#define MSG_STORE_EPROM                     "Salva in EEPROM"
#define MSG_LOAD_EPROM                      "Carica da EEPROM"
#define MSG_RESTORE_FAILSAFE                "Impostaz. default"
#define MSG_REFRESH                         "Aggiorna"
#define MSG_WATCH                           "Guarda"
#define MSG_PREPARE                         "Prepara"
#define MSG_TUNE                            "Adatta"
#define MSG_PAUSE_PRINT                     "Pausa"
#define MSG_RESUME_PRINT                    "Riprendi stampa"
#define MSG_STOP_PRINT                      "Arresta stampa"
#define MSG_CARD_MENU                       "SD Card Menu"
#define MSG_NO_CARD                         "No SD Card"
#define MSG_DWELL                           "Sospensione..."
#define MSG_USERWAIT                        "Attendi Utente..."
#define MSG_RESUMING                        "Riprendi Stampa"
#define MSG_PRINT_ABORTED                   "Stampa abortita"
#define MSG_NO_MOVE                         "Nessun Movimento"
#define MSG_KILLED                          "UCCISO "
#define MSG_STOPPED                         "ARRESTATO "
#define MSG_CONTROL_RETRACT                 "Ritrai mm"
#define MSG_CONTROL_RETRACT_SWAP            "Scamb. Ritrai mm"
#define MSG_CONTROL_RETRACTF                "Ritrai  V"
#define MSG_CONTROL_RETRACT_ZLIFT           "Salta mm"
#define MSG_CONTROL_RETRACT_RECOVER         "UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "Scamb. UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "UnRet  V"
#define MSG_AUTORETRACT                     "AutoArretramento"
#define MSG_FILAMENTCHANGE                  "Cambia filamento"
#define MSG_INIT_SDCARD                     "Iniz. SD-Card"
#define MSG_CNG_SDCARD                      "Cambia SD-Card"
#define MSG_ZPROBE_OUT                      "Z probe out. bed"
#define MSG_POSITION_UNKNOWN                "Home X/Y before Z"
#define MSG_ZPROBE_ZOFFSET                  "Z Offset"
#define MSG_BABYSTEP                        "Babystep"
#define MSG_BABYSTEP_X                      MSG_BABYSTEP " " MSG_X
#define MSG_BABYSTEP_Y                      MSG_BABYSTEP " " MSG_Y
#define MSG_BABYSTEP_Z                      MSG_BABYSTEP " " MSG_Z
#define MSG_ENDSTOP_ABORT                   "Finecorsa abort."
#define MSG_HEATING_FAILED_LCD              "Riscaldamento fallito"
#define MSG_ERR_REDUNDANT_TEMP              "REDUNDANT TEMP ERROR"
#define MSG_THERMAL_RUNAWAY                 "THERMAL RUNAWAY"
#define MSG_AD595                           "AD595 Offset & Gain"
#define MSG_ERR_MAXTEMP                     "MAXTEMP ERROR"
#define MSG_ERR_MINTEMP                     "MINTEMP ERROR"
#define MSG_ERR_MAXTEMP_BED                 "MAXTEMP BED ERROR"
#define MSG_ERR_MINTEMP_BED                 "MINTEMP BED ERROR"
#define MSG_END_DAY                         "giorni"
#define MSG_END_HOUR                        "ore"
#define MSG_END_MINUTE                      "minuti"

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
  #define MSG_DELTA_CALIBRATE               "Calibraz. Delta"
  #define MSG_DELTA_CALIBRATE_X             "Calibra X"
  #define MSG_DELTA_CALIBRATE_Y             "Calibra Y"
  #define MSG_DELTA_CALIBRATE_Z             "Calibra Z"
  #define MSG_DELTA_CALIBRATE_CENTER        "Calibra Centro"
#endif // DELTA

// Scara
#if MECH(SCARA)
  #define MSG_SCALE                         "Scale"
  #define MSG_XSCALE                        MSG_X " " MSG_SCALE
  #define MSG_YSCALE                        MSG_Y " " MSG_SCALE
#endif

#define MSG_HEATING                         "Riscaldamento..."
#define MSG_HEATING_COMPLETE                "Riscaldamento finito."
#define MSG_BED_HEATING                     "Riscaldamento piatto."
#define MSG_BED_DONE                        "Piatto riscaldato."

// Extra
#define MSG_LASER                           "Laser Preset"
#define MSG_CONFIG                          "Configurazione"
#define MSG_E_BOWDEN_LENGTH                 "Estrudi " STRINGIFY(BOWDEN_LENGTH) "mm"
#define MSG_R_BOWDEN_LENGTH                 "Retrai " STRINGIFY(BOWDEN_LENGTH) "mm"
#define MSG_PURGE_XMM                       "Purga " STRINGIFY(LCD_PURGE_LENGTH) "mm"
#define MSG_RETRACT_XMM                     "Retrai " STRINGIFY(LCD_RETRACT_LENGTH) "mm"
#define MSG_SAVED_POS                       "Posizione Salvata"
#define MSG_RESTORING_POS                   "Ripristino posizione"
#define MSG_INVALID_POS_SLOT                "Slot invalido, slot totali: "

// Rfid module
#if ENABLED(RFID_MODULE)
  #define MSG_RFID_SPOOL                    "Bobina su E"
  #define MSG_RFID_BRAND                    "Marca: "
  #define MSG_RFID_TYPE                     "Tipo: "
  #define MSG_RFID_COLOR                    "Colore: "
  #define MSG_RFID_SIZE                     "Size: "
  #define MSG_RFID_TEMP_HOTEND              "Temperatura Hotend: "
  #define MSG_RFID_TEMP_BED                 "Temperatura Bed: "
  #define MSG_RFID_TEMP_USER_HOTEND         "Temperatura utente Hotend: "
  #define MSG_RFID_TEMP_USER_BED            "Temperatura utente Bed: "
  #define MSG_RFID_DENSITY                  "Densita': "
  #define MSG_RFID_SPOOL_LENGHT             "Lunghezza bobina: "
#endif

// Firmware Test
#if ENABLED(FIRMWARE_TEST)
  #define MSG_FWTEST_YES                    "Dai il comando Y per andare avanti"
  #define MSG_FWTEST_NO                     "Dai il comando N per andare avanti"
  #define MSG_FWTEST_YES_NO                 "Dai il comando Y o N per andare avanti"
  #define MSG_FWTEST_ENDSTOP_ERR            "ENDSTOP ERROR! Controllare cavi e connessioni"
  #define MSG_FWTEST_PRESS                  "Premere e tenere premuto l'endstop "
  #define MSG_FWTEST_INVERT                 "Invertire valore di "
  #define MSG_FWTEST_XAXIS                  "Il nozzle si e' spostato a destra?"
  #define MSG_FWTEST_YAXIS                  "Il nozzle si e' spostato in avanti?"
  #define MSG_FWTEST_ZAXIS                  "Il nozzle si e' spostato in alto?"
  #define MSG_FWTEST_01                     "Muovi manualmente gli assi X, Y e Z lontano dagli endstop"
  #define MSG_FWTEST_02                     "Vuoi controllare gli ENDSTOP?"
  #define MSG_FWTEST_03                     "Inizio controllo ENDSTOP"
  #define MSG_FWTEST_04                     "Inizio controllo MOTORI"
  #define MSG_FWTEST_ATTENTION              "ATTENZIONE! Controlla che i tre assi siano a piu' di 5 mm dagli endstop!"
  #define MSG_FWTEST_END                    "Test finito. Disabilitare FIRMWARE_TEST e ricompilare."
  #define MSG_FWTEST_INTO                   "in "
  #define MSG_FWTEST_ERROR                  "ERRORE"
  #define MSG_FWTEST_OK                     "OK"
  #define MSG_FWTEST_NDEF                   "non definito"
#endif // FIRMWARE_TEST

#endif // LANGUAGE_IT_H
