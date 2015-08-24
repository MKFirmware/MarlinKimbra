/**
 * Configuration_Overall.h
 * Here you can define all your custom settings and they will overwrite configurations in the main configuration files.
 */


#define MECHANISM MECH_CARTESIAN
#define BAUDRATE 115200
#define MOTHERBOARD BOARD_RAMPS_13_EFB
#define POWER_SUPPLY 0
#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_BED 4
#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define HEATER_3_MAXTEMP 275
#define BED_MAXTEMP 150
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define HEATER_3_MINTEMP 5
#define BED_MINTEMP 5
#define PLA_PREHEAT_HOTEND_TEMP 190
#define PLA_PREHEAT_HPB_TEMP 60
#define PLA_PREHEAT_FAN_SPEED 255
#define ABS_PREHEAT_HOTEND_TEMP 240
#define ABS_PREHEAT_HPB_TEMP 100
#define ABS_PREHEAT_FAN_SPEED 255
#define GUM_PREHEAT_HOTEND_TEMP 230
#define GUM_PREHEAT_HPB_TEMP 60
#define GUM_PREHEAT_FAN_SPEED 255
#define EXTRUDERS 4
#define DRIVER_EXTRUDERS 1
#define LANGUAGE_CHOICE 7
#define CUSTOM_MACHINE_NAME "Prusa I3"
#undef ENDSTOPPULLUPS

#define ENDSTOPPULLUP_YMIN

#define ENDSTOPPULLUP_Z2MIN
#define ENDSTOPPULLUP_XMAX
#define ENDSTOPPULLUP_YMAX
#define ENDSTOPPULLUP_ZMAX
#define ENDSTOPPULLUP_Z2MAX
#define ENDSTOPPULLUP_ZPROBE
#define ENDSTOPPULLUP_EMIN
#define X_MIN_ENDSTOP_LOGIC false
#define Y_MIN_ENDSTOP_LOGIC false
#define Z_MIN_ENDSTOP_LOGIC false
#define Z2_MIN_ENDSTOP_LOGIC false
#define X_MAX_ENDSTOP_LOGIC false
#define Y_MAX_ENDSTOP_LOGIC false
#define Z_MAX_ENDSTOP_LOGIC false
#define Z2_MAX_ENDSTOP_LOGIC false
#define Z_PROBE_ENDSTOP_LOGIC false
#define E_MIN_ENDSTOP_LOGIC false
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1
#define E_HOME_DIR -1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0
#define INVERT_X_DIR true
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false
#define INVERT_E0_DIR false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false
#define X_MAX_POS 200
#define X_MIN_POS 0
#define Y_MAX_POS 200
#define Y_MIN_POS 0
#define Z_MAX_POS 200
#define Z_MIN_POS 0
#define LEFT_PROBE_BED_POSITION 20
#define RIGHT_PROBE_BED_POSITION 180
#define FRONT_PROBE_BED_POSITION 20
#define BACK_PROBE_BED_POSITION 180
#define DEFAULT_AXIS_STEPS_PER_UNIT {80,80,4000,625,625,625,625}
#define DEFAULT_MAX_FEEDRATE {300,300,2,100,100,100,100}
#define DEFAULT_MAX_ACCELERATION {3000,3000,50,3000,3000,3000,3000}
#define DEFAULT_RETRACT_ACCELERATION {10000,10000,10000,10000}
#define DEFAULT_ACCELERATION 2500
#define DEFAULT_TRAVEL_ACCELERATION 3000
#define DEFAULT_XYJERK 10
#define DEFAULT_ZJERK 0.4
#define DEFAULT_EJERK {5.0,5.0,5.0,5.0}
#define HOMING_FEEDRATE {100*60,100*60,2*60,0}
#define PIDTEMP
#define DEFAULT_Kp {41.51,41.51,41.51,41.51}
#define DEFAULT_Ki {7.28,7.28,7.28,7.28}
#define DEFAULT_Kd {59.17,59.17,59.17,59.17}
#define PIDTEMPBED
#define DEFAULT_bedKp 10
#define DEFAULT_bedKi 1
#define DEFAULT_bedKd 305
#define EXTRUDER_AUTO_FAN_TEMPERATURE 50
#define EXTRUDER_AUTO_FAN_SPEED 255
#define PREVENT_DANGEROUS_EXTRUDE
#define PREVENT_LENGTHY_EXTRUDE
#define SINGLENOZZLE
#define MKR4


#define BOWDEN_LENGTH 250
#define LCD_PURGE_LENGTH 10
#define LCD_RETRACT_LENGTH 5
#define LCD_PURGE_FEEDRATE 3
#define LCD_RETRACT_FEEDRATE 5
#define LCD_LOAD_FEEDRATE 20
#define LCD_UNLOAD_FEEDRATE 20

#define Z_PROBE_REPEATABILITY_TEST  // If not commented out, Z-Probe Repeatability test will be included if Auto Bed Leveling is Enabled.
#define AUTO_BED_LEVELING_GRID
#define AUTO_BED_LEVELING_GRID_POINTS 2
#define ABL_PROBE_PT_1_X 15
#define ABL_PROBE_PT_1_Y 180
#define ABL_PROBE_PT_2_X 15
#define ABL_PROBE_PT_2_Y 15
#define ABL_PROBE_PT_3_X 180
#define ABL_PROBE_PT_3_Y 15
#define X_PROBE_OFFSET_FROM_EXTRUDER 0
#define Y_PROBE_OFFSET_FROM_EXTRUDER 0
#define Z_PROBE_OFFSET_FROM_EXTRUDER -1
#define Z_RAISE_BEFORE_HOMING 10
#define Z_RAISE_BEFORE_PROBING 10
#define Z_RAISE_BETWEEN_PROBINGS 10
#define NUM_SERVOS 0

#define INVERT_Y2_VS_Y_DIR false


#define FILAMENT_SENSOR
#define FILAMENT_SENSOR_EXTRUDER_NUM 0
#define DEFAULT_NOMINAL_FILAMENT_DIA 1.75
#define MEASURED_UPPER_LIMIT 2
#define MEASURED_LOWER_LIMIT 1.35


#define FILRUNOUT_PIN_INVERTING false;
#define FILAMENT_RUNOUT_SCRIPT "M600"
#define EEPROM_SETTINGS
#define EEPROM_CHITCHAT // Uncomment this to enable EEPROM Serial responses.


#define NEXTION

#define PROGRESS_BAR_BAR_TIME 3000
#define PROGRESS_BAR_MSG_TIME 1000
#define PROGRESS_MSG_EXPIRE 0

#define MICROSTEP_MODES {16,16,16,16}
#define MOTOR_CURRENT {1,1,1,1,1,1,1}


/* Below you will find the configuration string, that created with Configurator tool online marlinkimbra.it

========== Start configuration string ==========
{
"mechanism": 0,
"motherboards": "BOARD_RAMPS_13_EFB",
"printer": "custom",
"baudrates": 115200,
"testmode": "0",
"processor": 0,
"version": 0,
"extruders": 4,
"driverextruders": 1,
"singlenozzle": "1",
"mkr4": "1",
"npr2": "0",
"E0E1pin": -1,
"E0E2pin": -1,
"E0E3pin": -1,
"E1E3pin": -1,
"power": "0",
"defaultpower": "0",
"tempsensor0": "1",
"tempsensor1": "0",
"tempsensor2": "0",
"tempsensor3": "0",
"tempsensorbed": "4",
"heater0pin": "ORIG_HEATER_0_PIN",
"heater1pin": "ORIG_HEATER_1_PIN",
"heater2pin": "ORIG_HEATER_2_PIN",
"heater3pin": "ORIG_HEATER_3_PIN",
"heaterbedpin": "ORIG_HEATER_BED_PIN",
"temp0pin": "ORIG_TEMP_0_PIN",
"temp1pin": "ORIG_TEMP_1_PIN",
"temp2pin": "ORIG_TEMP_2_PIN",
"temp3pin": "ORIG_TEMP_3_PIN",
"tempbedpin": "ORIG_TEMP_BED_PIN",
"mintemp0": 5,
"mintemp1": 5,
"mintemp2": 5,
"mintemp3": 5,
"mintempbed": 5,
"maxtemp0": 275,
"maxtemp1": 275,
"maxtemp2": 275,
"maxtemp3": 275,
"maxtempbed": 150,
"pidtemp": "1",
"pidkp0": 41.51,
"pidki0": 7.28,
"pidkd0": 59.17,
"pidkp1": 41.51,
"pidki1": 7.28,
"pidkd1": 59.17,
"pidkp2": 41.51,
"pidki2": 7.28,
"pidkd2": 59.17,
"pidkp3": 41.51,
"pidki3": 7.28,
"pidkd3": 59.17,
"pidbedtemp": "1",
"pidbedkp": 10,
"pidbedki": 1,
"pidbedkd": 305,
"dangerousextrude": "1",
"lengthyextrude": "1",
"extrudemintemp": 170,
"autobed": "0",
"zprobingrepeat": "1",
"gridmode": "1",
"gridpoint": 2,
"Zsafehoming": "0",
"ZsafehomingX": 100,
"ZsafehomingY": 100,
"leftprobe": 20,
"rightprobe": 180,
"backprobe": 180,
"frontprobe": 20,
"Xprobe1": 15,
"Yprobe1": 180,
"Xprobe2": 15,
"Yprobe2": 15,
"Xprobe3": 180,
"Yprobe3": 15,
"Xprobeoffset": 0,
"Yprobeoffset": 0,
"Zprobeoffset": -1,
"Zraisebeforehoming": 10,
"Zraisebeforeprobe": 10,
"Zraisebetweenprobe": 10,
"Xmotor": {
  "name": "X motor",
  "step": "ORIG_X_STEP_PIN",
  "dir": "ORIG_X_DIR_PIN",
  "enable": "ORIG_X_ENABLE_PIN"
},
"Ymotor": {
  "name": "Y motor",
  "step": "ORIG_Y_STEP_PIN",
  "dir": "ORIG_Y_DIR_PIN",
  "enable": "ORIG_Y_ENABLE_PIN"
},
"Zmotor": {
  "name": "Z motor",
  "step": "ORIG_Z_STEP_PIN",
  "dir": "ORIG_Z_DIR_PIN",
  "enable": "ORIG_Z_ENABLE_PIN"
},
"Y2motor": {
  "name": "Extruder 1",
  "step": "ORIG_E1_STEP_PIN",
  "dir": "ORIG_E1_DIR_PIN",
  "enable": "ORIG_E1_ENABLE_PIN"
},
"Z2motor": {
  "name": "Extruder 1",
  "step": "ORIG_E1_STEP_PIN",
  "dir": "ORIG_E1_DIR_PIN",
  "enable": "ORIG_E1_ENABLE_PIN"
},
"E0motor": {
  "name": "Extruder 0",
  "step": "ORIG_E0_STEP_PIN",
  "dir": "ORIG_E0_DIR_PIN",
  "enable": "ORIG_E0_ENABLE_PIN"
},
"E1motor": {
  "name": "Extruder 1",
  "step": "ORIG_E1_STEP_PIN",
  "dir": "ORIG_E1_DIR_PIN",
  "enable": "ORIG_E1_ENABLE_PIN"
},
"E2motor": {
  "name": "Extruder 2",
  "step": "ORIG_E2_STEP_PIN",
  "dir": "ORIG_E2_DIR_PIN",
  "enable": "ORIG_E2_ENABLE_PIN"
},
"E3motor": {
  "name": "Extruder 3",
  "step": "ORIG_E3_STEP_PIN",
  "dir": "ORIG_E3_DIR_PIN",
  "enable": "ORIG_E3_ENABLE_PIN"
},
"Ydualstepper": "0",
"Y2vsYdir": "0",
"Zdualstepper": "0",
"Zdualendstop": "0",
"Xminpos": 0,
"Xmaxpos": 200,
"Yminpos": 0,
"Ymaxpos": 200,
"Zminpos": 0,
"Zmaxpos": 200,
"defaultacceleration": 2500,
"defaulttravelacceleration": 3000,
"maxXYjerk": 10,
"maxZjerk": 0.4,
"maxE0jerk": 5,
"maxE1jerk": 5,
"maxE2jerk": 5,
"maxE3jerk": 5,
"deltasegmentpersecond": 200,
"deltadiagonalrod": 220,
"deltasmoothrodoffset": 145,
"deltaeffectoroffset": 20,
"deltacarriageoffset": 20,
"deltaprinterradius": 70,
"deltaheight": 210,
"deltaautoprecision": 0.1,
"deltaautogrid": 20,
"deltaXprobeoffset": 0,
"deltaYprobeoffset": 0,
"deltaZprobeoffset": -10,
"deltaXdeploystart": 0,
"deltaYdeploystart": 0,
"deltaZdeploystart": 30,
"deltaXdeployend": 0,
"deltaYdeployend": 0,
"deltaZdeployend": 0,
"deltaXretractstart": 0,
"deltaYretractstart": 0,
"deltaZretractstart": 30,
"deltaXretractend": 0,
"deltaYretractend": 0,
"deltaZretractend": 0,
"Xmicrostep": 16,
"Ymicrostep": 16,
"Zmicrostep": 16,
"Emicrostep": 16,
"Xcurrent": 1000,
"Ycurrent": 1000,
"Zcurrent": 1000,
"E0current": 1000,
"E1current": 1000,
"E2current": 1000,
"E3current": 1000,
"Xstepspermm": 80,
"Ystepspermm": 80,
"Zstepspermm": 4000,
"E0stepspermm": 625,
"E1stepspermm": 625,
"E2stepspermm": 625,
"E3stepspermm": 625,
"Xmaxspeed": 300,
"Ymaxspeed": 300,
"Zmaxspeed": 2,
"E0maxspeed": 100,
"E1maxspeed": 100,
"E2maxspeed": 100,
"E3maxspeed": 100,
"E0retractionspeed": 100,
"E1retractionspeed": 100,
"E2retractionspeed": 100,
"E3retractionspeed": 100,
"Xhomingspeed": 100,
"Yhomingspeed": 100,
"Zhomingspeed": 2,
"Xmaxacceleration": 3000,
"Ymaxacceleration": 3000,
"Zmaxacceleration": 50,
"E0maxacceleration": 3000,
"E1maxacceleration": 3000,
"E2maxacceleration": 3000,
"E3maxacceleration": 3000,
"E0retractacceleration": 10000,
"E1retractacceleration": 10000,
"E2retractacceleration": 10000,
"E3retractacceleration": 10000,
"Xinvert": "1",
"Yinvert": 0,
"Zinvert": 0,
"E0invert": 0,
"E1invert": 0,
"E2invert": 0,
"E3invert": 0,
"Xinvertenable": 0,
"Yinvertenable": 0,
"Zinvertenable": 0,
"Einvertenable": 0,
"disableX": 0,
"disableY": 0,
"disableZ": 0,
"disableE": 0,
"Xhoming": 0,
"Yhoming": 0,
"Zhoming": 0,
"Ehoming": 0,
"Xminendstop": "0",
"Xmaxendstop": "0",
"Yminendstop": "0",
"Ymaxendstop": "0",
"Zminendstop": "0",
"Zmaxendstop": "0",
"Z2minendstop": "0",
"Z2maxendstop": "0",
"Zprobeendstop": "0",
"Eminendstop": "0",
"Xminpullup": "0",
"Xmaxpullup": "1",
"Yminpullup": "1",
"Ymaxpullup": "1",
"Zminpullup": "0",
"Zmaxpullup": "1",
"Z2minpullup": "1",
"Z2maxpullup": "1",
"Zprobepullup": "1",
"Eminpullup": "1",
"E0coolerpin": -1,
"E1coolerpin": -1,
"E2coolerpin": -1,
"E3coolerpin": -1,
"Ecoolerspeed": 255,
"Ecoolertemp": 50,
"fanpin": "ORIG_FAN_PIN",
"sdsupport": "0",
"eeprom": "1",
"eepromchitchat": "1",
"laserbeam": "0",
"laserpwrpin": 41,
"laserttlpin": 42,
"toshiba": "0",
"powerconsumption": "1",
"powerconsumptionpin": 11,
"filamentrunout": "0",
"filrunoutpin": 7,
"filamentrunoutsensor": "0",
"filamentrunoutscript": "M600",
"servos": "0",
"numservos": 0,
"Xservo": -1,
"Yservo": -1,
"Zservo": -1,
"angleextendservosX": 0,
"angleretractservosX": 0,
"angleextendservosY": 0,
"angleretractservosY": 0,
"angleextendservosZ": 0,
"angleretractservosZ": 0,
"displays": 12,
"invertrotaryswitch": 0,
"uilanguages": 7,
"uiprintername": "Prusa I3",
"easyload": 0,
"bowdenlenght": 250,
"lcdpurgelenght": 10,
"lcdretractlenght": 5,
"lcdpurgefeedrate": 3,
"lcdretractfeedrate": 5,
"lcdloadfeedrate": 20,
"lcdunloadfeedrate": 20,
"lcdprogressbar": 0,
"lcdprogressbarbartime": 3,
"lcdprogressbarmsgtime": 1,
"lcdprogressbarmsgexpire": 0,
"plahotendtemp": 190,
"plabedtemp": 60,
"plafanspeed": 255,
"abshotendtemp": 240,
"absbedtemp": 100,
"absfanspeed": 255,
"gumhotendtemp": 230,
"gumbedtemp": 60,
"gumfanspeed": 255,
"filamentsensor": "1",
"filamentsensorpin": 10,
"filamentsensorlcd": "0",
"filamentsensorextruder": 0,
"filamentsensordia": 1.75,
"filamentsensormaxdia": 2,
"filamentsensormindia": 1.35
}
========== End configuration string ==========
*/
