/* -*- c++ -*- */

/*
 Reprap firmware based on Marlin
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 Copyright (C) 2014 MagoKimbra

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Marlin.h"

#ifdef ENABLE_AUTO_BED_LEVELING
  #include "vector_3.h"
  #ifdef AUTO_BED_LEVELING_GRID
    #include "qr_solve.h"
  #endif
#endif // ENABLE_AUTO_BED_LEVELING


#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"

#ifdef BLINKM
  #include "BlinkM.h"
  #include "Wire.h"
#endif

#if NUM_SERVOS > 0
  #include "Servo.h"
#endif

#if HAS_DIGIPOTSS
  #include <SPI.h>
#endif

#ifdef FIRMWARE_TEST
  #include "firmware_test.h"
#endif

/**
 * Look here for descriptions of G-codes:
 *  - http://linuxcnc.org/handbook/gcode/g-code.html
 *  - http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes
 *
 * Help us document these G-codes online:
 *  - http://reprap.org/wiki/G-code
 *  - https://github.com/MagoKimbra/MarlinKimbra/blob/master/Documentation/GCodes.md
 */

/**
 * Implemented Codes
 * --------------------------------------------------------------------------------------
 *
 * "G" Codes
 *
 * G0  -> G1
 * G1  - Coordinated Movement X Y Z E
 * G2  - CW ARC
 * G3  - CCW ARC
 * G4  - Dwell S<seconds> or P<milliseconds>
 * G10 - retract filament according to settings of M207
 * G11 - retract recover filament according to settings of M208
 * G28 - Home one or more axes
 * G29 - Detailed Z-Probe, probes the bed at 3 or more points.  Will fail if you haven't homed yet.
 * G30 - Single Z Probe, probes bed at current XY location. - Bed Probe and Delta geometry Autocalibration
 * G31 - Dock sled (Z_PROBE_SLED only)
 * G32 - Undock sled (Z_PROBE_SLED only)
 * G60 - Store in memory actual position
 * G61 - Move X Y Z to position in memory
 * G90 - Use Absolute Coordinates
 * G91 - Use Relative Coordinates
 * G92 - Set current position to coordinates given
 * -------------------------------------------------------------------------------------
 * "M" Codes
 *
 *
 * M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
 * M1   - Same as M0
 * M3   - Put S<value> in laser beam control
 * M4   - Turn on laser beam
 * M5   - Turn off laser beam
 * M11  - Start printer for pause mode
 * M17  - Enable/Power all stepper motors
 * M18  - Disable all stepper motors; same as M84
 * M20  - List SD card
 * M21  - Init SD card
 * M22  - Release SD card
 * M23  - Select SD file (M23 filename.g)
 * M24  - Start/resume SD print
 * M25  - Pause SD print
 * M26  - Set SD position in bytes (M26 S12345)
 * M27  - Report SD print status
 * M28  - Start SD write (M28 filename.g)
 * M29  - Stop SD write
 * M30  - Delete file from SD (M30 filename.g)
 * M31  - Output time since last M109 or SD card start to serial
 * M32  - Select file and start SD print (Can be used _while_ printing from SD card files):
 *        syntax "M32 /path/filename#", or "M32 S<startpos bytes> !filename#"
 *        Call gcode file : "M32 P !filename#" and return to caller file after finishing (similar to #include).
 *        The '#' is necessary when calling from within sd files, as it stops buffer prereading
 * M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
 * M49  - Measure Z_Probe repeatability. M49 [P # of points] [X position] [Y position] [V_erboseness #] [E_ngage Probe] [L # of legs of travel]
 * M80  - Turn on Power Supply
 * M81  - Turn off Power Supply
 * M82  - Set E codes absolute (default)
 * M83  - Set E codes relative while in Absolute Coordinates (G90) mode
 * M84  - Disable steppers until next move,
 *        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
 * M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 * M92  - Set axis_steps_per_unit - same syntax as G92
 * M104 - Set extruder target temp
 * M105 - Read current temp
 * M106 - Fan on
 * M107 - Fan off
 * M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
 *        Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
 *        IF AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
 * M111 - Debug mode
 * M112 - Emergency stop
 * M114 - Output current position to serial port
 * M115 - Capabilities string
 * M117 - display message
 * M119 - Output Endstop status to serial port
 * M120 - Enable endstop detection
 * M121 - Disable endstop detection
 * M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
 * M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
 * M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
 * M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
 * M140 - Set bed target temp
 * M150 - Set BlinkM Color Output R: Red<0-255> U(!): Green<0-255> B: Blue<0-255> over i2c, G for green does not work.
 * M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
 *        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
 * M200 - set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).:D<millimeters>- 
 * M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 * M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
 * M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
 * M204 - Set default acceleration: P for Printing moves, R for Retract only (no X, Y, Z) moves and T for Travel (non printing) moves (ex. M204 P800 T3000 R9000) in mm/sec^2
 * M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
 * M206 - Set additional homing offset
 * M207 - Set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop], stays in mm regardless of M200 setting
 * M208 - Set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
 * M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
 * M218 - Set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
 * M220 - Set speed factor override percentage: S<factor in percent>
 * M221 - Set extrude factor override percentage: S<factor in percent>
 * M226 - Wait until the specified pin reaches the state required: P<pin number> S<pin state>
 * M240 - Trigger a camera to take a photograph
 * M250 - Set LCD contrast C<contrast value> (value 0..63)
 * M280 - Set servo position absolute. P: servo index, S: angle or microseconds
 * M300 - Play beep sound S<frequency Hz> P<duration ms>
 * M301 - Set PID parameters P I and D
 * M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
 * M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
 * M304 - Set bed PID parameters P I and D
 * M350 - Set microstepping mode.
 * M351 - Toggle MS1 MS2 pins directly.
 * M380 - Activate solenoid on active extruder
 * M381 - Disable all solenoids
 * M400 - Finish all moves
 * M401 - Lower z-probe if present
 * M402 - Raise z-probe if present
 * M404 - N<dia in mm> Enter the nominal filament width (3mm, 1.75mm ) or will display nominal filament width without parameters
 * M405 - Turn on Filament Sensor extrusion control.  Optional D<delay in cm> to set delay in centimeters between sensor and extruder
 * M406 - Turn off Filament Sensor extrusion control
 * M407 - Display measured filament diameter
 * M500 - Store parameters in EEPROM
 * M501 - Read parameters from EEPROM (if you need reset them after you changed them temporarily).
 * M502 - Revert to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.
 * M503 - Print the current settings (from memory not from EEPROM). Use S0 to leave off headings.
 * M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
 * M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
 * M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ]
 * M666 - Set z probe offset or Endstop and delta geometry adjustment
 * M907 - Set digital trimpot motor current using axis codes.
 * M908 - Control digital trimpot directly.
 *
 * ************ SCARA Specific - This can change to suit future G-code regulations
 * M360 - SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
 * M361 - SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
 * M362 - SCARA calibration: Move to cal-position PsiA (0 deg calibration)
 * M363 - SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
 * M364 - SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
 * M365 - SCARA calibration: Scaling factor, X, Y, Z axis
 * ************* SCARA End ***************
 *
 * M928 - Start SD logging (M928 filename.g) - ended by M29
 * M997 - NPR2 Color rotate
 * M999 - Restart after being stopped by error
 */

#ifdef SDSUPPORT
  CardReader card;
#endif

bool Running = true;

static float feedrate = 1500.0, next_feedrate, saved_feedrate;
float current_position[NUM_AXIS] = { 0.0 };
float destination[NUM_AXIS] = { 0.0 };
float lastpos[NUM_AXIS] = { 0.0 };
bool axis_known_position[3] = { false };

static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static int cmd_queue_index_r = 0;
static int cmd_queue_index_w = 0;
static int commands_in_queue = 0;
static char command_queue[BUFSIZE][MAX_CMD_SIZE];

float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedrate_multiplier = 100; //100->1 200->2
int saved_feedrate_multiplier;
int extruder_multiply[EXTRUDERS] = ARRAY_BY_EXTRUDERS(100, 100, 100, 100);
bool volumetric_enabled = false;
float filament_size[EXTRUDERS] = ARRAY_BY_EXTRUDERS(DEFAULT_NOMINAL_FILAMENT_DIA, DEFAULT_NOMINAL_FILAMENT_DIA, DEFAULT_NOMINAL_FILAMENT_DIA, DEFAULT_NOMINAL_FILAMENT_DIA);
float volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS(1.0, 1.0, 1.0, 1.0);
float home_offset[3] = { 0 };
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };

uint8_t active_extruder = 0;
uint8_t active_driver = 0;
uint8_t debugLevel = 0;
int fanSpeed = 0;
bool cancel_heatup = false;

const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};

static float offset[3] = { 0 };
static bool relative_mode = false;  //Determines Absolute or Relative Coordinates
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; ///< A pointer to find chars in the command string (X, Y, Z, E, etc.)
const char* queued_commands_P= NULL; /* pointer to the current line in the active sequence of commands, or NULL when none */
const int sensitive_pins[] = SENSITIVE_PINS; ///< Sensitive pin list for M42
// Inactivity shutdown
millis_t previous_cmd_ms = 0;
static millis_t max_inactive_time = 0;
static millis_t stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME * 1000L;
millis_t print_job_start_ms = 0; ///< Print job start time
millis_t print_job_stop_ms = 0;  ///< Print job stop time
static uint8_t target_extruder;
bool no_wait_for_cooling = true;
bool target_direction;

unsigned long printer_usage_seconds;

#ifndef DELTA
  int xy_travel_speed = XY_TRAVEL_SPEED;
  float zprobe_zoffset = 0;
#endif

#if defined(Z_DUAL_ENDSTOPS) && !defined(DELTA)
  float z_endstop_adj = 0;
#endif

// Hotend offset
#if HOTENDS > 1
  #ifndef DUAL_X_CARRIAGE
    #define NUM_HOTEND_OFFSETS 2 // only in XY plane
  #else
    #define NUM_HOTEND_OFFSETS 3 // supports offsets in XYZ plane
  #endif
  float hotend_offset[NUM_HOTEND_OFFSETS][HOTENDS];
#endif

#ifdef NPR2
  int old_color = 99;
#endif

#if NUM_SERVOS > 0
  int servo_endstops[] = SERVO_ENDSTOPS;
  int servo_endstop_angles[] = SERVO_ENDSTOP_ANGLES;
#endif

#ifdef BARICUDA
  int ValvePressure = 0;
  int EtoPPressure = 0;
#endif

#ifdef FWRETRACT

  bool autoretract_enabled = false;
  bool retracted[EXTRUDERS] = { false };
  bool retracted_swap[EXTRUDERS] = { false };

  float retract_length = RETRACT_LENGTH;
  float retract_length_swap = RETRACT_LENGTH_SWAP;
  float retract_feedrate = RETRACT_FEEDRATE;
  float retract_zlift = RETRACT_ZLIFT;
  float retract_recover_length = RETRACT_RECOVER_LENGTH;
  float retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
  float retract_recover_feedrate = RETRACT_RECOVER_FEEDRATE;

#endif // FWRETRACT

#if defined(ULTIPANEL) && HAS_POWER_SWITCH
  bool powersupply = 
    #ifdef PS_DEFAULT_OFF
      false
    #else
      true
    #endif
  ;
#endif

#ifdef DELTA
  float endstop_adj[3] = { 0, 0, 0 };
  float tower_adj[6] = { 0, 0, 0, 0, 0, 0 };
  float delta_radius; // = DEFAULT_delta_radius;
  float delta_diagonal_rod; // = DEFAULT_DELTA_DIAGONAL_ROD;
  float DELTA_DIAGONAL_ROD_2;
  float ac_prec = AUTOCALIBRATION_PRECISION / 2;
  float bed_radius = PRINTER_RADIUS;
  float delta_tower1_x, delta_tower1_y;
  float delta_tower2_x, delta_tower2_y;
  float delta_tower3_x, delta_tower3_y;
  float base_max_pos[3] = {X_MAX_POS, Y_MAX_POS, Z_MAX_POS};
  float base_home_pos[3] = {X_HOME_POS, Y_HOME_POS, Z_HOME_POS};
  float max_length[3] = {X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH};
  float saved_position[3] = { 0.0, 0.0, 0.0 };
  float saved_positions[7][3] = {
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      };
  float delta[3] = { 0.0, 0.0, 0.0 };
  float delta_tmp[3] = { 0.0, 0.0, 0.0 };
  float probing_feedrate = PROBING_FEEDRATE;
  float default_z_probe_offset[] = Z_PROBE_OFFSET;
  float z_probe_offset[3];
  float z_probe_deploy_start_location[] = Z_PROBE_DEPLOY_START_LOCATION;
  float z_probe_deploy_end_location[] = Z_PROBE_DEPLOY_END_LOCATION;
  float z_probe_retract_start_location[] = Z_PROBE_RETRACT_START_LOCATION;
  float z_probe_retract_end_location[] = Z_PROBE_RETRACT_END_LOCATION;
  #define SIN_60 0.8660254037844386
  #define COS_60 0.5
  static float bed_level[7][7] = {
      { 0, 0, 0, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0 },
      };
  static float z_offset;
  static float bed_level_x, bed_level_y, bed_level_z;
  static float bed_level_c = 20; //used for inital bed probe safe distance (to avoid crashing into bed)
  static float bed_level_ox, bed_level_oy, bed_level_oz;
  static int loopcount;
  static bool home_all_axis = true;
#else
  static bool home_all_axis = true;
#endif // DELTA

#ifdef SCARA
  static float delta[3] = { 0 };
  float axis_scaling[3] = { 1, 1, 1 };    // Build size scaling, default to 1
#endif

#if HAS_FILAMENT_SENSOR
  //Variables for Filament Sensor input 
  float filament_width_nominal = DEFAULT_NOMINAL_FILAMENT_DIA;  //Set nominal filament width, can be changed with M404 
  bool filament_sensor = false;                                 //M405 turns on filament_sensor control, M406 turns it off 
  float filament_width_meas = DEFAULT_MEASURED_FILAMENT_DIA;    //Stores the measured filament diameter 
  signed char measurement_delay[MAX_MEASUREMENT_DELAY+1];       //ring buffer to delay measurement  store extruder factor after subtracting 100 
  int delay_index1 = 0;                                         //index into ring buffer
  int delay_index2 = -1;                                        //index into ring buffer - set to -1 on startup to indicate ring buffer needs to be initialized
  float delay_dist = 0;                                         //delay distance counter
  int meas_delay_cm = MEASUREMENT_DELAY_CM;                     //distance delay setting
#endif

#ifdef FILAMENT_RUNOUT_SENSOR
  static bool filrunoutEnqueued = false;
  bool printing = false;
#endif

#ifdef SDSUPPORT
  static bool fromsd[BUFSIZE];
  #ifdef SD_SETTINGS
    unsigned long config_last_update = 0;
	bool config_readed = false;
  #endif
#endif

#ifdef FILAMENTCHANGEENABLE
	bool filament_changing = false;
#endif

#if defined(IDLE_OOZING_PREVENT) || defined(EXTRUDER_RUNOUT_PREVENT)
  unsigned long axis_last_activity = 0;
  bool axis_is_moving = false;
#endif

#ifdef IDLE_OOZING_PREVENT
  bool idleoozing_enabled = true;
  bool IDLE_OOZING_retracted[EXTRUDERS] = ARRAY_BY_EXTRUDERS(false, false, false, false);
#endif

#if HAS_POWER_CONSUMPTION_SENSOR
  float power_consumption_meas = 0.0;
  unsigned long power_consumption_hour;
  unsigned long startpower = 0;
  unsigned long stoppower = 0;
#endif

#ifdef LASERBEAM
  int laser_ttl_modulation = 0;
#endif

#ifdef NPR2
  static float color_position[] = COLOR_STEP;  //variabile per la scelta del colore
  static float color_step_moltiplicator = (DRIVER_MICROSTEP / MOTOR_ANGLE) * CARTER_MOLTIPLICATOR;
#endif // NPR2

#ifdef EASY_LOAD
  bool allow_lengthy_extrude_once; // for load/unload
#endif

#if NUM_SERVOS > 0
  Servo servo[NUM_SERVOS];
#endif

#ifdef CHDK
  unsigned long chdkHigh = 0;
  boolean chdkActive = false;
#endif

//===========================================================================
//================================ Functions ================================
//===========================================================================
void get_arc_coordinates();
bool setTargetedHotend(int code);

#ifdef PREVENT_DANGEROUS_EXTRUDE
  float extrude_min_temp = EXTRUDE_MINTEMP;
#endif

#ifndef __SAM3X8E__
#ifdef SDSUPPORT
  #include "SdFatUtil.h"
  int freeMemory() { return SdFatUtil::FreeRam(); }
#else
  extern "C" {
    extern unsigned int __bss_end;
    extern unsigned int __heap_start;
    extern void *__brkval;

    int freeMemory() {
      int free_memory;

      if ((int)__brkval == 0)
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
      else
        free_memory = ((int)&free_memory) - ((int)__brkval);

      return free_memory;
    }
  }
#endif 
#endif

/**
 * Inject the next command from the command queue, when possible
 * Return false only if no command was pending
 */
static bool drain_queued_commands_P() {
  if (!queued_commands_P) return false;

  // Get the next 30 chars from the sequence of gcodes to run
  char cmd[30];
  strncpy_P(cmd, queued_commands_P, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = '\0';

  // Look for the end of line, or the end of sequence
  size_t i = 0;
  char c;
  while((c = cmd[i]) && c != '\n') i++; // find the end of this gcode command
  cmd[i] = '\0';
  if (enqueuecommand(cmd)) {      // buffer was not full (else we will retry later)
    if (c)
      queued_commands_P += i + 1; // move to next command
    else
      queued_commands_P = NULL;   // will have no more commands in the sequence
  }
  return true;
}

/**
 * Record one or many commands to run from program memory.
 * Aborts the current queue, if any.
 * Note: drain_queued_commands_P() must be called repeatedly to drain the commands afterwards
 */
void enqueuecommands_P(const char* pgcode) {
  queued_commands_P = pgcode;
  drain_queued_commands_P(); // first command executed asap (when possible)
}

/**
 * Copy a command directly into the main command buffer, from RAM.
 *
 * This is done in a non-safe way and needs a rework someday.
 * Returns false if it doesn't add any command
 */
bool enqueuecommand(const char *cmd) {

  if (*cmd == ';' || commands_in_queue >= BUFSIZE) return false;

  // This is dangerous if a mixing of serial and this happens
  char *command = command_queue[cmd_queue_index_w];
  strcpy(command, cmd);
  ECHO_LMV(OK, MSG_ENQUEUEING,command);
  cmd_queue_index_w = (cmd_queue_index_w + 1) % BUFSIZE;
  commands_in_queue++;
  return true;
}


#if MB(ALLIGATOR)
  void setup_alligator_board() {
    // Init Expansion Port Voltage logic Selector
    SET_OUTPUT(EXP_VOLTAGE_LEVEL_PIN);
    WRITE(EXP_VOLTAGE_LEVEL_PIN,UI_VOLTAGE_LEVEL);
    ExternalDac::begin(); //initialize ExternalDac
  }
#endif

void setup_killpin() {
  #if HAS_KILL
    SET_INPUT(KILL_PIN);
    WRITE(KILL_PIN, HIGH);
  #endif
}

void setup_filrunoutpin() {
  #if HAS_FILRUNOUT
    pinMode(FILRUNOUT_PIN, INPUT);
    #ifdef ENDSTOPPULLUP_FIL_RUNOUT
      WRITE(FILLRUNOUT_PIN, HIGH);
    #endif
  #endif
}

// Set home pin
void setup_homepin(void) {
  #if HAS_HOME
    SET_INPUT(HOME_PIN);
    WRITE(HOME_PIN, HIGH);
  #endif
}


void setup_photpin() {
  #if HAS_PHOTOGRAPH
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
}

void setup_laserbeampin() {
  #ifdef LASERBEAM
    OUT_WRITE(LASER_PWR_PIN, LOW);
    OUT_WRITE(LASER_TTL_PIN, LOW);
  #endif
}

void setup_powerhold() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if HAS_POWER_SWITCH
    #ifdef PS_DEFAULT_OFF
      OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #else
      OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
    #endif
  #endif
}

void suicide() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init() {
  #if NUM_SERVOS >= 1 && HAS_SERVO_0
    servo[0].attach(SERVO0_PIN);
  #endif
  #if NUM_SERVOS >= 2 && HAS_SERVO_1
    servo[1].attach(SERVO1_PIN);
  #endif
  #if NUM_SERVOS >= 3 && HAS_SERVO_2
    servo[2].attach(SERVO2_PIN);
  #endif
  #if NUM_SERVOS >= 4 && HAS_SERVO_3
    servo[3].attach(SERVO3_PIN);
  #endif

  // Set position of Servo Endstops that are defined
  #if (NUM_SERVOS > 0)
    for (int i = 0; i < 3; i++)
      if (servo_endstops[i] >= 0)
        servo[servo_endstops[i]].write(servo_endstop_angles[i * 2 + 1]);
  #endif //NUM_SERVOS

  #if SERVO_LEVELING_DELAY
    delay(PROBE_SERVO_DEACTIVATION_DELAY);
    servo[servo_endstops[Z_AXIS]].detach();
  #endif
}

/**
 * Marlin entry-point: Set up before the program loop
 *  - Set up Alligator Board
 *  - Set up the kill pin, filament runout, power hold
 *  - Start the serial port
 *  - Print startup messages and diagnostics
 *  - Get EEPROM or default settings
 *  - Initialize managers for:
 *    • temperature
 *    • planner
 *    • watchdog
 *    • stepper
 *    • photo pin
 *    • laserbeam
 *    • servos
 *    • LCD controller
 *    • Digipot I2C
 *    • Z probe sled
 *    • status LEDs
 */
void setup() {
  #if MB(ALLIGATOR)
    setup_alligator_board();// Initialize Alligator Board
  #endif
  setup_killpin();
  setup_filrunoutpin();
  setup_powerhold();
  SERIAL_INIT(BAUDRATE);
  
  ECHO_S(START);
  ECHO_E;

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if (mcu & 1) ECHO_LM(OK,MSG_POWERUP);
  if (mcu & 2) ECHO_LM(OK,MSG_EXTERNAL_RESET);
  if (mcu & 4) ECHO_LM(OK,MSG_BROWNOUT_RESET);
  if (mcu & 8) ECHO_LM(OK,MSG_WATCHDOG_RESET);
  if (mcu & 32) ECHO_LM(OK,MSG_SOFTWARE_RESET);
  MCUSR = 0;

  ECHO_LM(OK, MSG_MARLIN " " STRING_VERSION);

  #ifdef STRING_VERSION_CONFIG_H
    #ifdef STRING_CONFIG_H_AUTHOR
      ECHO_LM(OK, MSG_CONFIGURATION_VER STRING_VERSION_CONFIG_H MSG_AUTHOR STRING_CONFIG_H_AUTHOR MSG_COMPILED __DATE__);
    #endif // STRING_CONFIG_H_AUTHOR
  #endif // STRING_VERSION_CONFIG_H

  ECHO_SMV(OK, MSG_FREE_MEMORY, freeMemory());
  ECHO_M(MSG_PLANNER_BUFFER_BYTES);
  ECHO_EV((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);

  #ifdef SDSUPPORT
    for (int8_t i = 0; i < BUFSIZE; i++) fromsd[i] = false;
  #endif

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();
  
  // loads custom configuration from SDCARD if available else uses defaults
  #if defined(SDSUPPORT) && defined(SD_SETTINGS)
    ConfigSD_ResetDefault(); //this reset variable to default value as we can't load in the setup for different reason. Will load the value in the loop()
  #else
    ConfigSD_RetrieveSettings();
  #endif

  tp_init();    // Initialize temperature loop
  plan_init();  // Initialize planner;
  watchdog_init();
  st_init();    // Initialize stepper, this enables interrupts!
  setup_photpin();
  setup_laserbeampin();   // Initialize Laserbeam pin
  servo_init();
  
  lcd_init();
  _delay_ms(1000);  // wait 1sec to display the splash screen

  #if HAS_CONTROLLERFAN
    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
  #endif

  #ifdef DIGIPOT_I2C
    digipot_i2c_init();
  #endif

  #ifdef Z_PROBE_SLED
    OUT_WRITE(SERVO0_PIN, LOW); // turn it off
  #endif // Z_PROBE_SLED

  setup_homepin();

  #ifdef STAT_LED_RED
    pinMode(STAT_LED_RED, OUTPUT);
    digitalWrite(STAT_LED_RED, LOW); // turn it off
  #endif
  #ifdef STAT_LED_BLUE
    pinMode(STAT_LED_BLUE, OUTPUT);
    digitalWrite(STAT_LED_BLUE, LOW); // turn it off
  #endif

  #ifdef FIRMWARE_TEST
    FirmwareTest();
  #endif // FIRMWARE_TEST
}

/**
 * The main Marlin program loop
 *
 *  - Save or log commands to SD
 *  - Process available commands (if not saving)
 *  - Call heater manager
 *  - Call inactivity manager
 *  - Call endstop manager
 *  - Call LCD update
 */
void loop() {
  if (commands_in_queue < BUFSIZE - 1) get_command();

  #ifdef SDSUPPORT
    card.checkautostart(false);
  #endif

  if (commands_in_queue) {

    #ifdef SDSUPPORT

      if (card.saving) {
        char *command = command_queue[cmd_queue_index_r];
        if (strstr_P(command, PSTR("M29"))) {
          // M29 closes the file
          card.closeFile();
          ECHO_EM(MSG_FILE_SAVED);
        }
        else {
          // Write the string from the read buffer to SD
          card.write_command(command);
          if (card.logging)
            process_commands(); // The card is saving because it's logging
          else
            ECHO_EM(MSG_OK);
        }
      }
      else
        process_commands();

    #else

      process_commands();

    #endif // SDSUPPORT

    commands_in_queue--;
    cmd_queue_index_r = (cmd_queue_index_r + 1) % BUFSIZE;
  }
  // Check heater every n milliseconds
  manage_heater();
  manage_inactivity();
  checkHitEndstops();
  lcd_update();
}

/**
 * Add to the circular command queue the next command from:
 *  - The command-injection queue (queued_commands_P)
 *  - The active serial input (usually USB)
 *  - The SD card file being actively printed
 */
void get_command() {

  if (drain_queued_commands_P()) return; // priority is given to non-serial commands
  
  while (MYSERIAL.available() > 0 && commands_in_queue < BUFSIZE) {

    serial_char = MYSERIAL.read();

    if (serial_char == '\n' || serial_char == '\r' ||
       serial_count >= (MAX_CMD_SIZE - 1)
    ) {
      // end of line == end of comment
      comment_mode = false;

      if (!serial_count) return; // shortcut for empty lines

      char *command = command_queue[cmd_queue_index_w];
      command[serial_count] = 0; // terminate string

      #ifdef SDSUPPORT
        fromsd[cmd_queue_index_w] = false;
      #endif

      if (strchr(command, 'N') != NULL) {
        strchr_pointer = strchr(command, 'N');
        gcode_N = (strtol(strchr_pointer + 1, NULL, 10));
        if (gcode_N != gcode_LastN + 1 && strstr_P(command, PSTR("M110")) == NULL) {
          ECHO_LMV(ER, MSG_ERR_LINE_NO, gcode_LastN);
          FlushSerialRequestResend();
          serial_count = 0;
          return;
        }

        if (strchr(command, '*') != NULL) {
          byte checksum = 0;
          byte count = 0;
          while (command[count] != '*') checksum ^= command[count++];
          strchr_pointer = strchr(command, '*');

          if (strtol(strchr_pointer + 1, NULL, 10) != checksum) {
            ECHO_LMV(ER, MSG_ERR_CHECKSUM_MISMATCH, gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }
          //if no errors, continue parsing
        }
        else {
          ECHO_LMV(ER, MSG_ERR_NO_CHECKSUM, gcode_LastN);
          FlushSerialRequestResend();
          serial_count = 0;
          return;
        }

        gcode_LastN = gcode_N;
        //if no errors, continue parsing
      }
      else {  // if we don't receive 'N' but still see '*'
        if ((strchr(command, '*') != NULL)) {
          ECHO_LMV(ER, MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM, gcode_LastN);
          serial_count = 0;
          return;
        }
      }

      if (strchr(command, 'G') != NULL) {
        strchr_pointer = strchr(command, 'G');
        switch (strtol(strchr_pointer + 1, NULL, 10)) {
          case 0:
          case 1:
          case 2:
          case 3:
            if (IsStopped()) {
              ECHO_LM(ER,MSG_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
            }
            break;
          default:
            break;
        }
      }

      // If command was e-stop process now
      if (strcmp(command, "M112") == 0) kill();

      cmd_queue_index_w = (cmd_queue_index_w + 1) % BUFSIZE;
      commands_in_queue += 1;

      serial_count = 0; //clear buffer
    }
    else if (serial_char == '\\') {  // Handle escapes
      if (MYSERIAL.available() > 0  && commands_in_queue < BUFSIZE) {
        // if we have one more character, copy it over
        serial_char = MYSERIAL.read();
        command_queue[cmd_queue_index_w][serial_count++] = serial_char;
      }
      // otherwise do nothing
    }
    else { // its not a newline, carriage return or escape char
      if (serial_char == ';') comment_mode = true;
      if (!comment_mode) command_queue[cmd_queue_index_w][serial_count++] = serial_char;
    }
  }

  #ifdef SDSUPPORT

    if (!card.sdprinting || serial_count) return;

    // '#' stops reading from SD to the buffer prematurely, so procedural macro calls are possible
    // if it occurs, stop_buffering is triggered and the buffer is ran dry.
    // this character _can_ occur in serial com, due to checksums. however, no checksums are used in SD printing

    static bool stop_buffering = false;
    if (commands_in_queue == 0) stop_buffering = false;

    while (!card.eof() && commands_in_queue < BUFSIZE && !stop_buffering) {
      int16_t n = card.get();
      serial_char = (char)n;
      if (serial_char == '\n' || serial_char == '\r' ||
          ((serial_char == '#' || serial_char == ':') && !comment_mode) ||
          serial_count >= (MAX_CMD_SIZE - 1) || n == -1
      ) {
        if (card.eof()) {
          ECHO_LM(OK, MSG_FILE_PRINTED);
          print_job_stop_ms = millis();
          char time[30];
          millis_t t = (print_job_stop_ms - print_job_start_ms) / 1000;
          int hours = t / 60 / 60, minutes = (t / 60) % 60;
          sprintf_P(time, PSTR("%i " MSG_END_HOUR " %i " MSG_END_MINUTE), hours, minutes);
          ECHO_LV(OK,time);
          lcd_setstatus(time, true);
          card.printingHasFinished();
          card.checkautostart(true);
        }
        if (serial_char == '#') stop_buffering = true;

        if (!serial_count) {
          comment_mode = false; //for new command
          return; //if empty line
        }
        command_queue[cmd_queue_index_w][serial_count] = 0; //terminate string
        // if (!comment_mode) {
        fromsd[cmd_queue_index_w] = true;
        commands_in_queue += 1;
        cmd_queue_index_w = (cmd_queue_index_w + 1) % BUFSIZE;
        // }
        comment_mode = false; //for new command
        serial_count = 0; //clear buffer
      }
      else {
        if (serial_char == ';') comment_mode = true;
        if (!comment_mode) command_queue[cmd_queue_index_w][serial_count++] = serial_char;
      }
    }

  #endif // SDSUPPORT
}

bool code_has_value() {
  char c = strchr_pointer[1];
  return (c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.';
}

float code_value() {
  float ret;
  char *e = strchr(strchr_pointer, 'E');
  if (e) {
    *e = 0;
    ret = strtod(strchr_pointer+1, NULL);
    *e = 'E';
  }
  else
    ret = strtod(strchr_pointer+1, NULL);
  return ret;
}

long code_value_long() { return strtol(strchr_pointer + 1, NULL, 10); }

int16_t code_value_short() { return (int16_t)strtol(strchr_pointer + 1, NULL, 10); }

bool code_seen(char code) {
  strchr_pointer = strchr(command_queue[cmd_queue_index_r], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
    static inline type pgm_read_any(const type *p)  \
    { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)          \
    { return pgm_read_any(&array##_P[axis]); }

#if defined(CARTESIAN) || defined(COREXY) || defined(SCARA)
  XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,  MAX_POS);
  XYZ_CONSTS_FROM_CONFIG(float, base_home_pos, HOME_POS);
  XYZ_CONSTS_FROM_CONFIG(float, max_length,    MAX_LENGTH);
#endif
XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm,    HOME_BUMP_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

#ifdef DUAL_X_CARRIAGE

  #define DXC_FULL_CONTROL_MODE 0
  #define DXC_AUTO_PARK_MODE    1
  #define DXC_DUPLICATION_MODE  2

  static int dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;

  static float x_home_pos(int extruder) {
    if (extruder == 0)
      return base_home_pos(X_AXIS) + home_offset[X_AXIS];
    else
      // In dual carriage mode the extruder offset provides an override of the
      // second X-carriage offset when homed - otherwise X2_HOME_POS is used.
      // This allow soft recalibration of the second extruder offset position without firmware reflash
      // (through the M218 command).
      return (hotend_offset[X_AXIS][1] > 0) ? hotend_offset[X_AXIS][1] : X2_HOME_POS;
  }

  static int x_home_dir(int extruder) {
    return (extruder == 0) ? X_HOME_DIR : X2_HOME_DIR;
  }

  static float inactive_extruder_x_pos = X2_MAX_POS; // used in mode 0 & 1
  static bool active_extruder_parked = false; // used in mode 1 & 2
  static float raised_parked_position[NUM_AXIS]; // used in mode 1
  static millis_t delayed_move_time = 0; // used in mode 1
  static float duplicate_extruder_x_offset = DEFAULT_DUPLICATION_X_OFFSET; // used in mode 2
  static float duplicate_extruder_temp_offset = 0; // used in mode 2
  bool extruder_duplication_enabled = false; // used in mode 2

#endif //DUAL_X_CARRIAGE

/**
 * Some planner shorthand inline functions
 */
inline void set_homing_bump_feedrate(AxisEnum axis) {
  #ifdef DELTA
    const int homing_bump_divisor[] = HOMING_BUMP_DIVISOR;
    if (homing_bump_divisor[X_AXIS] >= 1)
      feedrate = homing_feedrate[axis] / homing_bump_divisor[X_AXIS];
    else {
      feedrate = homing_feedrate[axis] / 10;
      ECHO_LM(ER, MSG_ERR_HOMING_DIV);
    }
  #else // No DELTA
    const int homing_bump_divisor[] = HOMING_BUMP_DIVISOR;
    if (homing_bump_divisor[axis] >= 1)
      feedrate = homing_feedrate[axis] / homing_bump_divisor[axis];
    else {
      feedrate = homing_feedrate[axis] / 10;
      ECHO_LM(ER, MSG_ERR_HOMING_DIV);
    }
  #endif // No DELTA
}
inline void line_to_current_position() {
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder, active_driver);
}
inline void line_to_z(float zPosition) {
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder, active_driver);
}
inline void line_to_destination(float mm_m) {
  plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], mm_m/60, active_extruder, active_driver);
}
inline void line_to_destination() {
  line_to_destination(feedrate);
}
inline void sync_plan_position() {
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
#if defined(DELTA) || defined(SCARA)
  inline void sync_plan_position_delta() {
    calculate_delta(current_position);
    plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
  }
#endif
inline void set_current_to_destination() { memcpy(current_position, destination, sizeof(current_position)); }
inline void set_destination_to_current() { memcpy(destination, current_position, sizeof(destination)); }

#if defined(CARTESIAN) || defined(COREXY) || defined(SCARA)
  static void axis_is_at_home(int axis) {

    #ifdef DUAL_X_CARRIAGE
      if (axis == X_AXIS) {
        if (active_extruder != 0) {
          current_position[X_AXIS] = x_home_pos(active_extruder);
                   min_pos[X_AXIS] = X2_MIN_POS;
                   max_pos[X_AXIS] = max(hotend_offset[X_AXIS][1], X2_MAX_POS);
          return;
        }
        else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE) {
          float xoff = home_offset[X_AXIS];
          current_position[X_AXIS] = base_home_pos(X_AXIS) + xoff;
                   min_pos[X_AXIS] = base_min_pos(X_AXIS) + xoff;
                   max_pos[X_AXIS] = min(base_max_pos(X_AXIS) + xoff, max(hotend_offset[X_AXIS][1], X2_MAX_POS) - duplicate_extruder_x_offset);
          return;
        }
      }
    #endif

    #ifdef SCARA
      float homeposition[3];

      if (axis < 2) {
        for (int i = 0; i < 3; i++) homeposition[i] = base_home_pos(i);

        // ECHO_SMV(DB, "homeposition[x]= ", homeposition[0]);
        // ECHO_EMV("homeposition[y]= ", homeposition[1]);
        // Works out real Homeposition angles using inverse kinematics, 
        // and calculates homing offset using forward kinematics
        calculate_delta(homeposition);

        // ECHO_SMV(DB,"base Theta= ", delta[X_AXIS]);
        // ECHO_EMV(" base Psi+Theta=", delta[Y_AXIS]);

        for (int i = 0; i < 2; i++) delta[i] -= home_offset[i];

        // ECHO_SMV(DB,"addhome X=", home_offset[X_AXIS]);
        // ECHO_MV(" addhome Y=", home_offset[Y_AXIS]);
        // ECHO_MV(" addhome Theta=", delta[X_AXIS]);
        // ECHO_EMV(" addhome Psi+Theta=", delta[Y_AXIS]);

        calculate_SCARA_forward_Transform(delta);

        // ECHO_SMV(DB,"Delta X=", delta[X_AXIS]);
        // ECHO_EMV(" Delta Y=", delta[Y_AXIS]);
        
        current_position[axis] = delta[axis];

        // SCARA home positions are based on configuration since the actual limits are determined by the 
        // inverse kinematic transform.
        min_pos[axis] = base_min_pos(axis); // + (delta[axis] - base_home_pos(axis));
        max_pos[axis] = base_max_pos(axis); // + (delta[axis] - base_home_pos(axis));
      } 
      else {
        current_position[axis] = base_home_pos(axis) + home_offset[axis];
        min_pos[axis] = base_min_pos(axis) + home_offset[axis];
        max_pos[axis] = base_max_pos(axis) + home_offset[axis];
      }
    #else
      current_position[axis] = base_home_pos(axis) + home_offset[axis];
      min_pos[axis] = base_min_pos(axis) + home_offset[axis];
      max_pos[axis] = base_max_pos(axis) + home_offset[axis];
    #endif

    #if defined(ENABLE_AUTO_BED_LEVELING) && Z_HOME_DIR < 0
      if (axis == Z_AXIS) current_position[Z_AXIS] += zprobe_zoffset;
    #endif
  }

  static void do_blocking_move_to(float x, float y, float z) {
    float oldFeedRate = feedrate;
    feedrate = homing_feedrate[Z_AXIS];

    current_position[Z_AXIS] = z;
    line_to_current_position();
    st_synchronize();

    feedrate = xy_travel_speed;

    current_position[X_AXIS] = x;
    current_position[Y_AXIS] = y;
    line_to_current_position();
    st_synchronize();

    feedrate = oldFeedRate;
  }
    
  #ifdef ENABLE_AUTO_BED_LEVELING
    #ifdef AUTO_BED_LEVELING_GRID
      static void set_bed_level_equation_lsq(double *plane_equation_coefficients) {
        vector_3 planeNormal = vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1);
        planeNormal.debug("planeNormal");
        plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);
        //bedLevel.debug("bedLevel");

        //plan_bed_level_matrix.debug("bed level before");
        //vector_3 uncorrected_position = plan_get_position_mm();
        //uncorrected_position.debug("position before");

        vector_3 corrected_position = plan_get_position();
        //corrected_position.debug("position after");
        current_position[X_AXIS] = corrected_position.x;
        current_position[Y_AXIS] = corrected_position.y;
        current_position[Z_AXIS] = corrected_position.z;

        // put the bed at 0 so we don't go below it.
        current_position[Z_AXIS] = zprobe_zoffset; // in the lsq we reach here after raising the extruder due to the loop structure

        sync_plan_position();
      }
    #else // not AUTO_BED_LEVELING_GRID
      static void set_bed_level_equation_3pts(float z_at_pt_1, float z_at_pt_2, float z_at_pt_3) {

        plan_bed_level_matrix.set_to_identity();

        vector_3 pt1 = vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, z_at_pt_1);
        vector_3 pt2 = vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, z_at_pt_2);
        vector_3 pt3 = vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, z_at_pt_3);
        vector_3 planeNormal = vector_3::cross(pt1 - pt2, pt3 - pt2).get_normal();

        if (planeNormal.z < 0) {
          planeNormal.x = -planeNormal.x;
          planeNormal.y = -planeNormal.y;
          planeNormal.z = -planeNormal.z;
        }

        plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);

        vector_3 corrected_position = plan_get_position();
        current_position[X_AXIS] = corrected_position.x;
        current_position[Y_AXIS] = corrected_position.y;
        current_position[Z_AXIS] = corrected_position.z;

        // put the bed at 0 so we don't go below it.
        current_position[Z_AXIS] += zprobe_zoffset;
        sync_plan_position();
      }

    #endif // AUTO_BED_LEVELING_GRID

    static void run_z_probe() {

      plan_bed_level_matrix.set_to_identity();
      feedrate = homing_feedrate[Z_AXIS];

      // move down until you find the bed
      float zPosition = -10;
      line_to_z(zPosition);
      st_synchronize();

      // we have to let the planner know where we are right now as it is not where we said to go.
      zPosition = st_get_position_mm(Z_AXIS);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);

      // move up the retract distance
      zPosition += home_bump_mm(Z_AXIS);
      line_to_z(zPosition);
      st_synchronize();
      endstops_hit_on_purpose(); // clear endstop hit flags

      // move back down slowly to find bed
      set_homing_bump_feedrate(Z_AXIS);

      zPosition -= home_bump_mm(Z_AXIS) * 2;
      line_to_z(zPosition);
      st_synchronize();
      endstops_hit_on_purpose(); // clear endstop hit flags

      current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
      // make sure the planner knows where we are as it may be a bit different than we last said to move to
      sync_plan_position();
    }

    static void setup_for_endstop_move() {
      saved_feedrate = feedrate;
      saved_feedrate_multiplier = feedrate_multiplier;
      feedrate_multiplier = 100;
      refresh_cmd_timeout();
      enable_endstops(true);
    }

    static void clean_up_after_endstop_move() {
      #ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(false);
      #endif
      feedrate = saved_feedrate;
      feedrate_multiplier = saved_feedrate_multiplier;
      refresh_cmd_timeout();
    }

    static void deploy_z_probe() {
      #if NUM_SERVOS > 0
        // Engage Z Servo endstop if enabled
        if (servo_endstops[Z_AXIS] >= 0) {
          #if SERVO_LEVELING
            servo[servo_endstops[Z_AXIS]].attach(0);
          #endif
          servo[servo_endstops[Z_AXIS]].write(servo_endstop_angles[Z_AXIS * 2]);
          #if SERVO_LEVELING_DELAY
            delay(PROBE_SERVO_DEACTIVATION_DELAY);
            servo[servo_endstops[Z_AXIS]].detach();
          #endif
        }
      #endif //NUM_SERVOS > 0
    }

    static void stow_z_probe() {
      #if NUM_SERVOS > 0
        // Retract Z Servo endstop if enabled
        if (servo_endstops[Z_AXIS] >= 0) {

          /* NON FUNZIONA DA VERIFICARE
          #if Z_RAISE_AFTER_PROBING > 0
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], Z_RAISE_AFTER_PROBING);
          #endif
          */

          #if SERVO_LEVELING
            servo[servo_endstops[Z_AXIS]].attach(0);
          #endif

          servo[servo_endstops[Z_AXIS]].write(servo_endstop_angles[Z_AXIS * 2 + 1]);

          #if SERVO_LEVELING
            delay(PROBE_SERVO_DEACTIVATION_DELAY);
            servo[servo_endstops[Z_AXIS]].detach();
          #endif
        }
      #endif //NUM_SERVOS > 0
    }

    enum ProbeAction {
      ProbeStay             = 0,
      ProbeDeploy           = BIT(0),
      ProbeStow             = BIT(1),
      ProbeDeployAndStow    = (ProbeDeploy | ProbeStow)
    };

    // Probe bed height at position (x,y), returns the measured z value
    static float probe_pt(float x, float y, float z_before, ProbeAction retract_action=ProbeDeployAndStow, int verbose_level=1) {
      // move to right place
      do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z_before);
      do_blocking_move_to(x - X_PROBE_OFFSET_FROM_EXTRUDER, y - Y_PROBE_OFFSET_FROM_EXTRUDER, current_position[Z_AXIS]);

      #ifndef Z_PROBE_SLED
        if (retract_action & ProbeDeploy) deploy_z_probe();
      #endif // Z_PROBE_SLED

      run_z_probe();
      float measured_z = current_position[Z_AXIS];

	  /*     NON FUNZIONA CONTROLLARE
      #if Z_RAISE_BETWEEN_PROBINGS > 0
        if (retract_action == ProbeStay) {
          do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);
          st_synchronize();
        }
      #endif
		*/


      #ifndef Z_PROBE_SLED
        if (retract_action & ProbeStow) stow_z_probe();
      #endif

      if (verbose_level > 2) {
        ECHO_SM(DB, MSG_BED_LEVELLING_BED);
        ECHO_MV(MSG_BED_LEVELLING_X, x, 3);
        ECHO_MV(MSG_BED_LEVELLING_Y, y, 3);
        ECHO_EMV(MSG_BED_LEVELLING_Z, measured_z, 3);
      }
      return measured_z;
    }
  #endif //ENABLE_AUTO_BED_LEVELING

  static void homeaxis(AxisEnum axis) {
  #define HOMEAXIS_DO(LETTER) \
    ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

    if (axis == X_AXIS ? HOMEAXIS_DO(X) : axis == Y_AXIS ? HOMEAXIS_DO(Y) : axis == Z_AXIS ? HOMEAXIS_DO(Z) : 0) {

      int axis_home_dir;

      #ifdef DUAL_X_CARRIAGE
        if (axis == X_AXIS) axis_home_dir = x_home_dir(active_extruder);
      #else
        axis_home_dir = home_dir(axis);
      #endif

      // Set the axis position as setup for the move
      current_position[axis] = 0;
      sync_plan_position();

      // Engage Servo endstop if enabled
      #if (NUM_SERVOS > 0) && !defined(Z_PROBE_SLED)
        #if SERVO_LEVELING
          if (axis == Z_AXIS) deploy_z_probe(); else
        #endif
          {
            if (servo_endstops[axis] > -1)
              servo[servo_endstops[axis]].write(servo_endstop_angles[axis * 2]);
          }
      #endif // SERVO_ENDSTOPS && !Z_PROBE_SLED

      #ifdef Z_DUAL_ENDSTOPS
        if (axis == Z_AXIS) In_Homing_Process(true);
      #endif

      // Move towards the endstop until an endstop is triggered
      destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
      feedrate = homing_feedrate[axis];
      line_to_destination();
      st_synchronize();

      // Set the axis position as setup for the move
      current_position[axis] = 0;
      sync_plan_position();

      // Move away from the endstop by the axis HOME_BUMP_MM
      destination[axis] = -home_bump_mm(axis) * axis_home_dir;
      line_to_destination();
      st_synchronize();

      // Slow down the feedrate for the next move
      set_homing_bump_feedrate(axis);

      // Move slowly towards the endstop until triggered
      destination[axis] = 2 * home_bump_mm(axis) * axis_home_dir;
      line_to_destination();
      st_synchronize();

      #ifdef Z_DUAL_ENDSTOPS
        if (axis == Z_AXIS) {
          float adj = fabs(z_endstop_adj);
          bool lockZ1;
          if (axis_home_dir > 0) {
            adj = -adj;
            lockZ1 = (z_endstop_adj > 0);
          }
          else
            lockZ1 = (z_endstop_adj < 0);

          if (lockZ1) Lock_z_motor(true); else Lock_z2_motor(true);
          sync_plan_position();

          // Move to the adjusted endstop height
          feedrate = homing_feedrate[axis];
          destination[Z_AXIS] = adj;
          line_to_destination();
          st_synchronize();

          if (lockZ1) Lock_z_motor(false); else Lock_z2_motor(false);
          In_Homing_Process(false);
        } // Z_AXIS
      #endif

      // Set the axis position to its home position (plus home offsets)
      axis_is_at_home(axis);

      destination[axis] = current_position[axis];
      feedrate = 0.0;
      endstops_hit_on_purpose(); // clear endstop hit flags
      axis_known_position[axis] = true;

      // Retract Servo endstop if enabled
      #if NUM_SERVOS > 0
        if (servo_endstops[axis] > -1)
          servo[servo_endstops[axis]].write(servo_endstop_angles[axis * 2 + 1]);
      #endif

      #if SERVO_LEVELING && !defined(Z_PROBE_SLED)
        if (axis == Z_AXIS) stow_z_probe();
      #endif
    }
  }
  #define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)
#endif // Cartesian || CoreXY || Scara

#ifdef DELTA
  static void axis_is_at_home(int axis) {
    current_position[axis] = base_home_pos[axis] + home_offset[axis];
    min_pos[axis] = base_min_pos(axis) + home_offset[axis];
    max_pos[axis] = base_max_pos[axis] + home_offset[axis];
  }

  static void homeaxis(AxisEnum axis) {
    #define HOMEAXIS_DO(LETTER) \
      ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

    if (axis == X_AXIS ? HOMEAXIS_DO(X) : axis == Y_AXIS ? HOMEAXIS_DO(Y) : axis == Z_AXIS ? HOMEAXIS_DO(Z) : 0) {

      int axis_home_dir = home_dir(axis);
      current_position[axis] = 0;
      sync_plan_position();
      destination[axis] = 1.5 * max_length[axis] * axis_home_dir;
      feedrate = homing_feedrate[axis];
      line_to_destination();
      st_synchronize();

      // Set the axis position as setup for the move
      current_position[axis] = 0;
      sync_plan_position();
      // Move away from the endstop by the axis HOME_BUMP_MM
      destination[axis] = -home_bump_mm(axis) * axis_home_dir;
      line_to_destination();
      st_synchronize();

      // Slow down the feedrate for the next move
      set_homing_bump_feedrate(axis);

      // retrace by the amount specified in endstop_adj
      if (endstop_adj[axis] * axis_home_dir < 0) {
        sync_plan_position();
        destination[axis] = endstop_adj[axis];
        line_to_destination();
        st_synchronize();
      }

      // Set the axis position to its home position (plus home offsets)
      axis_is_at_home(axis);

      destination[axis] = current_position[axis];
      feedrate = 0.0;
      endstops_hit_on_purpose(); // clear endstop hit flags
      axis_known_position[axis] = true;
    }
  }
  #define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

  void set_default_z_probe_offset() {
    z_probe_offset[X_AXIS] = default_z_probe_offset[X_AXIS];
    z_probe_offset[Y_AXIS] = default_z_probe_offset[Y_AXIS];
    z_probe_offset[Z_AXIS] = default_z_probe_offset[Z_AXIS];
  }

  void set_delta_constants() {
    max_length[Z_AXIS] = max_pos[Z_AXIS] - Z_MIN_POS;
    base_max_pos[Z_AXIS]  = max_pos[Z_AXIS];
    base_home_pos[Z_AXIS] = max_pos[Z_AXIS];

    DELTA_DIAGONAL_ROD_2 = pow(delta_diagonal_rod,2);

    // Effective X/Y positions of the three vertical towers.
    /*
    delta_tower1_x = (-SIN_60 * delta_radius) + tower_adj[0]; // front left tower + xa
    delta_tower1_y = (-COS_60 * delta_radius) - tower_adj[0] ;
    delta_tower2_x = -(-SIN_60 * delta_radius) + tower_adj[1]; // front right tower + xb
    delta_tower2_y = (-COS_60 * delta_radius) + tower_adj[1]; // 
    delta_tower3_x = tower_adj[2] ; // back middle tower + xc
    delta_tower3_y = -2 * (-COS_60 * delta_radius);  
    */

    delta_tower1_x = (delta_radius + tower_adj[3]) * cos((210 + tower_adj[0]) * PI/180); // front left tower
    delta_tower1_y = (delta_radius + tower_adj[3]) * sin((210 + tower_adj[0]) * PI/180); 
    delta_tower2_x = (delta_radius + tower_adj[4]) * cos((330 + tower_adj[1]) * PI/180); // front right tower
    delta_tower2_y = (delta_radius + tower_adj[4]) * sin((330 + tower_adj[1]) * PI/180); 
    delta_tower3_x = (delta_radius + tower_adj[5]) * cos((90 + tower_adj[2]) * PI/180);  // back middle tower
    delta_tower3_y = (delta_radius + tower_adj[5]) * sin((90 + tower_adj[2]) * PI/180); 
  }

  void deploy_z_probe() {
    feedrate = homing_feedrate[X_AXIS];
    destination[X_AXIS] = z_probe_deploy_start_location[X_AXIS];
    destination[Y_AXIS] = z_probe_deploy_start_location[Y_AXIS];
    destination[Z_AXIS] = z_probe_deploy_start_location[Z_AXIS];
    prepare_move_raw();

    feedrate = homing_feedrate[X_AXIS]/10;
    destination[X_AXIS] = z_probe_deploy_end_location[X_AXIS];
    destination[Y_AXIS] = z_probe_deploy_end_location[Y_AXIS];
    destination[Z_AXIS] = z_probe_deploy_end_location[Z_AXIS];
    prepare_move_raw();

    feedrate = homing_feedrate[X_AXIS];
    destination[X_AXIS] = z_probe_deploy_start_location[X_AXIS];
    destination[Y_AXIS] = z_probe_deploy_start_location[Y_AXIS];
    destination[Z_AXIS] = z_probe_deploy_start_location[Z_AXIS];
    prepare_move_raw();
    st_synchronize();
  }

  void retract_z_probe() {
    feedrate = homing_feedrate[X_AXIS];
    destination[Z_AXIS] = 50;
    prepare_move_raw();

    destination[X_AXIS] = z_probe_retract_start_location[X_AXIS];
    destination[Y_AXIS] = z_probe_retract_start_location[Y_AXIS];
    destination[Z_AXIS] = z_probe_retract_start_location[Z_AXIS];
    prepare_move();
    prepare_move_raw();

    // Move the nozzle below the print surface to push the probe up.
    feedrate = homing_feedrate[Z_AXIS]/10;
    destination[X_AXIS] = z_probe_retract_end_location[X_AXIS];
    destination[Y_AXIS] = z_probe_retract_end_location[Y_AXIS];
    destination[Z_AXIS] = z_probe_retract_end_location[Z_AXIS];
    prepare_move_raw();

    feedrate = homing_feedrate[Z_AXIS];
    destination[X_AXIS] = z_probe_retract_start_location[X_AXIS];
    destination[Y_AXIS] = z_probe_retract_start_location[Y_AXIS];
    destination[Z_AXIS] = z_probe_retract_start_location[Z_AXIS];
    prepare_move_raw();
    st_synchronize();
  }

  float z_probe() {

    enable_endstops(true);
    feedrate = homing_feedrate[X_AXIS];
    prepare_move_raw();
    st_synchronize();

    float start_z = current_position[Z_AXIS];
    long start_steps = st_get_position(Z_AXIS);

    feedrate = probing_feedrate;
    destination[Z_AXIS] = -20;
    prepare_move_raw();
    st_synchronize();
    endstops_hit_on_purpose(); // clear endstop hit flags

    #ifdef ENDSTOPS_ONLY_FOR_HOMING
       enable_endstops(false);
    #endif

    long stop_steps = st_get_position(Z_AXIS);

    saved_position[X_AXIS] = float((st_get_position(X_AXIS)) / axis_steps_per_unit[X_AXIS]);
    saved_position[Y_AXIS] = float((st_get_position(Y_AXIS)) / axis_steps_per_unit[Y_AXIS]);
    saved_position[Z_AXIS] = float((st_get_position(Z_AXIS)) / axis_steps_per_unit[Z_AXIS]);

    float mm = start_z - float(start_steps - stop_steps) / axis_steps_per_unit[Z_AXIS];
    current_position[Z_AXIS] = mm;
    sync_plan_position_delta();

    saved_position[X_AXIS] = float((st_get_position(X_AXIS)) / axis_steps_per_unit[X_AXIS]);
    saved_position[Y_AXIS] = float((st_get_position(Y_AXIS)) / axis_steps_per_unit[Y_AXIS]);
    saved_position[Z_AXIS] = float((st_get_position(Z_AXIS)) / axis_steps_per_unit[Z_AXIS]);

    feedrate = homing_feedrate[Z_AXIS];
    destination[Z_AXIS] = mm + 2;
    prepare_move_raw();
    return mm;
  }

  void calibrate_print_surface(float z_offset) {
    float probe_bed_z, probe_z, probe_h, probe_l;
    int probe_count;
      
    for (int y = 3; y >= -3; y--)
    {
      int dir = y % 2 ? -1 : 1;
      for (int x = -3*dir; x != 4*dir; x += dir)
      {
        if (x*x + y*y < 11)
        {
          destination[X_AXIS] = AUTOLEVEL_GRID * x - z_probe_offset[X_AXIS];
          if (destination[X_AXIS]<X_MIN_POS) destination[X_AXIS]=X_MIN_POS;
          if (destination[X_AXIS]>X_MAX_POS) destination[X_AXIS]=X_MAX_POS;
          destination[Y_AXIS] = AUTOLEVEL_GRID * y - z_probe_offset[Y_AXIS];
          if (destination[Y_AXIS]<Y_MIN_POS) destination[Y_AXIS]=Y_MIN_POS;
          if (destination[Y_AXIS]>Y_MAX_POS) destination[Y_AXIS]=Y_MAX_POS;
          probe_count = 0;
          probe_z = -100;
          probe_h = -100;
          probe_l = 100;
          do
          {
            probe_bed_z = probe_z;
            probe_z = z_probe() + z_offset;
            if (probe_z > probe_h) probe_h = probe_z;
            if (probe_z < probe_l) probe_l = probe_z;
            probe_count ++;
          } while ((probe_z != probe_bed_z) and (probe_count < 21));

          bed_level[x+3][3-y] = probe_bed_z;
        }
        else
        {
          bed_level[x+3][3-y] = 0.0;
        }
      }
      // For unprobed positions just copy nearest neighbor.
      if (abs(y) >= 3)
      {
        bed_level[1][3-y] = bed_level[2][3-y];
        bed_level[5][3-y] = bed_level[4][3-y];
      }
      if (abs(y) >=2)
      {
        bed_level[0][3-y] = bed_level[1][3-y];
        bed_level[6][3-y] = bed_level[5][3-y];
      }
      // Print calibration results for manual frame adjustment.
      ECHO_S(DB);
      for (int x = -3; x <= 3; x++)
      {
        ECHO_VM(bed_level[x+3][3-y], " ", 3);
      }
      ECHO_E;
    }
  }

  float probe_bed(float x, float y) {
    //Probe bed at specified location and return z height of bed
    float probe_bed_z, probe_z, probe_h, probe_l;
    int probe_count;
    //  feedrate = homing_feedrate[Z_AXIS];
    destination[X_AXIS] = x - z_probe_offset[X_AXIS];
    if (destination[X_AXIS] < X_MIN_POS) destination[X_AXIS] = X_MIN_POS;
    if (destination[X_AXIS] > X_MAX_POS) destination[X_AXIS] = X_MAX_POS;
    destination[Y_AXIS] = y - z_probe_offset[Y_AXIS];
    if (destination[Y_AXIS] < Y_MIN_POS) destination[Y_AXIS] = Y_MIN_POS;
    if (destination[Y_AXIS] > Y_MAX_POS) destination[Y_AXIS] = Y_MAX_POS;
    destination[Z_AXIS] = bed_level_c - z_probe_offset[Z_AXIS] + 3;
    prepare_move();
    st_synchronize();

    probe_count = 0;
    probe_z = -100;
    probe_h = -100;
    probe_l = 100;
    do
    {
      probe_bed_z = probe_z;
      probe_z = z_probe() + z_probe_offset[Z_AXIS];
      if (probe_z > probe_h) probe_h = probe_z;
      if (probe_z < probe_l) probe_l = probe_z;
      probe_count ++;
    } while ((probe_z != probe_bed_z) and (probe_count < 21));

    return probe_bed_z;
  }

  float z_probe_accuracy() {
    //Perform z-probe accuracy test
    float probe_h[7];
    float probe_l[7];
    float range_h = 0, range_l = 0;

    for(int x = 0; x < 7; x++)
    {
      probe_h[x] = -100;
      probe_l[x] = 100;
    }
    
    // probe test loop  
    for(int x = 0; x < 3; x++)
    {
      bed_probe_all();

      if (bed_level_c > probe_h[0]) probe_h[0] = bed_level_c;
      if (bed_level_c < probe_l[0]) probe_l[0] = bed_level_c;
      if (bed_level_z > probe_h[1]) probe_h[1] = bed_level_z;
      if (bed_level_z < probe_l[1]) probe_l[1] = bed_level_z;
      if (bed_level_oy > probe_h[2]) probe_h[2] = bed_level_oy;
      if (bed_level_oy < probe_l[2]) probe_l[2] = bed_level_oy;
      if (bed_level_x > probe_h[3]) probe_h[3] = bed_level_x;
      if (bed_level_x < probe_l[3]) probe_l[3] = bed_level_x;
      if (bed_level_oz > probe_h[4]) probe_h[4] = bed_level_oz;
      if (bed_level_oz < probe_l[4]) probe_l[4] = bed_level_oz;
      if (bed_level_y > probe_h[5]) probe_h[5] = bed_level_y;
      if (bed_level_y < probe_l[5]) probe_l[5] = bed_level_y;
      if (bed_level_ox > probe_h[6]) probe_h[6] = bed_level_ox;
      if (bed_level_ox < probe_l[6]) probe_l[6] = bed_level_ox;
    }
    for(int x = 0; x < 7; x++)
    {
      if (probe_h[x] - probe_l[x] > range_h) range_h = probe_h[x] - probe_l[x];
      if (probe_h[x] - probe_l[x] < range_l) range_l = probe_h[x] - probe_l[x];
    }
    return range_h - range_l;
  }

  void bed_probe_all() {
    //Probe all bed positions & store carriage positions
    bed_level_c = probe_bed(0.0, 0.0);
    save_carriage_positions(0);
    bed_level_z = probe_bed(0.0, bed_radius);
    save_carriage_positions(1);
    bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
    save_carriage_positions(2);
    bed_level_x = probe_bed(-SIN_60 * bed_radius, -COS_60 * bed_radius);
    save_carriage_positions(3);
    bed_level_oz = probe_bed(0.0, -bed_radius);
    save_carriage_positions(4);
    bed_level_y = probe_bed(SIN_60 * bed_radius, -COS_60 * bed_radius);
    save_carriage_positions(5);
    bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
    save_carriage_positions(6);
  }
    
  void calibration_report() {
    //Display Report
    ECHO_SMV(DB, "\tZ-Tower\t\t\tEndstop Offsets\t", bed_level_z, 4);
    ECHO_MV("\t\t\tX:", endstop_adj[0]);
    ECHO_MV(" Y:", endstop_adj[1]);
    ECHO_EMV(" Z:", endstop_adj[2]);

    ECHO_SVM(DB, bed_level_oy, "\t\t", 4);
    ECHO_EVM(bed_level_ox, "\t\tTower Position Adjust", 4);

    ECHO_SMV(DB, "\t", bed_level_c, 4);
    ECHO_MV("\t\t\tA:",tower_adj[0]);
    ECHO_MV(" B:",tower_adj[1]);
    ECHO_EMV(" C:",tower_adj[2]);

    ECHO_SV(DB, bed_level_x, 4);
    ECHO_V("\t\t", bed_level_y, 4);
    ECHO_MV("\t\tI:",tower_adj[3]);
    ECHO_MV(" J:",tower_adj[4]);
    ECHO_EMV(" K:",tower_adj[5]);

    ECHO_SMV(DB, "\t", bed_level_oz, 4);
    ECHO_EMV("\t\t\tDelta Radius: ", delta_radius, 4);

    ECHO_LM(DB, "X-Tower\t\tY-Tower\t\tDiag Rod: ", delta_diagonal_rod, 4);
  }

  void save_carriage_positions(int position_num) {
    for(int8_t i=0; i < NUM_AXIS; i++) {
      saved_positions[position_num][i] = saved_position[i];    
    }
  }

  void home_delta_axis() {
    saved_feedrate = feedrate;
    saved_feedrate_multiplier = feedrate_multiplier;
    feedrate_multiplier = 100;
    refresh_cmd_timeout();

    enable_endstops(true);

    set_destination_to_current();

    feedrate = 0.0;

    // Pretend the current position is 0,0,0
    for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = 0;
    sync_plan_position();

    // Move all carriages up together until the first endstop is hit.
    for (int i = X_AXIS; i <= Z_AXIS; i++) destination[i] = 3 * Z_MAX_LENGTH;
    feedrate = 1.732 * homing_feedrate[X_AXIS];
    line_to_destination();
    st_synchronize();
    endstops_hit_on_purpose(); // clear endstop hit flags

    // Destination reached
    for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = destination[i];

    // take care of back off and rehome now we are all at the top
    HOMEAXIS(X);
    HOMEAXIS(Y);
    HOMEAXIS(Z);

    sync_plan_position_delta();

    #ifdef ENDSTOPS_ONLY_FOR_HOMING
      enable_endstops(false);
    #endif

    feedrate = saved_feedrate;
    feedrate_multiplier = saved_feedrate_multiplier;
    refresh_cmd_timeout();
    endstops_hit_on_purpose(); // clear endstop hit flags
  }

  void prepare_move_raw() {
    refresh_cmd_timeout();
    calculate_delta(destination);
    plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], destination[E_AXIS], feedrate*feedrate_multiplier/60/100.0, active_extruder, active_driver);
    set_current_to_destination();
  }

  void calculate_delta(float cartesian[3]) {
    delta[X_AXIS] = sqrt(DELTA_DIAGONAL_ROD_2
                         - sq(delta_tower1_x-cartesian[X_AXIS])
                         - sq(delta_tower1_y-cartesian[Y_AXIS])
                         ) + cartesian[Z_AXIS];
    delta[Y_AXIS] = sqrt(DELTA_DIAGONAL_ROD_2
                         - sq(delta_tower2_x-cartesian[X_AXIS])
                         - sq(delta_tower2_y-cartesian[Y_AXIS])
                         ) + cartesian[Z_AXIS];
    delta[Z_AXIS] = sqrt(DELTA_DIAGONAL_ROD_2
                         - sq(delta_tower3_x-cartesian[X_AXIS])
                         - sq(delta_tower3_y-cartesian[Y_AXIS])
                         ) + cartesian[Z_AXIS];
  }

  // Adjust print surface height by linear interpolation over the bed_level array.
  void adjust_delta(float cartesian[3]) {
    float grid_x = max(-2.999, min(2.999, cartesian[X_AXIS] / AUTOLEVEL_GRID));
    float grid_y = max(-2.999, min(2.999, cartesian[Y_AXIS] / AUTOLEVEL_GRID));
    int floor_x = floor(grid_x);
    int floor_y = floor(grid_y);
    float ratio_x = grid_x - floor_x;
    float ratio_y = grid_y - floor_y;
    float z1 = bed_level[floor_x+3][floor_y+3];
    float z2 = bed_level[floor_x+3][floor_y+4];
    float z3 = bed_level[floor_x+4][floor_y+3];
    float z4 = bed_level[floor_x+4][floor_y+4];
    float left = (1-ratio_y)*z1 + ratio_y*z2;
    float right = (1-ratio_y)*z3 + ratio_y*z4;
    float offset = (1-ratio_x)*left + ratio_x*right;

    delta[X_AXIS] += offset;
    delta[Y_AXIS] += offset;
    delta[Z_AXIS] += offset;

    /*
    ECHO_MSV(DB,"grid_x=", grid_x);
    ECHO_MV(" grid_y=", grid_y);
    ECHO_MV(" floor_x=", floor_x);
    ECHO_MV(" floor_y=", floor_y);
    ECHO_MV(" ratio_x=", ratio_x);
    ECHO_MV(" ratio_y=", ratio_y);
    ECHO_MV(" z1=", z1);
    ECHO_MV(" z2=", z2);
    ECHO_MV(" z3=", z3);
    ECHO_MV(" z4=", z4);
    ECHO_MV(" left=", left);
    ECHO_MV(" right=", right);
    ECHO_EMV(" offset=", offset);
    */
  }

  inline void delta_autocalibration(int iterations){
    //int iterations = 100; // Maximum number of iterations
    int loopcount = 1;
    float adj_r_target, adj_dr_target;
    float adj_r_target_delta = 0, adj_dr_target_delta = 0;
    float adj_AlphaA, adj_AlphaB, adj_AlphaC;
    float adj_RadiusA, adj_RadiusB, adj_RadiusC;
    float radiusErrorA, radiusErrorB, radiusErrorC;
    float adj_r = 0, adj_dr = 0;
    boolean equalAB, equalBC, equalCA;
    boolean adj_r_done, adj_dr_done, adj_tower_done;
    boolean adj_dr_allowed = true;
    float h_endstop = -100, l_endstop = 100;
    float probe_error, ftemp;

    ECHO_SMV(OK,MSG_DELTA_AUTO_CALIBRATION1, ac_prec, 3);
    ECHO_EMV(MSG_DELTA_AUTO_CALIBRATION2, iterations);
    
    LCD_MESSAGEPGM("Auto Calibration...");

    if (code_seen('D')) {
      delta_diagonal_rod = code_value();
      adj_dr_allowed = false;
      ECHO_SMV(DB, "Using diagional rod length: ", delta_diagonal_rod);
      ECHO_EM(DB, "mm (will not be adjusted)");
    }

    // First Check for control endstop
    ECHO_LM(DB, "First check for adjust Z-Height");
    home_delta_axis();
    deploy_z_probe(); 
    //Probe all points
    bed_probe_all();
    //Show calibration report      
    calibration_report();
    
    //Check that endstop are within limits
    if (bed_level_x + endstop_adj[0] > h_endstop) h_endstop = bed_level_x + endstop_adj[0];
    if (bed_level_x + endstop_adj[0] < l_endstop) l_endstop = bed_level_x + endstop_adj[0];
    if (bed_level_y + endstop_adj[1] > h_endstop) h_endstop = bed_level_y + endstop_adj[1];
    if (bed_level_y + endstop_adj[1] < l_endstop) l_endstop = bed_level_y + endstop_adj[1];
    if (bed_level_z + endstop_adj[2] > h_endstop) h_endstop = bed_level_z + endstop_adj[2];
    if (bed_level_z + endstop_adj[2] < l_endstop) l_endstop = bed_level_z + endstop_adj[2];

    if (h_endstop - l_endstop > 3) {
      ECHO_LM(DB, "The position of the endstop switches on this printer are not within limits");
      ECHO_LM(DB, "Adjust endstop switches so that they are within 3mm Z-height of each other");
      ECHO_SMV(DB, "Current Endstop Positions - X: ", bed_level_x + endstop_adj[0]); 
      ECHO_MV(" Y: ", bed_level_y + endstop_adj[1]);
      ECHO_EMV(" Z: ", bed_level_z + endstop_adj[2]);
      ECHO_LV(ER,"Auto calibration aborted");

      retract_z_probe();

      //Restore saved variables
      feedrate = saved_feedrate;
      feedrate_multiplier = saved_feedrate_multiplier;
      return;
    }

    do {
      ECHO_LMV(DB, "Iteration: ", loopcount);

      if ((bed_level_c > 3) or (bed_level_c < -3)) {
        //Build height is not set correctly .. 
        max_pos[Z_AXIS] -= bed_level_c + 2;
        set_delta_constants();
        ECHO_SMV(DB, "Adjusting Z-Height to: ", max_pos[Z_AXIS]);
        ECHO_EM(" mm..");
      } 
      else {
        if ((bed_level_x < -ac_prec) or (bed_level_x > ac_prec) or (bed_level_y < -ac_prec) or (bed_level_y > ac_prec) or (bed_level_z < -ac_prec) or (bed_level_z > ac_prec)) {
          //Endstop req adjustment
          ECHO_LM(DB,"Adjusting Endstop..");
          endstop_adj[0] += bed_level_x / 1.05;
          endstop_adj[1] += bed_level_y / 1.05;
          endstop_adj[2] += bed_level_z / 1.05; 

          //Check that no endstop adj values are > 0 (not allowed).. if they are, reduce the build height to compensate.
          h_endstop = 0;
          for(int x = 0; x < 3; x++) { 
            if (endstop_adj[x] > h_endstop) h_endstop = endstop_adj[x]; 
          }
          if (h_endstop > 0) {
            //Reduce build height and adjust endstop
            for(int x=0; x < 3; x++) {
              endstop_adj[x] -= h_endstop + 2;
            }
            max_pos[Z_AXIS] -= h_endstop + 2;
            set_delta_constants();
            ECHO_SMV(DB,"Adjusting Z-Height to: ", max_pos[Z_AXIS]);
            ECHO_EM(" mm..");                
          }
        }
        else {
          ECHO_LM(DB,"Endstop: OK");
          adj_r_target = (bed_level_x + bed_level_y + bed_level_z) / 3;
          adj_dr_target = (bed_level_ox + bed_level_oy + bed_level_oz) / 3;

          //Determine which parameters require adjustment
          if ((bed_level_c >= adj_r_target - ac_prec) and (bed_level_c <= adj_r_target + ac_prec)) adj_r_done = true; 
          else adj_r_done = false;
          if ((adj_dr_target >= adj_r_target - ac_prec) and (adj_dr_target <= adj_r_target + ac_prec)) adj_dr_done = true; 
          else adj_dr_done = false;
          if ((bed_level_x != bed_level_ox) or (bed_level_y != bed_level_oy) or (bed_level_z != bed_level_oz)) adj_tower_done = false; 
          else adj_tower_done = true;
          if ((adj_r_done == false) or (adj_dr_done == false) or (adj_tower_done == false)) {
            //delta geometry adjustment required
            ECHO_LM(DB,"Adjusting Delta Geometry..");
            //set initial direction and magnitude for delta radius & diagonal rod adjustment
            if (adj_r == 0) {
              if (adj_r_target > bed_level_c) adj_r = 1; 
              else adj_r = -1;
            }

            if (adj_dr == 0) {
              if (adj_r_target > adj_dr_target) adj_dr = 1; 
              else adj_dr = -1;
            }

            //Don't adjust tower positions on first iteration
            adj_AlphaA = adj_AlphaB = adj_AlphaC = 0; 
            adj_RadiusA = adj_RadiusB = adj_RadiusC = 0;

            do {   
              //Apply adjustments 
              if (adj_r_done == false) {
                ECHO_SMV(DB,"Adjusting Delta Radius (", delta_radius);
                ECHO_MV(" -> ", delta_radius + adj_r);
                ECHO_EM(")");
                delta_radius += adj_r;
              }

              if (adj_dr_allowed == false) adj_dr_done = true;
              if (adj_dr_done == false) {
                ECHO_SMV(DB,"Adjusting Diagonal Rod Length (", delta_diagonal_rod);
                ECHO_MV(" -> ", delta_diagonal_rod + adj_dr);
                ECHO_EM(")");
                delta_diagonal_rod += adj_dr;
              }

              tower_adj[0] -= adj_AlphaA;
              tower_adj[1] -= adj_AlphaB;
              tower_adj[2] -= adj_AlphaC;
              tower_adj[3] += adj_RadiusA;
              tower_adj[4] += adj_RadiusB;
              tower_adj[5] += adj_RadiusC;

              set_delta_constants();

              bed_probe_all();
              calibration_report();

              //Check to see if auto calc is complete to within limits..
              if (adj_dr_allowed == true) {
                if   ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)
                  and (bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)
                  and (bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)
                  and (bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)
                  and (bed_level_ox >= -ac_prec) and (bed_level_ox <= ac_prec)
                  and (bed_level_oy >= -ac_prec) and (bed_level_oy <= ac_prec)
                  and (bed_level_oz >= -ac_prec) and (bed_level_oz <= ac_prec)) loopcount = iterations;
              } 
              else {
                if   ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)
                  and (bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)
                  and (bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)
                  and (bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)) loopcount = iterations;
              }

              //set delta radius and diagonal rod targets
              adj_r_target = (bed_level_x + bed_level_y + bed_level_z) / 3;
              adj_dr_target = (bed_level_ox + bed_level_oy + bed_level_oz) / 3;

              //set Tower position adjustment values
              adj_AlphaA = bed_level_oy - bed_level_oz;
              adj_AlphaB = bed_level_oz - bed_level_ox;
              adj_AlphaC = bed_level_ox - bed_level_oy;

              //set tower radius errors
              radiusErrorA = bed_level_x - bed_level_ox;
              radiusErrorB = bed_level_y - bed_level_oy;
              radiusErrorC = bed_level_z - bed_level_oz;

              if ((radiusErrorA >= (radiusErrorB - 0.02)) and (radiusErrorA <= (radiusErrorB + 0.02))) equalAB = true;
              else equalAB = false;
              if ((radiusErrorB >= (radiusErrorC - 0.02)) and (radiusErrorB <= (radiusErrorC + 0.02))) equalBC = true;
              else equalBC = false;
              if ((radiusErrorC >= (radiusErrorA - 0.02)) and (radiusErrorC <= (radiusErrorA + 0.02))) equalCA = true;
              else equalCA = false;

              #ifdef DEBUG_MESSAGES
                if (equalAB == true) {
                  ECHO_SMV(DB, "Tower AB Equal (A = ", radiusErrorA);
                  ECHO_MV(" B=", radiusErrorB);
                  ECHO_EM(")");
                }
                else ECHO_LM(DB, "equalAB = false");

                if (equalBC == true) {
                  ECHO_SMV(DB, "Tower BC Equal (B=", radiusErrorB);
                  ECHO_MV(" C = ", radiusErrorC);
                  ECHO_EM(")");
                }
                else ECHO_LM(DB, "equalBC = false");

                if (equalCA == true) {
                  ECHO_SMV(DB, "Tower CA Equal (C = ", radiusErrorC);
                  ECHO_MV(" A = ", radiusErrorA);
                  ECHO_EM(")");
                }
                else ECHO_LM(DB, "equalCA = false");
              #endif //DEBUG_MESSAGES

              if ((equalAB == true) and (equalBC == true) and (equalCA == true)) {
                // all tower radius out by the same amount (within 0.02) - allow adjustment with delta rod length
                #ifdef DEBUG_MESSAGES
                  ECHO_LM(DB, "All tower radius errors equal");
                #endif
                adj_RadiusA = adj_RadiusB = adj_RadiusC = 0;
              }

              if ((equalAB == true) and (equalBC == false) and (equalCA == false)) {
                //Tower C radius error.. adjust it
                ECHO_LM(DB, "TowerC Radius error - adjusting");
                if (adj_RadiusC == 0) {
                  if (bed_level_z < bed_level_oz) adj_RadiusC = 0.5;
                  if (bed_level_z > bed_level_oz) adj_RadiusC = -0.5;
                  #ifdef DEBUG_MESSAGES
                    ECHO_LMV(DB, "adj_RadiusC set to ",adj_RadiusC);
                  #endif
                }
              }

              if ((equalBC == true) and (equalAB == false) and (equalCA == false)) {
                //Tower A radius error .. adjust it
                ECHO_LM(DB, "TowerA Radius error - adjusting");
                if (adj_RadiusA == 0) {
                  if (bed_level_x < bed_level_ox) adj_RadiusA = 0.5;
                  if (bed_level_x > bed_level_ox) adj_RadiusA = -0.5;  
                  #ifdef DEBUG_MESSAGES
                    ECHO_LMV(DB, "adj_RadiusA set to ",adj_RadiusA);
                  #endif
                }
              } 

              if ((equalCA == true) and (equalAB == false) and (equalBC == false)) {
                //Tower B radius error .. adjust it
                ECHO_LM(DB, "TowerB Radius error - adjusting");
                if (adj_RadiusB == 0) {
                  if (bed_level_y < bed_level_oy) adj_RadiusB = 0.5;
                  if (bed_level_y > bed_level_oy) adj_RadiusB = -0.5;                     
                  #ifdef DEBUG_MESSAGES
                    ECHO_LMV(DB, "adj_RadiusB set to ",adj_RadiusB);
                  #endif
                }
              }                   

              if (((adj_r > 0) and (bed_level_c > adj_r_target)) or ((adj_r < 0) and (bed_level_c < adj_r_target))) {
                //overshot target .. reverse & scale down
                adj_r = -(adj_r / 2);
              }

              if (((adj_dr > 0) and (adj_dr_target > adj_r_target)) or ((adj_dr < 0) and (adj_dr_target < adj_r_target))) {
                //overshot target .. reverse & scale down
                adj_dr = -(adj_dr / 2);
              }

              //Tower radius overshot targets?
              if (((adj_RadiusA > 0) and (bed_level_x > bed_level_ox)) or ((adj_RadiusA < 0) and (bed_level_x < bed_level_ox))) adj_RadiusA = -(adj_RadiusA / 2);
              if (((adj_RadiusB > 0) and (bed_level_y > bed_level_oy)) or ((adj_RadiusB < 0) and (bed_level_y < bed_level_oy))) adj_RadiusB = -(adj_RadiusB / 2);
              if (((adj_RadiusC > 0) and (bed_level_z > bed_level_oz)) or ((adj_RadiusC < 0) and (bed_level_z < bed_level_oz))) adj_RadiusC = -(adj_RadiusC / 2);

              //Delta radius adjustment complete?                       
              if ((bed_level_c >= (adj_r_target - ac_prec)) and (bed_level_c <= (adj_r_target + ac_prec))) adj_r_done = true; 
              else adj_r_done = false;

              //Diag Rod adjustment complete?
              if ((adj_dr_target >= (adj_r_target - ac_prec)) and (adj_dr_target <= (adj_r_target + ac_prec))) adj_dr_done = true; 
              else adj_dr_done = false;

              #ifdef DEBUG_MESSAGES
                ECHO_SMV(DB, "c: ", bed_level_c);
                ECHO_MV(" x: ", bed_level_x);
                ECHO_MV(" y: ", bed_level_y);
                ECHO_MV(" z: ", bed_level_z);
                ECHO_MV(" ox: ", bed_level_ox);
                ECHO_MV(" oy: ", bed_level_oy);
                ECHO_EMV(" oz: ", bed_level_oz);
                ECHO_SMV("radius:", delta_radius, 4);
                ECHO_EMV(" diagrod:", delta_diagonal_rod, 4);
                ECHO_SM(DB, "Radius Adj Complete: ");
                if (adj_r_done == true) ECHO_M("Yes"); 
                else ECHO_M("No");
                ECHO_M(" DiagRod Adj Complete: ");
                if (adj_dr_done == true) ECHO_EM("Yes"); 
                else ECHO_EM("No");
                ECHO_SMV(DB, "RadiusA Error: ",radiusErrorA);
                ECHO_MV(" (adjust: ",adj_RadiusA);
                ECHO_EM(")");
                ECHO_SMV(DB, "RadiusB Error: ",radiusErrorB);
                ECHO_MV(" (adjust: ",adj_RadiusB);
                ECHO_EM(")");
                ECHO_SMV(DB, "RadiusC Error: ",radiusErrorC);
                ECHO_MV(" (adjust: ",adj_RadiusC);
                ECHO_EM(")");
                ECHO_LMV(DB, "DeltaAlphaA: ",adj_AlphaA);
                ECHO_LMV(DB, "DeltaAlphaB: ",adj_AlphaB);
                ECHO_LMV(DB, "DeltaAlphaC: ",adj_AlphaC);
              #endif
            } while (((adj_r_done == false) or (adj_dr_done = false)) and (loopcount < iterations));
          }
          else {
            ECHO_LM(DB, "Delta Geometry: OK");
          }
        }
      }

      if (loopcount < iterations) {
        home_delta_axis();

        //probe bed and display report
        bed_probe_all();
        calibration_report();

        //Check to see if autocalc is complete to within limits..
        if (adj_dr_allowed == true) {
          if   ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)
            and (bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)
            and (bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)
            and (bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)
            and (bed_level_ox >= -ac_prec) and (bed_level_ox <= ac_prec)
            and (bed_level_oy >= -ac_prec) and (bed_level_oy <= ac_prec)
            and (bed_level_oz >= -ac_prec) and (bed_level_oz <= ac_prec)) loopcount = iterations;
        }
        else {
          if   ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)
            and (bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)
            and (bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)
            and (bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)) loopcount = iterations;
        }
      }

      loopcount ++;

      manage_heater();
      manage_inactivity();
      lcd_update();

    } while(loopcount < iterations);

    ECHO_LM(DB, "Auto Calibration Complete");
    LCD_MESSAGEPGM("Complete");
    ECHO_LM(DB, "Issue M500 Command to save calibration settings to EPROM (if enabled)");
    /*   
     if ((abs(delta_diagonal_rod - saved_delta_diagonal_rod) > 1) and (adj_dr_allowed == true)) {
       ECHO_SMV(DB, "WARNING: The length of diagonal rods specified (", saved_delta_diagonal_rod);
       ECHO_EV(" mm) appears to be incorrect");
       ECHO_LM(DB, "If you have measured your rods and you believe that this value is correct, this could indicate");
       ECHO_LM(DB,"excessive twisting movement of carriages and/or loose screws/joints on carriages or end effector");
     }
     */

    retract_z_probe();

    //Restore saved variables
    feedrate = saved_feedrate;
    feedrate_multiplier = saved_feedrate_multiplier;
    return;
  }
#endif //DELTA

#ifdef IDLE_OOZING_PREVENT
  void IDLE_OOZING_retract(bool retracting) {  
    if (retracting && !IDLE_OOZING_retracted[active_extruder]) {
  	  set_destination_to_current();
  	  current_position[E_AXIS]+=IDLE_OOZING_LENGTH/volumetric_multiplier[active_extruder];
  	  plan_set_e_position(current_position[E_AXIS]);
  	  float oldFeedrate = feedrate;
  	  feedrate=IDLE_OOZING_FEEDRATE*60;
  	  IDLE_OOZING_retracted[active_extruder]=true;
  	  prepare_move();
  	  feedrate = oldFeedrate;
    }
    else if(!retracting && IDLE_OOZING_retracted[active_extruder]) {
  	  set_destination_to_current();
  	  current_position[E_AXIS]-=(IDLE_OOZING_LENGTH+IDLE_OOZING_RECOVER_LENGTH)/volumetric_multiplier[active_extruder];
  	  plan_set_e_position(current_position[E_AXIS]);
  	  float oldFeedrate = feedrate;
  	  feedrate=IDLE_OOZING_RECOVER_FEEDRATE * 60;
  	  IDLE_OOZING_retracted[active_extruder] = false;
  	  prepare_move();
      feedrate = oldFeedrate;
    }
  }
#endif

#ifdef FWRETRACT
  void retract(bool retracting, bool swapretract = false) {

    if (retracting == retracted[active_extruder]) return;

    float oldFeedrate = feedrate;

    set_destination_to_current();

    if (retracting) {

      feedrate = retract_feedrate * 60;
      current_position[E_AXIS] += (swapretract ? retract_length_swap : retract_length) / volumetric_multiplier[active_extruder];
      plan_set_e_position(current_position[E_AXIS]);
      prepare_move();

      if (retract_zlift > 0.01) {
        current_position[Z_AXIS] -= retract_zlift;
        #ifdef DELTA
          sync_plan_position_delta();
        #else
          sync_plan_position();
        #endif
        prepare_move();
      }
    }
    else {

      if (retract_zlift > 0.01) {
        current_position[Z_AXIS] += retract_zlift;
        #ifdef DELTA
          sync_plan_position_delta();
        #else
          sync_plan_position();
        #endif
        //prepare_move();
      }

      feedrate = retract_recover_feedrate * 60;
      float move_e = swapretract ? retract_length_swap + retract_recover_length_swap : retract_length + retract_recover_length;
      current_position[E_AXIS] -= move_e / volumetric_multiplier[active_extruder];
      plan_set_e_position(current_position[E_AXIS]);
      prepare_move();
    }

    feedrate = oldFeedrate;
    retracted[active_extruder] = retracting;

  } // retract()
#endif //FWRETRACT

#ifdef Z_PROBE_SLED

  #ifndef SLED_DOCKING_OFFSET
    #define SLED_DOCKING_OFFSET 0
  #endif

  /**
   * Method to dock/undock a sled designed by Charles Bell.
   *
   * dock[in]     If true, move to MAX_X and engage the electromagnet
   * offset[in]   The additional distance to move to adjust docking location
   */
  static void dock_sled(bool dock, int offset=0) {
    if (!axis_known_position[X_AXIS] || !axis_known_position[Y_AXIS]) {
      LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
      ECHO_LM(OK, MSG_POSITION_UNKNOWN);
      return;
    }

    if (dock) {
      do_blocking_move_to(X_MAX_POS + SLED_DOCKING_OFFSET + offset, current_position[Y_AXIS], current_position[Z_AXIS]); // this also updates current_position
      digitalWrite(SERVO0_PIN, LOW); // turn off magnet
    } else {
      float z_loc = current_position[Z_AXIS];
      if (z_loc < Z_RAISE_BEFORE_PROBING + 5) z_loc = Z_RAISE_BEFORE_PROBING;
      do_blocking_move_to(X_MAX_POS + SLED_DOCKING_OFFSET + offset, Y_PROBE_OFFSET_FROM_EXTRUDER, z_loc); // this also updates current_position
      digitalWrite(SERVO0_PIN, HIGH); // turn on magnet
    }
  }
#endif //Z_PROBE_SLED

inline void lcd_beep(int number_beep = 3) {
  #ifdef LCD_USE_I2C_BUZZER
    #if !defined(LCD_FEEDBACK_FREQUENCY_HZ) || !defined(LCD_FEEDBACK_FREQUENCY_DURATION_MS)
      for(int8_t i = 0; i < 3; i++) {
        lcd_buzz(1000/6,100);
      }
    #else
      for(int8_t i = 0; i < number_beep; i++) {
        lcd_buzz(LCD_FEEDBACK_FREQUENCY_DURATION_MS,LCD_FEEDBACK_FREQUENCY_HZ);
      }
    #endif
  #elif defined(BEEPER) && BEEPER > -1
    SET_OUTPUT(BEEPER);
    #if !defined(LCD_FEEDBACK_FREQUENCY_HZ) || !defined(LCD_FEEDBACK_FREQUENCY_DURATION_MS)
      for(int8_t i = 0; i < number_beep; i++) {
        WRITE(BEEPER,HIGH);
        delay(100);
        WRITE(BEEPER,LOW);
        delay(100);
      }
    #else
      for(int8_t i=0;i<number_beep;i++) {
        WRITE(BEEPER,HIGH);
        delay(1000000 / LCD_FEEDBACK_FREQUENCY_HZ / 2);
        WRITE(BEEPER,LOW);
        delay(1000000 / LCD_FEEDBACK_FREQUENCY_HZ / 2);
      }
    #endif
  #endif
}

inline void wait_heater() {
  setWatch();

  millis_t temp_ms = millis();

  /* See if we are heating up or cooling down */
  target_direction = isHeatingHotend(target_extruder); // true if heating, false if cooling

  cancel_heatup = false;

  #ifdef TEMP_RESIDENCY_TIME
    long residency_start_ms = -1;
    /* continue to loop until we have reached the target temp
      _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
    while((!cancel_heatup)&&((residency_start_ms == -1) ||
          (residency_start_ms >= 0 && (((unsigned int) (millis() - residency_start_ms)) < (TEMP_RESIDENCY_TIME * 1000UL)))) )
  #else
    while ( target_direction ? (isHeatingHotend(target_extruder)) : (isCoolingHotend(target_extruder)&&(no_wait_for_cooling==false)) )
  #endif //TEMP_RESIDENCY_TIME

    { // while loop
      if (millis() > temp_ms + 1000UL) { //Print temp & remaining time every 1s while waiting
        ECHO_SMV(OK, "T:", degHotend(target_extruder),1);
        ECHO_MV(" E:", target_extruder);
        #ifdef TEMP_RESIDENCY_TIME
          ECHO_M(" W:");
          if (residency_start_ms > -1) {
            temp_ms = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residency_start_ms)) / 1000UL;
            ECHO_EV(temp_ms);
          }
          else {
            ECHO_EM("?");
          }
        #else
          ECHO_E;
        #endif
        temp_ms = millis();
      }
      manage_heater();
      manage_inactivity();
      lcd_update();
      #ifdef TEMP_RESIDENCY_TIME
        // start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
        // or when current temp falls outside the hysteresis after target temp was reached
        if ((residency_start_ms == -1 &&  target_direction && (degHotend(target_extruder) >= (degTargetHotend(target_extruder)-TEMP_WINDOW))) ||
            (residency_start_ms == -1 && !target_direction && (degHotend(target_extruder) <= (degTargetHotend(target_extruder)+TEMP_WINDOW))) ||
            (residency_start_ms > -1 && labs(degHotend(target_extruder) - degTargetHotend(target_extruder)) > TEMP_HYSTERESIS) )
        {
          residency_start_ms = millis();
        }
      #endif //TEMP_RESIDENCY_TIME
    }

  LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
  refresh_cmd_timeout();
  print_job_start_ms = previous_cmd_ms;
}

inline void wait_bed() {
  millis_t temp_ms = millis();

  cancel_heatup = false;
  target_direction = isHeatingBed(); // true if heating, false if cooling

  while ((target_direction && !cancel_heatup) ? isHeatingBed() : isCoolingBed() && !no_wait_for_cooling) {
    millis_t ms = millis();
    if (ms > temp_ms + 1000UL) { //Print Temp Reading every 1 second while heating up.
      temp_ms = ms;
      float tt = degHotend(active_extruder);
      ECHO_SMV(OK, "T:", tt);
      ECHO_MV(" E:", active_extruder);
      ECHO_EMV(" B:", degBed(), 1);
    }
    manage_heater();
    manage_inactivity();
    lcd_update();
  }
  LCD_MESSAGEPGM(MSG_BED_DONE);
  refresh_cmd_timeout();
}

/******************************************************************************
***************************** G-Code Functions ********************************
*******************************************************************************/


/**
 * G0, G1: Coordinated movement of X Y Z E axes
 */
inline void gcode_G0_G1() {
  if (IsRunning()) {

    #ifdef IDLE_OOZING_PREVENT
      IDLE_OOZING_retract(false);
    #endif

    get_coordinates(); // For X Y Z E F

    #ifdef FWRETRACT
      if (autoretract_enabled) {
        if (!(code_seen('X') || code_seen('Y') || code_seen('Z')) && code_seen('E')) {
          float echange = destination[E_AXIS] - current_position[E_AXIS];
          // Is this move an attempt to retract or recover?
          if ((echange < -MIN_RETRACT && !retracted[active_extruder]) || (echange > MIN_RETRACT && retracted[active_extruder])) {
            current_position[E_AXIS] = destination[E_AXIS]; // hide the slicer-generated retract/recover from calculations
            plan_set_e_position(current_position[E_AXIS]);  // AND from the planner
            retract(!retracted[active_extruder]);
            return;
          }
        }
      }
    #endif //FWRETRACT
    prepare_move();
    //ClearToSend();
  }
}

/**
 * G2: Clockwise Arc
 * G3: Counterclockwise Arc
 */
inline void gcode_G2_G3(bool clockwise) {
  if (IsRunning()) {
    get_arc_coordinates();
    prepare_arc_move(clockwise);
  }
}

/**
 * G4: Dwell S<seconds> or P<milliseconds>
 */
inline void gcode_G4() {
  millis_t codenum = 0;

  LCD_MESSAGEPGM(MSG_DWELL);

  if (code_seen('P')) codenum = code_value_long(); // milliseconds to wait
  if (code_seen('S')) codenum = code_value_long() * 1000; // seconds to wait

  st_synchronize();
  refresh_cmd_timeout();
  codenum += previous_cmd_ms;  // keep track of when we started waiting
  while (millis() < codenum) {
    manage_heater();
    manage_inactivity();
    lcd_update();
  }
}

#ifdef FWRETRACT

  /**
   * G10 - Retract filament according to settings of M207
   * G11 - Recover filament according to settings of M208
   */
  inline void gcode_G10_G11(bool doRetract = false) {
    #if EXTRUDERS > 1
      if (doRetract) {
        retracted_swap[active_extruder] = (code_seen('S') && code_value_short() == 1); // checks for swap retract argument
      }
    #endif
    retract(doRetract
     #if EXTRUDERS > 1
      , retracted_swap[active_extruder]
     #endif
    );
  }

#endif //FWRETRACT

/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 * Cartesian parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *
 * If numbers are included with XYZ set the position as with G92
 * Currently adds the home_offset, which may be wrong and removed soon.
 *
 *  Xn  Home X, setting X to n + home_offset[X_AXIS]
 *  Yn  Home Y, setting Y to n + home_offset[Y_AXIS]
 *  Zn  Home Z, setting Z to n + home_offset[Z_AXIS]
 */
inline void gcode_G28(boolean home_x = false, boolean home_y = false) {

  // For auto bed leveling, clear the level matrix
  #ifdef ENABLE_AUTO_BED_LEVELING
    plan_bed_level_matrix.set_to_identity();
  #endif

  saved_feedrate = feedrate;
  saved_feedrate_multiplier = feedrate_multiplier;
  feedrate_multiplier = 100;
  refresh_cmd_timeout();

  enable_endstops(true);

  set_destination_to_current();

  feedrate = 0.0;

  bool  homeX = code_seen(axis_codes[X_AXIS]),
        homeY = code_seen(axis_codes[Y_AXIS]),
        homeZ = code_seen(axis_codes[Z_AXIS]),
        homeE = code_seen(axis_codes[E_AXIS]);
        
  home_all_axis = !(homeX || homeY || homeZ || homeE || home_x || home_y) || (homeX && homeY && homeZ);

  #ifdef NPR2
    if((home_all_axis) || (code_seen(axis_codes[E_AXIS]))) {
      active_driver = active_extruder = 1;
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], -200, COLOR_HOMERATE, active_extruder, active_driver);
      st_synchronize();
      old_color = 99;
      active_driver = active_extruder = 0;
    }
  #endif

  #ifdef DELTA
    // A delta can only safely home all axis at the same time
    // all axis have to home at the same time

    // Pretend the current position is 0,0,0
    for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = 0;
    sync_plan_position();

    // Move all carriages up together until the first endstop is hit.
    for (int i = X_AXIS; i <= Z_AXIS; i++) destination[i] = 3 * Z_MAX_LENGTH;
    feedrate = 1.732 * homing_feedrate[X_AXIS];
    line_to_destination();
    st_synchronize();
    endstops_hit_on_purpose(); // clear endstop hit flags

    // Destination reached
    for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = destination[i];

    // take care of back off and rehome now we are all at the top
    HOMEAXIS(X);
    HOMEAXIS(Y);
    HOMEAXIS(Z);

    sync_plan_position_delta();

  #else // NOT DELTA

    if (home_all_axis || homeZ) {

      #if Z_HOME_DIR > 0  // If homing away from BED do Z first

        HOMEAXIS(Z);

      #elif !defined(Z_SAFE_HOMING) && defined(Z_RAISE_BEFORE_HOMING) && Z_RAISE_BEFORE_HOMING > 0

        // Raise Z before homing any other axes

        destination[Z_AXIS] = -Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS); // Set destination away from bed
        feedrate = max_feedrate[Z_AXIS] * 60;
        line_to_destination();
        st_synchronize();

      #endif

    } // home_all_axis || homeZ

    #ifdef QUICK_HOME

      if (home_all_axis || (homeX && homeY)) {  // First diagonal move

        current_position[X_AXIS] = current_position[Y_AXIS] = 0;

        #ifdef DUAL_X_CARRIAGE
          int x_axis_home_dir = x_home_dir(active_extruder);
          extruder_duplication_enabled = false;
        #else
          int x_axis_home_dir = home_dir(X_AXIS);
        #endif

        sync_plan_position();

        float mlx = max_length(X_AXIS), mly = max_length(Y_AXIS),
              mlratio = mlx>mly ? mly/mlx : mlx/mly;

        destination[X_AXIS] = 1.5 * mlx * x_axis_home_dir;
        destination[Y_AXIS] = 1.5 * mly * home_dir(Y_AXIS);
        feedrate = min(homing_feedrate[X_AXIS], homing_feedrate[Y_AXIS]) * sqrt(mlratio * mlratio + 1);
        line_to_destination();
        st_synchronize();

        axis_is_at_home(X_AXIS);
        axis_is_at_home(Y_AXIS);
        sync_plan_position();

        destination[X_AXIS] = current_position[X_AXIS];
        destination[Y_AXIS] = current_position[Y_AXIS];
        line_to_destination();
        feedrate = 0.0;
        st_synchronize();
        endstops_hit_on_purpose(); // clear endstop hit flags

        current_position[X_AXIS] = destination[X_AXIS];
        current_position[Y_AXIS] = destination[Y_AXIS];
        #ifndef SCARA
          current_position[Z_AXIS] = destination[Z_AXIS];
        #endif
      }

    #endif // QUICK_HOME

    // Home X
    if (home_all_axis || homeX) {
      #ifdef DUAL_X_CARRIAGE
        int tmp_extruder = active_extruder;
        extruder_duplication_enabled = false;
        active_extruder = !active_extruder;
        HOMEAXIS(X);
        inactive_extruder_x_pos = current_position[X_AXIS];
        active_extruder = tmp_extruder;
        HOMEAXIS(X);
        // reset state used by the different modes
        memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
        delayed_move_time = 0;
        active_extruder_parked = true;
      #else
        HOMEAXIS(X);
      #endif
    }

    // Home Y
    if (home_all_axis || homeY) HOMEAXIS(Y);

    // Set the X position, if included
    if (code_seen(axis_codes[X_AXIS]) && code_has_value())
      current_position[X_AXIS] = code_value();

    // Set the Y position, if included
    if (code_seen(axis_codes[Y_AXIS]) && code_has_value())
      current_position[Y_AXIS] = code_value();

    // Home Z last if homing towards the bed
    #if Z_HOME_DIR < 0
      #ifndef Z_SAFE_HOMING
        if (code_seen('M') && !(homeX || homeY)) {
          // Manual G28 bed level
          #ifdef ULTIPANEL
            ECHO_LM(OK, " --LEVEL PLATE SCRIPT--");
            set_ChangeScreen(true);
            while(!lcd_clicked()) {
              set_pageShowInfo(0);
              lcd_update();
            }
            saved_feedrate = feedrate;
            saved_feedrate_multiplier = feedrate_multiplier;
            feedrate_multiplier = 100;
            refresh_cmd_timeout();

            enable_endstops(true);
            for(int8_t i=0; i < NUM_AXIS; i++) {
              destination[i] = current_position[i];
            }
            feedrate = 0.0;
            #if Z_HOME_DIR > 0  // If homing away from BED do Z first
              HOMEAXIS(Z);
            #endif
            HOMEAXIS(X);
            HOMEAXIS(Y);
            #if Z_HOME_DIR < 0
              HOMEAXIS(Z);
            #endif
            sync_plan_position();

            #ifdef ENDSTOPS_ONLY_FOR_HOMING
              enable_endstops(false);
            #endif

            feedrate = saved_feedrate;
            feedrate_multiplier = saved_feedrate_multiplier;
            refresh_cmd_timeout();
            endstops_hit_on_purpose(); // clear endstop hit flags

            sync_plan_position();

            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], Z_MIN_POS + 5);

            // PROBE FIRST POINT
            set_pageShowInfo(1);
            set_ChangeScreen(true);
            do_blocking_move_to(LEFT_PROBE_BED_POSITION, FRONT_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], Z_MIN_POS);
            while(!lcd_clicked()) {          
              manage_heater();
              manage_inactivity();
            }

            // PROBE SECOND POINT
            set_ChangeScreen(true);
            set_pageShowInfo(2);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS + 5);
            do_blocking_move_to(RIGHT_PROBE_BED_POSITION, FRONT_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS);
            while(!lcd_clicked()) {
              manage_heater();
              manage_inactivity();
            }

            // PROBE THIRD POINT
            set_ChangeScreen(true);
            set_pageShowInfo(3);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS + 5);
            do_blocking_move_to(RIGHT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS);
            while(!lcd_clicked()) {
              manage_heater();
              manage_inactivity();
            }     

            // PROBE FOURTH POINT
            set_ChangeScreen(true);
            set_pageShowInfo(4);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS + 5);
            do_blocking_move_to(LEFT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS);
            while(!lcd_clicked()) {
              manage_heater();
              manage_inactivity();
            }

            // PROBE CENTER
            set_ChangeScreen(true);
            set_pageShowInfo(5);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS + 5);
            do_blocking_move_to((X_MAX_POS-X_MIN_POS)/2, (Y_MAX_POS-Y_MIN_POS)/2, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS);
            while(!lcd_clicked()) {
              manage_heater();
              manage_inactivity();
            }

            // FINISH MANUAL BED LEVEL
            set_ChangeScreen(true);
            set_pageShowInfo(6);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS + 5);
            enqueuecommands_P(PSTR("G28 X0 Y0\nG4 P0\nG4 P0\nG4 P0"));
          #endif // ULTIPANEL
        }
        else if(home_all_axis || homeZ) HOMEAXIS(Z);
      #elif defined(Z_SAFE_HOMING) && defined(ENABLE_AUTO_BED_LEVELING)// Z Safe mode activated.
        if (home_all_axis) {
          destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - X_PROBE_OFFSET_FROM_EXTRUDER);
          destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - Y_PROBE_OFFSET_FROM_EXTRUDER);
          destination[Z_AXIS] = -Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS);    // Set destination away from bed
          feedrate = xy_travel_speed;
          current_position[Z_AXIS] = 0;

          sync_plan_position();
          line_to_destination();
          st_synchronize();
          current_position[X_AXIS] = destination[X_AXIS];
          current_position[Y_AXIS] = destination[Y_AXIS];
          HOMEAXIS(Z);
        }
        // Let's see if X and Y are homed and probe is inside bed area.
        if (homeZ) {
          if (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) {
            float cpx = current_position[X_AXIS], cpy = current_position[Y_AXIS];
            if (   cpx >= X_MIN_POS - X_PROBE_OFFSET_FROM_EXTRUDER
                && cpx <= X_MAX_POS - X_PROBE_OFFSET_FROM_EXTRUDER
                && cpy >= Y_MIN_POS - Y_PROBE_OFFSET_FROM_EXTRUDER
                && cpy <= Y_MAX_POS - Y_PROBE_OFFSET_FROM_EXTRUDER) {
              current_position[Z_AXIS] = 0;
              plan_set_position(cpx, cpy, 0, current_position[E_AXIS]);
              destination[Z_AXIS] = -Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS);    // Set destination away from bed
              feedrate = max_feedrate[Z_AXIS] * 60;
              line_to_destination();
              st_synchronize();
              HOMEAXIS(Z);
            }
            else {
              LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
              ECHO_LM(OK, MSG_ZPROBE_OUT);
            }
          }
          else {
            LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
            ECHO_LM(OK, MSG_POSITION_UNKNOWN);
          }
        }
      #elif defined(Z_SAFE_HOMING)
        if(home_all_axis || homeZ) {
          destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT);
          destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT);
          feedrate = xy_travel_speed;
          destination[Z_AXIS] = current_position[Z_AXIS] = 0;
          sync_plan_position();
          line_to_destination();
          st_synchronize();
          current_position[X_AXIS] = destination[X_AXIS];
          current_position[Y_AXIS] = destination[Y_AXIS];

          HOMEAXIS(Z);
        }
      #endif //Z_SAFE_HOMING
    #endif //Z_HOME_DIR < 0

    // Set the Z position, if included
    if (code_seen(axis_codes[Z_AXIS]) && code_has_value())
      current_position[Z_AXIS] = code_value();

    sync_plan_position();

  #endif // else DELTA

  #ifdef SCARA
    sync_plan_position_delta();
  #endif

  #ifdef ENDSTOPS_ONLY_FOR_HOMING
    enable_endstops(false);
  #endif

  feedrate = saved_feedrate;
  feedrate_multiplier = saved_feedrate_multiplier;
  refresh_cmd_timeout();
  endstops_hit_on_purpose(); // clear endstop hit flags
}

#ifdef ENABLE_AUTO_BED_LEVELING

  /**
   * G29: Detailed Z-Probe, probes the bed at 3 or more points.
   *      Will fail if the printer has not been homed with G28.
   *
   * Enhanced G29 Auto Bed Leveling Probe Routine
   * 
   * Parameters With AUTO_BED_LEVELING_GRID:
   *
   *  P  Set the size of the grid that will be probed (P x P points).
   *     Example: "G29 P4"
   *
   *  S  Set the XY travel speed between probe points (in mm/min)
   *
   *  D  Dry-Run mode. Just evaluate the bed Topology - Don't apply
   *     or clean the rotation Matrix. Useful to check the topology
   *     after a first run of G29.
   *
   *  V  Set the verbose level (0-4). Example: "G29 V3"
   *
   *  T  Generate a Bed Topology Report. Example: "G29 P5 T" for a detailed report.
   *     This is useful for manual bed leveling and finding flaws in the bed (to
   *     assist with part placement).
   *
   *  F  Set the Front limit of the probing grid
   *  B  Set the Back limit of the probing grid
   *  L  Set the Left limit of the probing grid
   *  R  Set the Right limit of the probing grid
   *
   * Global Parameters:
   *
   * E/e By default G29 will engage the probe, test the bed, then disengage.
   *     Include "E" to engage/disengage the probe for each sample.
   *     There's no extra effect if you have a fixed probe.
   *     Usage: "G29 E" or "G29 e"
   *
   */
  inline void gcode_G29() {

    // Don't allow auto-leveling without homing first
    if (!axis_known_position[X_AXIS] || !axis_known_position[Y_AXIS]) {
      LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
      ECHO_LM(ER, MSG_POSITION_UNKNOWN);
      return;
    }

    int verbose_level = code_seen('V') || code_seen('v') ? code_value_short() : 1;
    if (verbose_level < 0 || verbose_level > 4) {
      ECHO_LM(ER,"?(V)erbose Level is implausible (0-4).");
      return;
    }

    bool dryrun = code_seen('D') || code_seen('d'),
         deploy_probe_for_each_reading = code_seen('E') || code_seen('e');

    #ifdef AUTO_BED_LEVELING_GRID

      bool do_topography_map = verbose_level > 2 || code_seen('T') || code_seen('t');

      if (verbose_level > 0) {
        ECHO_LM(DB, "G29 Auto Bed Leveling");
        if (dryrun) ECHO_LM(DB,"Running in DRY-RUN mode");
      }

      int auto_bed_leveling_grid_points = code_seen('P') ? code_value_long() : AUTO_BED_LEVELING_GRID_POINTS;
      if (auto_bed_leveling_grid_points < 2) {
        ECHO_LM(ER, "?Number of probed (P)oints is implausible (2 minimum).\n");
        return;
      }

      xy_travel_speed = code_seen('S') ? code_value_short() : XY_TRAVEL_SPEED;

      int left_probe_bed_position = code_seen('L') ? code_value_short() : LEFT_PROBE_BED_POSITION,
          right_probe_bed_position = code_seen('R') ? code_value_short() : RIGHT_PROBE_BED_POSITION,
          front_probe_bed_position = code_seen('F') ? code_value_short() : FRONT_PROBE_BED_POSITION,
          back_probe_bed_position = code_seen('B') ? code_value_short() : BACK_PROBE_BED_POSITION;

      bool left_out_l = left_probe_bed_position < MIN_PROBE_X,
           left_out = left_out_l || left_probe_bed_position > right_probe_bed_position - MIN_PROBE_EDGE,
           right_out_r = right_probe_bed_position > MAX_PROBE_X,
           right_out = right_out_r || right_probe_bed_position < left_probe_bed_position + MIN_PROBE_EDGE,
           front_out_f = front_probe_bed_position < MIN_PROBE_Y,
           front_out = front_out_f || front_probe_bed_position > back_probe_bed_position - MIN_PROBE_EDGE,
           back_out_b = back_probe_bed_position > MAX_PROBE_Y,
           back_out = back_out_b || back_probe_bed_position < front_probe_bed_position + MIN_PROBE_EDGE;

      if (left_out || right_out || front_out || back_out) {
        if (left_out) {
          ECHO_LM(ER, "?Probe (L)eft position out of range.\n");
          left_probe_bed_position = left_out_l ? MIN_PROBE_X : right_probe_bed_position - MIN_PROBE_EDGE;
        }
        if (right_out) {
          ECHO_LM(ER, "?Probe (R)ight position out of range.\n");
          right_probe_bed_position = right_out_r ? MAX_PROBE_X : left_probe_bed_position + MIN_PROBE_EDGE;
        }
        if (front_out) {
          ECHO_LM(ER, "?Probe (F)ront position out of range.\n");
          front_probe_bed_position = front_out_f ? MIN_PROBE_Y : back_probe_bed_position - MIN_PROBE_EDGE;
        }
        if (back_out) {
          ECHO_LM(ER, "?Probe (B)ack position out of range.\n");
          back_probe_bed_position = back_out_b ? MAX_PROBE_Y : front_probe_bed_position + MIN_PROBE_EDGE;
        }
        return;
      }

    #endif // AUTO_BED_LEVELING_GRID

    #ifdef Z_PROBE_SLED
      dock_sled(false); // engage (un-dock) the probe
    #endif

    st_synchronize();

    if (!dryrun) {
      // make sure the bed_level_rotation_matrix is identity or the planner will get it wrong
      plan_bed_level_matrix.set_to_identity();

      //vector_3 corrected_position = plan_get_position_mm();
      //corrected_position.debug("position before G29");
      vector_3 uncorrected_position = plan_get_position();
      //uncorrected_position.debug("position during G29");
      current_position[X_AXIS] = uncorrected_position.x;
      current_position[Y_AXIS] = uncorrected_position.y;
      current_position[Z_AXIS] = uncorrected_position.z;
      sync_plan_position();
    }

    setup_for_endstop_move();
    feedrate = homing_feedrate[Z_AXIS];

    #ifdef AUTO_BED_LEVELING_GRID

      // probe at the points of a lattice grid
      const int xGridSpacing = (right_probe_bed_position - left_probe_bed_position) / (auto_bed_leveling_grid_points - 1),
                yGridSpacing = (back_probe_bed_position - front_probe_bed_position) / (auto_bed_leveling_grid_points - 1);

      // solve the plane equation ax + by + d = z
      // A is the matrix with rows [x y 1] for all the probed points
      // B is the vector of the Z positions
      // the normal vector to the plane is formed by the coefficients of the plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
      // so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z

      int abl2 = auto_bed_leveling_grid_points * auto_bed_leveling_grid_points;

      double eqnAMatrix[abl2 * 3], // "A" matrix of the linear system of equations
             eqnBVector[abl2],     // "B" vector of Z points
             mean = 0.0;

      int probePointCounter = 0;
      bool zig = true;

      for (int yCount = 0; yCount < auto_bed_leveling_grid_points; yCount++) {
        double yProbe = front_probe_bed_position + yGridSpacing * yCount;
        int xStart, xStop, xInc;

        if (zig) {
          xStart = 0;
          xStop = auto_bed_leveling_grid_points;
          xInc = 1;
        }
        else {
          xStart = auto_bed_leveling_grid_points - 1;
          xStop = -1;
          xInc = -1;
        }


        // If topo_flag is set then don't zig-zag. Just scan in one direction.
        // This gets the probe points in more readable order.
        if (!do_topography_map) zig = !zig;
        for (int xCount = xStart; xCount != xStop; xCount += xInc) {
          double xProbe = left_probe_bed_position + xGridSpacing * xCount;

          // raise extruder
          float measured_z,
                z_before = probePointCounter ? Z_RAISE_BETWEEN_PROBINGS + current_position[Z_AXIS] : Z_RAISE_BEFORE_PROBING;

          // Enhanced G29 - Do not retract servo between probes
          ProbeAction act;
          if (deploy_probe_for_each_reading) // G29 E - Stow between probes
            act = ProbeDeployAndStow;
          else if (yCount == 0 && xCount == xStart)
            act = ProbeDeploy;
          else if (yCount == auto_bed_leveling_grid_points - 1 && xCount == xStop - xInc)
            act = ProbeStow;
          else
            act = ProbeStay;

          measured_z = probe_pt(xProbe, yProbe, z_before, act, verbose_level);

          mean += measured_z;

          eqnBVector[probePointCounter] = measured_z;
          eqnAMatrix[probePointCounter + 0 * abl2] = xProbe;
          eqnAMatrix[probePointCounter + 1 * abl2] = yProbe;
          eqnAMatrix[probePointCounter + 2 * abl2] = 1;

          probePointCounter++;

          manage_heater();
          manage_inactivity();
          lcd_update();

        } //xProbe
      } //yProbe

      clean_up_after_endstop_move();

      // solve lsq problem
      double *plane_equation_coefficients = qr_solve(abl2, 3, eqnAMatrix, eqnBVector);

      mean /= abl2;

      if (verbose_level) {
        ECHO_SMV(DB, "Eqn coefficients: a: ", plane_equation_coefficients[0], 8);
        ECHO_MV(" b: ", plane_equation_coefficients[1], 8);
        ECHO_EMV(" d: ", plane_equation_coefficients[2], 8);
        if (verbose_level > 2) {
          ECHO_LMV(DB, "Mean of sampled points: ", mean, 8);
        }
      }

      if (do_topography_map) {

        ECHO_LM(DB, "\nBed Height Topography:");
        ECHO_LM(DB, "+-----------+");
        ECHO_LM(DB, "|...Back....|");
        ECHO_LM(DB, "|Left..Right|");
        ECHO_LM(DB, "|...Front...|");
        ECHO_LM(DB, "+-----------+");

        for (int yy = auto_bed_leveling_grid_points - 1; yy >= 0; yy--) {
          for (int xx = 0; xx < auto_bed_leveling_grid_points; xx++) {
            int ind = yy * auto_bed_leveling_grid_points + xx;
            float diff = eqnBVector[ind] - mean;
            if (diff >= 0.0) ECHO_M(" +");   // Include + for column alignment
            else ECHO_M(" ");
            ECHO_V(diff, 5);
          } // xx
          ECHO_E;
        } // yy
        ECHO_E;

      } //do_topography_map

      if (!dryrun) set_bed_level_equation_lsq(plane_equation_coefficients);
      free(plane_equation_coefficients);

    #else // !AUTO_BED_LEVELING_GRID

      // Actions for each probe
      ProbeAction p1, p2, p3;
      if (deploy_probe_for_each_reading)
        p1 = p2 = p3 = ProbeDeployAndStow;
      else
        p1 = ProbeDeploy, p2 = ProbeStay, p3 = ProbeStow;

      // Probe at 3 arbitrary points
      float z_at_pt_1 = probe_pt(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, Z_RAISE_BEFORE_PROBING, p1, verbose_level),
            z_at_pt_2 = probe_pt(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS, p2, verbose_level),
            z_at_pt_3 = probe_pt(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS, p3, verbose_level);
      clean_up_after_endstop_move();
      if (!dryrun) set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);

    #endif // !AUTO_BED_LEVELING_GRID

    if (verbose_level > 0)
      plan_bed_level_matrix.debug(" \n\nBed Level Correction Matrix:");

    if (!dryrun) {
      // Correct the Z height difference from z-probe position and hotend tip position.
      // The Z height on homing is measured by Z-Probe, but the probe is quite far from the hotend.
      // When the bed is uneven, this height must be corrected.
      float x_tmp = current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER,
            y_tmp = current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER,
            z_tmp = current_position[Z_AXIS],
            real_z = (float)st_get_position(Z_AXIS) / axis_steps_per_unit[Z_AXIS];  //get the real Z (since the auto bed leveling is already correcting the plane)

      apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);         //Apply the correction sending the probe offset
      current_position[Z_AXIS] = z_tmp - real_z + current_position[Z_AXIS];   //The difference is added to current position and sent to planner.
      sync_plan_position();
    }

    #ifdef Z_PROBE_SLED
      dock_sled(true, -SLED_DOCKING_OFFSET); // dock the probe, correcting for over-travel
    #endif
  }

  #ifndef Z_PROBE_SLED
    inline void gcode_G30() {
      deploy_z_probe(); // Engage Z Servo endstop if available
      st_synchronize();
      // TODO: make sure the bed_level_rotation_matrix is identity or the planner will get set incorectly
      setup_for_endstop_move();

      feedrate = homing_feedrate[Z_AXIS];

      run_z_probe();
      ECHO_SM(DB, "Bed");
      ECHO_M(" X: ", current_position[X_AXIS] + 0.0001);
      ECHO_M(" Y: ", current_position[Y_AXIS] + 0.0001);
      ECHO_EMV(" Z: ", current_position[Z_AXIS] + 0.0001);

      clean_up_after_endstop_move();
      stow_z_probe(); // Retract Z Servo endstop if available
    }
  #endif //Z_PROBE_SLED
#endif //ENABLE_AUTO_BED_LEVELING

#if defined(DELTA) && defined(Z_PROBE_ENDSTOP)

  /**
   * G29: Delta Z-Probe, probes the bed at more points.
   */
  inline void gcode_G29() {
    if (code_seen('D')) {
      ECHO_LM(DB, "Current bed level array values:");
      for (int y = 0; y < 7; y++) {
        ECHO_S(DB); 
        for (int x = 0; x < 7; x++) {
          ECHO_VM(bed_level[x][y], " ", 3);
        }
        ECHO_E;
      }
      return;
    }
    saved_feedrate = feedrate;
    saved_feedrate_multiplier = feedrate_multiplier;
    feedrate_multiplier = 100;

    deploy_z_probe();
    calibrate_print_surface(z_probe_offset[Z_AXIS] + (code_seen(axis_codes[Z_AXIS]) ? code_value() : 0.0));
    retract_z_probe();

    feedrate = saved_feedrate;
    feedrate_multiplier = saved_feedrate_multiplier;
    refresh_cmd_timeout();
    endstops_hit_on_purpose(); // clear endstop hit flags
  }

  // G30: Delta AutoCalibration
  inline void gcode_G30() {

    int iterations = 100; // Maximum number of iterations

    //Zero the bed level array
    for (int y = 0; y < 7; y++) {
      for (int x = 0; x < 7; x++) {
        bed_level[x][y] = 0.0;
      }
    }

    if (code_seen('C')) {
      //Show carriage positions 
      ECHO_LM(DB, "Carriage Positions for last scan: ");
      for(int8_t i=0; i < 7; i++) {
        ECHO_SMV(DB, "[", saved_positions[i][X_AXIS]);
        ECHO_MV(", ", saved_positions[i][Y_AXIS]);
        ECHO_MV(", ", saved_positions[i][Z_AXIS]);
        ECHO_EM("]");
      }
      return;
    }

    if (code_seen('F')) probing_feedrate = code_value_short();

    if (code_seen('I')) iterations = code_value_short();

    if (code_seen('X') and code_seen('Y')) {
      //Probe specified X,Y point
      float x = code_seen('X') ? code_value():0.00;
      float y = code_seen('Y') ? code_value():0.00;
      float probe_value;

      deploy_z_probe();
      probe_value = probe_bed(x, y);
      ECHO_SMV(BD, "Bed Z-Height at X:", x);
      ECHO_MV(" Y:", y);
      ECHO_EMV(" = ", probe_value, 4);

      ECHO_SMV(BD, "Carriage Positions: [", saved_position[X_AXIS]);
      ECHO_MV(", ", saved_position[Y_AXIS]);
      ECHO_MV(", ", saved_position[Z_AXIS]);
      ECHO_EM("]");
      retract_z_probe();
      return;
    }

    saved_feedrate = feedrate;
    saved_feedrate_multiplier = feedrate_multiplier;
    feedrate_multiplier = 100;

    if (code_seen('A')) {
      if (code_has_value()) ac_prec = (float)(code_value() / 2);
      delta_autocalibration(iterations);
      return;
    }

    home_delta_axis();
    deploy_z_probe(); 

    //Probe all points
    bed_probe_all();

    //Show calibration report      
    calibration_report();

    retract_z_probe();

    //Restore saved variables
    feedrate = saved_feedrate;
    feedrate_multiplier = saved_feedrate_multiplier;
  }
#endif // DELTA && Z_PROBE_ENDSTOP

// G60: Store in memory actual position
inline void gcode_G60() {
  memcpy(lastpos, current_position, sizeof(lastpos));
  //ECHO_SMV(DB, " Lastpos X: ", lastpos[X_AXIS]);
  //ECHO_MV(" Lastpos Y: ", lastpos[Y_AXIS]);
  //ECHO_MV(" Lastpos Z: ", lastpos[Z_AXIS]);
  //ECHO_EMV(" Lastpos E: ", lastpos[E_AXIS]);
}

// G61: move to X Y Z in memory
inline void gcode_G61() {
  for(int8_t i = 0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i])) {
      destination[i] = (float)code_value() + lastpos[i];
    }
    else {
      destination[i] = current_position[i];
    }
  }
  //ECHO_SMV(DB, " Move to X: ", destination[X_AXIS]);
  //ECHO_MV(" Move to Y: ", destination[Y_AXIS]);
  //ECHO_MV(" Move to Z: ", destination[Z_AXIS]);
  //ECHO_EMV(" Move to E: ", destination[E_AXIS]);

  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
  //finish moves
  prepare_move();
}

/**
 * G92: Set current position to given X Y Z E
 */
inline void gcode_G92() {
  if (!code_seen(axis_codes[E_AXIS]))
    st_synchronize();

  bool didXYZ = false;
  for (int i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      float v = current_position[i] = code_value();
      if (i == E_AXIS)
        plan_set_e_position(v);
      else
        didXYZ = true;
    }
  }
  if (didXYZ) sync_plan_position();
}

#ifdef ULTIPANEL

  /**
   * M0: // M0 - Unconditional stop - Wait for user button press on LCD
   * M1: // M1 - Conditional stop - Wait for user button press on LCD
   */
  inline void gcode_M0_M1() {
    char *src = strchr_pointer + 2;

    millis_t codenum = 0;
    bool hasP = false, hasS = false;
    if (code_seen('P')) {
      codenum = code_value_short(); // milliseconds to wait
      hasP = codenum > 0;
    }
    if (code_seen('S')) {
      codenum = code_value_short() * 1000UL; // seconds to wait
      hasS = codenum > 0;
    }
    char* starpos = strchr(src, '*');
    if (starpos != NULL) *(starpos) = '\0';
    while (*src == ' ') ++src;
    if (!hasP && !hasS && *src != '\0')
      lcd_setstatus(src, true);
    else {
      LCD_MESSAGEPGM(MSG_USERWAIT);
      #if defined(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
        dontExpireStatus();
      #endif
    }

    lcd_ignore_click();
    st_synchronize();
    refresh_cmd_timeout();
    if (codenum > 0) {
      codenum += previous_cmd_ms;  // keep track of when we started waiting
      while(millis() < codenum && !lcd_clicked()) {
        manage_heater();
        manage_inactivity();
        lcd_update();
      }
      lcd_ignore_click(false);
    }
    else {
      if (!lcd_detected()) return;
      while (!lcd_clicked()) {
        manage_heater();
        manage_inactivity();
        lcd_update();
      }
    }
    if (IS_SD_PRINTING)
      LCD_MESSAGEPGM(MSG_RESUMING);
    else
      LCD_MESSAGEPGM(WELCOME_MSG);
  }
#endif //ULTIPANEL

#ifdef LASERBEAM
  /**
   * M3: S - Setting laser beam
   */
  inline void gcode_M3() {
    if (code_seen('S')) {
      laser_ttl_modulation = constrain(code_value(), 0, 255);
    }
    else {
      laser_ttl_modulation = 0;
    }
  }

  /**
   * M4: Turn on laser beam
   */
  inline void gcode_M4() {
    WRITE(LASER_PWR_PIN, HIGH);
    laser_ttl_modulation = 0;
  }

  /**
   * M5: Turn off laser beam
   */
  inline void gcode_M5() {
    WRITE(LASER_PWR_PIN, LOW);
    laser_ttl_modulation = 0;
  }
#endif //LASERBEAM

#ifdef FILAMENT_END_SWITCH
  /**
   * M11: Start printing
   */
  inline void gcode_M11() {
    printing = true;
    paused = false;
    ECHO_LM(OK, "Start Printing, pause pin active.");
    ECHO_S(RESUME);
    ECHO_E;
    #if HAS_POWER_CONSUMPTION_SENSOR
      startpower = power_consumption_hour;
    #endif
  }
#endif

/**
 * M17: Enable power on all stepper motors
 */
inline void gcode_M17() {
  LCD_MESSAGEPGM(MSG_NO_MOVE);
  enable_all_steppers();
}

#ifdef SDSUPPORT

  /**
   * M20: List SD card to serial output
   */
  inline void gcode_M20() {
    ECHO_EM(MSG_BEGIN_FILE_LIST);
    card.ls();
    ECHO_EM(MSG_END_FILE_LIST);
  }

  /**
   * M21: Init SD Card
   */
  inline void gcode_M21() {
    card.initsd();
  }

  /**
   * M22: Release SD Card
   */
  inline void gcode_M22() {
    card.release();
  }

  /**
   * M23: Select a file
   */
  inline void gcode_M23() {
    char* codepos = strchr_pointer + 4;
    char* starpos = strchr(codepos, '*');
    if (starpos) *starpos = '\0';
    card.openFile(codepos, true);
  }

  /**
   * M24: Start SD Print
   */
  inline void gcode_M24() {
    card.startFileprint();
    print_job_start_ms = millis();
    #if HAS_POWER_CONSUMPTION_SENSOR
      startpower = power_consumption_hour;
    #endif
  }

  /**
   * M25: Pause SD Print
   */
  inline void gcode_M25() {
    card.pauseSDPrint();
  }

  /**
   * M26: Set SD Card file index
   */
  inline void gcode_M26() {
    if (card.cardOK && code_seen('S'))
      card.setIndex(code_value_short());
  }

  /**
   * M27: Get SD Card status
   */
  inline void gcode_M27() {
    card.getStatus();
  }

  /**
   * M28: Start SD Write
   */
  inline void gcode_M28() {
    char* codepos = strchr_pointer + 4;
    char* starpos = strchr(codepos, '*');
    if (starpos) {
      char* npos = strchr(command_queue[cmd_queue_index_r], 'N');
      strchr_pointer = strchr(npos, ' ') + 1;
      *(starpos) = '\0';
    }
    card.openFile(codepos, false);
  }

  /**
   * M29: Stop SD Write
   * Processed in write to file routine above
   */
  inline void gcode_M29() {
    // card.saving = false;
  }

  /**
   * M30 <filename>: Delete SD Card file
   */
  inline void gcode_M30() {
    if (card.cardOK) {
      card.closeFile();
      char* starpos = strchr(strchr_pointer + 4, '*');
      if (starpos) {
        char* npos = strchr(command_queue[cmd_queue_index_r], 'N');
        strchr_pointer = strchr(npos, ' ') + 1;
        *(starpos) = '\0';
      }
      card.removeFile(strchr_pointer + 4);
    }
  }
#endif

/**
 * M31: Get the time since the start of SD Print (or last M109)
 */
inline void gcode_M31() {
  print_job_stop_ms = millis();
  millis_t t = (print_job_stop_ms - print_job_start_ms) / 1000;
  int min = t / 60, sec = t % 60;
  char time[30];
  sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
  ECHO_LV(OK, time);
  lcd_setstatus(time);
  autotempShutdown();
}

#ifdef SDSUPPORT

  /**
   * M32: Select file and start SD Print
   */
  inline void gcode_M32() {
    if (card.sdprinting)
      st_synchronize();

    char* codepos = strchr_pointer + 4;

    char* namestartpos = strchr(codepos, '!');   //find ! to indicate filename string start.
    if (! namestartpos)
      namestartpos = codepos; //default name position, 4 letters after the M
    else
      namestartpos++; //to skip the '!'

    char* starpos = strchr(codepos, '*');
    if (starpos) *(starpos) = '\0';

    bool call_procedure = code_seen('P') && (strchr_pointer < namestartpos);

    if (card.cardOK) {
      card.openFile(namestartpos, true, !call_procedure);

      if (code_seen('S') && strchr_pointer < namestartpos) // "S" (must occur _before_ the filename!)
        card.setIndex(code_value_short());

      card.startFileprint();
      if (!call_procedure) {
        print_job_start_ms = millis(); //procedure calls count as normal print time.
        #if HAS_POWER_CONSUMPTION_SENSOR
          startpower = power_consumption_hour;
        #endif
      }
    }
  }

  /**
   * M928: Start SD Write
   */
  inline void gcode_M928() {
    char* starpos = strchr(strchr_pointer + 5, '*');
    if (starpos) {
      char* npos = strchr(command_queue[cmd_queue_index_r], 'N');
      strchr_pointer = strchr(npos, ' ') + 1;
      *(starpos) = '\0';
    }
    card.openLogFile(strchr_pointer + 5);
  }

#endif // SDSUPPORT

/**
 * M42: Change pin status via GCode
 */
inline void gcode_M42() {
  if (code_seen('S')) {
    int pin_status = code_value_short(),
        pin_number = LED_PIN;

    if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
      pin_number = code_value_short();

    for (int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins) / sizeof(*sensitive_pins)); i++) {
      if (sensitive_pins[i] == pin_number) {
        pin_number = -1;
        break;
      }
    }

    #if HAS_FAN
      if (pin_number == FAN_PIN) fanSpeed = pin_status;
    #endif

    if (pin_number > -1) {
      pinMode(pin_number, OUTPUT);
      digitalWrite(pin_number, pin_status);
      analogWrite(pin_number, pin_status);
    }
  } // code_seen('S')
}

#if defined(ENABLE_AUTO_BED_LEVELING) && defined(Z_PROBE_REPEATABILITY_TEST)
  /**
   * M49: Z-Probe repeatability measurement function.
   *
   * Usage:
   *   M49 <P#> <X#> <Y#> <V#> <E> <L#>
   *     P = Number of sampled points (4-50, default 10)
   *     X = Sample X position
   *     Y = Sample Y position
   *     V = Verbose level (0-4, default=1)
   *     E = Engage probe for each reading
   *     L = Number of legs of movement before probe
   *  
   * This function assumes the bed has been homed.  Specifically, that a G28 command
   * as been issued prior to invoking the M48 Z-Probe repeatability measurement function.
   * Any information generated by a prior G29 Bed leveling command will be lost and need to be
   * regenerated.
   */
  inline void gcode_M49() {

    double sum = 0.0, mean = 0.0, sigma = 0.0, sample_set[50];
    uint8_t verbose_level = 1, n_samples = 10, n_legs = 0;

    if (code_seen('V') || code_seen('v')) {
      verbose_level = code_value_short();
      if (verbose_level < 0 || verbose_level > 4 ) {
        ECHO_LM(ER,"?Verbose Level not plausible (0-4).");
        return;
      }
    }

    if (code_seen('P') || code_seen('p')) {
      n_samples = code_value_short();
      if (n_samples < 4 || n_samples > 50) {
        ECHO_LM(ER, "?Sample size not plausible (4-50).");
        return;
      }
    }
    
    if (verbose_level > 0) ECHO_LM(DB, "M49 Z-Probe Repeatability test");

    double X_probe_location, Y_probe_location,
           X_current = X_probe_location = st_get_position_mm(X_AXIS),
           Y_current = Y_probe_location = st_get_position_mm(Y_AXIS),
           Z_current = st_get_position_mm(Z_AXIS),
           Z_start_location = Z_current + Z_RAISE_BEFORE_PROBING,
           ext_position = st_get_position_mm(E_AXIS);

    bool deploy_probe_for_each_reading = code_seen('E') || code_seen('e');

    if (code_seen('X') || code_seen('x')) {
      X_probe_location = code_value() - X_PROBE_OFFSET_FROM_EXTRUDER;
      if (X_probe_location < X_MIN_POS || X_probe_location > X_MAX_POS) {
        ECHO_LM(ER, "?X position out of range.");
        return;
      }
    }

    if (code_seen('Y') || code_seen('y')) {
      Y_probe_location = code_value() -  Y_PROBE_OFFSET_FROM_EXTRUDER;
      if (Y_probe_location < Y_MIN_POS || Y_probe_location > Y_MAX_POS) {
        ECHO_LM(ER, "?Y position out of range.");
        return;
      }
    }

    if (code_seen('L') || code_seen('l')) {
      n_legs = code_value_short();
      if (n_legs == 1) n_legs = 2;
      if (n_legs < 0 || n_legs > 15) {
        ECHO_LM(ER, "?Number of legs in movement not plausible (0-15).");
        return;
      }
    }

    //
    // Do all the preliminary setup work. First raise the probe.
    //

    st_synchronize();
    plan_bed_level_matrix.set_to_identity();
    plan_buffer_line(X_current, Y_current, Z_start_location, ext_position, homing_feedrate[Z_AXIS]/60, active_extruder, active_driver);
    st_synchronize();

    //
    // Now get everything to the specified probe point So we can safely do a probe to
    // get us close to the bed.  If the Z-Axis is far from the bed, we don't want to 
    // use that as a starting point for each probe.
    //
    if (verbose_level > 2)
      ECHO_LM(DB, "Positioning the probe...");

    plan_buffer_line(X_probe_location, Y_probe_location, Z_start_location, ext_position, homing_feedrate[X_AXIS]/60, active_extruder, active_driver);
    st_synchronize();

    current_position[X_AXIS] = X_current = st_get_position_mm(X_AXIS);
    current_position[Y_AXIS] = Y_current = st_get_position_mm(Y_AXIS);
    current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);
    current_position[E_AXIS] = ext_position = st_get_position_mm(E_AXIS);

    // 
    // OK, do the initial probe to get us close to the bed.
    // Then retrace the right amount and use that in subsequent probes
    //

    deploy_z_probe();

    setup_for_endstop_move();
    run_z_probe();

    current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);
    Z_start_location = st_get_position_mm(Z_AXIS) + Z_RAISE_BEFORE_PROBING;

    plan_buffer_line(X_probe_location, Y_probe_location, Z_start_location, ext_position, homing_feedrate[X_AXIS]/60, active_extruder, active_driver);
    st_synchronize();
    current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);

    if (deploy_probe_for_each_reading) stow_z_probe();

    for (uint8_t n=0; n < n_samples; n++) {
      // Make sure we are at the probe location
      do_blocking_move_to(X_probe_location, Y_probe_location, Z_start_location); // this also updates current_position

      if (n_legs) {
        millis_t ms = millis();
        double radius = ms % (X_MAX_LENGTH / 4),       // limit how far out to go
               theta = RADIANS(ms % 360L);
        float dir = (ms & 0x0001) ? 1 : -1;            // clockwise or counter clockwise

        //ECHO_SMV(DB, "starting radius: ",radius);
        //ECHO_MV("   theta: ",theta);
        //ECHO_EMV("   direction: ",dir);

        for (uint8_t l = 0; l < n_legs - 1; l++) {
          ms = millis();
          theta += RADIANS(dir * (ms % 20L));
          radius += (ms % 10L) - 5L;
          if (radius < 0.0) radius = -radius;

          X_current = X_probe_location + cos(theta) * radius;
          Y_current = Y_probe_location + sin(theta) * radius;
          X_current = constrain(X_current, X_MIN_POS, X_MAX_POS);
          Y_current = constrain(Y_current, Y_MIN_POS, Y_MAX_POS);

          if (verbose_level > 3) {
            ECHO_SMV(DB, "x: ", X_current);
            ECHO_EMV("y: ", Y_current);
          }

          do_blocking_move_to(X_current, Y_current, Z_current); // this also updates current_position

        } // n_legs loop

        // Go back to the probe location
        do_blocking_move_to(X_probe_location, Y_probe_location, Z_start_location); // this also updates current_position

      } // n_legs

      if (deploy_probe_for_each_reading) {
        deploy_z_probe(); 
        delay(1000);
      }

      setup_for_endstop_move();
      run_z_probe();

      sample_set[n] = current_position[Z_AXIS];

      //
      // Get the current mean for the data points we have so far
      //
      sum = 0.0;
      for (uint8_t j = 0; j <= n; j++) sum += sample_set[j];
      mean = sum / (n + 1);

      //
      // Now, use that mean to calculate the standard deviation for the
      // data points we have so far
      //
      sum = 0.0;
      for (uint8_t j = 0; j <= n; j++) {
        float ss = sample_set[j] - mean;
        sum += ss * ss;
      }
      sigma = sqrt(sum / (n + 1));

      if (verbose_level > 1) {
        ECHO_SV(n+1);
        ECHO_MV(" of ", n_samples);
        ECHO_MV("   z: ", current_position[Z_AXIS], 6);
        if (verbose_level > 2) {
          ECHO_MV(" mean: ", mean,6);
          ECHO_MV("   sigma: ", sigma,6);
        }
        ECHO_E;
      }

      plan_buffer_line(X_probe_location, Y_probe_location, Z_start_location, current_position[E_AXIS], homing_feedrate[Z_AXIS]/60, active_extruder, active_driver);
      st_synchronize();

      if (deploy_probe_for_each_reading) {
        stow_z_probe();
        delay(1000);
      }
    }

    if (!deploy_probe_for_each_reading) {
      stow_z_probe();
      delay(1000);
    }

    clean_up_after_endstop_move();

    // enable_endstops(true);

    if (verbose_level > 0) {
      ECHO_LMV("Mean: ",mean, 6);
    }

    ECHO_LMV("Standard Deviation: ", sigma, 6);
  }

#endif // ENABLE_AUTO_BED_LEVELING && Z_PROBE_REPEATABILITY_TEST

#if HAS_POWER_SWITCH

  /**
   * M80: Turn on Power Supply
   */
  inline void gcode_M80() {
    OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE); //GND

    // If you have a switch on suicide pin, this is useful
    // if you want to start another print with suicide feature after
    // a print without suicide...
    #if HAS_SUICIDE
      OUT_WRITE(SUICIDE_PIN, HIGH);
    #endif

    #ifdef ULTIPANEL
      powersupply = true;
      LCD_MESSAGEPGM(WELCOME_MSG);
      lcd_update();
    #endif
  }
#endif // HAS_POWER_SWITCH

/**
 * M81: Turn off Power, including Power Supply, if there is one.
 *
 *      This code should ALWAYS be available for EMERGENCY SHUTDOWN!
 */
inline void gcode_M81() {
  disable_all_heaters();
  st_synchronize();
  disable_e();
  finishAndDisableSteppers();
  fanSpeed = 0;
  delay(1000); // Wait 1 second before switching off
  #if HAS_SUICIDE
    st_synchronize();
    suicide();
  #elif HAS_POWER_SWITCH
    OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
  #endif
  #ifdef ULTIPANEL
    #if HAS_POWER_SWITCH
      powersupply = false;
    #endif
    LCD_MESSAGEPGM(MACHINE_NAME " " MSG_OFF ".");
    lcd_update();
  #endif
}

/**
 * M82: Set E codes absolute (default)
 */
inline void gcode_M82() { axis_relative_modes[E_AXIS] = false; }

/**
 * M83: Set E codes relative while in Absolute Coordinates (G90) mode
 */
inline void gcode_M83() { axis_relative_modes[E_AXIS] = true; }

/**
 * M18, M84: Disable all stepper motors
 */
inline void gcode_M18_M84() {
  if (code_seen('S')) {
    stepper_inactive_time = code_value() * 1000;
  }
  else {
    bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
    if (all_axis) {
      st_synchronize();
      disable_e();
      finishAndDisableSteppers();
    }
    else {
      st_synchronize();
      if (code_seen('X')) disable_x();
      if (code_seen('Y')) disable_y();
      if (code_seen('Z')) disable_z();
      #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
        if (code_seen('E')) {
          disable_e();
        }
      #endif
    }
  }
}

/**
 * M85: Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 */
inline void gcode_M85() {
  if (code_seen('S')) max_inactive_time = code_value() * 1000;
}

/**
 * M92: Set axis_steps_per_unit - same syntax as G92
 */
inline void gcode_M92() {
  for(int8_t i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      if (i == E_AXIS) {
        float value = code_value();
        if (value < 20.0) {
          float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
          max_e_jerk *= factor;
          max_feedrate[i] *= factor;
          axis_steps_per_sqr_second[i] *= factor;
        }
        axis_steps_per_unit[i] = value;
      }
      else {
        axis_steps_per_unit[i] = code_value();
      }
    }
  }
}

/**
 * M104: Set hot end temperature
 */
inline void gcode_M104() {
  if (setTargetedHotend(104)) return;
  if (debugDryrun()) return;
  #if HOTENDS == 1
    if (target_extruder != active_extruder) return;
  #endif
  if (code_seen('S')) {
    float temp = code_value();
    setTargetHotend(temp, target_extruder);
    #ifdef DUAL_X_CARRIAGE
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && target_extruder == 0)
        setTargetHotend1(temp == 0.0 ? 0.0 : temp + duplicate_extruder_temp_offset);
    #endif
    setWatch();
  }
}

/**
 * M105: Read hot end and bed temperature
 */
inline void gcode_M105() {
  if (setTargetedHotend(105)) return;

  #if HAS_TEMP_0 || HAS_TEMP_BED
    ECHO_S(OK);
    #if HAS_TEMP_0
      ECHO_MV(" T:", degHotend(target_extruder), 1);
      ECHO_MV(" /", degTargetHotend(target_extruder), 1);
    #endif
    #if HAS_TEMP_BED
      ECHO_MV(" B:", degBed(), 1);
      ECHO_MV(" /", degTargetBed(), 1);
    #endif
    for (int8_t e = 0; e < EXTRUDERS; ++e) {
      ECHO_MV(" T", e);
      ECHO_MV(":", degHotend(e), 1);
      ECHO_MV(" /", degTargetHotend(e), 1);
    }
  #else // !HAS_TEMP_0 && !HAS_TEMP_BED
    ECHO_LM(ER, MSG_ERR_NO_THERMISTORS);
    return;
  #endif

  ECHO_M(" @:");
  #ifdef HOTEND_WATTS
    ECHO_VM((HOTEND_WATTS * getHeaterPower(target_extruder))/127, "W");
  #else
    ECHO_V(getHeaterPower(target_extruder));
  #endif

  ECHO_M(" B@:");
  #ifdef BED_WATTS
    ECHO_VM((BED_WATTS * getHeaterPower(-1))/127, "W");
  #else
    ECHO_V(getHeaterPower(-1));
  #endif

  #ifdef SHOW_TEMP_ADC_VALUES
    #if HAS_TEMP_BED
      ECHO_MV("    ADC B:", degBed(),1);
      ECHO_MV("C->", rawBedTemp()/OVERSAMPLENR,0);
    #endif
    for (int8_t cur_extruder = 0; cur_extruder < HOTENDS; ++cur_extruder) {
      ECHO_MV("  T", cur_extruder);
      ECHO_MV(":", degHotend(cur_extruder),1);
      ECHO_MV("C->", rawHotendTemp(cur_extruder)/OVERSAMPLENR,0);
    }
  #endif
  ECHO_E;
}

#if HAS_FAN
  /**
   * M106: Set Fan Speed
   */
  inline void gcode_M106() { fanSpeed = code_seen('S') ? constrain(code_value_short(), 0, 255) : 255; }

  /**
   * M107: Fan Off
   */
  inline void gcode_M107() { fanSpeed = 0; }

#endif // HAS_FAN

/**
 * M109: Wait for extruder(s) to reach temperature
 */
inline void gcode_M109() {
  if (setTargetedHotend(109)) return;
  if (debugDryrun()) return;
  #if HOTENDS == 1
    if (target_extruder != active_extruder) return;
  #endif

  LCD_MESSAGEPGM(MSG_HEATING);

  no_wait_for_cooling = code_seen('S');
  if (no_wait_for_cooling || code_seen('R')) {
    float temp = code_value();
    setTargetHotend(temp, target_extruder);
    #ifdef DUAL_X_CARRIAGE
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && target_extruder == 0)
        setTargetHotend1(temp == 0.0 ? 0.0 : temp + duplicate_extruder_temp_offset);
    #endif
  }

  #ifdef AUTOTEMP
    autotemp_enabled = code_seen('F');
    if (autotemp_enabled) autotemp_factor = code_value();
    if (code_seen('S')) autotemp_min = code_value();
    if (code_seen('B')) autotemp_max = code_value();
  #endif

  wait_heater();
}

/**
 * M111: Debug mode Repetier Host compatibile
 */
inline void gcode_M111() {
  if (code_seen('S')) debugLevel = code_value_short();
  if (debugDryrun()) {
    ECHO_LM(OK, MSG_DRYRUN_ENABLED);
    setTargetBed(0);
    for (int8_t cur_hotend = 0; cur_hotend < HOTENDS; ++cur_hotend) {
      setTargetHotend(0, cur_hotend);
    }
  }
}

/**
 * M112: Emergency Stop
 */
inline void gcode_M112() {
  kill();
}

/**
 * M114: Output current position to serial port
 */
inline void gcode_M114() {
  ECHO_SMV(OK, "X:", current_position[X_AXIS]);
  ECHO_MV(" Y:", current_position[Y_AXIS]);
  ECHO_MV(" Z:", current_position[Z_AXIS]);
  ECHO_MV(" E:", current_position[E_AXIS]);

  ECHO_MV(MSG_COUNT_X, float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
  ECHO_MV(" Y:", float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
  ECHO_EMV(" Z:", float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

  #ifdef SCARA
    ECHO_SMV(OK, "SCARA Theta:", delta[X_AXIS]);
    ECHO_EMV("   Psi+Theta:", delta[Y_AXIS]);
    
    ECHO_SMV(OK, "SCARA Cal - Theta:", delta[X_AXIS]+home_offset[X_AXIS]);
    ECHO_EMV("   Psi+Theta (90):", delta[Y_AXIS]-delta[X_AXIS]-90+home_offset[Y_AXIS]);
    
    ECHO_SMV(OK, "SCARA step Cal - Theta:", delta[X_AXIS]/90*axis_steps_per_unit[X_AXIS]);
    ECHO_EMV("   Psi+Theta:", (delta[Y_AXIS]-delta[X_AXIS])/90*axis_steps_per_unit[Y_AXIS]);
  #endif
}

/**
 * M115: Capabilities string
 */
inline void gcode_M115() {
  ECHO_M(MSG_M115_REPORT);
}

/**
 * M117: Set LCD Status Message
 */
inline void gcode_M117() {
  char* codepos = strchr_pointer + 5;
  char* starpos = strchr(codepos, '*');
  if (starpos) *starpos = '\0';
  lcd_setstatus(codepos);
}

/**
 * M119: Output endstop states to serial output
 */
inline void gcode_M119() {
  ECHO_LV(OK, MSG_M119_REPORT);
  #if HAS_X_MIN
    ECHO_MV(MSG_X_MIN, ((READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_X_MAX
    ECHO_MV(MSG_X_MAX, ((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y_MIN
    ECHO_MV(MSG_Y_MIN, ((READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y_MAX
    ECHO_MV(MSG_Y_MAX, ((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_MIN
    ECHO_MV(MSG_Z_MIN, ((READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_MAX
    ECHO_MV(MSG_Z_MAX, ((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z2_MAX
    ECHO_MV(MSG_Z2_MAX, ((READ(Z2_MAX_PIN)^Z2_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_PROBE
    ECHO_MV(MSG_Z_PROBE, ((READ(Z_PROBE_PIN)^Z_PROBE_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_E_MIN
    ECHO_MV(MSG_E_MIN, ((READ(E_MIN_PIN)^E_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_FILRUNOUT
    ECHO_MV(MSG_FILRUNOUT_PIN, ((READ(FILRUNOUT_PIN)^FILRUNOUT_PIN_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  ECHO_E;
}

/**
 * M120: Enable endstops
 */
inline void gcode_M120() { enable_endstops(false); }

/**
 * M121: Disable endstops
 */
inline void gcode_M121() { enable_endstops(true); }

#ifdef BARICUDA
  #if HAS_HEATER_1
    /**
     * M126: Heater 1 valve open
     */
    inline void gcode_M126() { ValvePressure = code_seen('S') ? constrain(code_value(), 0, 255) : 255; }
    /**
     * M127: Heater 1 valve close
     */
    inline void gcode_M127() { ValvePressure = 0; }
  #endif

  #if HAS_HEATER_2
    /**
     * M128: Heater 2 valve open
     */
    inline void gcode_M128() { EtoPPressure = code_seen('S') ? constrain(code_value(), 0, 255) : 255; }
    /**
     * M129: Heater 2 valve close
     */
    inline void gcode_M129() { EtoPPressure = 0; }
  #endif

#endif //BARICUDA

/**
 * M140: Set bed temperature
 */
inline void gcode_M140() {
  if (debugDryrun()) return;
  if (code_seen('S')) setTargetBed(code_value());
}

#ifdef BLINKM
  /**
   * M150: Set Status LED Color - Use R-U-B for R-G-B
   */
  inline void gcode_M150() {
    SendColors(
      code_seen('R') ? (byte)code_value_short() : 0,
      code_seen('U') ? (byte)code_value_short() : 0,
      code_seen('B') ? (byte)code_value_short() : 0
    );
  }

#endif // BLINKM

#if HAS_TEMP_BED
  /**
   * M190: Sxxx Wait for bed current temp to reach target temp. Waits only when heating
   *       Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
   */
  inline void gcode_M190() {
    if (debugDryrun()) return;
    LCD_MESSAGEPGM(MSG_BED_HEATING);
    no_wait_for_cooling = code_seen('S');
    if (no_wait_for_cooling || code_seen('R'))
      setTargetBed(code_value());

    wait_bed();
  }
#endif // HAS_TEMP_BED

/**
 * M200: Set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
 *       T<extruder>
 *       D<millimeters>
 */
inline void gcode_M200() {
  int tmp_extruder = active_extruder;
  if (code_seen('T')) {
    tmp_extruder = code_value_short();
    if (tmp_extruder >= EXTRUDERS) {
      ECHO_LM(ER, MSG_M200_INVALID_EXTRUDER);
      return;
    }
  }

  if (code_seen('D')) {
    float diameter = code_value();
    // setting any extruder filament size disables volumetric on the assumption that
    // slicers either generate in extruder values as cubic mm or as as filament feeds
    // for all extruders
    volumetric_enabled = (diameter != 0.0);
    if (volumetric_enabled) {
      filament_size[tmp_extruder] = diameter;
      // make sure all extruders have some sane value for the filament size
      for (int i=0; i<EXTRUDERS; i++)
        if (! filament_size[i]) filament_size[i] = DEFAULT_NOMINAL_FILAMENT_DIA;
    }
  }
  else {
    //reserved for setting filament diameter via UFID or filament measuring device
    return;
  }
  calculate_volumetric_multipliers();
}

/**
 * M201: Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 */
inline void gcode_M201() {
  for (int8_t i=0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      max_acceleration_units_per_sq_second[i] = code_value();
    }
  }
  // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
  reset_acceleration_rates();
}

#if 0 // Not used for Sprinter/grbl gen6
  inline void gcode_M202() {
    for(int8_t i=0; i < NUM_AXIS; i++) {
      if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
    }
  }
#endif


/**
 * M203: Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
 */
inline void gcode_M203() {
  for (int8_t i=0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      max_feedrate[i] = code_value();
    }
  }
}

/**
 * M204: Set Accelerations in mm/sec^2 (M204 P1200 R3000 T3000)
 *
 *    P = Printing moves
 *    R = Retract only (no X, Y, Z) moves
 *    T = Travel (non printing) moves
 *
 *  Also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
 */
inline void gcode_M204() {
  if (code_seen('S')) {  // Kept for legacy compatibility. Should NOT BE USED for new developments.
    acceleration = code_value();
    travel_acceleration = acceleration;
    ECHO_LMV(OK, "Setting Print and Travel Acceleration: ", acceleration );
  }
  if (code_seen('P')) {
    acceleration = code_value();
    ECHO_LMV(OK, "Setting Print Acceleration: ", acceleration );
  }
  if (code_seen('R')) {
    retract_acceleration = code_value();
    ECHO_LMV(OK, "Setting Retract Acceleration: ", retract_acceleration );
  }
  if (code_seen('T')) {
    travel_acceleration = code_value();
    ECHO_LMV(OK, "Setting Travel Acceleration: ", travel_acceleration );
  }
}

/**
 * M205: Set Advanced Settings
 *
 *    S = Min Feed Rate (mm/s)
 *    T = Min Travel Feed Rate (mm/s)
 *    B = Min Segment Time (µs)
 *    X = Max XY Jerk (mm/s/s)
 *    Z = Max Z Jerk (mm/s/s)
 *    E = Max E Jerk (mm/s/s)
 */
inline void gcode_M205() {
  if (code_seen('S')) minimumfeedrate = code_value();
  if (code_seen('T')) mintravelfeedrate = code_value();
  if (code_seen('B')) minsegmenttime = code_value();
  if (code_seen('X')) max_xy_jerk = code_value();
  if (code_seen('Z')) max_z_jerk = code_value();
  if (code_seen('E')) max_e_jerk = code_value();
}

/**
 * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
 */
inline void gcode_M206() {
  for (int8_t i=X_AXIS; i <= Z_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      home_offset[i] = code_value();
    }
  }
  #ifdef SCARA
    if (code_seen('T')) home_offset[X_AXIS] = code_value(); // Theta
    if (code_seen('P')) home_offset[Y_AXIS] = code_value(); // Psi
  #endif
}

#ifdef FWRETRACT

  /**
   * M207: Set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop]
   */
  inline void gcode_M207() {
    if (code_seen('S')) retract_length = code_value();
    if (code_seen('F')) retract_feedrate = code_value() / 60;
    if (code_seen('Z')) retract_zlift = code_value();
  }

  /**
   * M208: Set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
   */
  inline void gcode_M208() {
    if (code_seen('S')) retract_recover_length = code_value();
    if (code_seen('F')) retract_recover_feedrate = code_value() / 60;
  }

  /**
   * M209: Enable automatic retract (M209 S1)
   *       detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
   */
  inline void gcode_M209() {
    if (code_seen('S')) {
      int t = code_value_short();
      switch(t) {
        case 0:
          autoretract_enabled = false;
          break;
        case 1:
          autoretract_enabled = true;
          break;
        default:
          ECHO_SM(ER, MSG_UNKNOWN_COMMAND);
          ECHO_EVM(command_queue[cmd_queue_index_r], "\"");
          return;
      }
      for (int i=0; i < EXTRUDERS; i++) retracted[i] = false;
    }
  }
#endif // FWRETRACT

#if HOTENDS > 1

  /**
   * M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
   */
  inline void gcode_M218() {
    if (setTargetedHotend(218)) return;

    if (code_seen('X')) hotend_offset[X_AXIS][target_extruder] = code_value();
    if (code_seen('Y')) hotend_offset[Y_AXIS][target_extruder] = code_value();

    #ifdef DUAL_X_CARRIAGE
      if (code_seen('Z')) hotend_offset[Z_AXIS][target_extruder] = code_value();
    #endif

    ECHO_SM(OK, MSG_HOTEND_OFFSET);
    for (int e = 0; e < EXTRUDERS; e++) {
      ECHO_MV(" ", hotend_offset[X_AXIS][e]);
      ECHO_MV(",", hotend_offset[Y_AXIS][e]);
      #ifdef DUAL_X_CARRIAGE
        ECHO_MV(",", hotend_offset[Z_AXIS][e]);
      #endif
    }
    ECHO_E;
  }
#endif //HOTENDS > 1

/**
 * M220: Set speed percentage factor, aka "Feed Rate" (M220 S95)
 */
inline void gcode_M220() {
  if (code_seen('S')) feedrate_multiplier = code_value();
}

/**
 * M221: Set extrusion percentage (M221 T0 S95)
 */
inline void gcode_M221() {
  if (code_seen('S')) {
    int sval = code_value();
    if (code_seen('T')) {
      if (setTargetedHotend(221)) return;
      extruder_multiply[target_extruder] = sval;
    }
    else {
      extruder_multiply[active_extruder] = sval;
    }
  }
}

/**
 * M226: Wait until the specified pin reaches the state required (M226 P<pin> S<state>)
 */
inline void gcode_M226() {
  if (code_seen('P')) {
    int pin_number = code_value();

    int pin_state = code_seen('S') ? code_value() : -1; // required pin state - default is inverted

    if (pin_state >= -1 && pin_state <= 1) {

      for (int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins)/sizeof(*sensitive_pins)); i++) {
        if (sensitive_pins[i] == pin_number) {
          pin_number = -1;
          break;
        }
      }

      if (pin_number > -1) {
        int target = LOW;

        st_synchronize();

        pinMode(pin_number, INPUT);

        switch(pin_state){
          case 1:
            target = HIGH;
            break;

          case 0:
            target = LOW;
            break;

          case -1:
            target = !digitalRead(pin_number);
            break;
        }

        while(digitalRead(pin_number) != target) {
          manage_heater();
          manage_inactivity();
          lcd_update();
        }

      } // pin_number > -1
    } // pin_state -1 0 1
  } // code_seen('P')
}

#if defined(CHDK) || HAS_PHOTOGRAPH
  /**
   * M240: Trigger a camera by emulating a Canon RC-1
   *       See http://www.doc-diy.net/photo/rc-1_hacked/
   */
  inline void gcode_M240() {
    #ifdef CHDK
       OUT_WRITE(CHDK, HIGH);
       chdkHigh = millis();
       chdkActive = true;
    #elif HAS_PHOTOGRAPH
      const uint8_t NUM_PULSES = 16;
      const float PULSE_LENGTH = 0.01524;
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
      }
      delay(7.33);
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
      }
    #endif // !CHDK && HAS_PHOTOGRAPH
  }
#endif // CHDK || PHOTOGRAPH_PIN

#if defined(DOGLCD) && LCD_CONTRAST >= 0
  /**
   * M250: Read and optionally set the LCD contrast
   */
  inline void gcode_M250() {
    if (code_seen('C')) lcd_setcontrast(code_value_short() & 0x3F);
    ECHO_LMV(OK, "lcd contrast value: ", lcd_contrast);
  }

#endif // DOGLCD

#if NUM_SERVOS > 0
  /**
   * M280: Set servo position absolute. P: servo index, S: angle or microseconds
   */
  inline void gcode_M280() {
    int servo_index = code_seen('P') ? code_value() : -1;
    int servo_position = 0;
    if (code_seen('S')) {
      servo_position = code_value();
      if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
        #if SERVO_LEVELING
          servo[servo_index].attach(0);
        #endif
        servo[servo_index].write(servo_position);
        #if SERVO_LEVELING
          delay(PROBE_SERVO_DEACTIVATION_DELAY);
          servo[servo_index].detach();
        #endif
      }
      else {
        ECHO_SM(ER, "Servo ");
        ECHO_EMV(servo_index, " out of range");
      }
    }
    else if (servo_index >= 0) {
      ECHO_S(OK);
      ECHO_MV("Servo ", servo_index);
      ECHO_EMV(": ", servo[servo_index].read());
    }
  }
#endif // NUM_SERVOS > 0

#if BEEPER > 0 || defined(ULTRALCD) || defined(LCD_USE_I2C_BUZZER)

  /**
   * M300: Play beep sound S<frequency Hz> P<duration ms>
   */
  inline void gcode_M300() {
    uint16_t beepS = code_seen('S') ? code_value_short() : 110;
    uint32_t beepP = code_seen('P') ? code_value_long() : 1000;
    if (beepS > 0) {
      #if BEEPER > 0
        tone(BEEPER, beepS);
        delay(beepP);
        noTone(BEEPER);
      #elif defined(ULTRALCD)
        lcd_buzz(beepS, beepP);
      #elif defined(LCD_USE_I2C_BUZZER)
        lcd_buzz(beepP, beepS);
      #endif
    }
    else {
      delay(beepP);
    }
  }

#endif // BEEPER>0 || ULTRALCD || LCD_USE_I2C_BUZZER

#ifdef PIDTEMP
  /**
   * M301: Set PID parameters P I D
   */
  inline void gcode_M301() {

    // multi-hotend PID patch: M301 updates or prints a single hotend's PID values
    // default behaviour (omitting E parameter) is to update for hotend 0 only
    int e = code_seen('E') ? code_value() : 0; // hotend being updated

    if (e < HOTENDS) { // catch bad input value
      if (code_seen('P')) PID_PARAM(Kp, e) = code_value();
      if (code_seen('I')) PID_PARAM(Ki, e) = scalePID_i(code_value());
      if (code_seen('D')) PID_PARAM(Kd, e) = scalePID_d(code_value());

      updatePID();
      ECHO_S(OK);
      ECHO_MV(" e:", e);
      ECHO_MV(" p:", PID_PARAM(Kp, e));
      ECHO_MV(" i:", unscalePID_i(PID_PARAM(Ki, e)));
      ECHO_EMV(" d:", unscalePID_d(PID_PARAM(Kd, e)));
    }
    else {
      ECHO_LM(ER, MSG_INVALID_EXTRUDER);
    }
  }
#endif // PIDTEMP

#ifdef PREVENT_DANGEROUS_EXTRUDE

  void set_extrude_min_temp(float temp) { extrude_min_temp = temp; }

  /**
   * M302: Allow cold extrudes, or set the minimum extrude S<temperature>.
   */
  inline void gcode_M302() {
    set_extrude_min_temp(code_seen('S') ? code_value() : 0);
  }

#endif // PREVENT_DANGEROUS_EXTRUDE

/**
 * M303: PID relay autotune
 *       S<temperature> sets the target temperature. (default target temperature = 150C)
 *       E<extruder> (-1 for the bed)
 *       C<cycles>
 */
inline void gcode_M303() {
  int e = code_seen('E') ? code_value_short() : 0;
  int c = code_seen('C') ? code_value_short() : 5;
  float temp = code_seen('S') ? code_value() : (e < 0 ? 70.0 : 150.0);
  PID_autotune(temp, e, c);
}

#ifdef PIDTEMPBED
  // M304: Set bed PID parameters P I and D
  inline void gcode_M304() {
    if (code_seen('P')) bedKp = code_value();
    if (code_seen('I')) bedKi = scalePID_i(code_value());
    if (code_seen('D')) bedKd = scalePID_d(code_value());

    updatePID();
    ECHO_S(OK);
    ECHO_MV(" p:", bedKp);
    ECHO_MV(" i:", unscalePID_i(bedKi));
    ECHO_EMV(" d:", unscalePID_d(bedKd));
  }
#endif // PIDTEMPBED

#if HAS_MICROSTEPS
  // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
  inline void gcode_M350() {
    if(code_seen('S')) for(int i=0;i<=4;i++) microstep_mode(i,code_value());
    for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
    if(code_seen('B')) microstep_mode(4,code_value());
    microstep_readings();
  }

  /**
   * M351: Toggle MS1 MS2 pins directly with axis codes X Y Z E B
   *       S# determines MS1 or MS2, X# sets the pin high/low.
   */
  inline void gcode_M351() {
    if (code_seen('S')) switch(code_value_short()) {
      case 1:
        for(int i=0;i<NUM_AXIS;i++) if (code_seen(axis_codes[i])) microstep_ms(i, code_value(), -1);
        if (code_seen('B')) microstep_ms(4, code_value(), -1);
        break;
      case 2:
        for(int i=0;i<NUM_AXIS;i++) if (code_seen(axis_codes[i])) microstep_ms(i, -1, code_value());
        if (code_seen('B')) microstep_ms(4, -1, code_value());
        break;
    }
    microstep_readings();
  }
#endif // HAS_MICROSTEPS

#ifdef SCARA
  bool SCARA_move_to_cal(uint8_t delta_x, uint8_t delta_y) {
    //SoftEndsEnabled = false;              // Ignore soft endstops during calibration
    //ECHO_LM(DB, " Soft endstops disabled ");
    if (IsRunning()) {
      //get_coordinates(); // For X Y Z E F
      delta[X_AXIS] = delta_x;
      delta[Y_AXIS] = delta_y;
      calculate_SCARA_forward_Transform(delta);
      destination[X_AXIS] = delta[X_AXIS]/axis_scaling[X_AXIS];
      destination[Y_AXIS] = delta[Y_AXIS]/axis_scaling[Y_AXIS];
      prepare_move();
      //ClearToSend();
      return true;
    }
    return false;
  }

  /**
   * M360: SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
   */
  inline bool gcode_M360() {
    ECHO_LM(DB, " Cal: Theta 0 ");
    return SCARA_move_to_cal(0, 120);
  }

  /**
   * M361: SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M361() {
    ECHO_LM(DB, " Cal: Theta 90 ");
    return SCARA_move_to_cal(90, 130);
  }

  /**
   * M362: SCARA calibration: Move to cal-position PsiA (0 deg calibration)
   */
  inline bool gcode_M362() {
    ECHO_LM(DB, " Cal: Psi 0 ");
    return SCARA_move_to_cal(60, 180);
  }

  /**
   * M363: SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M363() {
    ECHO_LM(DB," Cal: Psi 90 ");
    return SCARA_move_to_cal(50, 90);
  }

  /**
   * M364: SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
   */
  inline bool gcode_M364() {
    ECHO_LM(DB, " Cal: Theta-Psi 90 ");
    return SCARA_move_to_cal(45, 135);
  }

  /**
   * M365: SCARA calibration: Scaling factor, X, Y, Z axis
   */
  inline void gcode_M365() {
    for (int8_t i = X_AXIS; i <= Z_AXIS; i++) {
      if (code_seen(axis_codes[i])) {
        axis_scaling[i] = code_value();
      }
    }
  }
#endif // SCARA

#ifdef EXT_SOLENOID
  void enable_solenoid(uint8_t num) {
    switch(num) {
      case 0:
        OUT_WRITE(SOL0_PIN, HIGH);
        break;
        #if HAS_SOLENOID_1
          case 1:
            OUT_WRITE(SOL1_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_2
          case 2:
            OUT_WRITE(SOL2_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_3
          case 3:
            OUT_WRITE(SOL3_PIN, HIGH);
            break;
        #endif
      default:
        ECHO_LM(ER, MSG_INVALID_SOLENOID);
        break;
    }
  }

  void enable_solenoid_on_active_extruder() { enable_solenoid(active_extruder); }

  void disable_all_solenoids() {
    OUT_WRITE(SOL0_PIN, LOW);
    OUT_WRITE(SOL1_PIN, LOW);
    OUT_WRITE(SOL2_PIN, LOW);
    OUT_WRITE(SOL3_PIN, LOW);
  }

  /**
   * M380: Enable solenoid on the active extruder
   */
  inline void gcode_M380() { enable_solenoid_on_active_extruder(); }

  /**
   * M381: Disable all solenoids
   */
  inline void gcode_M381() { disable_all_solenoids(); }

#endif // EXT_SOLENOID

/**
 * M400: Finish all moves
 */
inline void gcode_M400() { st_synchronize(); }

#if SERVO_LEVELING

  /**
   * M401: Engage Z Servo endstop if available
   */
  inline void gcode_M401() { deploy_z_probe(); }
  /**
   * M402: Retract Z Servo endstop if enabled
   */
  inline void gcode_M402() { stow_z_probe(); }

#endif

#ifdef FILAMENT_SENSOR

  /**
   * M404: Display or set the nominal filament width (3mm, 1.75mm ) W<3.0>
   */
  inline void gcode_M404() {
    #if HAS_FILWIDTH
      if (code_seen('D')) {
        filament_width_nominal = code_value();
      }
      else {
        ECHO_LMV(OK, "Filament dia (nominal mm):", filament_width_nominal);
      }
    #endif
  }
    
  /**
   * M405: Turn on filament sensor for control
   */
  inline void gcode_M405() {
    if (code_seen('D')) meas_delay_cm = code_value();
    if (meas_delay_cm > MAX_MEASUREMENT_DELAY) meas_delay_cm = MAX_MEASUREMENT_DELAY;

    if (delay_index2 == -1) { //initialize the ring buffer if it has not been done since startup
      int temp_ratio = widthFil_to_size_ratio();

      for (delay_index1 = 0; delay_index1 < MAX_MEASUREMENT_DELAY + 1; ++delay_index1)
        measurement_delay[delay_index1] = temp_ratio - 100;  //subtract 100 to scale within a signed byte

      delay_index1 = delay_index2 = 0;
    }

    filament_sensor = true;

    //ECHO_SMV(DB, "Filament dia (measured mm):", filament_width_meas);
    //ECHO_EMV("Extrusion ratio(%):", extruder_multiply[active_extruder]);
  }

  /**
   * M406: Turn off filament sensor for control
   */
  inline void gcode_M406() { filament_sensor = false; }
  
  /**
   * M407: Get measured filament diameter on serial output
   */
  inline void gcode_M407() {
    ECHO_LM(OK, "Filament dia (measured mm):", filament_width_meas);   
  }

#endif // FILAMENT_SENSOR

/**
 * M500: Store settings in EEPROM
 */
inline void gcode_M500() {
  Config_StoreSettings();
}

/**
 * M501: Read settings from EEPROM
 */
inline void gcode_M501() {
  Config_RetrieveSettings();
}

/**
 * M502: Revert to default settings
 */
inline void gcode_M502() {
  Config_ResetDefault();
}

/**
 * M503: print settings currently in memory
 */
inline void gcode_M503() {
  Config_PrintSettings(code_seen('S') && code_value() == 0);
}

#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

  /**
   * M540: Set whether SD card print should abort on endstop hit (M540 S<0|1>)
   */
  inline void gcode_M540() {
    if (code_seen('S')) abort_on_endstop_hit = (code_value() > 0);
  }

#endif // ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

#ifdef FILAMENTCHANGEENABLE
  /**
   * M600: Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
   */
  inline void gcode_M600() {
    float target[NUM_AXIS], lastpos[NUM_AXIS], fr60 = feedrate / 60;
    filament_changing = true;
    for (int i=0; i < NUM_AXIS; i++)
      target[i] = lastpos[i] = current_position[i];

    #define BASICPLAN plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], fr60, active_extruder, active_driver);
    #ifdef DELTA
      #define RUNPLAN calculate_delta(target); BASICPLAN
    #else
      #define RUNPLAN BASICPLAN
    #endif

    //retract by E
    if (code_seen('E')) target[E_AXIS] += code_value();
    #ifdef FILAMENTCHANGE_FIRSTRETRACT
      else target[E_AXIS] += FILAMENTCHANGE_FIRSTRETRACT;
    #endif

    RUNPLAN;

    //lift Z
    if (code_seen('Z')) target[Z_AXIS] += code_value();
    #ifdef FILAMENTCHANGE_ZADD
      else target[Z_AXIS] += FILAMENTCHANGE_ZADD;
    #endif

    RUNPLAN;

    //move xy
    if (code_seen('X')) target[X_AXIS] = code_value();
    #ifdef FILAMENTCHANGE_XPOS
      else target[X_AXIS] = FILAMENTCHANGE_XPOS;
    #endif

    if (code_seen('Y')) target[Y_AXIS] = code_value();
    #ifdef FILAMENTCHANGE_YPOS
      else target[Y_AXIS] = FILAMENTCHANGE_YPOS;
    #endif

    RUNPLAN;

    if (code_seen('L')) target[E_AXIS] += code_value();
    #ifdef FILAMENTCHANGE_FINALRETRACT
      else target[E_AXIS] += FILAMENTCHANGE_FINALRETRACT;
    #endif

    RUNPLAN;

    //finish moves
    st_synchronize();
    //disable extruder steppers so filament can be removed
    disable_e();
    delay(100);
    boolean beep = true;
    boolean sleep = false;
    int cnt = 0;
    
    int old_target_temperature[HOTENDS] = { 0 };
    for (int8_t e = 0; e < HOTENDS; e++)
    {
      old_target_temperature[e] = target_temperature[e];
    }
    int old_target_temperature_bed = target_temperature_bed;
    millis_t last_set = millis();
    
    PRESSBUTTON:
    LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);
    while (!lcd_clicked()) {
      manage_heater();
      manage_inactivity(true);
      lcd_update();
      if ((millis() - last_set > 60000) && cnt <= FILAMENTCHANGE_PRINTEROFF) beep = true;
      if (cnt >= FILAMENTCHANGE_PRINTEROFF && !sleep) {
        disable_all_heaters();
        disable_x();
        disable_y();
        disable_z();
        disable_e();
        sleep = true;
        lcd_reset_alert_level();
        LCD_ALERTMESSAGEPGM("Zzzz Zzzz Zzzz");
      }
      if (beep) {
        last_set = millis();
        lcd_beep(3);
        beep = false;
        cnt += 1;
      }
    }

    //reset LCD alert message
    lcd_reset_alert_level();

    if (sleep) {
      for(int8_t e = 0; e < HOTENDS; e++)
      {
        setTargetHotend(old_target_temperature[e], e);
        no_wait_for_cooling = true;
        wait_heater();
      }
      setTargetBed(old_target_temperature_bed);
      no_wait_for_cooling = true;
      wait_bed();
      sleep = false;
      beep = true;
      cnt = 0;
      goto PRESSBUTTON;
    }

    //return to normal
    if (code_seen('L')) target[E_AXIS] -= code_value();
    #ifdef FILAMENTCHANGE_FINALRETRACT
      else target[E_AXIS] -= FILAMENTCHANGE_FINALRETRACT;
    #endif

    current_position[E_AXIS] = target[E_AXIS]; //the long retract of L is compensated by manual filament feeding
    plan_set_e_position(current_position[E_AXIS]);

    // HOME X & Y & Z(only Delta)
    gcode_G28(true,true);

    #ifdef DELTA
      calculate_delta(lastpos);
      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder, active_driver); //move xyz back
      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder, active_driver); //final unretract
      for (int8_t i = 0; i < NUM_AXIS; i++) current_position[i] = lastpos[i];
      calculate_delta(current_position);
      plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
    #else
      plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder, active_driver); //move xy back
      plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder, active_driver); //move z back
      plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder, active_driver); //final unretract
      for (int8_t i = 0; i < NUM_AXIS; i++) current_position[i] = lastpos[i];
      sync_plan_position();
    #endif

    #ifdef FILAMENT_RUNOUT_SENSOR
      filrunoutEnqueued = false;
    #endif
  }
#endif //FILAMENTCHANGEENABLE

#ifdef DUAL_X_CARRIAGE
  /**
   * M605: Set dual x-carriage movement mode
   *
   *    M605 S0: Full control mode. The slicer has full control over x-carriage movement
   *    M605 S1: Auto-park mode. The inactive head will auto park/unpark without slicer involvement
   *    M605 S2 [Xnnn] [Rmmm]: Duplication mode. The second extruder will duplicate the first with nnn
   *                         millimeters x-offset and an optional differential hotend temperature of
   *                         mmm degrees. E.g., with "M605 S2 X100 R2" the second extruder will duplicate
   *                         the first with a spacing of 100mm in the x direction and 2 degrees hotter.
   *
   *    Note: the X axis should be homed after changing dual x-carriage mode.
   */
  inline void gcode_M605() {
    st_synchronize();
    if (code_seen('S')) dual_x_carriage_mode = code_value();
    switch(dual_x_carriage_mode) {
      case DXC_DUPLICATION_MODE:
        if (code_seen('X')) duplicate_extruder_x_offset = max(code_value(), X2_MIN_POS - x_home_pos(0));
        if (code_seen('R')) duplicate_extruder_temp_offset = code_value();
        ECHO_SM(OK, MSG_HOTEND_OFFSET);
        ECHO_MV(" ", extruder_offset[X_AXIS][0]);
        ECHO_MV(",", extruder_offset[Y_AXIS][0]);
        ECHO_MV(" ", duplicate_extruder_x_offset);
        ECHO_EMV(",", extruder_offset[Y_AXIS][1]);
        break;
      case DXC_FULL_CONTROL_MODE:
      case DXC_AUTO_PARK_MODE:
        break;
      default:
        dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
        break;
    }
    active_extruder_parked = false;
    extruder_duplication_enabled = false;
    delayed_move_time = 0;
  }
#endif // DUAL_X_CARRIAGE

#ifdef ENABLE_AUTO_BED_LEVELING
  //M666: Set Z probe offset
  inline void gcode_M666() {
    if (code_seen('P')) {
      zprobe_zoffset = code_value();
    }
    if (code_seen('L')) {
      ECHO_LMV(OK, "P (Z-Probe Offset):", zprobe_zoffset);
    }
  }
#endif

#ifdef DELTA
  //M666: Set delta endstop and geometry adjustment
  inline void gcode_M666() {
    if ( !(code_seen('P'))) {
      for(int8_t i=0; i < 3; i++) {
        if (code_seen(axis_codes[i])) endstop_adj[i] = code_value();
      }
    }
    if (code_seen('A')) {
      tower_adj[0] = code_value();
      set_delta_constants();
    }
    if (code_seen('B')) {
      tower_adj[1] = code_value();
      set_delta_constants();
    }
    if (code_seen('C')) {
      tower_adj[2] = code_value();
      set_delta_constants();
    }
    if (code_seen('I')) {
      tower_adj[3] = code_value();
      set_delta_constants();
    }
    if (code_seen('J')) {
      tower_adj[4] = code_value();
      set_delta_constants();
    }
    if (code_seen('K')) {
      tower_adj[5] = code_value();
      set_delta_constants();
    }
    if (code_seen('R')) {
      delta_radius = code_value();
      set_delta_constants();
    }
    if (code_seen('D')) {
      delta_diagonal_rod = code_value();
      set_delta_constants();
    }
    if (code_seen('H')) {
      max_pos[Z_AXIS]= code_value();
      set_delta_constants();
    }
    if (code_seen('P')) {
      float pz = code_value();
      if (!(code_seen(axis_codes[0]) || code_seen(axis_codes[1]) || code_seen(axis_codes[2]))) {  // Allow direct set of Z offset without an axis code
        z_probe_offset[Z_AXIS]= pz;
      }
      else {
        for(int8_t i=0; i < 3; i++) {
          if (code_seen(axis_codes[i])) z_probe_offset[i] = code_value();
        }
      }
    }
    if (code_seen('L')) {
      ECHO_LM(DB, "Current Delta geometry values:");
      ECHO_LMV(DB, "X (Endstop Adj): ",endstop_adj[0]);
      ECHO_LMV(DB, "Y (Endstop Adj): ",endstop_adj[1]);
      ECHO_LMV(DB, "Z (Endstop Adj): ",endstop_adj[2]);
      ECHO_SMV(DB, "P (Z-Probe Offset): X", z_probe_offset[0]);
      ECHO_MV(" Y", z_probe_offset[1]);
      ECHO_EMV(" Z", z_probe_offset[2]);
      ECHO_LMV(DB, "A (Tower A Position Correction): ",tower_adj[0]);
      ECHO_LMV(DB, "B (Tower B Position Correction): ",tower_adj[1]);
      ECHO_LMV(DB, "C (Tower C Position Correction): ",tower_adj[2]);
      ECHO_LMV(DB, "I (Tower A Radius Correction): ",tower_adj[3]);
      ECHO_LMV(DB, "J (Tower B Radius Correction): ",tower_adj[4]);
      ECHO_LMV(DB, "K (Tower C Radius Correction): ",tower_adj[5]);
      ECHO_LMV(DB, "R (Delta Radius): ",delta_radius);
      ECHO_LMV(DB, "D (Diagonal Rod Length): ",delta_diagonal_rod);
      ECHO_LMV(DB, "H (Z-Height): ",max_pos[Z_AXIS]);
    }
  }
#endif

/**
 * M907: Set digital trimpot motor current using axis codes X, Y, Z, E, B, S
 */
inline void gcode_M907() {
  #if HAS_DIGIPOTSS
    for (int i=0;i<NUM_AXIS;i++)
      if (code_seen(axis_codes[i])) digipot_current(i, code_value());
    if (code_seen('B')) digipot_current(4, code_value());
    if (code_seen('S')) for (int i=0; i<=4; i++) digipot_current(i, code_value());
  #endif
  #ifdef MOTOR_CURRENT_PWM_XY_PIN
    if (code_seen('X')) digipot_current(0, code_value());
  #endif
  #ifdef MOTOR_CURRENT_PWM_Z_PIN
    if (code_seen('Z')) digipot_current(1, code_value());
  #endif
  #ifdef MOTOR_CURRENT_PWM_E_PIN
    if (code_seen('E')) digipot_current(2, code_value());
  #endif
  #ifdef DIGIPOT_I2C
    // this one uses actual amps in floating point
    for (int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_i2c_set_current(i, code_value());
    // for each additional extruder (named B,C,D,E..., channels 4,5,6,7...)
    for (int i=NUM_AXIS;i<DIGIPOT_I2C_NUM_CHANNELS;i++) if(code_seen('B'+i-NUM_AXIS)) digipot_i2c_set_current(i, code_value());
  #endif
}

#if HAS_DIGIPOTSS
  /**
   * M908: Control digital trimpot directly (M908 P<pin> S<current>)
   */
  inline void gcode_M908() {
    digitalPotWrite(
      code_seen('P') ? code_value() : 0,
      code_seen('S') ? code_value() : 0
    );
  }
#endif // HAS_DIGIPOTSS

#ifdef NPR2
  /**
   * M997: Cxx Move Carter xx gradi
   */
  inline void gcode_M997() {
    long csteps;
    if (code_seen('C')) {
      csteps = code_value() * color_step_moltiplicator;
      ECHO_LMV(OK, "csteps: ", csteps);
      if (csteps < 0) colorstep(-csteps,false);
      if (csteps > 0) colorstep(csteps,true);
    }
  }
#endif

/**
 * M999: Restart after being stopped
 */
inline void gcode_M999() {
  Running = true;
  lcd_reset_alert_level();
  gcode_LastN = Stopped_gcode_LastN;
  FlushSerialRequestResend();
}

#ifdef CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
  inline void gcode_SET_Z_PROBE_OFFSET() {
    float value;
    if (code_seen('Z')) {
      value = code_value();
      if (Z_PROBE_OFFSET_RANGE_MIN <= value && value <= Z_PROBE_OFFSET_RANGE_MAX) {
        zprobe_zoffset = -value; // compare w/ line 278 of ConfigurationStore.cpp
        ECHO_LM(OK, MSG_ZPROBE_ZOFFSET " " MSG_OK);
      }
      else {
        ECHO_S(OK, MSG_ZPROBE_ZOFFSET);
        ECHO_LMV(OK, MSG_Z_MIN, Z_PROBE_OFFSET_RANGE_MIN);
        ECHO_LMV(OK, MSG_Z_MAX,Z_PROBE_OFFSET_RANGE_MAX);
      }
    }
    else {
      ECHO_LM(OK, MSG_ZPROBE_ZOFFSET " : ", -zprobe_zoffset);
    }
  }

#endif // CUSTOM_M_CODE_SET_Z_PROBE_OFFSET

inline void gcode_T() {
  int tmp_extruder = code_value();
  long csteps;
  if (tmp_extruder >= EXTRUDERS) {
    ECHO_SMV(OK, "T", tmp_extruder);
    ECHO_EM(MSG_INVALID_EXTRUDER);
  }
  else {
    target_extruder = tmp_extruder;

    #if EXTRUDERS > 1
      bool make_move = false;
    #endif

    if (code_seen('F')) {

      #if EXTRUDERS > 1
        make_move = true;
      #endif

      next_feedrate = code_value();
      if (next_feedrate > 0.0) feedrate = next_feedrate;
    }
    #if EXTRUDERS > 1
      #ifdef NPR2
        if(target_extruder != old_color)
      #else
        if(target_extruder != active_extruder)
      #endif // NPR2
      {
        // Save current position to return to after applying extruder offset
        set_destination_to_current();
        #ifdef DUAL_X_CARRIAGE
          if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE && IsRunning() &&
                (delayed_move_time != 0 || current_position[X_AXIS] != x_home_pos(active_extruder))) {
            // Park old head: 1) raise 2) move to park position 3) lower
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,
                  current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder, active_driver);
            plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,
                  current_position[E_AXIS], max_feedrate[X_AXIS], active_extruder, active_driver);
            plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS],
                  current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder, active_driver);
            st_synchronize();
          }

          // apply Y & Z extruder offset (x offset is already used in determining home pos)
          current_position[Y_AXIS] = current_position[Y_AXIS] -
                        hotend_offset[Y_AXIS][active_extruder] +
                        hotend_offset[Y_AXIS][target_extruder];
          current_position[Z_AXIS] = current_position[Z_AXIS] -
                        hotend_offset[Z_AXIS][active_extruder] +
                        hotend_offset[Z_AXIS][target_extruder];

          active_extruder = target_extruder;

          // This function resets the max/min values - the current position may be overwritten below.
          axis_is_at_home(X_AXIS);

          if (dual_x_carriage_mode == DXC_FULL_CONTROL_MODE) {
            current_position[X_AXIS] = inactive_extruder_x_pos;
            inactive_extruder_x_pos = destination[X_AXIS];
          }
          else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE) {
            active_extruder_parked = (active_extruder == 0); // this triggers the second extruder to move into the duplication position
            if (active_extruder == 0 || active_extruder_parked)
              current_position[X_AXIS] = inactive_extruder_x_pos;
            else
              current_position[X_AXIS] = destination[X_AXIS] + duplicate_extruder_x_offset;
            inactive_extruder_x_pos = destination[X_AXIS];
            extruder_duplication_enabled = false;
          }
          else {
            // record raised toolhead position for use by unpark
            memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
            raised_parked_position[Z_AXIS] += TOOLCHANGE_UNPARK_ZLIFT;
            active_extruder_parked = true;
            delayed_move_time = 0;
          }
        #else // !DUAL_X_CARRIAGE
          // Offset hotend (only by XY)
          #if HOTENDS > 1
            for (int i=X_AXIS; i<=Y_AXIS; i++)
              current_position[i] += hotend_offset[i][target_extruder] - hotend_offset[i][active_extruder];
          #endif // HOTENDS > 1

          #if defined(MKR4) && (EXTRUDERS > 1)
            #if (EXTRUDERS == 4) && (E0E2_CHOICE_PIN >1) && (E1E3_CHOICE_PIN > 1) && (DRIVER_EXTRUDERS == 2)
              st_synchronize(); // Finish all movement
              disable_e();
              switch(target_extruder)
              {
              case 0:
                WRITE(E0E2_CHOICE_PIN,LOW);
                WRITE(E1E3_CHOICE_PIN,LOW);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e0();
                break;
              case 1:
                WRITE(E0E2_CHOICE_PIN,LOW);
                WRITE(E1E3_CHOICE_PIN,LOW);
                active_driver = 1;
                delay(500); // 500 microseconds delay for relay
                enable_e1();             
                break;
              case 2:
                WRITE(E0E2_CHOICE_PIN,HIGH);
                WRITE(E1E3_CHOICE_PIN,LOW);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e2();
                break;
              case 3:
                WRITE(E0E2_CHOICE_PIN,LOW);
                WRITE(E1E3_CHOICE_PIN,HIGH);
                active_driver = 1;
                delay(500); // 500 microseconds delay for relay
                enable_e3();
                break;
              }            
            #elif (EXTRUDERS == 3) && (E0E2_CHOICE_PIN >1) && (DRIVER_EXTRUDERS == 2)
              st_synchronize(); // Finish all movement
              disable_e();
              switch(target_extruder)
              {
              case 0:
                WRITE(E0E2_CHOICE_PIN,LOW);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e0();
                break;
              case 1:
                WRITE(E0E2_CHOICE_PIN,LOW);
                active_driver = 1;
                delay(500); // 500 microseconds delay for relay
                enable_e1();
                break;
              case 2:
                WRITE(E0E2_CHOICE_PIN,HIGH);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e2();
                break;
              }
            #elif (EXTRUDERS == 3) && (E0E1_CHOICE_PIN >1) && (E0E2_CHOICE_PIN >1) && (DRIVER_EXTRUDERS == 1)
              st_synchronize(); // Finish all movement
              disable_e();
              switch(target_extruder)
              {
              case 0:
                WRITE(E0E1_CHOICE_PIN,LOW);
                WRITE(E0E2_CHOICE_PIN,LOW);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e0();
                break;
              case 1:
                WRITE(E0E1_CHOICE_PIN,HIGH);
                WRITE(E0E2_CHOICE_PIN,LOW);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e0();
                break;
              case 2:
                WRITE(E0E1_CHOICE_PIN,HIGH);
                WRITE(E0E2_CHOICE_PIN,HIGH);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e0();
                break;
              }
            #elif (EXTRUDERS == 2) && (E0E1_CHOICE_PIN >1) && (DRIVER_EXTRUDERS == 1)
              st_synchronize(); // Finish all movement
              disable_e();
              switch(target_extruder)
              {
              case 0:
                WRITE(E0E1_CHOICE_PIN,LOW);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e0();
                break;
              case 1:
                WRITE(E0E1_CHOICE_PIN,HIGH);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e0();
                break;
              }
            #endif // E0E1_CHOICE_PIN E0E2_CHOICE_PIN E1E3_CHOICE_PIN
            active_extruder = target_extruder;
            ECHO_LM(OK, MSG_ACTIVE_DRIVER, active_driver);
            ECHO_LM(OK, MSG_ACTIVE_EXTRUDER, active_extruder);
          #elif defined(NPR2)
            st_synchronize(); // Finish all movement
            if (old_color == 99)
            {
              csteps = (color_position[target_extruder]) * color_step_moltiplicator;
            }
            else
            {
              csteps = (color_position[target_extruder] - color_position[old_color]) * color_step_moltiplicator;
            }
            if (csteps < 0) colorstep(-csteps,false);
            if (csteps > 0) colorstep(csteps,true);
            old_color = active_extruder = target_extruder;
            active_driver = 0;
            ECHO_LM(OK, MSG_ACTIVE_COLOR, active_extruder);
          #else 
            active_driver = active_extruder = target_extruder;
            ECHO_S(OK, MSG_ACTIVE_EXTRUDER, active_extruder);

          #endif // end MKR4 || NPR2
        #endif // end no DUAL_X_CARRIAGE

        #ifdef DELTA 
          sync_plan_position_delta();
        #else // NO DELTA
          sync_plan_position();
        #endif // DELTA
        // Move to the old position if 'F' was in the parameters
        if (make_move && IsRunning()) prepare_move();
      }

      #ifdef EXT_SOLENOID
        st_synchronize();
        disable_all_solenoids();
        enable_solenoid_on_active_extruder();
      #endif // EXT_SOLENOID

    #endif // EXTRUDERS > 1
  }
}


/*****************************************************
*** Process Commands and dispatch them to handlers ***
******************************************************/
void process_commands() {
  if(code_seen('G')) {
    int gCode = code_value_short();
    switch(gCode) {
      //G0 -> G1
      case 0:
      case 1:
        gcode_G0_G1();
        break;

      // G2, G3
      #ifndef SCARA
        case 2: // G2  - CW ARC
        case 3: // G3  - CCW ARC
          gcode_G2_G3(gCode == 2); break;
      #endif

      // G4 Dwell
      case 4:
        gcode_G4(); break;

      #ifdef FWRETRACT
        case 10: // G10: retract
        case 11: // G11: retract_recover
          gcode_G10_G11(gCode == 10); break;
      #endif //FWRETRACT

      case 28: //G28: Home all axes, one at a time
        gcode_G28(); break;

      #ifdef ENABLE_AUTO_BED_LEVELING
        case 29: // G29 Detailed Z-Probe, probes the bed at 3 or more points.
          gcode_G29(); break;
        #ifndef Z_PROBE_SLED
          case 30: // G30 Single Z Probe
            gcode_G30(); break;
        #else // Z_PROBE_SLED
          case 31: // G31: dock the sled
          case 32: // G32: undock the sled
            dock_sled(gCode == 31); break;
        #endif // Z_PROBE_SLED
      #endif // ENABLE_AUTO_BED_LEVELING

      #if defined(DELTA) && defined(Z_PROBE_ENDSTOP)
        case 29: // G29 Detailed Z-Probe, probes the bed at more points.
          gcode_G29(); break;
        case 30:  // G30 Delta AutoCalibration
          gcode_G30(); break;
      #endif // DELTA && Z_PROBE_ENDSTOP

      case 60: // G60 Store in memory actual position
        gcode_G60(); break;
      case 61: // G61 move to X Y Z in memory
        gcode_G61(); break;
      case 90: // G90
        relative_mode = false; break;
      case 91: // G91
        relative_mode = true; break;
      case 92: // G92
        gcode_G92(); break;
    }
  }

  else if (code_seen('M')) {
    switch(code_value_short()) {
      #ifdef ULTIPANEL
        case 0: // M0 - Unconditional stop - Wait for user button press on LCD
        case 1: // M1 - Conditional stop - Wait for user button press on LCD
          gcode_M0_M1(); break;
      #endif //ULTIPANEL

      #ifdef LASERBEAM
        case 3: // M03 S - Setting laser beam
          gcode_M3(); break;
        case 4: // M04 - Turn on laser beam
          gcode_M4(); break;
        case 5: // M05 - Turn off laser beam
          gcode_M5(); break;
      #endif //LASERBEAM

      #ifdef FILAMENT_END_SWITCH
        case 11: //M11 - Start printing
          gcode_M11(); break;
      #endif

      case 17: //M17 - Enable/Power all stepper motors
        gcode_M17(); break;

      #ifdef SDSUPPORT
        case 20: // M20 - list SD card
          gcode_M20(); break;
        case 21: // M21 - init SD card
          gcode_M21(); break;
        case 22: //M22 - release SD card
          gcode_M22(); break;
        case 23: //M23 - Select file
          gcode_M23(); break;
        case 24: //M24 - Start SD print
          gcode_M24(); break;
        case 25: //M25 - Pause SD print
          gcode_M25(); break;
        case 26: //M26 - Set SD index
          gcode_M26(); break;
        case 27: //M27 - Get SD status
          gcode_M27(); break;
        case 28: //M28 - Start SD write
          gcode_M28(); break;
        case 29: //M29 - Stop SD write
          gcode_M29(); break;
        case 30: //M30 <filename> Delete File
          gcode_M30(); break;
        case 32: //M32 - Select file and start SD print
          gcode_M32(); break;
        case 928: //M928 - Start SD write
          gcode_M928(); break;

      #endif //SDSUPPORT


      case 31: //M31 take time since the start of the SD print or an M109 command
        gcode_M31(); break;
      case 42: //M42 -Change pin status via gcode
        gcode_M42(); break;

      #if defined(ENABLE_AUTO_BED_LEVELING) && defined(Z_PROBE_REPEATABILITY_TEST)
        case 49: //M49 Z-Probe repeatability
          gcode_M49(); break;
      #endif //defined(ENABLE_AUTO_BED_LEVELING) && defined(Z_PROBE_REPEATABILITY_TEST)

      #if HAS_POWER_SWITCH
        case 80: //M80 - Turn on Power Supply
          gcode_M80(); break;
      #endif //HAS_POWER_SWITCH
      
      case 81: // M81 - Turn off Power, including Power Supply, if possible
        gcode_M81(); break;
      case 82:
        gcode_M82(); break;
      case 83:
        gcode_M83(); break;
      case 18: //compatibility
      case 84: // M84
        gcode_M18_M84(); break;
      case 85: // M85
        gcode_M85(); break;
      case 92: // M92
        gcode_M92(); break;  
      case 104: // M104
        gcode_M104(); break;
      case 105: // M105 Read current temperature
        gcode_M105(); return; break;

      #if HAS_FAN
        case 106: //M106 Fan On
          gcode_M106(); break;
        case 107: //M107 Fan Off
          gcode_M107(); break;
      #endif // HAS_FAN

      case 109: // M109 Wait for temperature
        gcode_M109(); break;
      case 111: // M111 - Debug mode
        gcode_M111(); break;
      case 112: //  M112 Emergency Stop
        gcode_M112(); break;
      case 114: // M114
        gcode_M114(); break;
      case 115: // M115
        gcode_M115(); break;  
      case 117: // M117 display message
        gcode_M117(); break;  
      case 119: // M119
        gcode_M119(); break;  
      case 120: // M120
        gcode_M120(); break;
      case 121: // M121
        gcode_M121(); break;  

      #ifdef BARICUDA
        // PWM for HEATER_1_PIN
        #if HAS_HEATER_1
          case 126: // M126 valve open
            gcode_M126(); break;
          case 127: // M127 valve closed
            gcode_M127(); break;
        #endif // HAS_HEATER_1

        // PWM for HEATER_2_PIN
        #if HAS_HEATER_2
          case 128: // M128 valve open
            gcode_M128(); break;
          case 129: // M129 valve closed
            gcode_M129(); break;
        #endif // HAS_HEATER_2
      #endif // BARICUDA

      case 140: // M140 Set bed temp
        gcode_M140(); break;

      #ifdef BLINKM
        case 150: // M150
          gcode_M150(); break;
      #endif //BLINKM

      #if HAS_TEMP_BED
        case 190: // M190 - Wait for bed heater to reach target.
          gcode_M190(); break;
      #endif //TEMP_BED_PIN

      case 200: // M200 D<millimetres> set filament diameter and set E axis units to cubic millimetres (use S0 to set back to millimeters).
        gcode_M200(); break;
      case 201: // M201
        gcode_M201(); break;
      #if 0 // Not used for Sprinter/grbl gen6
      case 202: // M202
        gcode_M202();
        break;
      #endif
      case 203: // M203 max feedrate mm/sec
        gcode_M203(); break;
      case 204: // M204 acceleration S normal moves T filament only moves
        gcode_M204(); break;
      case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
        gcode_M205(); break;
      case 206: // M206 additional homing offset
        gcode_M206(); break;

      #ifdef FWRETRACT
        case 207: //M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop]
          gcode_M207(); break;
        case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
          gcode_M208(); break;
        case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
          gcode_M209(); break;
      #endif // FWRETRACT

      #if HOTENDS > 1
        case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
          gcode_M218(); break;
      #endif

      case 220: // M220 S<factor in percent>- set speed factor override percentage
        gcode_M220(); break;
      case 221: // M221 S<factor in percent>- set extrude factor override percentage
        gcode_M221(); break;
      case 226: // M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
        gcode_M226(); break;

      #if defined(CHDK) || (defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1)
        case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
          gcode_M240(); break;
      #endif // CHDK || PHOTOGRAPH_PIN

      #if defined(DOGLCD) && LCD_CONTRAST >= 0
        case 250: // M250  Set LCD contrast value: C<value> (value 0..63)
          gcode_M250(); break;
      #endif // DOGLCD

      #if NUM_SERVOS > 0
        case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
          gcode_M280(); break;
      #endif // NUM_SERVOS > 0

      #if BEEPER > 0 || defined(ULTRALCD) || defined(LCD_USE_I2C_BUZZER)
        case 300: // M300 - Play beep tone
          gcode_M300(); break;
      #endif // LARGE_FLASH && (BEEPER>0 || ULTRALCD || LCD_USE_I2C_BUZZER)

      #ifdef PIDTEMP
        case 301: // M301
          gcode_M301(); break;
      #endif // PIDTEMP

      #ifdef PREVENT_DANGEROUS_EXTRUDE
        case 302: // allow cold extrudes, or set the minimum extrude temperature
          gcode_M302(); break;
      #endif // PREVENT_DANGEROUS_EXTRUDE

      case 303: // M303 PID autotune
        gcode_M303(); break;

      #ifdef PIDTEMPBED
        case 304: // M304
          gcode_M304(); break;
      #endif // PIDTEMPBED

      #if HAS_MICROSTEPS
        case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
          gcode_M350();
          break;
        case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
          gcode_M351();
          break;
      #endif // HAS_MICROSTEPS

      #ifdef SCARA
        case 360:  // M360 SCARA Theta pos1
          if (gcode_M360()) return; break;
        case 361:  // M361 SCARA Theta pos2
          if (gcode_M361()) return; break;
        case 362:  // M362 SCARA Psi pos1
          if (gcode_M362()) return; break;
        case 363:  // M363 SCARA Psi pos2
          if (gcode_M363()) return; break;
        case 364:  // M364 SCARA Psi pos3 (90 deg to Theta)
          if (gcode_M364()) return; break;
        case 365: // M365 Set SCARA scaling for X Y Z
          gcode_M365(); break;
      #endif // SCARA

      case 400: // M400 finish all moves
        gcode_M400(); break;

      #if SERVO_LEVELING
        case 401:
          gcode_M401(); break;
        case 402:
          gcode_M402(); break;
      #endif

      #ifdef FILAMENT_SENSOR
        case 404:  //M404 Enter the nominal filament width (3mm, 1.75mm ) N<3.0> or display nominal filament width
          gcode_M404(); break;
        case 405:  //M405 Turn on filament sensor for control
          gcode_M405(); break;
        case 406:  //M406 Turn off filament sensor for control
          gcode_M406(); break;
        case 407:   //M407 Display measured filament diameter
          gcode_M407(); break;
      #endif // FILAMENT_SENSOR 

      case 500: // M500 Store settings in EEPROM
        gcode_M500(); break;
      case 501: // M501 Read settings from EEPROM
        gcode_M501(); break;
      case 502: // M502 Revert to default settings
        gcode_M502(); break;
      case 503: // M503 print settings currently in memory
        gcode_M503(); break;

      #ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
        case 540:
          gcode_M540(); break;
      #endif

      #ifdef FILAMENTCHANGEENABLE
        case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
          gcode_M600(); break;
      #endif // FILAMENTCHANGEENABLE

      #ifdef DUAL_X_CARRIAGE
        case 605:
          gcode_M605(); break;
      #endif // DUAL_X_CARRIAGE

      #if defined(ENABLE_AUTO_BED_LEVELING) || defined(DELTA)
        case 666: //M666 Set Z probe offset or set delta endstop and geometry adjustment
          gcode_M666(); break;
      #endif //defined(ENABLE_AUTO_BED_LEVELING) || defined(DELTA)

      case 907: // M907 Set digital trimpot motor current using axis codes.
        gcode_M907(); break;

      #if HAS_DIGIPOTSS
        case 908: // M908 Control digital trimpot directly.
          gcode_M908(); break;
      #endif // HAS_DIGIPOTSS

      #ifdef NPR2
        case 997: // M997 Cxx Move Carter xx gradi
          gcode_M997(); break;
      #endif // NPR2

       case 999: // M999: Restart after being Stopped
        gcode_M999(); break;

        #ifdef CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
        case CUSTOM_M_CODE_SET_Z_PROBE_OFFSET:
          gcode_SET_Z_PROBE_OFFSET(); break;
      #endif // CUSTOM_M_CODE_SET_Z_PROBE_OFFSET

    }
  }

  else if (code_seen('T')) {
    gcode_T();
  }

  else {
    ECHO_SM(ER, MSG_UNKNOWN_COMMAND);
    ECHO_EVM(command_queue[cmd_queue_index_r], "\"");
  }

  ClearToSend();
}

void FlushSerialRequestResend() {
  //char command_queue[cmd_queue_index_r][100]="Resend:";
  MYSERIAL.flush();
  ECHO_LV(RS, gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend() {
  refresh_cmd_timeout();
  #ifdef SDSUPPORT
    if (fromsd[cmd_queue_index_r]) return;
  #endif
  ECHO_S(OK);
  ECHO_E;
}

void get_coordinates() {
  for (int i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i]))
      destination[i] = code_value() + (axis_relative_modes[i] || relative_mode ? current_position[i] : 0);
    else
      destination[i] = current_position[i];
  }
  if (code_seen('F')) {
    next_feedrate = code_value();
    if (next_feedrate > 0.0) feedrate = next_feedrate;
  }

  #ifdef LASERBEAM
    if(code_seen('L')) {
      laser_ttl_modulation = constrain(code_value(), 0, 255);
    }
  #endif // LASERBEAM
}

void get_arc_coordinates() {
  #ifdef SF_ARC_FIX
    bool relative_mode_backup = relative_mode;
    relative_mode = true;
  #endif
    get_coordinates();
  #ifdef SF_ARC_FIX
    relative_mode = relative_mode_backup;
  #endif

  offset[0] = code_seen('I') ? code_value() : 0;
  offset[1] = code_seen('J') ? code_value() : 0;
}

void clamp_to_software_endstops(float target[3]) {
  if (min_software_endstops) {
    NOLESS(target[X_AXIS], min_pos[X_AXIS]);
    NOLESS(target[Y_AXIS], min_pos[Y_AXIS]);
    
    float negative_z_offset = 0;
    #ifdef ENABLE_AUTO_BED_LEVELING
      if (Z_PROBE_OFFSET_FROM_EXTRUDER < 0) negative_z_offset += Z_PROBE_OFFSET_FROM_EXTRUDER;
      if (home_offset[Z_AXIS] < 0) negative_z_offset += home_offset[Z_AXIS];
    #endif
    NOLESS(target[Z_AXIS], min_pos[Z_AXIS] + negative_z_offset);
  }

  if (max_software_endstops) {
    NOMORE(target[X_AXIS], max_pos[X_AXIS]);
    NOMORE(target[Y_AXIS], max_pos[Y_AXIS]);
    NOMORE(target[Z_AXIS], max_pos[Z_AXIS]);
  }
}

#ifdef PREVENT_DANGEROUS_EXTRUDE

  inline float prevent_dangerous_extrude(float &curr_e, float &dest_e) {
    float de = dest_e - curr_e;
    if (debugDryrun()) return de;
    if (de) {
      if (degHotend(active_extruder) < extrude_min_temp) {
        curr_e = dest_e; // Behave as if the move really took place, but ignore E part
        ECHO_LM(ER, MSG_ERR_COLD_EXTRUDE_STOP);
        return 0;
      }
      #ifdef PREVENT_LENGTHY_EXTRUDE
        if (labs(de) > EXTRUDE_MAXLENGTH) {
          curr_e = dest_e; // Behave as if the move really took place, but ignore E part
          ECHO_LM(ER, MSG_ERR_LONG_EXTRUDE_STOP);
          return 0;
        }
      #endif
    }
    return de;
  }

#endif // PREVENT_DANGEROUS_EXTRUDE

void prepare_move() {
  clamp_to_software_endstops(destination);
  refresh_cmd_timeout();

  #ifdef PREVENT_DANGEROUS_EXTRUDE
    (void)prevent_dangerous_extrude(current_position[E_AXIS], destination[E_AXIS]);
  #endif

  #ifdef SCARA //for now same as delta-code

    float difference[NUM_AXIS];
    for (int8_t i = 0; i < NUM_AXIS; i++) difference[i] = destination[i] - current_position[i];

    float cartesian_mm = sqrt(sq(difference[X_AXIS]) + sq(difference[Y_AXIS]) + sq(difference[Z_AXIS]));
    if (cartesian_mm < 0.000001) { cartesian_mm = abs(difference[E_AXIS]); }
    if (cartesian_mm < 0.000001) { return; }
    float seconds = 6000 * cartesian_mm / feedrate / feedrate_multiplier;
    int steps = max(1, int(scara_segments_per_second * seconds));

    //ECHO_SMV(DB, "mm=", cartesian_mm);
    //ECHO_MV(" seconds=", seconds);
    //ECHO_EMV(" steps=", steps);

    for (int s = 1; s <= steps; s++) {
      float fraction = float(s) / float(steps);
      for (int8_t i = 0; i < NUM_AXIS; i++) destination[i] = current_position[i] + difference[i] * fraction;

      calculate_delta(destination);
      //ECHO_SMV(DB, "destination[X_AXIS]=", destination[X_AXIS]);
      //ECHO_MV("destination[Y_AXIS]=", destination[Y_AXIS]);
      //ECHO_MV("destination[Z_AXIS]=", destination[Z_AXIS]);
      //ECHO_MV("delta[X_AXIS]=", delta[X_AXIS]);
      //ECHO_MV("delta[Y_AXIS]=", delta[Y_AXIS]);
      //ECHO_EMV("delta[Z_AXIS]=", delta[Z_AXIS]);

      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],
        destination[E_AXIS], feedrate*feedrate_multiplier/60/100.0,
        active_extruder);
    }

  #endif // SCARA

  #ifdef DELTA
    float difference[NUM_AXIS];
    for (int8_t i=0; i < NUM_AXIS; i++) difference[i] = destination[i] - current_position[i];

    float cartesian_mm = sqrt(sq(difference[X_AXIS]) + sq(difference[Y_AXIS]) + sq(difference[Z_AXIS]));
    if (cartesian_mm < 0.000001) cartesian_mm = abs(difference[E_AXIS]);
    if (cartesian_mm < 0.000001) return;
    float seconds = 6000 * cartesian_mm / feedrate / feedrate_multiplier;
    int steps = max(1, int(DELTA_SEGMENTS_PER_SECOND * seconds));

    // ECHO_SMV(DB,"mm=", cartesian_mm);
    // ECHO_MV(" seconds=", seconds);
    // ECHO_EMV(" steps=", steps);

    for (int s = 1; s <= steps; s++) {
      float fraction = float(s) / float(steps);
      for (int8_t i = 0; i < NUM_AXIS; i++) destination[i] = current_position[i] + difference[i] * fraction;
      calculate_delta(destination);
      adjust_delta(destination);
      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], destination[E_AXIS], feedrate*feedrate_multiplier/60/100.0, active_extruder, active_driver);
    }

  #endif // DELTA

  #ifdef DUAL_X_CARRIAGE
    if (active_extruder_parked) {
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && active_extruder == 0) {
        // move duplicate extruder into correct duplication position.
        plan_set_position(inactive_extruder_x_pos, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        plan_buffer_line(current_position[X_AXIS] + duplicate_extruder_x_offset, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS], 1, active_driver);
        sync_plan_position();
        st_synchronize();
        extruder_duplication_enabled = true;
        active_extruder_parked = false;
      }
      else if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE) { // handle unparking of head
        if (current_position[E_AXIS] == destination[E_AXIS]) {
          // This is a travel move (with no extrusion)
          // Skip it, but keep track of the current position
          // (so it can be used as the start of the next non-travel move)
          if (delayed_move_time != 0xFFFFFFFFUL) {
            set_current_to_destination();
            NOLESS(raised_parked_position[Z_AXIS], destination[Z_AXIS]);
            delayed_move_time = millis();
            return;
          }
        }
        delayed_move_time = 0;
        // unpark extruder: 1) raise, 2) move into starting XY position, 3) lower
        plan_buffer_line(raised_parked_position[X_AXIS], raised_parked_position[Y_AXIS], raised_parked_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder, active_driver);
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], raised_parked_position[Z_AXIS], current_position[E_AXIS], min(max_feedrate[X_AXIS],max_feedrate[Y_AXIS]), active_extruder, active_driver);
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder, active_driver);
        active_extruder_parked = false;
      }
    }
  #endif //DUAL_X_CARRIAGE

  #if !defined(DELTA) && !defined(SCARA)
    // Do not use feedrate_multiplier for E or Z only moves
    if ((current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
      line_to_destination();
    }
    else {
      line_to_destination(feedrate * feedrate_multiplier / 100.0);
    }
  #endif // !defined(DELTA) && !defined(SCARA)

  #ifdef IDLE_OOZING_PREVENT || EXTRUDER_RUNOUT_PREVENT
    axis_last_activity = millis();
    axis_is_moving = false;
  #endif

  set_current_to_destination();
}

void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedrate_multiplier/60/100.0, r, isclockwise, active_extruder, active_driver);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  set_current_to_destination();
  refresh_cmd_timeout();
}

#if HAS_CONTROLLERFAN

millis_t lastMotor = 0; // Last time a motor was turned on
millis_t lastMotorCheck = 0; // Last time the state was checked

void controllerFan() {
  millis_t ms = millis();
  if (ms >= lastMotorCheck + 2500) { // Not a time critical function, so we only check every 2500ms
    lastMotorCheck = ms;
    if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON || soft_pwm_bed > 0
      || E0_ENABLE_READ == E_ENABLE_ON // If any of the drivers are enabled...
      #if EXTRUDERS > 1
        || E1_ENABLE_READ == E_ENABLE_ON
        #if HAS_X2_ENABLE
          || X2_ENABLE_READ == X_ENABLE_ON
        #endif
        #if EXTRUDERS > 2
          || E2_ENABLE_READ == E_ENABLE_ON
          #if EXTRUDERS > 3
            || E3_ENABLE_READ == E_ENABLE_ON
          #endif
        #endif
      #endif
    ) {
      lastMotor = ms; //... set time to NOW so the fan will turn on
    }
    uint8_t speed = (lastMotor == 0 || ms >= lastMotor + (CONTROLLERFAN_SECS * 1000UL)) ? 0 : CONTROLLERFAN_SPEED;
    // allows digital or PWM fan output to be used (see M42 handling)
    digitalWrite(CONTROLLERFAN_PIN, speed);
    analogWrite(CONTROLLERFAN_PIN, speed);
  }
}
#endif

#ifdef SCARA
void calculate_SCARA_forward_Transform(float f_scara[3])
{
  // Perform forward kinematics, and place results in delta[3]
  // The maths and first version has been done by QHARLEY . Integrated into masterbranch 06/2014 and slightly restructured by Joachim Cerny in June 2014
  
  float x_sin, x_cos, y_sin, y_cos;
  
    //ECHO_SMV(DB,"f_delta x=", f_scara[X_AXIS]);
    //ECHO_MV(" y=", f_scara[Y_AXIS]);
  
    x_sin = sin(f_scara[X_AXIS]/SCARA_RAD2DEG) * Linkage_1;
    x_cos = cos(f_scara[X_AXIS]/SCARA_RAD2DEG) * Linkage_1;
    y_sin = sin(f_scara[Y_AXIS]/SCARA_RAD2DEG) * Linkage_2;
    y_cos = cos(f_scara[Y_AXIS]/SCARA_RAD2DEG) * Linkage_2;
   
    //ECHO_MV(" x_sin=", x_sin);
    //ECHO_MV(" x_cos=", x_cos);
    //ECHO_MV(" y_sin=", y_sin);
    //ECHO_MV(" y_cos=", y_cos);
  
    delta[X_AXIS] = x_cos + y_cos + SCARA_offset_x;  //theta
    delta[Y_AXIS] = x_sin + y_sin + SCARA_offset_y;  //theta+phi
  
    //ECHO_MV(" delta[X_AXIS]=", delta[X_AXIS]);
    //ECHO_EMV(" delta[Y_AXIS]=", delta[Y_AXIS]);
}

void calculate_delta(float cartesian[3]){
  //reverse kinematics.
  // Perform reversed kinematics, and place results in delta[3]
  // The maths and first version has been done by QHARLEY . Integrated into masterbranch 06/2014 and slightly restructured by Joachim Cerny in June 2014
  
  float SCARA_pos[2];
  static float SCARA_C2, SCARA_S2, SCARA_K1, SCARA_K2, SCARA_theta, SCARA_psi; 
  
  SCARA_pos[X_AXIS] = cartesian[X_AXIS] * axis_scaling[X_AXIS] - SCARA_offset_x;  //Translate SCARA to standard X Y
  SCARA_pos[Y_AXIS] = cartesian[Y_AXIS] * axis_scaling[Y_AXIS] - SCARA_offset_y;  // With scaling factor.
  
  #if (Linkage_1 == Linkage_2)
    SCARA_C2 = ( ( sq(SCARA_pos[X_AXIS]) + sq(SCARA_pos[Y_AXIS]) ) / (2 * (float)L1_2) ) - 1;
  #else
    SCARA_C2 =   ( sq(SCARA_pos[X_AXIS]) + sq(SCARA_pos[Y_AXIS]) - (float)L1_2 - (float)L2_2 ) / 45000; 
  #endif
  
  SCARA_S2 = sqrt( 1 - sq(SCARA_C2) );
  
  SCARA_K1 = Linkage_1 + Linkage_2 * SCARA_C2;
  SCARA_K2 = Linkage_2 * SCARA_S2;
  
  SCARA_theta = ( atan2(SCARA_pos[X_AXIS],SCARA_pos[Y_AXIS])-atan2(SCARA_K1, SCARA_K2) ) * -1;
  SCARA_psi   =   atan2(SCARA_S2,SCARA_C2);
  
  delta[X_AXIS] = SCARA_theta * SCARA_RAD2DEG;  // Multiply by 180/Pi  -  theta is support arm angle
  delta[Y_AXIS] = (SCARA_theta + SCARA_psi) * SCARA_RAD2DEG;  //       -  equal to sub arm angle (inverted motor)
  delta[Z_AXIS] = cartesian[Z_AXIS];
  
  /*
  ECHO_SMV(DB, "cartesian x=", cartesian[X_AXIS]);
  ECHO_MV(" y=", cartesian[Y_AXIS]);
  ECHO_MV(" z=", cartesian[Z_AXIS]);
  
  ECHO_MV("scara x=", SCARA_pos[X_AXIS]);
  ECHO_MV(" y=", Y_AXIS]);
  
  ECHO_MV("delta x=", delta[X_AXIS]);
  ECHO_MV(" y=", delta[Y_AXIS]);
  ECHO_MV(" z=", delta[Z_AXIS]);
  
  ECHO_MV("C2=", SCARA_C2);
  ECHO_MV(" S2=", SCARA_S2);
  ECHO_MV(" Theta=", SCARA_theta);
  ECHO_EMV(" Psi=", SCARA_psi);
  */
}

#endif

#ifdef TEMP_STAT_LEDS

  static bool red_led = false;
  static millis_t next_status_led_update_ms = 0;

  void handle_status_leds(void) {
    float max_temp = 0.0;
    if (millis() > next_status_led_update_ms) {
      next_status_led_update_ms += 500; // Update every 0.5s
      for (int8_t cur_hotend = 0; cur_hotend < EXTRUDERS; ++cur_hotend)
         max_temp = max(max(max_temp, degHotend(cur_hotend)), degTargetHotend(cur_hotend));
      #if HAS_TEMP_BED
        max_temp = max(max(max_temp, degTargetBed()), degBed());
      #endif
      bool new_led = (max_temp > 55.0) ? true : (max_temp < 54.0) ? false : red_led;
      if (new_led != red_led) {
        red_led = new_led;
        digitalWrite(STAT_LED_RED, new_led ? HIGH : LOW);
        digitalWrite(STAT_LED_BLUE, new_led ? LOW : HIGH);
      }
    }
  }

#endif

void enable_all_steppers() {
  enable_x();
  enable_y();
  enable_z();
  enable_e0();
  enable_e1();
  enable_e2();
  enable_e3();
}

void disable_all_steppers() {
  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();
  disable_e3();
}

/**
 * Manage several activities:
 *  - Check for Filament Runout
 *  - Keep the command buffer full
 *  - Check for maximum inactive time between commands
 *  - Check for maximum inactive time between stepper commands
 *  - Check if pin CHDK needs to go LOW
 *  - Check for KILL button held down
 *  - Check for HOME button held down
 *  - Check if cooling fan needs to be switched on
 *  - Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 *  - check oozing prevent
 */
void manage_inactivity(bool ignore_stepper_queue/*=false*/) {

  #if HAS_FILRUNOUT
    if ((printing || card.sdprinting) && (READ(FILRUNOUT_PIN) ^ FIL_RUNOUT_INVERTING))
      filrunout();
  #endif

  if (commands_in_queue < BUFSIZE - 1) get_command();

  millis_t ms = millis();

  if (max_inactive_time && ms > previous_cmd_ms + max_inactive_time) kill();

  if (stepper_inactive_time && ms > previous_cmd_ms + stepper_inactive_time
      && !ignore_stepper_queue && !blocks_queued())
    disable_all_steppers();

  #ifdef CHDK // Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive && ms > chdkHigh + CHDK_DELAY) {
      chdkActive = false;
      WRITE(CHDK, LOW);
    }
  #endif

  #if HAS_KILL
    
    // Check if the kill button was pressed and wait just in case it was an accidental
    // key kill key press
    // -------------------------------------------------------------------------------
    static int killCount = 0;   // make the inactivity button a bit less responsive
    const int KILL_DELAY = 750;
    if (!READ(KILL_PIN))
       killCount++;
    else if (killCount > 0)
       killCount--;

    // Exceeded threshold and we can confirm that it was not accidental
    // KILL the machine
    // ----------------------------------------------------------------
    if (killCount >= KILL_DELAY) kill();
  #endif

  #if HAS_HOME
    // Check to see if we have to home, use poor man's debouncer
    // ---------------------------------------------------------
    static int homeDebounceCount = 0;   // poor man's debouncing count
    const int HOME_DEBOUNCE_DELAY = 750;
    if (!READ(HOME_PIN)) {
      if (!homeDebounceCount) {
        enqueuecommands_P(PSTR("G28"));
        LCD_ALERTMESSAGEPGM(MSG_AUTO_HOME);
      }
      if (homeDebounceCount < HOME_DEBOUNCE_DELAY)
        homeDebounceCount++;
      else
        homeDebounceCount = 0;
    }
  #endif
    
  #if HAS_CONTROLLERFAN
    controllerFan(); // Check if fan should be turned on to cool stepper drivers down
  #endif

  #ifdef EXTRUDER_RUNOUT_PREVENT
    if (ms > previous_cmd_ms + EXTRUDER_RUNOUT_SECONDS * 1000)
    if (degHotend(active_extruder) > EXTRUDER_RUNOUT_MINTEMP) {
      bool oldstatus;
      switch(active_extruder) {
        case 0:
          oldstatus = E0_ENABLE_READ;
          enable_e0();
          break;
        #if EXTRUDERS > 1
          case 1:
            oldstatus = E1_ENABLE_READ;
            enable_e1();
            break;
          #if EXTRUDERS > 2
            case 2:
              oldstatus = E2_ENABLE_READ;
              enable_e2();
              break;
            #if EXTRUDERS > 3
              case 3:
                oldstatus = E3_ENABLE_READ;
                enable_e3();
                break;
            #endif
          #endif
        #endif
      }
      float oldepos = current_position[E_AXIS], oldedes = destination[E_AXIS];
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
                      destination[E_AXIS] + EXTRUDER_RUNOUT_EXTRUDE * EXTRUDER_RUNOUT_ESTEPS / axis_steps_per_unit[E_AXIS],
                      EXTRUDER_RUNOUT_SPEED / 60. * EXTRUDER_RUNOUT_ESTEPS / axis_steps_per_unit[E_AXIS], active_extruder, active_driver);
      current_position[E_AXIS] = oldepos;
      destination[E_AXIS] = oldedes;
      plan_set_e_position(oldepos);
      previous_cmd_ms = ms; // refresh_cmd_timeout()
      st_synchronize();
      switch(active_extruder) {
        case 0:
          E0_ENABLE_WRITE(oldstatus);
          break;
        #if EXTRUDERS > 1
          case 1:
            E1_ENABLE_WRITE(oldstatus);
            break;
          #if EXTRUDERS > 2
            case 2:
              E2_ENABLE_WRITE(oldstatus);
              break;
            #if EXTRUDERS > 3
              case 3:
                E3_ENABLE_WRITE(oldstatus);
                break;
            #endif
          #endif
        #endif
      }
    }
  #endif

  #ifdef DUAL_X_CARRIAGE
    // handle delayed move timeout
    if (delayed_move_time && ms > delayed_move_time + 1000 && IsRunning()) {
      // travel moves have been received so enact them
      delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      set_destination_to_current();
      prepare_move();
    }
  #endif

  #ifdef IDLE_OOZING_PREVENT
    if (degHotend(active_extruder) > IDLE_OOZING_MINTEMP && !debugDryrun() && !axis_is_moving && idleoozing_enabled) {
      #ifdef FILAMENTCHANGEENABLE
        if (!filament_changing)
      #endif
      {
        if(degTargetHotend(active_extruder) < IDLE_OOZING_MINTEMP) {
          IDLE_OOZING_retract(false);
        }
        else if((millis() - axis_last_activity) >  IDLE_OOZING_SECONDS*1000) {
          IDLE_OOZING_retract(true);
        }
      }
    }
  #endif
  
  #if defined(SDSUPPORT) && defined(SD_SETTINGS)
    if(IS_SD_INSERTED && !IS_SD_PRINTING) {
      if(!config_readed) {
        ConfigSD_RetrieveSettings(true);
        ConfigSD_StoreSettings();
      }
      else if((millis() - config_last_update) >  SD_CFG_SECONDS*1000) {
        ConfigSD_StoreSettings();
      }
    }
  #endif
  #ifdef TEMP_STAT_LEDS
    handle_status_leds();
  #endif

  #ifdef TEMP_STAT_LEDS
    handle_status_leds();
  #endif

  check_axes_activity();
}

void kill()
{
  cli(); // Stop interrupts
  disable_all_heaters();

  disable_all_steppers();

  #if HAS_POWER_SWITCH
    SET_INPUT(PS_ON_PIN);
  #endif

  ECHO_LM(ER, MSG_ERR_KILLED);
  LCD_ALERTMESSAGEPGM(MSG_KILLED);
  
  // FMC small patch to update the LCD before ending
  sei();   // enable interrupts
  for (int i = 5; i--; lcd_update()) delay(200); // Wait a short time
  cli();   // disable interrupts
  suicide();
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

#if HAS_FILRUNOUT
  void filrunout() {
    if (!filrunoutEnqueued) {
      filrunoutEnqueued = true;
      enqueuecommand("M600");
    }
  }
#endif

void Stop() {
  disable_all_heaters();
  if (IsRunning()) {
    Running = false;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    ECHO_LM(ER, MSG_ERR_STOPPED);
    ECHO_S(PAUSE);
    ECHO_E;
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

#ifdef FAST_PWM_FAN
  void setPwmFrequency(uint8_t pin, int val) {
    val &= 0x07;
    switch(digitalPinToTimer(pin)) {
      #if defined(TCCR0A)
      case TIMER0A:
      case TIMER0B:
  //         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
  //         TCCR0B |= val;
           break;
      #endif

      #if defined(TCCR1A)
      case TIMER1A:
      case TIMER1B:
  //         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
  //         TCCR1B |= val;
           break;
      #endif

      #if defined(TCCR2)
      case TIMER2:
      case TIMER2:
           TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
           TCCR2 |= val;
           break;
      #endif

      #if defined(TCCR2A)
      case TIMER2A:
      case TIMER2B:
           TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
           TCCR2B |= val;
           break;
      #endif

      #if defined(TCCR3A)
      case TIMER3A:
      case TIMER3B:
      case TIMER3C:
           TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
           TCCR3B |= val;
           break;
      #endif

      #if defined(TCCR4A)
      case TIMER4A:
      case TIMER4B:
      case TIMER4C:
           TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
           TCCR4B |= val;
           break;
     #endif

      #if defined(TCCR5A)
      case TIMER5A:
      case TIMER5B:
      case TIMER5C:
           TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
           TCCR5B |= val;
           break;
     #endif

    }
  }
#endif //FAST_PWM_FAN

bool setTargetedHotend(int code) {
  target_extruder = active_extruder;
  if (code_seen('T')) {
    target_extruder = code_value_short();
    if (target_extruder >= EXTRUDERS) {
      ECHO_S(ER);
      switch(code) {
        case 104:
          ECHO_M(MSG_M104_INVALID_EXTRUDER);
          break;
        case 105:
          ECHO_M(MSG_M105_INVALID_EXTRUDER);
          break;
        case 109:
          ECHO_M(MSG_M109_INVALID_EXTRUDER);
          break;
        case 218:
          ECHO_M(MSG_M218_INVALID_EXTRUDER);
          break;
        case 221:
          ECHO_M(MSG_M221_INVALID_EXTRUDER);
          break;
      }
      ECHO_EV(target_extruder);
      return true;
    }
  }
  return false;
}

float calculate_volumetric_multiplier(float diameter) {
  if (!volumetric_enabled || diameter == 0) return 1.0;
  float d2 = diameter * 0.5;
  return 1.0 / (M_PI * d2 * d2);
}

void calculate_volumetric_multipliers() {
  for (int i=0; i<EXTRUDERS; i++)
    volumetric_multiplier[i] = calculate_volumetric_multiplier(filament_size[i]);
}
