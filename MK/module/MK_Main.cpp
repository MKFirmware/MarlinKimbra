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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * About Marlin
 *
 * This firmware is a mashup between Sprinter and grbl.
 *  - https://github.com/kliment/Sprinter
 *  - https://github.com/simen/grbl/tree
 *
 * It has preliminary support for Matthew Roberts advance algorithm
 *  - http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "../base.h"

#if ENABLED(RFID_MODULE)
  MFRC522 RFID522;
#endif

#if ENABLED(M100_FREE_MEMORY_WATCHER)
  void gcode_M100();
#endif

#if ENABLED(SDSUPPORT)
  CardReader card;
#endif

bool Running = true;
bool Printing = false;

uint8_t mk_debug_flags = DEBUG_NONE;

static float feedrate = 1500.0, saved_feedrate;
float current_position[NUM_AXIS] = { 0.0 };
float destination[NUM_AXIS] = { 0.0 };
uint8_t axis_known_position = 0;
uint8_t axis_was_homed = 0;

bool pos_saved = false;
float stored_position[NUM_POSITON_SLOTS][NUM_AXIS];

static long gcode_N, gcode_LastN;

static char* current_command, *current_command_args;
static int cmd_queue_index_r = 0;
static int cmd_queue_index_w = 0;
static int commands_in_queue = 0;
static char command_queue[BUFSIZE][MAX_CMD_SIZE];

float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedrate_multiplier = 100; // 100->1 200->2
int saved_feedrate_multiplier;
int extruder_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS(100);
int density_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS(100);
bool volumetric_enabled = false;
float filament_size[EXTRUDERS] = ARRAY_BY_EXTRUDERS(DEFAULT_NOMINAL_FILAMENT_DIA);
float volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS(1.0);
float home_offset[3] = { 0 };
float hotend_offset[3][HOTENDS];
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };

uint8_t active_extruder = 0;
uint8_t previous_extruder = 0;
uint8_t active_driver = 0;

int fanSpeed = 0;

const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};

// Relative Mode. Enable with G91, disable with G90.
static bool relative_mode = false;

bool cancel_heatup = false;

static int serial_count = 0;

// GCode parameter pointer used by code_seen(), code_value(), etc.
static char* seen_pointer;

// Next Immediate GCode Command pointer. NULL if none.
const char* queued_commands_P = NULL;

const int sensitive_pins[] = SENSITIVE_PINS; ///< Sensitive pin list for M42

// Inactivity shutdown
millis_t previous_cmd_ms = 0;
static millis_t max_inactive_time = 0;
static millis_t stepper_inactive_time = (DEFAULT_STEPPER_DEACTIVE_TIME) * 1000UL;

// Print Job Timer
Stopwatch print_job_timer = Stopwatch();

static uint8_t target_extruder;

bool no_wait_for_cooling = true;
bool software_endstops = true;

unsigned long printer_usage_seconds;
double printer_usage_filament;

#if !MECH(DELTA)
  int xy_travel_speed = XY_TRAVEL_SPEED;
  float zprobe_zoffset = 0;
#endif

#if ENABLED(Z_DUAL_ENDSTOPS) && !MECH(DELTA)
  float z_endstop_adj = 0;
#endif

#if HEATER_USES_AD595
  float ad595_offset[HOTENDS] = ARRAY_BY_HOTENDS1(TEMP_SENSOR_AD595_OFFSET);
  float ad595_gain[HOTENDS] = ARRAY_BY_HOTENDS1(TEMP_SENSOR_AD595_GAIN);
#endif

#if ENABLED(NPR2)
  uint8_t old_color = 99;
#endif

#if ENABLED(RFID_MODULE)
  bool RFID_ON = false;
  unsigned long Spool_ID[EXTRUDERS] = ARRAY_BY_EXTRUDERS (0);
  bool Spool_must_read[EXTRUDERS]   = ARRAY_BY_EXTRUDERS (false);
  bool Spool_must_write[EXTRUDERS]  = ARRAY_BY_EXTRUDERS (false);
#endif

#if HAS(SERVOS)
  Servo servo[NUM_SERVOS];
#endif

#if HAS(SERVO_ENDSTOPS)
  const int servo_endstop_id[] = SERVO_ENDSTOP_IDS;
  const int servo_endstop_angle[][2] = {X_ENDSTOP_SERVO_ANGLES, Y_ENDSTOP_SERVO_ANGLES, Z_ENDSTOP_SERVO_ANGLES};
#endif

#if ENABLED(BARICUDA)
  int baricuda_valve_pressure = 0;
  int baricuda_e_to_p_pressure = 0;
#endif

#if ENABLED(FWRETRACT)

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

#if HAS(POWER_SWITCH)
  bool powersupply = 
    #if ENABLED(PS_DEFAULT_OFF)
      false
    #else
      true
    #endif
  ;
#endif

#if MECH(DELTA)

  #define TOWER_1 X_AXIS
  #define TOWER_2 Y_AXIS
  #define TOWER_3 Z_AXIS

  float delta[3] = { 0.0 };
  float delta_tmp[3] = { 0.0 };
  float endstop_adj[3] = { 0 };
  float diagrod_adj[3] = { 0 };
  float saved_endstop_adj[3] = { 0 };
  float tower_adj[6] = { 0 };
  float delta_radius; // = delta_radius;
  float delta_diagonal_rod; // = DELTA_DIAGONAL_ROD;
  float delta_diagonal_rod_1;
  float delta_diagonal_rod_2;
  float delta_diagonal_rod_3;
  float ac_prec = AUTOCALIBRATION_PRECISION;
  float delta_tower1_x, delta_tower1_y,
        delta_tower2_x, delta_tower2_y,
        delta_tower3_x, delta_tower3_y;
  float base_max_pos[3] = {X_MAX_POS, Y_MAX_POS, Z_MAX_POS};
  float base_home_pos[3] = {X_HOME_POS, Y_HOME_POS, Z_HOME_POS};
  float max_length[3] = {X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH};
  float z_probe_offset[3];
  float bed_level[AUTO_BED_LEVELING_GRID_POINTS][AUTO_BED_LEVELING_GRID_POINTS];
  int delta_grid_spacing[2] = { 0, 0 };
  const float bed_radius = DELTA_PROBABLE_RADIUS;
  const float z_probe_deploy_start_location[] = Z_PROBE_DEPLOY_START_LOCATION;
  const float z_probe_deploy_end_location[] = Z_PROBE_DEPLOY_END_LOCATION;
  const float z_probe_retract_start_location[] = Z_PROBE_RETRACT_START_LOCATION;
  const float z_probe_retract_end_location[] = Z_PROBE_RETRACT_END_LOCATION;
  static float saved_position[3] = { 0.0 };
  static float saved_positions[7][3] = {
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      };
  static float adj_t1_Radius = 0;
  static float adj_t2_Radius = 0;
  static float adj_t3_Radius = 0;
  static float z_offset;
  static float bed_level_c, bed_level_x, bed_level_y, bed_level_z;
  static float bed_safe_z = 45; //used for initial bed probe safe distance (to avoid crashing into bed)
  static float bed_level_ox, bed_level_oy, bed_level_oz;
  static int loopcount;
  static bool home_all_axis = true;
#else
  static bool home_all_axis = true;
#endif

#if MECH(SCARA)
  #define DELTA_SEGMENTS_PER_SECOND SCARA_SEGMENTS_PER_SECOND
  static float delta[3] = { 0 };
  float axis_scaling[3] = { 1, 1, 1 };    // Build size scaling, default to 1
#endif

#if ENABLED(FILAMENT_SENSOR)
  //Variables for Filament Sensor input
  float filament_width_nominal = DEFAULT_NOMINAL_FILAMENT_DIA;  //Set nominal filament width, can be changed with M404 
  bool filament_sensor = false;                                 //M405 turns on filament_sensor control, M406 turns it off 
  float filament_width_meas = DEFAULT_MEASURED_FILAMENT_DIA;    //Stores the measured filament diameter 
  signed char measurement_delay[MAX_MEASUREMENT_DELAY + 1];     //ring buffer to delay measurement  store extruder factor after subtracting 100 
  int delay_index1 = 0;                                         //index into ring buffer
  int delay_index2 = -1;                                        //index into ring buffer - set to -1 on startup to indicate ring buffer needs to be initialized
  float delay_dist = 0;                                         //delay distance counter
  int meas_delay_cm = MEASUREMENT_DELAY_CM;                     //distance delay setting
#endif

#if HAS(FILRUNOUT)
  static bool filrunoutEnqueued = false;
#endif

#if MB(ALLIGATOR)
  float motor_current[DRIVER_EXTRUDERS + 3];
#endif

#if ENABLED(COLOR_MIXING_EXTRUDER)
  float mixing_factor[DRIVER_EXTRUDERS];
  #if MIXING_VIRTUAL_TOOLS  > 1
    float mixing_virtual_tool_mix[MIXING_VIRTUAL_TOOLS][DRIVER_EXTRUDERS];
  #endif
#endif

#if ENABLED(SDSUPPORT)
  static bool fromsd[BUFSIZE];
  #if ENABLED(SD_SETTINGS)
    millis_t config_last_update = 0;
    bool config_readed = false;
  #endif
#endif

#if ENABLED(FILAMENTCHANGEENABLE)
  bool filament_changing = false;
#endif

#if ENABLED(IDLE_OOZING_PREVENT)
  unsigned long axis_last_activity = 0;
  bool IDLE_OOZING_enabled = true;
  bool IDLE_OOZING_retracted[EXTRUDERS] = ARRAY_BY_EXTRUDERS(false);
#endif

#if HAS(POWER_CONSUMPTION_SENSOR)
  float power_consumption_meas = 0.0;
  unsigned long power_consumption_hour;
  unsigned long startpower = 0;
  unsigned long stoppower = 0;
#endif

#if ENABLED(LASERBEAM)
  int laser_ttl_modulation = 0;
#endif

#if ENABLED(NPR2)
  static float color_position[] = COLOR_STEP;
  static float color_step_moltiplicator = (DRIVER_MICROSTEP / MOTOR_ANGLE) * CARTER_MOLTIPLICATOR;
#endif // NPR2

#if ENABLED(EASY_LOAD)
  bool allow_lengthy_extrude_once; // for load/unload
#endif

static bool send_ok[BUFSIZE];

#if HAS(CHDK)
  unsigned long chdkHigh = 0;
  boolean chdkActive = false;
#endif

#if ENABLED(PIDTEMP) && ENABLED(PID_ADD_EXTRUSION_RATE)
  int lpq_len = 20;
#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  // States for managing MK and host communication
  // MK sends messages if blocked or busy
  enum MKBusyState {
    NOT_BUSY,           // Not in a handler
    IN_HANDLER,         // Processing a GCode
    IN_PROCESS,         // Known to be blocking command input (as in G29)
    PAUSED_FOR_USER,    // Blocking pending any input
    PAUSED_FOR_INPUT    // Blocking pending text input (concept)
  };

  static MKBusyState busy_state = NOT_BUSY;
  static millis_t next_busy_signal_ms = 0;
  uint8_t host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;
  #define KEEPALIVE_STATE(n) do{ busy_state = n; }while(0)
#else
  #define host_keepalive() ;
  #define KEEPALIVE_STATE(n) ;
#endif // HOST_KEEPALIVE_FEATURE

/**
 * ***************************************************************************
 * ******************************** FUNCTIONS ********************************
 * ***************************************************************************
 */

void stop();

void get_available_commands();
void process_next_command();

inline void refresh_cmd_timeout() { previous_cmd_ms = millis(); }

void delay_ms(millis_t ms) {
  ms += millis();
  while (millis() < ms) idle();
}

void plan_arc(float target[NUM_AXIS], float* offset, uint8_t clockwise);

#if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
  float extrude_min_temp = EXTRUDE_MINTEMP;
#endif

#if ENABLED(M100_FREE_MEMORY_WATCHER)
  // top_of_stack() returns the location of a variable on its stack frame.  The value returned is above
  // the stack once the function returns to the caller.

  unsigned char* top_of_stack() {
    unsigned char x;
    return &x + 1; // x is pulled on return;
  }

  //
  // 3 support routines to print hex numbers.  We can print a nibble, byte and word
  //
  void prt_hex_nibble( unsigned int n ) {
    if ( n <= 9 )
      ECHO_V(n);
    else
      ECHO_V( (char) ('A'+n-10) );
    HAL::delayMilliseconds(2);
  }

  void prt_hex_byte(unsigned int b) {
    prt_hex_nibble( ( b & 0xf0 ) >> 4 );
    prt_hex_nibble(  b & 0x0f );
  }

  void prt_hex_word(unsigned int w) {
    prt_hex_byte( ( w & 0xff00 ) >> 8 );
    prt_hex_byte(  w & 0x0ff );
  }

  // how_many_E5s_are_here() is a utility function to easily find out how many 0xE5's are
  // at the specified location.  Having this logic as a function simplifies the search code.
  //
  int how_many_E5s_are_here( unsigned char* p) {
    int n;

    for(n = 0; n < 32000; n++) {
      if ( *(p+n) != (unsigned char) 0xe5)
        return n-1;
    }
    return -1;
  }

#endif

/**
 * Inject the next command from the command queue, when possible
 * Return false only if no command was pending
 */
static bool drain_queued_commands_P() {
  if (queued_commands_P != NULL) {
    size_t i = 0;
    char c, cmd[30];
    strncpy_P(cmd, queued_commands_P, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';
    while ((c = cmd[i]) && c != '\n') i++; // find the end of this gcode command
    cmd[i] = '\0';
    if (enqueue_and_echo_command(cmd)) {   // success?
      if (c)                               // newline char?
        queued_commands_P += i + 1;        // advance to the next command
      else
        queued_commands_P = NULL;          // nul char? no more commands
    }
  }
  return (queued_commands_P != NULL);      // return whether any more remain
}

/**
 * Record one or many commands to run from program memory.
 * Aborts the current queue, if any.
 * Note: drain_queued_commands_P() must be called repeatedly to drain the commands afterwards
 */
void enqueue_and_echo_commands_P(const char* pgcode) {
  queued_commands_P = pgcode;
  drain_queued_commands_P(); // first command executed asap (when possible)
}

/**
 * Once a new command is in the ring buffer, call this to commit it
 */
inline void _commit_command(bool say_ok) {
  send_ok[cmd_queue_index_w] = say_ok;
  cmd_queue_index_w = (cmd_queue_index_w + 1) % BUFSIZE;
  commands_in_queue++;
}

/**
 * Copy a command directly into the main command buffer, from RAM.
 * Returns true if successfully adds the command
 */
inline bool _enqueuecommand(const char* cmd, bool say_ok = false) {
  if (*cmd == ';' || commands_in_queue >= BUFSIZE) return false;
  strcpy(command_queue[cmd_queue_index_w], cmd);
  _commit_command(say_ok);
  return true;
}

void enqueue_and_echo_command_now(const char* cmd) {
  while (!enqueue_and_echo_command(cmd)) idle();
}

/**
 * Enqueue with Serial Echo
 */
bool enqueue_and_echo_command(const char* cmd, bool say_ok/*=false*/) {
  if (_enqueuecommand(cmd, say_ok)) {
    ECHO_SM(DB, SERIAL_ENQUEUEING);
    ECHO_T(cmd);
    ECHO_EM("\"");
    return true;
  }
  return false;
}

#if MB(ALLIGATOR)
  void setup_alligator_board() {
    // Init Expansion Port Voltage logic Selector
    SET_OUTPUT(EXP_VOLTAGE_LEVEL_PIN);
    WRITE(EXP_VOLTAGE_LEVEL_PIN, UI_VOLTAGE_LEVEL);
    ExternalDac::begin(); // Initialize ExternalDac
    #if HAS(BUZZER)
      buzz(10,10);
    #endif
  }
#endif

#if HAS(KILL)
  void setup_killpin() {
    SET_INPUT(KILL_PIN);
    WRITE(KILL_PIN, HIGH);
  }
#endif

#if HAS(FILRUNOUT)
  void setup_filrunoutpin() {
    pinMode(FILRUNOUT_PIN, INPUT);
    #if ENABLED(ENDSTOPPULLUP_FIL_RUNOUT)
      WRITE(FILRUNOUT_PIN, HIGH);
    #endif
  }
#endif

// Set home pin
#if HAS(HOME)
  void setup_homepin(void) {
    SET_INPUT(HOME_PIN);
    WRITE(HOME_PIN, HIGH);
  }
#endif

#if HAS(PHOTOGRAPH)
  void setup_photpin() {
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  }
#endif

#if ENABLED(LASERBEAM)
  void setup_laserbeampin() {
    OUT_WRITE(LASER_PWR_PIN, LOW);
    OUT_WRITE(LASER_TTL_PIN, LOW);
  }
#endif

#if HAS(POWER_SWITCH)
  void setup_powerhold() {
    #if HAS(SUICIDE)
      OUT_WRITE(SUICIDE_PIN, HIGH);
    #endif
    #if ENABLED(PS_DEFAULT_OFF)
      OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #else
      OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
    #endif
  }
#endif

#if HAS(SUICIDE)
  void suicide() {
    OUT_WRITE(SUICIDE_PIN, LOW);
  }
#endif

#if HAS(SERVOS)
  void servo_init() {
    #if HAS(SERVO_0)
      servo[0].attach(SERVO0_PIN);
      servo[0].detach(); // Just set up the pin. We don't have a position yet. Don't move to a random position.
    #endif
    #if HAS(SERVO_1)
      servo[1].attach(SERVO1_PIN);
      servo[1].detach();
    #endif
    #if HAS(SERVO_2)
      servo[2].attach(SERVO2_PIN);
      servo[2].detach();
    #endif
    #if HAS(SERVO_3)
      servo[3].attach(SERVO3_PIN);
      servo[3].detach();
    #endif

    #if ENABLED(DONDOLO)
      servo[DONDOLO_SERVO_INDEX].attach(0);
  		servo[DONDOLO_SERVO_INDEX].write(DONDOLO_SERVOPOS_E0);
  		delay_ms(DONDOLO_SERVO_DELAY);
      servo[DONDOLO_SERVO_INDEX].detach();
  	#endif

    // Set position of Servo Endstops that are defined
    #if HAS(SERVO_ENDSTOPS)
      #if ENABLED(DONDOLO)
        for (int i = 0; i < 3; i++) {
          if (servo_endstop_id[i] >= 0 && servo_endstop_id[i] != DONDOLO_SERVO_INDEX)
            servo[servo_endstop_id[i]].write(servo_endstop_angle[i][1]);
        }
      #else
        for (int i = 0; i < 3; i++) {
          if (servo_endstop_id[i] >= 0)
          servo[servo_endstop_id[i]].write(servo_endstop_angle[i][1]);
        }
      #endif
    #endif
  }
#endif

/**
 * Led init
 */
#if ENABLED(TEMP_STAT_LEDS)
  void setup_statled() {
    #if ENABLED(STAT_LED_RED)
      pinMode(STAT_LED_RED, OUTPUT);
      digitalWrite(STAT_LED_RED, LOW); // turn it off
    #endif

    #if ENABLED(STAT_LED_BLUE)
      pinMode(STAT_LED_BLUE, OUTPUT);
      digitalWrite(STAT_LED_BLUE, LOW); // turn it off
    #endif
  }
#endif

#if HAS(Z_PROBE_SLED)
  void setup_zprobesled() {
    pinMode(SLED_PIN, OUTPUT);
    digitalWrite(SLED_PIN, LOW); // turn it off
  }
#endif

/**
 * Stepper Reset (RigidBoard, et.al.)
 */
#if HAS(STEPPER_RESET)
  void disableStepperDrivers() {
    pinMode(STEPPER_RESET_PIN, OUTPUT);
    digitalWrite(STEPPER_RESET_PIN, LOW);  // drive it down to hold in reset motor driver chips
  }
  void enableStepperDrivers() { pinMode(STEPPER_RESET_PIN, INPUT); }  // set to input, which allows it to be pulled high by pullups
#endif

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
  #if HAS(KILL)
    setup_killpin();
  #endif
  #if HAS(FILRUNOUT)
    setup_filrunoutpin();
  #endif
  #if HAS(POWER_SWITCH)
    setup_powerhold();
  #endif
  #if HAS(STEPPER_RESET)
    disableStepperDrivers();
  #endif

  SERIAL_INIT(BAUDRATE);
  ECHO_EM(START);
  HAL::showStartReason();
  ECHO_EM(BUILD_VERSION);

  #if ENABLED(STRING_DISTRIBUTION_DATE) && ENABLED(STRING_CONFIG_H_AUTHOR)
    ECHO_EM(SERIAL_CONFIGURATION_VER STRING_DISTRIBUTION_DATE SERIAL_AUTHOR STRING_CONFIG_H_AUTHOR);
    ECHO_EM(SERIAL_COMPILED __DATE__);
  #endif // STRING_DISTRIBUTION_DATE

  ECHO_MV(SERIAL_FREE_MEMORY, HAL::getFreeRam());
  ECHO_EMV(SERIAL_PLANNER_BUFFER_BYTES, (int)sizeof(block_t)*BLOCK_BUFFER_SIZE);

  #if ENABLED(SDSUPPORT)
    for (uint8_t i = 0; i < BUFSIZE; i++) fromsd[i] = false;
  #endif

  // loads custom configuration from SDCARD if available else uses defaults
  ConfigSD_RetrieveSettings();

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();

  lcd_init();   // Initialize LCD
  tp_init();    // Initialize temperature loop
  plan_init();  // Initialize planner;

  #if ENABLED(USE_WATCHDOG)
    watchdog_init();
  #endif

  st_init();    // Initialize stepper, this enables interrupts!

  #if HAS(PHOTOGRAPH)
    setup_photpin();
  #endif

  #if ENABLED(LASERBEAM)
    setup_laserbeampin();
  #endif

  #if HAS(SERVOS)
    servo_init();
  #endif

  #if HAS(STEPPER_RESET)
    enableStepperDrivers();
  #endif

  #if ENABLED(DIGIPOT_I2C)
    digipot_i2c_init();
  #endif

  #if HAS(Z_PROBE_SLED)
    setup_zprobesled();
  #endif

  #if HAS(HOME)
    setup_homepin();
  #endif

  #if ENABLED(TEMP_STAT_LEDS)
    setup_statled();
  #endif

  #if ENABLED(COLOR_MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1
    // Initialize mixing to 100% color 1
    for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
      mixing_factor[i] = (i == 0) ? 1 : 0;
    }
    for (uint8_t t = 0; t < MIXING_VIRTUAL_TOOLS; t++) {
      for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
        mixing_virtual_tool_mix[t][i] = mixing_factor[i];
      }
    }
  #endif

  #if ENABLED(RFID_MODULE)
    RFID_ON = RFID522.init();
    if (RFID_ON)
      ECHO_LM(INFO, "RFID CONNECT");
  #endif

  #if ENABLED(FIRMWARE_TEST)
    FirmwareTest();
  #endif
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
  if (commands_in_queue < BUFSIZE - 1) get_available_commands();

  #if ENABLED(SDSUPPORT)
    card.checkautostart(false);
  #endif

  if (commands_in_queue) {

    #if ENABLED(SDSUPPORT)

      if (card.saving) {
        char* command = command_queue[cmd_queue_index_r];
        if (strstr_P(command, PSTR("M29"))) {
          // M29 closes the file
          card.finishWrite();
        }
        else {
          // Write the string from the read buffer to SD
          card.write_command(command);
          ECHO_L(OK);
        }
      }
      else
        process_next_command();

    #else

      process_next_command();

    #endif // SDSUPPORT

    commands_in_queue--;
    cmd_queue_index_r = (cmd_queue_index_r + 1) % BUFSIZE;
  }
  checkHitEndstops();
  idle();
}

void gcode_line_error(const char* err, bool doFlush = true) {
  ECHO_ST(ER, err);
  ECHO_EV(gcode_LastN);
  //Serial.println(gcode_N);
  if (doFlush) FlushSerialRequestResend();
  serial_count = 0;
}

inline void get_serial_commands() {
  static char serial_line_buffer[MAX_CMD_SIZE];
  static boolean serial_comment_mode = false;

  // If the command buffer is empty for too long,
  // send "wait" to indicate Marlin is still waiting.
  #if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
    static millis_t last_command_time = 0;
    millis_t ms = millis();
    if (!MKSERIAL.available() && commands_in_queue == 0 && ELAPSED(ms, last_command_time + NO_TIMEOUTS)) {
      ECHO_L(WT);
      last_command_time = ms;
    }
  #endif

  /**
   * Loop while serial characters are incoming and the queue is not full
   */
  while (MKSERIAL.available() > 0 && commands_in_queue < BUFSIZE) {

    char serial_char = MKSERIAL.read();

    /**
     * If the character ends the line
     */
    if (serial_char == '\n' || serial_char == '\r') {

      serial_comment_mode = false; // end of line == end of comment

      if (!serial_count) return; // skip empty lines

      serial_line_buffer[serial_count] = 0; // terminate string
      serial_count = 0; //reset buffer

      char* command = serial_line_buffer;

      while (*command == ' ') command++; // skip any leading spaces
      char* npos = (*command == 'N') ? command : NULL; // Require the N parameter to start the line
      char* apos = strchr(command, '*');

      if (npos) {

        boolean M110 = strstr_P(command, PSTR("M110")) != NULL;

        if (M110) {
          char* n2pos = strchr(command + 4, 'N');
          if (n2pos) npos = n2pos;
        }

        gcode_N = strtol(npos + 1, NULL, 10);

        if (gcode_N != gcode_LastN + 1 && !M110) {
          gcode_line_error(PSTR(SERIAL_ERR_LINE_NO));
          return;
        }

        if (apos) {
          byte checksum = 0, count = 0;
          while (command[count] != '*') checksum ^= command[count++];

          if (strtol(apos + 1, NULL, 10) != checksum) {
            gcode_line_error(PSTR(SERIAL_ERR_CHECKSUM_MISMATCH));
            return;
          }
          // if no errors, continue parsing
        }
        else {
          gcode_line_error(PSTR(SERIAL_ERR_NO_CHECKSUM));
          return;
        }

        gcode_LastN = gcode_N;
        // if no errors, continue parsing

      }
      else if (apos) { // No '*' without 'N'
        gcode_line_error(PSTR(SERIAL_ERR_NO_LINENUMBER_WITH_CHECKSUM), false);
        return;
      }

      // Movement commands alert when stopped
      if (IsStopped()) {
        char* gpos = strchr(command, 'G');
        if (gpos) {
          int codenum = strtol(gpos + 1, NULL, 10);
          switch (codenum) {
            case 0:
            case 1:
            case 2:
            case 3:
              ECHO_LM(ER, SERIAL_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
              break;
          }
        }
      }

      // If command was e-stop process now
      if (strcmp(command, "M112") == 0) kill(PSTR(MSG_KILLED));

      #if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
        last_command_time = ms;
      #endif

      // Add the command to the queue
      _enqueuecommand(serial_line_buffer, true);
    }
    else if (serial_count >= MAX_CMD_SIZE - 1) {
      // Keep fetching, but ignore normal characters beyond the max length
      // The command will be injected when EOL is reached
    }
    else if (serial_char == '\\') { // Handle escapes
      if (MKSERIAL.available() > 0) {
        // if we have one more character, copy it over
        serial_char = MKSERIAL.read();
        if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
      }
      // otherwise do nothing
    }
    else { // its not a newline, carriage return or escape char
      if (serial_char == ';') serial_comment_mode = true;
      if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
    }
  } // queue has space, serial has data
}

#if ENABLED(SDSUPPORT)
  inline void get_sdcard_commands() {
    static bool stop_buffering = false,
                sd_comment_mode = false;

    if (!card.sdprinting) return;

    /**
     * '#' stops reading from SD to the buffer prematurely, so procedural
     * macro calls are possible. If it occurs, stop_buffering is triggered
     * and the buffer is run dry; this character _can_ occur in serial com
     * due to checksums, however, no checksums are used in SD printing.
     */

    if (commands_in_queue == 0) stop_buffering = false;

    uint16_t sd_count = 0;
    bool card_eof = card.eof();
    while (commands_in_queue < BUFSIZE && !card_eof && !stop_buffering) {
      int16_t n = card.get();
      char sd_char = (char)n;
      card_eof = card.eof();
      if (card_eof || n == -1
          || sd_char == '\n' || sd_char == '\r'
          || ((sd_char == '#' || sd_char == ':') && !sd_comment_mode)
      ) {
        if (card_eof) {
          ECHO_EM(SERIAL_FILE_PRINTED);
          print_job_timer.stop();
          char time[30];
          millis_t t = print_job_timer.duration();
          int hours = t / 60 / 60, minutes = (t / 60) % 60;
          sprintf_P(time, PSTR("%i " MSG_END_HOUR " %i " MSG_END_MINUTE), hours, minutes);
          ECHO_LT(DB, time);
          lcd_setstatus(time, true);
          card.printingHasFinished();
          card.checkautostart(true);
        }
        if (sd_char == '#') stop_buffering = true;

        sd_comment_mode = false; // for new command

        if (!sd_count) continue; // skip empty lines

        command_queue[cmd_queue_index_w][sd_count] = '\0'; // terminate string
        sd_count = 0; // clear buffer

        _commit_command(false);
      }
      else if (sd_count >= MAX_CMD_SIZE - 1) {
        /**
         * Keep fetching, but ignore normal characters beyond the max length
         * The command will be injected when EOL is reached
         */
      }
      else {
        if (sd_char == ';') sd_comment_mode = true;
        if (!sd_comment_mode) command_queue[cmd_queue_index_w][sd_count++] = sd_char;
      }
    }
  }
#endif // SDSUPPORT

/**
 * Add to the circular command queue the next command from:
 *  - The command-injection queue (queued_commands_P)
 *  - The active serial input (usually USB)
 *  - The SD card file being actively printed
 */
void get_available_commands() {

  // if any immediate commands remain, don't get other commands yet
  if (drain_queued_commands_P()) return;

  get_serial_commands();

  #if ENABLED(SDSUPPORT)
    get_sdcard_commands();
  #endif
}

bool code_has_value() {
  int i = 1;
  char c = seen_pointer[i];
  while (c == ' ') c = seen_pointer[++i];
  if (c == '-' || c == '+') c = seen_pointer[++i];
  if (c == '.') c = seen_pointer[++i];
  return NUMERIC(c);
}

float code_value() {
  float ret;
  char* e = strchr(seen_pointer, 'E');
  if (e) {
    *e = 0;
    ret = strtod(seen_pointer + 1, NULL);
    *e = 'E';
  }
  else
    ret = strtod(seen_pointer + 1, NULL);
  return ret;
}

long code_value_long() { return strtol(seen_pointer + 1, NULL, 10); }

int16_t code_value_short() { return (int16_t)strtol(seen_pointer + 1, NULL, 10); }

bool code_seen(char code) {
  seen_pointer = strchr(current_command_args, code);
  return (seen_pointer != NULL); // Return TRUE if the code-letter was found
}

/**
 * Set target_extruder from the T parameter or the active_extruder
 *
 * Returns TRUE if the target is invalid
 */
bool setTargetedExtruder(int code) {
  if (code_seen('T')) {
    short t = code_value_short();
    if (t >= EXTRUDERS) {
      ECHO_SMV(ER, "M", code);
      ECHO_EMV(" " SERIAL_INVALID_EXTRUDER, t);
      return true;
    }
    target_extruder = t;
  }
  else
    target_extruder = active_extruder;

  return false;
}

/**
 * Set target_Hotend from the T parameter or the active_extruder
 *
 * Returns TRUE if the target is invalid
 */
bool setTargetedHotend(int code) {
  if (code_seen('H')) {
    short h = code_value_short();
    if (h >= HOTENDS) {
      ECHO_SMV(ER, "M", code);
      ECHO_EMV(" " SERIAL_INVALID_HOTEND, h);
      return true;
    }
    target_extruder = h;
  }
  else
    target_extruder = active_extruder;

  return false;
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

#if MECH(CARTESIAN) || MECH(COREXY) || MECH(COREYX) || MECH(COREXZ) || MECH(COREZX) || MECH(SCARA)
  XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,  MAX_POS);
  XYZ_CONSTS_FROM_CONFIG(float, base_home_pos, HOME_POS);
  XYZ_CONSTS_FROM_CONFIG(float, max_length,    MAX_LENGTH);
#endif
XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm,    HOME_BUMP_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

#if ENABLED(DUAL_X_CARRIAGE)

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
  static float duplicate_hotend_x_offset = DEFAULT_DUPLICATION_X_OFFSET; // used in mode 2
  static float duplicate_extruder_temp_offset = 0; // used in mode 2
  bool extruder_duplication_enabled = false; // used in mode 2

#endif //DUAL_X_CARRIAGE

void print_xyz(const char* prefix, const float x, const float y, const float z, bool eol = true) {
  ECHO_T(prefix);
  ECHO_MV(": (", x);
  ECHO_MV(", ", y);
  ECHO_MV(", ", z);
  ECHO_M(")");
  if (eol) ECHO_E;
}

void print_xyz(const char* prefix, const float xyz[], bool eol = true) {
  print_xyz(prefix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS], eol);
}

static void set_axis_is_at_home(AxisEnum axis) {
  #if ENABLED(DUAL_X_CARRIAGE)
    if (axis == X_AXIS) {
      if (active_extruder != 0) {
        current_position[X_AXIS] = x_home_pos(active_extruder);
        min_pos[X_AXIS] = X2_MIN_POS;
        max_pos[X_AXIS] = max(hotend_offset[X_AXIS][1], X2_MAX_POS);
        return;
      } else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE) {
        float xoff = home_offset[X_AXIS];
        current_position[X_AXIS] = base_home_pos(X_AXIS) + xoff;
        min_pos[X_AXIS] = base_min_pos(X_AXIS) + xoff;
        max_pos[X_AXIS] = min(base_max_pos(X_AXIS) + xoff, max(hotend_offset[X_AXIS][1], X2_MAX_POS) - duplicate_hotend_x_offset);
        return;
      }
    }
  #endif

  #if MECH(SCARA)
    if (axis == X_AXIS || axis == Y_AXIS) {
      float homeposition[3];
      for (int i = 0; i < 3; i++) homeposition[i] = base_home_pos(i);
      // ECHO_MV("homeposition[x]= ", homeposition[0]);
      // ECHO_EMV("homeposition[y]= ", homeposition[1]);
      // Works out real Home position angles using inverse kinematics, 
      // and calculates homing offset using forward kinematics
      calculate_delta(homeposition);
      // ECHO_MV("base Theta= ", delta[X_AXIS]);
      // ECHO_EMV(" base Psi+Theta=", delta[Y_AXIS]);
      for (int i = 0; i < 2; i++) delta[i] -= home_offset[i];
      // ECHO_MV("addhome X=", home_offset[X_AXIS]);
      // ECHO_MV(" addhome Y=", home_offset[Y_AXIS]);
      // ECHO_MV(" addhome Theta=", delta[X_AXIS]);
      // ECHO_EMV(" addhome Psi+Theta=", delta[Y_AXIS]);
      calculate_SCARA_forward_Transform(delta);
      // ECHO_MV("Delta X=", delta[X_AXIS]);
      // ECHO_EMV(" Delta Y=", delta[Y_AXIS]);
      current_position[axis] = delta[axis];
      // SCARA home positions are based on configuration since the actual limits are determined by the 
      // inverse kinematic transform.
      min_pos[axis] = base_min_pos(axis); // + (delta[axis] - base_home_pos(axis));
      max_pos[axis] = base_max_pos(axis); // + (delta[axis] - base_home_pos(axis));
    } else {
      current_position[axis] = base_home_pos(axis) + home_offset[axis];
               min_pos[axis] = base_min_pos(axis)  + home_offset[axis];
               max_pos[axis] = base_max_pos(axis)  + home_offset[axis];
    }
  #elif MECH(DELTA)
    current_position[axis] = base_home_pos[axis] + home_offset[axis];
             min_pos[axis] = base_min_pos(axis)  + home_offset[axis];
             max_pos[axis] = base_max_pos[axis]  + home_offset[axis];
  #else
    current_position[axis] = base_home_pos(axis) + home_offset[axis];
             min_pos[axis] = base_min_pos(axis)  + home_offset[axis];
             max_pos[axis] = base_max_pos(axis)  + home_offset[axis];
  #endif

  #if ENABLED(AUTO_BED_LEVELING_FEATURE) && Z_HOME_DIR < 0
    if (axis == Z_AXIS) current_position[Z_AXIS] -= zprobe_zoffset;
  #endif

  if (DEBUGGING(INFO)) {
    ECHO_SMV(INFO, "set_axis_is_at_home ", (unsigned long)axis);
    ECHO_MV(" > (home_offset[axis]==", home_offset[axis]);
    print_xyz(") > current_position", current_position);
  }
}

/**
 * Some planner shorthand inline functions
 */
inline void set_homing_bump_feedrate(AxisEnum axis) {
  const int homing_bump_divisor[] = HOMING_BUMP_DIVISOR;
  int hbd = homing_bump_divisor[axis];
  if (hbd < 1) {
    hbd = 10;
    ECHO_LM(ER, SERIAL_ERR_HOMING_DIV);
  }
  feedrate = homing_feedrate[axis] / hbd;
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
#if MECH(DELTA) || MECH(SCARA)
  inline void sync_plan_position_delta() {
    calculate_delta(current_position);
    plan_set_position(delta[TOWER_1], delta[TOWER_2], delta[TOWER_3], current_position[E_AXIS]);
  }
#endif
inline void set_current_to_destination() { memcpy(current_position, destination, sizeof(current_position)); }
inline void set_destination_to_current() { memcpy(destination, current_position, sizeof(destination)); }

static void setup_for_endstop_move() {
  saved_feedrate = feedrate;
  saved_feedrate_multiplier = feedrate_multiplier;
  feedrate_multiplier = 100;
  refresh_cmd_timeout();
  enable_endstops(true);
}

static void clean_up_after_endstop_move() {
  #if ENABLED(ENDSTOPS_ONLY_FOR_HOMING)
    enable_endstops(false);
  #endif
  feedrate = saved_feedrate;
  feedrate_multiplier = saved_feedrate_multiplier;
  refresh_cmd_timeout();
  endstops_hit_on_purpose(); // clear endstop hit flags
}

#if MECH(CARTESIAN) || MECH(COREXY) || MECH(COREYX) || MECH(COREXZ) || MECH(COREZX) || MECH(SCARA)

  /**
   *  Plan a move to (X, Y, Z) and set the current_position
   *  The final current_position may not be the one that was requested
   */
  static void do_blocking_move_to(float x, float y, float z) {
    float oldFeedRate = feedrate;
    feedrate = homing_feedrate[Z_AXIS];

    if (DEBUGGING(INFO)) {
      ECHO_S(INFO);
      print_xyz("do_blocking_move_to", x, y, z);
    }

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

  inline void do_blocking_move_to_xy(float x, float y) { do_blocking_move_to(x, y, current_position[Z_AXIS]); }
  inline void do_blocking_move_to_x(float x) { do_blocking_move_to(x, current_position[Y_AXIS], current_position[Z_AXIS]); }
  inline void do_blocking_move_to_z(float z) { do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z); }

  #if ENABLED(AUTO_BED_LEVELING_FEATURE)

    #if ENABLED(AUTO_BED_LEVELING_GRID)
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

        if (DEBUGGING(INFO)) {
          ECHO_S(INFO);
          print_xyz("set_bed_level_equation_lsq > current_position", current_position);
        }

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

        if (DEBUGGING(INFO)) {
          ECHO_S(INFO);
          print_xyz("set_bed_level_equation_3pts > current_position", current_position);
        }

        sync_plan_position();
      }

    #endif // AUTO_BED_LEVELING_GRID

    static void run_z_probe() {

      plan_bed_level_matrix.set_to_identity();
      feedrate = homing_feedrate[Z_AXIS];

      // Move down until the probe (or endstop?) is triggered
      float zPosition = -(Z_MAX_LENGTH + 10);
      line_to_z(zPosition);
      st_synchronize();

      // Tell the planner where we ended up - Get this from the stepper handler
      zPosition = st_get_axis_position_mm(Z_AXIS);
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

      // Get the current stepper position after bumping an endstop
      current_position[Z_AXIS] = st_get_axis_position_mm(Z_AXIS);
      sync_plan_position();

      if (DEBUGGING(INFO)) {
        ECHO_S(INFO);
        print_xyz("run_z_probe > current_position", current_position);
      }
    }

    static void deploy_z_probe() {
      if (DEBUGGING(INFO)) {
        ECHO_S(INFO);
        print_xyz("deploy_z_probe > current_position", current_position);
      }
      #if HAS(SERVO_ENDSTOPS)
        // Engage Z Servo endstop if enabled
        if (servo_endstop_id[Z_AXIS] >= 0) servo[servo_endstop_id[Z_AXIS]].move(servo_endstop_angle[Z_AXIS][0]);
      #endif
    }

    static void stow_z_probe(bool doRaise = true) {
      if (DEBUGGING(INFO)) {
        ECHO_S(INFO);
        print_xyz("stow_z_probe > current_position", current_position);
      }
      #if HAS(SERVO_ENDSTOPS)
        // Retract Z Servo endstop if enabled
        if (servo_endstop_id[Z_AXIS] >= 0) {

          #if Z_RAISE_AFTER_PROBING > 0
            if (doRaise) {
              if (DEBUGGING(INFO)) {
                ECHO_LMV(INFO, "Raise Z (after) by ", (float)Z_RAISE_AFTER_PROBING);
                ECHO_LMV(INFO, "> SERVO_ENDSTOPS > do_blocking_move_to_z ", current_position[Z_AXIS] + Z_RAISE_AFTER_PROBING);
              }
              do_blocking_move_to_z(current_position[Z_AXIS] + Z_RAISE_AFTER_PROBING); // this also updates current_position
              st_synchronize();
            }
          #endif

          // Change the Z servo angle
          servo[servo_endstop_id[Z_AXIS]].move(servo_endstop_angle[Z_AXIS][1]);
        }
      #endif
    }

    enum ProbeAction {
      ProbeStay             = 0,
      ProbeDeploy           = _BV(0),
      ProbeStow             = _BV(1),
      ProbeDeployAndStow    = (ProbeDeploy | ProbeStow)
    };

    // Probe bed height at position (x,y), returns the measured z value
    static float probe_pt(float x, float y, float z_before, ProbeAction probe_action = ProbeDeployAndStow, int verbose_level = 1) {
      if (DEBUGGING(INFO)) {
        ECHO_LM(INFO, "probe_pt >>>");
        ECHO_SMV(INFO, "> ProbeAction:", (unsigned long)probe_action);
        print_xyz(" > current_position", current_position);
        ECHO_SMV(INFO, "Z Raise to z_before ", z_before);
        ECHO_EMV(" > do_blocking_move_to_z ", z_before);
      }

      // Move Z up to the z_before height, then move the probe to the given XY
      do_blocking_move_to_z(z_before); // this also updates current_position

      if (DEBUGGING(INFO)) {
        ECHO_SMV(INFO, "> do_blocking_move_to_xy ", x - (X_PROBE_OFFSET_FROM_EXTRUDER));
        ECHO_EMV(", ", y - Y_PROBE_OFFSET_FROM_EXTRUDER);
      }

      do_blocking_move_to_xy(x - X_PROBE_OFFSET_FROM_EXTRUDER, y - (Y_PROBE_OFFSET_FROM_EXTRUDER)); // this also updates current_position

      #if HASNT(Z_PROBE_SLED)
        if (probe_action & ProbeDeploy) {
          if (DEBUGGING(INFO)) ECHO_LM(INFO, "> ProbeDeploy");
          deploy_z_probe();
        }
      #endif

      run_z_probe();
      float measured_z = current_position[Z_AXIS];

      #if HASNT(Z_PROBE_SLED)
        if (probe_action & ProbeStow) {
          if (DEBUGGING(INFO)) ECHO_LM(INFO, "> ProbeStow (stow_z_probe will do Z Raise)");
          stow_z_probe();
        }
      #endif

      if (verbose_level > 2) {
        ECHO_SM(DB, SERIAL_BED_LEVELLING_BED);
        ECHO_MV(SERIAL_BED_LEVELLING_X, x, 3);
        ECHO_MV(SERIAL_BED_LEVELLING_Y, y, 3);
        ECHO_EMV(SERIAL_BED_LEVELLING_Z, measured_z, 3);
      }

      if (DEBUGGING(INFO)) ECHO_LM(INFO, "<<< probe_pt");

      return measured_z;
    }
    
    #if HAS(SERVO_ENDSTOPS) && HASNT(Z_PROBE_SLED)
      void raise_z_for_servo() {
        float zpos = current_position[Z_AXIS], z_dest = Z_RAISE_BEFORE_PROBING;
        z_dest += TEST(axis_was_homed, Z_AXIS) ? zprobe_zoffset : zpos;
        if (zpos < z_dest) do_blocking_move_to_z(z_dest); // also updates current_position
      }
    #endif

  #endif //AUTO_BED_LEVELING_FEATURE

  static void homeaxis(AxisEnum axis) {
  #define HOMEAXIS_DO(LETTER) \
    ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

    if (DEBUGGING(INFO)) {
      ECHO_SMV(INFO, ">>> homeaxis(", (unsigned long)axis);
      ECHO_EM(")");
    }

    if (axis == X_AXIS ? HOMEAXIS_DO(X) : axis == Y_AXIS ? HOMEAXIS_DO(Y) : axis == Z_AXIS ? HOMEAXIS_DO(Z) : 0) {

      int axis_home_dir =
      #if ENABLED(DUAL_X_CARRIAGE)
        (axis == X_AXIS) ? x_home_dir(active_extruder) :
      #endif
      home_dir(axis);

      // Set the axis position as setup for the move
      current_position[axis] = 0;
      sync_plan_position();

      #if HAS(Z_PROBE_SLED)
        // Get Probe
        if (axis == Z_AXIS) {
          if (axis_home_dir < 0) dock_sled(false);
        }
      #endif

      #if HAS(SERVO_ENDSTOPS) && HASNT(Z_PROBE_SLED)
        // Deploy a probe if there is one, and homing towards the bed
        if (axis == Z_AXIS) {
          if (axis_home_dir < 0) deploy_z_probe();
        }
      #endif

      #if HAS(SERVO_ENDSTOPS)
        // Engage Servo endstop if enabled
        if (axis != Z_AXIS && servo_endstop_id[axis] >= 0)
          servo[servo_endstop_id[axis]].move(servo_endstop_angle[axis][0]);
      #endif

      // Set a flag for Z motor locking
      #if ENABLED(Z_DUAL_ENDSTOPS)
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

      enable_endstops(false); // Disable endstops while moving away

      // Move away from the endstop by the axis HOME_BUMP_MM
      destination[axis] = -home_bump_mm(axis) * axis_home_dir;
      line_to_destination();
      st_synchronize();

      enable_endstops(true); // Enable endstops for next homing move

      // Slow down the feedrate for the next move
      set_homing_bump_feedrate(axis);

      // Move slowly towards the endstop until triggered
      destination[axis] = 2 * home_bump_mm(axis) * axis_home_dir;
      line_to_destination();
      st_synchronize();

      if (DEBUGGING(INFO)) {
        ECHO_S(INFO);
        print_xyz("> TRIGGER ENDSTOP > current_position", current_position);
      }

      #if ENABLED(Z_DUAL_ENDSTOPS)
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
      set_axis_is_at_home(axis);
      sync_plan_position();

      if (DEBUGGING(INFO)) {
        ECHO_S(INFO);
        print_xyz("> AFTER set_axis_is_at_home > current_position", current_position);
      }

      destination[axis] = current_position[axis];
      feedrate = 0.0;
      endstops_hit_on_purpose(); // clear endstop hit flags
      SBI(axis_was_homed, axis);
      SBI(axis_known_position, axis);

      #if ENABLED(Z_PROBE_SLED)
        // bring probe back
          if (axis == Z_AXIS) {
            if (axis_home_dir < 0) dock_sled(true);
          }
      #endif

      #if HAS(SERVO_ENDSTOPS) && HASNT(Z_PROBE_SLED)
        // Deploy a probe if there is one, and homing towards the bed
        if (axis == Z_AXIS) {
          if (axis_home_dir < 0) {
            if (DEBUGGING(INFO)) ECHO_LM(INFO, "> SERVO_LEVELING > stow_z_probe");
            stow_z_probe();
          }
        }
        else
      #endif
      {
        #if HAS(SERVO_ENDSTOPS)
          // Retract Servo endstop if enabled
          if (servo_endstop_id[axis] >= 0) {
            if (DEBUGGING(INFO)) ECHO_LM(INFO, "> SERVO_ENDSTOPS > Stow with servo.move()");
            servo[servo_endstop_id[axis]].move(servo_endstop_angle[axis][1]);
          }
        #endif
      }
    }
    if (DEBUGGING(INFO)) {
      ECHO_SMV(INFO, "<<< homeaxis(", (unsigned long)axis);
      ECHO_EM(")");
    }
  }
  #define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)
#endif // CARTESIAN || COREXY || COREYX || COREXZ || COREZX || SCARA

#if MECH(DELTA)
  static void homeaxis(AxisEnum axis) {
    #define HOMEAXIS_DO(LETTER) \
      ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

    if (axis == X_AXIS ? HOMEAXIS_DO(X) : 
        axis == Y_AXIS ? HOMEAXIS_DO(Y) :
        axis == Z_AXIS ? HOMEAXIS_DO(Z) :
        0) {

      int axis_home_dir = home_dir(axis);
      current_position[axis] = 0;
      sync_plan_position();

      // Move towards the endstop until an endstop is triggered
      destination[axis] = 1.5 * max_length[axis] * axis_home_dir;
      feedrate = homing_feedrate[axis];
      line_to_destination();
      st_synchronize();

      // Set the axis position as setup for the move
      current_position[axis] = 0;
      sync_plan_position();

      enable_endstops(false); // Disable endstops while moving away

      // Move away from the endstop by the axis HOME_BUMP_MM
      destination[axis] = -home_bump_mm(axis) * axis_home_dir;
      line_to_destination();
      st_synchronize();

      enable_endstops(true); // Enable endstops for next homing move

      // Slow down the feedrate for the next move
      set_homing_bump_feedrate(axis);

      // Move slowly towards the endstop until triggered
      destination[axis] = 2 * home_bump_mm(axis) * axis_home_dir;
      line_to_destination();
      st_synchronize();

      // retrace by the amount specified in endstop_adj
      if (endstop_adj[axis] * axis_home_dir < 0) {
        enable_endstops(false); // Disable endstops while moving away
        sync_plan_position();
        destination[axis] = endstop_adj[axis];
        if (DEBUGGING(INFO)) {
          ECHO_SMV(INFO, "> endstop_adj = ", endstop_adj[axis]);
          print_xyz(" > destination", destination);
        }
        line_to_destination();
        st_synchronize();
        enable_endstops(true); // Enable endstops for next homing move
      }

      if (DEBUGGING(INFO)) ECHO_LMV(INFO, "> endstop_adj * axis_home_dir = ", endstop_adj[axis] * axis_home_dir);

      // Set the axis position to its home position (plus home offsets)
      set_axis_is_at_home(axis);
      sync_plan_position();

      if (DEBUGGING(INFO)) {
        ECHO_S(INFO);
        print_xyz("> AFTER set_axis_is_at_home > current_position", current_position);
      }

      destination[axis] = current_position[axis];
      feedrate = 0.0;
      endstops_hit_on_purpose(); // clear endstop hit flags
      SBI(axis_was_homed, axis);
      SBI(axis_known_position, axis);
    }
  }
  #define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

  void set_delta_constants() {
    max_length[Z_AXIS]    = max_pos[Z_AXIS] - Z_MIN_POS;
    base_max_pos[Z_AXIS]  = max_pos[Z_AXIS];
    base_home_pos[Z_AXIS] = max_pos[Z_AXIS];

    delta_diagonal_rod_1 = pow(delta_diagonal_rod + diagrod_adj[0], 2);
    delta_diagonal_rod_2 = pow(delta_diagonal_rod + diagrod_adj[1], 2);
    delta_diagonal_rod_3 = pow(delta_diagonal_rod + diagrod_adj[2], 2);

    // Effective X/Y positions of the three vertical towers.
    delta_tower1_x = (delta_radius + tower_adj[3]) * cos((210 + tower_adj[0]) * M_PI/180); // front left tower
    delta_tower1_y = (delta_radius + tower_adj[3]) * sin((210 + tower_adj[0]) * M_PI/180); 
    delta_tower2_x = (delta_radius + tower_adj[4]) * cos((330 + tower_adj[1]) * M_PI/180); // front right tower
    delta_tower2_y = (delta_radius + tower_adj[4]) * sin((330 + tower_adj[1]) * M_PI/180); 
    delta_tower3_x = (delta_radius + tower_adj[5]) * cos((90 + tower_adj[2]) * M_PI/180);  // back middle tower
    delta_tower3_y = (delta_radius + tower_adj[5]) * sin((90 + tower_adj[2]) * M_PI/180); 
  }

  bool Equal_AB(const float A, const float B, const float prec = 0.001) {
    if (abs(A - B) <= prec) return true;
    return false;
  }

  static void extrapolate_one_point(int x, int y, int xdir, int ydir) {
    if (bed_level[x][y] != 0.0) {
      return;  // Don't overwrite good values.
    }
    float a = 2 * bed_level[x + xdir][y] - bed_level[x + xdir * 2][y];  // Left to right.
    float b = 2 * bed_level[x][y + ydir] - bed_level[x][y + ydir * 2];  // Front to back.
    float c = 2 * bed_level[x + xdir][y + ydir] - bed_level[x + xdir * 2][y + ydir * 2];  // Diagonal.
    float median = c;  // Median is robust (ignores outliers).
    if (a < b) {
      if (b < c) median = b;
      if (c < a) median = a;
    } else {  // b <= a
      if (c < b) median = b;
      if (a < c) median = a;
    }
    bed_level[x][y] = median;
  }

  // Fill in the unprobed points (corners of circular print surface)
  // using linear extrapolation, away from the center.
  static void extrapolate_unprobed_bed_level() {
    int half = (AUTO_BED_LEVELING_GRID_POINTS - 1) / 2;
    for (int y = 0; y <= half; y++) {
      for (int x = 0; x <= half; x++) {
        if (x + y < 3) continue;
        extrapolate_one_point(half - x, half - y, x > 1 ? +1:0, y > 1 ? +1:0);
        extrapolate_one_point(half + x, half - y, x > 1 ? -1:0, y > 1 ? +1:0);
        extrapolate_one_point(half - x, half + y, x > 1 ? +1:0, y > 1 ? -1:0);
        extrapolate_one_point(half + x, half + y, x > 1 ? -1:0, y > 1 ? -1:0);
      }
    }
  }

  // Print calibration results for plotting or manual frame adjustment.
  static void print_bed_level() {
    for (int y = 0; y < AUTO_BED_LEVELING_GRID_POINTS; y++) {
      ECHO_S(DB);
      for (int x = 0; x < AUTO_BED_LEVELING_GRID_POINTS; x++) {
        ECHO_VM(bed_level[x][y], " ", 3);
      }
      ECHO_E;
    }
  }

  // Reset calibration results to zero.
  void reset_bed_level() {
    if (DEBUGGING(INFO)) ECHO_LM(INFO, "reset_bed_level");
    for (int y = 0; y < AUTO_BED_LEVELING_GRID_POINTS; y++) {
      for (int x = 0; x < AUTO_BED_LEVELING_GRID_POINTS; x++) {
        bed_level[x][y] = 0.0;
      }
    }
  }

  void deploy_z_probe() {
    #if HAS(SERVO_ENDSTOPS)
      // Engage Z Servo endstop if enabled
      if (servo_endstop_id[Z_AXIS] >= 0) servo[servo_endstop_id[Z_AXIS]].move(servo_endstop_angle[Z_AXIS][0]);
    #endif

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
    //destination[Z_AXIS] = 50;
    //prepare_move_raw();

    destination[X_AXIS] = z_probe_retract_start_location[X_AXIS];
    destination[Y_AXIS] = z_probe_retract_start_location[Y_AXIS];
    destination[Z_AXIS] = z_probe_retract_start_location[Z_AXIS];
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

     #if HAS(SERVO_ENDSTOPS)
      // Retract Z Servo endstop if enabled
      if (servo_endstop_id[Z_AXIS] >= 0)
        // Change the Z servo angle
        servo[servo_endstop_id[Z_AXIS]].move(servo_endstop_angle[Z_AXIS][1]);
    #endif
  }

  void apply_endstop_adjustment(float x_endstop, float y_endstop, float z_endstop) {
    memcpy(saved_endstop_adj, endstop_adj, sizeof(saved_endstop_adj));
    endstop_adj[X_AXIS] += x_endstop;
    endstop_adj[Y_AXIS] += y_endstop;
    endstop_adj[Z_AXIS] += z_endstop;

    calculate_delta(current_position);
    plan_set_position(delta[TOWER_1] - (endstop_adj[X_AXIS] - saved_endstop_adj[X_AXIS]) , delta[TOWER_2] - (endstop_adj[Y_AXIS] - saved_endstop_adj[Y_AXIS]), delta[TOWER_3] - (endstop_adj[Z_AXIS] - saved_endstop_adj[Z_AXIS]), current_position[E_AXIS]);  
    st_synchronize();
  }

  void adj_endstops() {
    boolean x_done = false;
    boolean y_done = false;
    boolean z_done = false;
    float prv_bed_level_x, prv_bed_level_y, prv_bed_level_z;

    do {
      bed_level_z = probe_bed(0.0, bed_radius);
      bed_level_x = probe_bed(-SIN_60 * bed_radius, -COS_60 * bed_radius);
      bed_level_y = probe_bed(SIN_60 * bed_radius, -COS_60 * bed_radius);

      apply_endstop_adjustment(bed_level_x, bed_level_y, bed_level_z);

      ECHO_SMV(DB, "x:", bed_level_x, 4);
      ECHO_MV(" (adj:", endstop_adj[0], 4);
      ECHO_MV(") y:", bed_level_y, 4);
      ECHO_MV(" (adj:", endstop_adj[1], 4);
      ECHO_MV(") z:", bed_level_z, 4);
      ECHO_MV(" (adj:", endstop_adj[2], 4);
      ECHO_EM(")");

      if ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)) {
        x_done = true;
        ECHO_SM(DB, "X=OK ");
      }
      else {
        x_done = false;
        ECHO_SM(DB, "X=ERROR ");
      }

      if ((bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)) {
        y_done = true;
        ECHO_M("Y=OK ");
      }
      else {
        y_done = false;
        ECHO_M("Y=ERROR ");
      }

      if ((bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)) {
        z_done = true;
        ECHO_EM("Z=OK");
      }
      else {
        z_done = false;
        ECHO_EM("Z=ERROR");
      }
    } while (((x_done == false) or (y_done == false) or (z_done == false)));

    float high_endstop = max(max(endstop_adj[0], endstop_adj[1]), endstop_adj[2]);

    if (DEBUGGING(INFO)) {
      ECHO_LMV(INFO, "High endstop: ", high_endstop, 4);
    }

    if (high_endstop > 0) {
      ECHO_LMV(DB, "Reducing Build height by ", high_endstop);
      for(uint8_t i = 0; i < 3; i++) {
        endstop_adj[i] -= high_endstop;
      }
      max_pos[Z_AXIS] -= high_endstop;
    }
    else if (high_endstop < 0) {
      ECHO_LMV(DB, "Increment Build height by ", abs(high_endstop));
      for(uint8_t i = 0; i < 3; i++) {
        endstop_adj[i] -= high_endstop;
      }
      max_pos[Z_AXIS] -= high_endstop;
    }

    set_delta_constants();

    bed_safe_z = 20;
  }

  int fix_tower_errors() {
    boolean t1_err, t2_err, t3_err;
    boolean xy_equal, xz_equal, yz_equal;
    float saved_tower_adj[6];
    int err_tower;
    float low_diff, high_diff;
    float x_diff, y_diff, z_diff;
    float xy_diff, yz_diff, xz_diff;
    float low_opp, high_opp;

    for (uint8_t i = 0; i < 6; i++) saved_tower_adj[i] = tower_adj[i];

    err_tower = 0;

    x_diff = abs(bed_level_x - bed_level_ox);
    high_diff = x_diff;
    y_diff = abs(bed_level_y - bed_level_oy);
    if (y_diff > high_diff) high_diff = y_diff;
    z_diff = abs(bed_level_z - bed_level_oz);
    if (z_diff > high_diff) high_diff = z_diff;

    if (x_diff <= ac_prec) t1_err = false; else t1_err = true;
    if (y_diff <= ac_prec) t2_err = false; else t2_err = true;
    if (z_diff <= ac_prec) t3_err = false; else t3_err = true;

    ECHO_LMV(DB, "x_diff = ", x_diff, 5);
    ECHO_LMV(DB, "y_diff = ", y_diff, 5);
    ECHO_LMV(DB, "z_diff = ", z_diff, 5);
    ECHO_LMV(DB, "high_diff = ", high_diff, 5);

    // Are all errors equal? (within defined precision)
    xy_equal = false;
    xz_equal = false;
    yz_equal = false;
    if (Equal_AB(x_diff, y_diff, ac_prec)) xy_equal = true;
    if (Equal_AB(x_diff, z_diff, ac_prec)) xz_equal = true;
    if (Equal_AB(y_diff, z_diff, ac_prec)) yz_equal = true;

    ECHO_SM(DB, "xy_equal = ");
    if (xy_equal == true) ECHO_EM("true"); else ECHO_EM("false");
    ECHO_SM(DB, "xz_equal = ");
    if (xz_equal == true) ECHO_EM("true"); else ECHO_EM("false");
    ECHO_SM(DB, "yz_equal = ");
    if (yz_equal == true) ECHO_EM("true"); else ECHO_EM("false");

    low_opp = bed_level_ox;
    high_opp = low_opp;
    if (bed_level_oy < low_opp) low_opp = bed_level_oy;
    if (bed_level_oy > high_opp) high_opp = bed_level_oy;
    if (bed_level_oz < low_opp) low_opp = bed_level_oz;
    if (bed_level_oz > high_opp) high_opp = bed_level_oz;

    ECHO_LMV(DB, "Opp Range = ", high_opp - low_opp, 5);

    if (Equal_AB(high_opp, low_opp, ac_prec)) {
      ECHO_LM(DB, "Opposite Points within Limits - Adjustment not required");
      t1_err = false;
      t2_err = false;
      t3_err = false;
    }

    // All Towers have errors
    if ((t1_err == true) and (t2_err == true) and (t3_err == true)) {
      if ((xy_equal == false) or (xz_equal == false) or (yz_equal == false)) {
        // Errors not equal .. select the tower that needs to be adjusted
        if (Equal_AB(high_diff, x_diff, 0.00001)) err_tower = 1;
        if (Equal_AB(high_diff, y_diff, 0.00001)) err_tower = 2;
        if (Equal_AB(high_diff, z_diff, 0.00001)) err_tower = 3;
        ECHO_SMV(DB, "Tower ", err_tower);
        ECHO_EM(" has largest error");
      }
      if ((xy_equal == true) and (xz_equal == true) and (yz_equal == true)) {
        ECHO_LM(DB, "All Towers Errors Equal");
        t1_err = false;
        t2_err = false;
        t3_err = false;
      }
    }

    /*
    // Two tower errors
    if ((t1_err == true) and (t2_err == true) and (t3_err == false)) err_tower = 3;
    if ((t1_err == true) and (t2_err == false) and (t3_err == true)) err_tower = 2;
    if ((t1_err == false) and (t2_err == true) and (t3_err == true)) err_tower = 1;
    */

    // Single tower error
    if ((t1_err == true) and (t2_err == false) and (t3_err == false)) err_tower = 1;
    if ((t1_err == false) and (t2_err == true) and (t3_err == false)) err_tower = 2;
    if ((t1_err == false) and (t2_err == false) and (t3_err == true)) err_tower = 3;

    ECHO_SM(DB, "t1:");
    if (t1_err == true) ECHO_M("Err"); else ECHO_M("OK");
    ECHO_M(" t2:");
    if (t2_err == true) ECHO_M("Err"); else ECHO_M("OK");
    ECHO_M(" t3:");
    if (t3_err == true) ECHO_M("Err"); else ECHO_M("OK");
    ECHO_E;

    if (err_tower == 0) {
      ECHO_LM(DB, "Tower geometry OK");
    }
    else {
      ECHO_SMV(DB, "Tower", int(err_tower));
      ECHO_EM(" Error: Adjusting");
      adj_tower_radius(err_tower);
    }

    //Set return value to indicate if anything has been changed (0 = no change)
    int retval = 0;
    for (uint8_t i = 0; i < 6; i++) if (saved_tower_adj[i] != tower_adj[i]) retval++;
    return retval;
  }

  bool adj_deltaradius() { 
    float adj_r;
    float prev_c;
    uint8_t c_nochange_count = 0;
    float nochange_r;

    bed_level_c = probe_bed(0.0, 0.0);

    if ((bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)) {
      ECHO_LM(DB, "Delta Radius OK");
      return false;
    }
    else {
      ECHO_LM(DB, "Adjusting Delta Radius");
      // set initial direction and magnitude for delta radius adjustment
      adj_r = 0.5;
      if (bed_level_c > 0) adj_r = -0.5;

      bed_safe_z = Z_RAISE_BETWEEN_PROBINGS - z_probe_offset[Z_AXIS];

      do {
        delta_radius += adj_r;
        set_delta_constants();

        prev_c = bed_level_c;
        bed_level_c = probe_bed(0.0, 0.0);

        //Show progress
        ECHO_SMV(DB, "r:", delta_radius, 4);
        ECHO_MV(" (adj:", adj_r, 6);
        ECHO_EMV(") c:", bed_level_c, 4);

        //Adjust delta radius
        if (((adj_r > 0) and (bed_level_c < prev_c)) or ((adj_r < 0) and (bed_level_c > prev_c))) adj_r = -(adj_r / 2);

        //Count iterations with no change to c probe point
        if (Equal_AB(bed_level_c, prev_c)) c_nochange_count ++;
        if (c_nochange_count == 1) nochange_r = delta_radius;

      } while(((bed_level_c < -ac_prec) or (bed_level_c > ac_prec)) and (c_nochange_count < 3));

      if (c_nochange_count > 0) {
        delta_radius = nochange_r;
        set_delta_constants();
        bed_safe_z = Z_RAISE_BETWEEN_PROBINGS - z_probe_offset[Z_AXIS];
      }
      return true;
    }
  }

  void adj_tower_radius(int tower) {
    boolean done,t1_done,t2_done,t3_done;
    int nochange_count;
    float target, prev_target, prev_bed_level;
    float temp, adj_target;

    //Set inital tower adjustment values
    adj_t1_Radius = 0;
    adj_t2_Radius = 0;
    adj_t3_Radius = 0;
    nochange_count = 0;

    if ((tower == 1) and (adj_t1_Radius == 0)) {
      target = (bed_level_oy + bed_level_oz) / 2;
      temp = (bed_level_ox - target) / 2;
      adj_target = target + temp;
      if (bed_level_ox < adj_target) adj_t1_Radius = -0.4;
      if (bed_level_ox > adj_target) adj_t1_Radius = 0.4;
    }
    if ((tower == 2) and (adj_t2_Radius == 0)) {
      target = (bed_level_ox + bed_level_oz) / 2;
      temp = (bed_level_oy - target) / 2;
      adj_target = target + temp;
      if (bed_level_oy < adj_target) adj_t2_Radius = -0.4;
      if (bed_level_oy > adj_target) adj_t2_Radius = 0.4;
    }
    if ((tower == 3) and (adj_t3_Radius == 0)) {
      target = (bed_level_oy + bed_level_ox) / 2;
      temp = (bed_level_oz - target) / 2;
      adj_target = target + temp;
      if (bed_level_oz < adj_target) adj_t3_Radius = -0.4; //0.4;
      if (bed_level_oz > adj_target) adj_t3_Radius = 0.4; //-0.4;
    }

    do {
      tower_adj[3] += adj_t1_Radius;
      tower_adj[4] += adj_t2_Radius;
      tower_adj[5] += adj_t3_Radius;
      set_delta_constants();

      //done = false;
      t1_done = false;
      t2_done = false;
      t3_done = false;
      if (tower == 1) {
        t2_done = true;
        t3_done = true;
        prev_target = adj_target;
        prev_bed_level = bed_level_ox;

        bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
        bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
        bed_level_oz = probe_bed(0.0, -bed_radius);

        target = (bed_level_oy + bed_level_oz) / 2;
        temp = (bed_level_ox - target) / 2;
        adj_target = target + temp;
        if (((bed_level_ox < adj_target) and (adj_t1_Radius > 0)) or ((bed_level_ox > adj_target) and (adj_t1_Radius < 0))) adj_t1_Radius = -(adj_t1_Radius / 2);
        if (Equal_AB(bed_level_ox, adj_target)) t1_done = true;
        if (Equal_AB(bed_level_ox, prev_bed_level) and Equal_AB(adj_target, prev_target)) nochange_count ++;
        if (nochange_count > 1) {
          ECHO_LM(DB, "Stuck in Loop.. Exiting");
          t1_done = true;
        }

        ECHO_SMV(DB, "target:", adj_target, 6);
        ECHO_MV(" ox:", bed_level_ox, 6);
        ECHO_MV(" tower radius adj:", tower_adj[3], 6);
        if (t1_done == true) ECHO_EM(" done:true"); else ECHO_EM(" done:false");
      }

      if (tower == 2) {
        t1_done = true;
        t3_done = true;
        prev_target = adj_target;
        prev_bed_level = bed_level_oy;

        bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
        bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
        bed_level_oz = probe_bed(0.0, -bed_radius);

        target = (bed_level_ox + bed_level_oz) /2;
        temp = (bed_level_oy - target) / 2;
        adj_target = target + temp;
        if (((bed_level_oy < adj_target) and (adj_t2_Radius > 0)) or ((bed_level_oy > adj_target) and (adj_t2_Radius < 0))) adj_t2_Radius = -(adj_t2_Radius / 2);
        if (Equal_AB(bed_level_oy, adj_target)) t2_done = true;
        if (Equal_AB(bed_level_oy, prev_bed_level) and Equal_AB(adj_target, prev_target)) nochange_count ++;
        if (nochange_count > 1) {
          ECHO_LM(DB, "Stuck in Loop.. Exiting");
          t2_done = true;
        }

        ECHO_SMV(DB, "target:", adj_target, 6);
        ECHO_MV(" oy:", bed_level_oy, 6);
        ECHO_MV(" tower radius adj:", tower_adj[4], 6);
        if (t2_done == true) ECHO_EM(" done:true"); else ECHO_EM(" done:false");
      }

      if (tower == 3) {
        t1_done = true;
        t2_done = true;
        prev_target = adj_target;
        prev_bed_level = bed_level_oz;

        bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
        bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
        bed_level_oz = probe_bed(0.0, -bed_radius);

        target = (bed_level_oy + bed_level_ox) / 2;
        temp = (bed_level_oz - target) / 2;
        adj_target = target + temp;
        if (((bed_level_oz < adj_target) and (adj_t3_Radius > 0)) or ((bed_level_oz > adj_target) and (adj_t3_Radius < 0))) adj_t3_Radius = -(adj_t3_Radius / 2);
        if (Equal_AB(bed_level_oz, adj_target)) t3_done = true;
        if (Equal_AB(bed_level_oz, prev_bed_level) and Equal_AB(adj_target, prev_target)) nochange_count ++;
        if (nochange_count > 1) {
          ECHO_LM(DB, "Stuck in Loop.. Exiting");
          t3_done = true;
        }
        ECHO_SMV(DB, "target:", adj_target, 6);
        ECHO_MV(" oz:", bed_level_oz, 6);
        ECHO_MV(" tower radius adj:", tower_adj[5], 6);
        if (t3_done == true) ECHO_EM(" done:true"); else ECHO_EM(" done:false");
      }
    } while ((t1_done == false) or (t2_done == false) or (t3_done == false));
  }

  void adj_tower_delta(int tower) {
    float adj_val = 0;
    float adj_mag = 0.2;
    float adj_prv;

    do {
      tower_adj[tower - 1] += adj_val;
      set_delta_constants();

      if ((tower == 1) or (tower == 3)) bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
      if ((tower == 1) or (tower == 2)) bed_level_oz = probe_bed(0.0, -bed_radius);
      if ((tower == 2) or (tower == 3)) bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);

      adj_prv = adj_val;
      adj_val = 0;

      if (tower == 1) {
        if (bed_level_oy < bed_level_oz) adj_val = adj_mag;
        if (bed_level_oy > bed_level_oz) adj_val = -adj_mag;
      }

      if (tower == 2) {
        if (bed_level_oz < bed_level_ox) adj_val = adj_mag;
        if (bed_level_oz > bed_level_ox) adj_val = -adj_mag;
      }

      if (tower == 3) {
        if (bed_level_ox < bed_level_oy) adj_val = adj_mag;
        if (bed_level_ox > bed_level_oy) adj_val = -adj_mag;
      }
         
      if ((adj_val > 0) and (adj_prv < 0)) {
        adj_mag = adj_mag / 2;
        adj_val = adj_mag;
      }

      if ((adj_val < 0) and (adj_prv > 0)) {
        adj_mag = adj_mag / 2;
        adj_val = -adj_mag;
      }

      // Show Adjustments made
      if (tower == 1) {
        ECHO_SMV(DB, "oy:", bed_level_oy, 4);
        ECHO_MV(" oz:", bed_level_oz, 4);
      }

      if (tower == 2) {
        ECHO_SMV(DB, "ox:", bed_level_ox, 4);
        ECHO_MV(" oz:", bed_level_oz, 4);
      }

      if (tower == 3) {
        ECHO_SMV(DB, "ox:", bed_level_ox, 4);
        ECHO_MV(" oy:", bed_level_oy, 4);
      }

      ECHO_EMV(" tower delta adj:", adj_val, 5);
    } while(adj_val != 0);
  }

  float adj_diagrod_length() {
    float adj_val = 0;
    float adj_mag = 0.2;
    float adj_prv, target;
    float prev_diag_rod = delta_diagonal_rod;

    do {
      delta_diagonal_rod += adj_val;
      set_delta_constants();

      bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
      bed_level_oz = probe_bed(0.0, -bed_radius);
      bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
      bed_level_c = probe_bed(0.0, 0.0);

      target = (bed_level_ox + bed_level_oy + bed_level_oz) / 3;
      adj_prv = adj_val;
      adj_val = 0;

      if (bed_level_c - 0.005 < target) adj_val = -adj_mag;
      if (bed_level_c + 0.005 > target) adj_val = adj_mag;

      if (((adj_val > 0) and (adj_prv < 0)) or ((adj_val < 0) and (adj_prv > 0))) {
        adj_val = adj_val / 2;
        adj_mag = adj_mag / 2;
      }

      if ((bed_level_c - 0.005 < target) and (bed_level_c + 0.005 > target)) adj_val = 0;

      // If adj magnatude is very small.. quit adjusting
      if ((abs(adj_val) < 0.001) and (adj_val != 0)) adj_val = 0;

      ECHO_SMV(DB, "target:", target, 4);
      ECHO_MV(" c:", bed_level_c, 4);
      ECHO_EMV(" adj:", adj_val, 5);
    } while(adj_val != 0);
    return (delta_diagonal_rod - prev_diag_rod);
  }

  float z_probe() {
    feedrate = AUTOCAL_TRAVELRATE * 60;
    prepare_move();
    st_synchronize();

    enable_endstops(true);
    float start_z = current_position[Z_AXIS];
    long start_steps = st_get_position(Z_AXIS);

    feedrate = AUTOCAL_PROBERATE * 60;
    destination[Z_AXIS] = -20;
    prepare_move_raw();
    st_synchronize();
    endstops_hit_on_purpose();

    enable_endstops(false);
    long stop_steps = st_get_position(Z_AXIS);

    float mm = start_z - float(start_steps - stop_steps) / axis_steps_per_unit[Z_AXIS];
    current_position[Z_AXIS] = mm;
    sync_plan_position_delta();

    // Save tower carriage positions for G30 diagnostic reports
    saved_position[X_AXIS] = st_get_axis_position_mm(X_AXIS);
    saved_position[Y_AXIS] = st_get_axis_position_mm(Y_AXIS);
    saved_position[Z_AXIS] = st_get_axis_position_mm(Z_AXIS);

    destination[Z_AXIS] = mm + Z_RAISE_BETWEEN_PROBINGS;
    prepare_move_raw();
    st_synchronize();
    return mm;
  }

  void calibrate_print_surface(float z_offset) {
    float probe_bed_z, probe_z, probe_h, probe_l;
    int probe_count, auto_bed_leveling_grid_points = AUTO_BED_LEVELING_GRID_POINTS;

    int left_probe_bed_position = LEFT_PROBE_BED_POSITION,
        right_probe_bed_position = RIGHT_PROBE_BED_POSITION,
        front_probe_bed_position = FRONT_PROBE_BED_POSITION,
        back_probe_bed_position = BACK_PROBE_BED_POSITION;

    // probe at the points of a lattice grid
    const int xGridSpacing = (right_probe_bed_position - left_probe_bed_position) / (auto_bed_leveling_grid_points - 1),
              yGridSpacing = (back_probe_bed_position - front_probe_bed_position) / (auto_bed_leveling_grid_points - 1);

    delta_grid_spacing[0] = xGridSpacing;
    delta_grid_spacing[1] = yGridSpacing;

    // First point
    bed_level_c = probe_bed(0.0, 0.0);

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

      zig = !zig;

      for (int xCount = xStart; xCount != xStop; xCount += xInc) {
        double xProbe = left_probe_bed_position + xGridSpacing * xCount;

        // Avoid probing the corners (outside the round or hexagon print surface) on a delta printer.
        float distance_from_center = sqrt(xProbe * xProbe + yProbe * yProbe);
        if (distance_from_center > DELTA_PROBABLE_RADIUS) continue;

        bed_level[xCount][yCount] = probe_bed(xProbe, yProbe);

        idle();
      } // xProbe
    } // yProbe

    extrapolate_unprobed_bed_level();
    print_bed_level();
  }

  float probe_bed(float x, float y) {
    //Probe bed at specified location and return z height of bed
    uint8_t probe_count = PROBE_COUNT;
    float probe_z, probe_bed_array[probe_count], probe_bed_mean = 0;

    destination[X_AXIS] = x - z_probe_offset[X_AXIS];
    if (destination[X_AXIS] < X_MIN_POS) destination[X_AXIS] = X_MIN_POS;
    if (destination[X_AXIS] > X_MAX_POS) destination[X_AXIS] = X_MAX_POS;
    destination[Y_AXIS] = y - z_probe_offset[Y_AXIS];
    if (destination[Y_AXIS] < Y_MIN_POS) destination[Y_AXIS] = Y_MIN_POS;
    if (destination[Y_AXIS] > Y_MAX_POS) destination[Y_AXIS] = Y_MAX_POS;

    for(int i = 0; i < probe_count; i++) {
      probe_bed_array[i] = z_probe() + z_probe_offset[Z_AXIS];
      probe_bed_mean += probe_bed_array[i];
    }

    probe_z = probe_bed_mean / probe_count;

    if (DEBUGGING(INFO)) {
      ECHO_SM(INFO, "Bed probe heights: ");
      for(int i = 0; i < probe_count; i++) {
        if (probe_bed_array[i] >= 0) ECHO_M(" ");
        ECHO_VM(probe_bed_array[i], " ", 4);
      }
      ECHO_M("mean");
      if (probe_z >= 0) ECHO_M(" ");
      ECHO_EV(probe_z, 4);
    }

    bed_safe_z = probe_z + 5;
    return probe_z;
  }

  void bed_probe_all() {
    // Do inital move to safe z level above bed
    feedrate = AUTOCAL_TRAVELRATE * 60;
    destination[Z_AXIS] = bed_safe_z;
    prepare_move_raw();
    st_synchronize();

    // Initial throwaway probe.. used to stabilize probe
    bed_level_c = probe_bed(0.0, 0.0);

    // Probe all bed positions & store carriage positions
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
    bed_level_c = probe_bed(0.0, 0.0);
    save_carriage_positions(0);
  }

  void calibration_report() {
    // Display Report
    ECHO_LM(DB, "|\tZ-Tower\t\t\tEndstop Offsets");

    ECHO_SM(DB, "| \t");
    if (bed_level_z >= 0) ECHO_M(" ");
    ECHO_MV("", bed_level_z, 4);
    ECHO_MV("\t\t\tX:", endstop_adj[0], 4);
    ECHO_MV(" Y:", endstop_adj[1], 4);
    ECHO_EMV(" Z:", endstop_adj[2], 4);

    ECHO_SM(DB, "| ");
    if (bed_level_ox >= 0) ECHO_M(" ");
    ECHO_MV("", bed_level_ox, 4);
    ECHO_M("\t");
    if (bed_level_oy >= 0) ECHO_M(" ");
    ECHO_MV("", bed_level_oy, 4);
    ECHO_EM("\t\tTower Offsets");

    ECHO_SM(DB, "| \t");
    if (bed_level_c >= 0) ECHO_M(" ");
    ECHO_MV("", bed_level_c, 4);
    ECHO_MV("\t\t\tA:",tower_adj[0]);
    ECHO_MV(" B:",tower_adj[1]);
    ECHO_EMV(" C:",tower_adj[2]);

    ECHO_SM(DB, "| ");
    if (bed_level_x >= 0) ECHO_M(" ");
    ECHO_MV("", bed_level_x, 4);
    ECHO_M("\t");
    if (bed_level_y >= 0) ECHO_M(" ");
    ECHO_MV("", bed_level_y, 4);
    ECHO_MV("\t\tI:",tower_adj[3]);
    ECHO_MV(" J:",tower_adj[4]);
    ECHO_EMV(" K:",tower_adj[5]);

    ECHO_SM(DB, "| \t");
    if (bed_level_oz >= 0) ECHO_M(" ");
    ECHO_MV("", bed_level_oz, 4);
    ECHO_EMV("\t\t\tDelta Radius: ", delta_radius, 4);

    ECHO_LMV(DB, "| X-Tower\tY-Tower\t\tDiagonal Rod: ", delta_diagonal_rod, 4);
    ECHO_E;
  }

  void save_carriage_positions(int position_num) {
    for(uint8_t i = 0; i < 3; i++) {
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
    for (int i = X_AXIS; i <= Z_AXIS; i++) destination[i] = 3 * max_length[Z_AXIS];
    feedrate = 1.732 * homing_feedrate[X_AXIS];
    line_to_destination();
    st_synchronize();
    endstops_hit_on_purpose(); // clear endstop hit flags

    // Destination reached
    set_current_to_destination();

    // take care of back off and rehome now we are all at the top
    HOMEAXIS(X);
    HOMEAXIS(Y);
    HOMEAXIS(Z);

    sync_plan_position_delta();

    #if ENABLED(ENDSTOPS_ONLY_FOR_HOMING)
      enable_endstops(false);
    #endif

    feedrate = saved_feedrate;
    feedrate_multiplier = saved_feedrate_multiplier;
    refresh_cmd_timeout();
    endstops_hit_on_purpose(); // clear endstop hit flags
  }

  void prepare_move_raw() {
    if (DEBUGGING(DEBUG)) {
      ECHO_S(DEB);
      print_xyz("prepare_move_raw > destination", destination);
    }
    refresh_cmd_timeout();
    calculate_delta(destination);
    plan_buffer_line(delta[TOWER_1], delta[TOWER_2], delta[TOWER_3], destination[E_AXIS], feedrate * feedrate_multiplier / 60 / 100.0, active_extruder, active_driver);
    set_current_to_destination();
  }

  void calculate_delta(float cartesian[3]) {
    delta[TOWER_1] = sqrt(delta_diagonal_rod_1
                         - sq(delta_tower1_x - cartesian[X_AXIS])
                         - sq(delta_tower1_y - cartesian[Y_AXIS])
                         ) + cartesian[Z_AXIS];
    delta[TOWER_2] = sqrt(delta_diagonal_rod_2
                         - sq(delta_tower2_x - cartesian[X_AXIS])
                         - sq(delta_tower2_y - cartesian[Y_AXIS])
                         ) + cartesian[Z_AXIS];
    delta[TOWER_3] = sqrt(delta_diagonal_rod_3
                         - sq(delta_tower3_x - cartesian[X_AXIS])
                         - sq(delta_tower3_y - cartesian[Y_AXIS])
                         ) + cartesian[Z_AXIS];
  }

  // Adjust print surface height by linear interpolation over the bed_level array.
  void adjust_delta(float cartesian[3]) {
    if (delta_grid_spacing[0] == 0 || delta_grid_spacing[1] == 0) return; // G29 not done!

    int half = (AUTO_BED_LEVELING_GRID_POINTS - 1) / 2;
    float h1 = 0.001 - half, h2 = half - 0.001,
          grid_x = max(h1, min(h2, cartesian[X_AXIS] / delta_grid_spacing[0])),
          grid_y = max(h1, min(h2, cartesian[Y_AXIS] / delta_grid_spacing[1]));
    int floor_x = floor(grid_x), floor_y = floor(grid_y);
    float ratio_x = grid_x - floor_x, ratio_y = grid_y - floor_y,
          z1 = bed_level[floor_x + half][floor_y + half],
          z2 = bed_level[floor_x + half][floor_y + half + 1],
          z3 = bed_level[floor_x + half + 1][floor_y + half],
          z4 = bed_level[floor_x + half + 1][floor_y + half + 1],
          left = (1 - ratio_y) * z1 + ratio_y * z2,
          right = (1 - ratio_y) * z3 + ratio_y * z4,
          offset = (1 - ratio_x) * left + ratio_x * right;

    delta[TOWER_1] += offset;
    delta[TOWER_2] += offset;
    delta[TOWER_3] += offset;

    if (DEBUGGING(DEBUG)) {
      ECHO_SMV(DEB, "grid_x=", grid_x);
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
    }
  }

#endif // DELTA

#if ENABLED(COLOR_MIXING_EXTRUDER)
  void normalize_mix() {
    float mix_total = 0.0;
    for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
      float v = mixing_factor[i];
      if (v < 0) v = mixing_factor[i] = 0;
      mix_total += v;
    }

    // Scale all values if they don't add up to ~1.0
    if (mix_total < 0.9999 || mix_total > 1.0001) {
      ECHO_EM("Warning: Mix factors must add up to 1.0. Scaling.");
      float mix_scale = 1.0 / mix_total;
      for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
        mixing_factor[i] *= mix_scale;
      }
    }
  }

  // Get mixing parameters from the GCode
  // Factors that are left out are set to 0
  // The total "must" be 1.0 (but it will be normalized)
  void gcode_get_mix() {
    const char* mixing_codes = "ABCDHI";
    for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
      mixing_factor[i] = code_seen(mixing_codes[i]) ? code_value() : 0;
    }
    normalize_mix();
  }
#endif

#if ENABLED(IDLE_OOZING_PREVENT)
  void IDLE_OOZING_retract(bool retracting) {  
    if (retracting && !IDLE_OOZING_retracted[active_extruder]) {
      float oldFeedrate = feedrate;
      set_destination_to_current();
      current_position[E_AXIS] += IDLE_OOZING_LENGTH / volumetric_multiplier[active_extruder];
      feedrate = IDLE_OOZING_FEEDRATE * 60;
      plan_set_e_position(current_position[E_AXIS]);
      prepare_move();
      feedrate = oldFeedrate;
      IDLE_OOZING_retracted[active_extruder] = true;
      //ECHO_EM("-");
    }
    else if (!retracting && IDLE_OOZING_retracted[active_extruder]) {
      float oldFeedrate = feedrate;
      set_destination_to_current();
      current_position[E_AXIS] -= (IDLE_OOZING_LENGTH+IDLE_OOZING_RECOVER_LENGTH) / volumetric_multiplier[active_extruder];
      feedrate = IDLE_OOZING_RECOVER_FEEDRATE * 60;
      plan_set_e_position(current_position[E_AXIS]);
      prepare_move();
      feedrate = oldFeedrate;
      IDLE_OOZING_retracted[active_extruder] = false;
      //ECHO_EM("+");
    }
  }
#endif

#if ENABLED(FWRETRACT)
  void retract(bool retracting, bool swapping = false) {

    if (retracting == retracted[active_extruder]) return;

    float oldFeedrate = feedrate;

    set_destination_to_current();

    if (retracting) {
      feedrate = retract_feedrate * 60;
      current_position[E_AXIS] += (swapping ? retract_length_swap : retract_length) / volumetric_multiplier[active_extruder];
      plan_set_e_position(current_position[E_AXIS]);
      prepare_move();

      if (retract_zlift > 0.01) {
        current_position[Z_AXIS] -= retract_zlift;
        #if MECH(DELTA) || MECH(SCARA)
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
        #if MECH(DELTA) || MECH(SCARA)
          sync_plan_position_delta();
        #else
          sync_plan_position();
        #endif
        //prepare_move();
      }

      feedrate = retract_recover_feedrate * 60;
      float move_e = swapping ? retract_length_swap + retract_recover_length_swap : retract_length + retract_recover_length;
      current_position[E_AXIS] -= move_e / volumetric_multiplier[active_extruder];
      plan_set_e_position(current_position[E_AXIS]);
      prepare_move();
    }

    feedrate = oldFeedrate;
    retracted[active_extruder] = retracting;

  } // retract()
#endif //FWRETRACT

#if HAS(Z_PROBE_SLED)

  #if DISABLED(SLED_DOCKING_OFFSET)
    #define SLED_DOCKING_OFFSET 0
  #endif

  /**
   * Method to dock/undock a sled designed by Charles Bell.
   *
   * dock[in]     If true, move to MAX_X and engage the electromagnet
   * offset[in]   The additional distance to move to adjust docking location
   */
  static void dock_sled(bool dock, int offset=0) {
    if (DEBUGGING(INFO)) ECHO_LMV(INFO, "dock_sled", dock);

    if (axis_known_position & (_BV(X_AXIS)|_BV(Y_AXIS)) != (_BV(X_AXIS)|_BV(Y_AXIS))) {
      LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
      ECHO_LM(DB, MSG_POSITION_UNKNOWN);
      return;
    }

    float oldXpos = current_position[X_AXIS]; // save x position
    if (dock) {
      #if Z_RAISE_AFTER_PROBING > 0
        do_blocking_move_to_z(current_position[Z_AXIS] + Z_RAISE_AFTER_PROBING); // raise Z
      #endif
      do_blocking_move_to_x(X_MAX_POS + SLED_DOCKING_OFFSET + offset - 1);  // Dock sled a bit closer to ensure proper capturing
      digitalWrite(SLED_PIN, LOW); // turn off magnet
    }
    else {
      float z_loc = current_position[Z_AXIS];
      if (z_loc < Z_RAISE_BEFORE_PROBING + 5) z_loc = Z_RAISE_BEFORE_PROBING;
      do_blocking_move_to(X_MAX_POS + SLED_DOCKING_OFFSET + offset, current_position[Y_AXIS], z_loc); // this also updates current_position
      digitalWrite(SLED_PIN, HIGH); // turn on magnet
    }
    do_blocking_move_to_x(oldXpos); // return to position before docking
  }
#endif // Z_PROBE_SLED

#if HAS(TEMP_0) || HAS(TEMP_BED) || ENABLED(HEATER_0_USES_MAX6675)
  void print_heaterstates() {
    #if HAS(TEMP_0) || ENABLED(HEATER_0_USES_MAX6675)
      ECHO_MV(SERIAL_T, degHotend(target_extruder), 1);
      ECHO_MV(" /", degTargetHotend(target_extruder), 1);
    #endif
    #if HAS(TEMP_BED)
      ECHO_MV(SERIAL_B, degBed(), 1);
      ECHO_MV(" /", degTargetBed(), 1);
    #endif
    #if HOTENDS > 1
      for (uint8_t h = 0; h < HOTENDS; ++h) {
        ECHO_MV(" T", (int)h);
        ECHO_C(':');
        ECHO_V(degHotend(h), 1);
        ECHO_MV(" /", degTargetHotend(h), 1);
      }
    #endif
    #if HAS(TEMP_BED)
      ECHO_M(SERIAL_BAT);
      #if ENABLED(BED_WATTS)
        ECHO_VM(((BED_WATTS) * getHeaterPower(-1)) / 127, "W");
      #else
        ECHO_V(getHeaterPower(-1));
      #endif
    #endif
    ECHO_M(SERIAL_AT ":");
    #if ENABLED(HOTEND_WATTS)
      ECHO_VM(((HOTEND_WATTS) * getHeaterPower(target_extruder)) / 127, "W");
    #else
      ECHO_V(getHeaterPower(target_extruder));
    #endif
    #if HOTENDS > 1
      for (uint8_t h = 0; h < HOTENDS; ++h) {
        ECHO_MV(SERIAL_AT, (int)h);
        ECHO_C(':');
        #if ENABLED(HOTEND_WATTS)
          ECHO_VM(((HOTEND_WATTS) * getHeaterPower(h)) / 127, "W");
        #else
          ECHO_V(getHeaterPower(h));
        #endif
      }
    #endif
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      #if HAS(TEMP_BED)
        ECHO_MV("    ADC B:", degBed(), 1);
        ECHO_MV("C->", rawBedTemp() / OVERSAMPLENR, 0);
      #endif
      for (uint8_t h = 0; h < HOTENDS; ++h) {
        ECHO_MV("  T", (int)h);
        ECHO_C(':');
        ECHO_V(degHotend(h), 1);
        ECHO_MV("C->", rawHotendTemp(h) / OVERSAMPLENR, 0);
      }
    #endif
  }
#endif

inline void wait_heater() {
  bool wants_to_cool = isCoolingHotend(target_extruder);

  // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
  if (no_wait_for_cooling && wants_to_cool) return;

  // Prevents a wait-forever situation if R is misused i.e. M109 R0
  // Try to calculate a ballpark safe margin by halving EXTRUDE_MINTEMP
  if (wants_to_cool && degTargetHotend(target_extruder) < (EXTRUDE_MINTEMP)/2) return;

  #if ENABLED(TEMP_RESIDENCY_TIME)
    long residency_start_ms = -1;
    // Loop until the temperature has stabilized
    #define TEMP_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_RESIDENCY_TIME) * 1000UL))
  #else
    // Loop until the temperature is exactly on target
    #define TEMP_CONDITIONS (wants_to_cool ? isCoolingHotend(target_extruder) : isHeatingHotend(target_extruder))
  #endif // TEMP_RESIDENCY_TIME

  cancel_heatup = false;
  millis_t now, next_temp_ms = 0;
  do {
    now = millis();
    if (ELAPSED(now, next_temp_ms)) { //Print temp & remaining time every 1s while waiting
      next_temp_ms = now + 1000UL;
      #if HAS(TEMP_0) || HAS(TEMP_BED) || ENABLED(HEATER_0_USES_MAX6675)
        print_heaterstates();
      #endif
      #if TEMP_RESIDENCY_TIME > 0
        ECHO_M(SERIAL_W);
        if (residency_start_ms) {
          long rem = ((TEMP_RESIDENCY_TIME * 1000UL) - (now - residency_start_ms)) / 1000UL;
          ECHO_EV(rem);
        }
        else {
          ECHO_EM("?");
        }
      #else
        ECHO_E;
      #endif
    }

    idle();
    refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

    #if TEMP_RESIDENCY_TIME > 0
      float temp_diff = fabs(degTargetHotend(target_extruder) - degHotend(target_extruder));

      if (!residency_start_ms) {
        // Start the TEMP_RESIDENCY_TIME timer when we reach target temp for the first time.
        if (temp_diff < TEMP_WINDOW) residency_start_ms = millis();
      }
      else if (temp_diff > TEMP_HYSTERESIS) {
        // Restart the timer whenever the temperature falls outside the hysteresis.
        residency_start_ms = millis();
      }
    #endif //TEMP_RESIDENCY_TIME > 0

  } while(!cancel_heatup && TEMP_CONDITIONS);

  LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
}

inline void wait_bed() {
  bool wants_to_cool = isCoolingBed();

  // Exit if the temperature is above target and not waiting for cooling
  if (no_wait_for_cooling && wants_to_cool) return;

  #if TEMP_BED_RESIDENCY_TIME > 0
    millis_t residency_start_ms = 0;
    // Loop until the temperature has stabilized
    #define TEMP_BED_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_BED_RESIDENCY_TIME) * 1000UL))
  #else
    // Loop until the temperature is very close target
    #define TEMP_BED_CONDITIONS (wants_to_cool ? isCoolingBed() : isHeatingBed())
  #endif // TEMP_BED_RESIDENCY_TIME > 0

  cancel_heatup = false;
  millis_t now, next_temp_ms = 0;

  // Wait for temperature to come close enough
  do {
    now = millis();
    if (ELAPSED(now, next_temp_ms)) { //Print Temp Reading every 1 second while heating up.
      next_temp_ms = now + 1000UL;
      print_heaterstates();
      #if TEMP_BED_RESIDENCY_TIME > 0
        ECHO_M(SERIAL_W);
        if (residency_start_ms) {
          long rem = (((TEMP_BED_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
          ECHO_EV(rem);
        }
        else {
          ECHO_EM("?");
        }
      #else
        ECHO_E;
      #endif
    }

    idle();
    refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

    #if TEMP_BED_RESIDENCY_TIME > 0
      float temp_diff = fabs(degBed() - degTargetBed());

      if (!residency_start_ms) {
        // Start the TEMP_BED_RESIDENCY_TIME timer when we reach target temp for the first time.
        if (temp_diff < TEMP_BED_WINDOW) residency_start_ms = millis();
      }
      else if (temp_diff > TEMP_BED_HYSTERESIS) {
        // Restart the timer whenever the temperature falls outside the hysteresis.
        residency_start_ms = millis();
      }
    #endif //TEMP_BED_RESIDENCY_TIME > 0

  } while (!cancel_heatup && TEMP_BED_CONDITIONS);
  LCD_MESSAGEPGM(MSG_BED_DONE);
}


/******************************************************************************
***************************** G-Code Functions ********************************
*******************************************************************************/

/**
 * Set XYZE destination and feedrate from the current GCode command
 *
 *  - Set destination from included axis codes
 *  - Set to current for missing axis codes
 *  - Set the feedrate, if included
 */
void gcode_get_destination() {
  #if ENABLED(IDLE_OOZING_PREVENT)
    if(code_seen(axis_codes[E_AXIS])) IDLE_OOZING_retract(false);
  #endif

  for (int i = 0; i < 3; i++) {
    if (code_seen(axis_codes[i]))
      destination[i] = code_value() + (axis_relative_modes[i] || relative_mode ? current_position[i] : -hotend_offset[i][active_extruder]);
    else
      destination[i] = current_position[i];
  }

  if(code_seen(axis_codes[E_AXIS]))
    destination[E_AXIS] = code_value() + (axis_relative_modes[E_AXIS] || relative_mode ? current_position[E_AXIS] : 0);
  else
    destination[E_AXIS] = current_position[E_AXIS];

  if (code_seen('F')) {
    float next_feedrate = code_value();
    if (next_feedrate > 0.0) feedrate = next_feedrate;
  }

  if (code_seen('P')) {
    destination[E_AXIS] = (code_value() * density_multiplier[previous_extruder] / 100) + current_position[E_AXIS];
  }

  printer_usage_filament += (destination[E_AXIS] - current_position[E_AXIS]);

  #if ENABLED(RFID_MODULE)
    RFID522.RfidData[active_extruder].data.lenght -= (destination[E_AXIS] - current_position[E_AXIS]);
  #endif

  #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
    #if MECH(DELTA)
      if((code_seen(axis_codes[X_AXIS]) || code_seen(axis_codes[Y_AXIS])) && code_seen(axis_codes[E_AXIS]))
        gfx_line_to(destination[X_AXIS] + (X_MAX_POS), destination[Y_AXIS] + (Y_MAX_POS), destination[Z_AXIS]);
      else
        gfx_cursor_to(destination[X_AXIS] + (X_MAX_POS), destination[Y_AXIS] + (Y_MAX_POS), destination[Z_AXIS]);
    #else
      if((code_seen(axis_codes[X_AXIS]) || code_seen(axis_codes[Y_AXIS])) && code_seen(axis_codes[E_AXIS]))
        gfx_line_to(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS]);
      else
        gfx_cursor_to(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS]);
    #endif
  #endif
}

void unknown_command_error() {
  ECHO_LMV(ER, SERIAL_UNKNOWN_COMMAND, current_command);
}

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  /**
   * Output a "busy" message at regular intervals
   * while the machine is not accepting commands.
   */
  void host_keepalive() {
    millis_t ms = millis();
    if (host_keepalive_interval && busy_state != NOT_BUSY) {
      if (PENDING(ms, next_busy_signal_ms)) return;
      switch (busy_state) {
        case IN_HANDLER:
        case IN_PROCESS:
          ECHO_LM(BUSY, SERIAL_BUSY_PROCESSING);
          break;
        case PAUSED_FOR_USER:
          ECHO_LM(BUSY, SERIAL_BUSY_PAUSED_FOR_USER);
          break;
        case PAUSED_FOR_INPUT:
          ECHO_LM(BUSY, SERIAL_BUSY_PAUSED_FOR_INPUT);
          break;
        default:
          break;
      }
    }
    next_busy_signal_ms = ms + host_keepalive_interval * 1000UL;
  }

#endif //HOST_KEEPALIVE_FEATURE

/**
 * G0, G1: Coordinated movement of X Y Z E axes
 */
inline void gcode_G0_G1() {
  if (IsRunning()) {
    gcode_get_destination(); // For X Y Z E F

    #if ENABLED(FWRETRACT)
      if (autoretract_enabled && !(code_seen('X') || code_seen('Y') || code_seen('Z')) && code_seen('E')) {
        float echange = destination[E_AXIS] - current_position[E_AXIS];
        // Is this move an attempt to retract or recover?
        if ((echange < -MIN_RETRACT && !retracted[active_extruder]) || (echange > MIN_RETRACT && retracted[active_extruder])) {
          current_position[E_AXIS] = destination[E_AXIS]; // hide the slicer-generated retract/recover from calculations
          plan_set_e_position(current_position[E_AXIS]);  // AND from the planner
          retract(!retracted[active_extruder]);
          return;
        }
      }
    #endif //FWRETRACT

    prepare_move();
  }
}

/**
 * G2: Clockwise Arc
 * G3: Counterclockwise Arc
 */
inline void gcode_G2_G3(bool clockwise) {
  if (IsRunning()) {

    #if ENABLED(SF_ARC_FIX)
      bool relative_mode_backup = relative_mode;
      relative_mode = true;
    #endif

    gcode_get_destination();

    #if ENABLED(SF_ARC_FIX)
      relative_mode = relative_mode_backup;
    #endif

    // Center of arc as offset from current_position
    float arc_offset[2] = {
      code_seen('I') ? code_value() : 0,
      code_seen('J') ? code_value() : 0
    };

    // Send an arc to the planner
    plan_arc(destination, arc_offset, clockwise);

    refresh_cmd_timeout();
  }
}

/**
 * G4: Dwell S<seconds> or P<milliseconds>
 */
inline void gcode_G4() {
  millis_t codenum = 0;

  if (code_seen('P')) codenum = code_value_long(); // milliseconds to wait
  if (code_seen('S')) codenum = code_value() * 1000; // seconds to wait

  st_synchronize();
  refresh_cmd_timeout();
  codenum += previous_cmd_ms;  // keep track of when we started waiting

  if (!lcd_hasstatus()) LCD_MESSAGEPGM(MSG_DWELL);

  while (millis() < codenum) idle();
}

#if ENABLED(FWRETRACT)

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
 */
inline void gcode_G28() {
  if (DEBUGGING(INFO)) ECHO_LM(INFO, "gcode_G28 >>>");

  // Wait for planner moves to finish!
  st_synchronize();

  // For auto bed leveling, clear the level matrix
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    plan_bed_level_matrix.set_to_identity();
  #elif MECH(DELTA)
    reset_bed_level();
  #endif

  setup_for_endstop_move();

  set_destination_to_current();

  bool come_back = code_seen('B');
  float lastpos[NUM_AXIS];
  float oldfeedrate;
  if(come_back) {
    oldfeedrate = feedrate;
    memcpy(lastpos, current_position, sizeof(lastpos));
  }

  feedrate = 0.0;

  bool  homeX = code_seen(axis_codes[X_AXIS]),
        homeY = code_seen(axis_codes[Y_AXIS]),
        homeZ = code_seen(axis_codes[Z_AXIS]),
        homeE = code_seen(axis_codes[E_AXIS]);

  home_all_axis = (!homeX && !homeY && !homeZ && !homeE) || (homeX && homeY && homeZ);

  #if ENABLED(NPR2)
    if((home_all_axis) || (code_seen(axis_codes[E_AXIS]))) {
      active_driver = active_extruder = 1;
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], -200, COLOR_HOMERATE, active_extruder, active_driver);
      st_synchronize();
      old_color = 99;
      active_driver = active_extruder = 0;
    }
  #endif

  #if MECH(DELTA)
    // A delta can only safely home all axis at the same time
    // all axis have to home at the same time

    // Pretend the current position is 0,0,0
    for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = 0;
    sync_plan_position();

    // Move all carriages up together until the first endstop is hit.
    for (int i = X_AXIS; i <= Z_AXIS; i++) destination[i] = 3 * (Z_MAX_LENGTH);
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

    if (DEBUGGING(INFO)) {
      ECHO_S(INFO);
      print_xyz("(DELTA) > current_position", current_position);
    }

  #else // NOT DELTA

    if (home_all_axis || homeZ) {

      #if Z_HOME_DIR > 0  // If homing away from BED do Z first

        HOMEAXIS(Z);
        if (DEBUGGING(INFO)) {
          ECHO_S(INFO);
          print_xyz("> HOMEAXIS(Z) > current_position", current_position);
        }

      #elif DISABLED(Z_SAFE_HOMING) && ENABLED(AUTO_BED_LEVELING_FEATURE) && Z_RAISE_BEFORE_HOMING > 0

        // Raise Z before homing any other axes
        destination[Z_AXIS] = -(Z_RAISE_BEFORE_HOMING) * home_dir(Z_AXIS); // Set destination away from bed
        if (DEBUGGING(INFO)) {
          ECHO_SMV(INFO, "Raise Z (before homing) by ", (float)Z_RAISE_BEFORE_HOMING);
          print_xyz(" > (home_all_axis || homeZ) > destination", destination);
        }
        feedrate = max_feedrate[Z_AXIS] * 60;
        line_to_destination();
        st_synchronize();

      #endif

    } // home_all_axis || homeZ

    #if ENABLED(QUICK_HOME)

      if (home_all_axis || (homeX && homeY)) {  // First diagonal move

        current_position[X_AXIS] = current_position[Y_AXIS] = 0;

        #if ENABLED(DUAL_X_CARRIAGE)
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

        set_axis_is_at_home(X_AXIS);
        set_axis_is_at_home(Y_AXIS);
        sync_plan_position();

        if (DEBUGGING(INFO)) {
          ECHO_S(INFO);
          print_xyz("> QUICK_HOME > current_position 1", current_position);
        }

        destination[X_AXIS] = current_position[X_AXIS];
        destination[Y_AXIS] = current_position[Y_AXIS];
        line_to_destination();
        feedrate = 0.0;
        st_synchronize();
        endstops_hit_on_purpose(); // clear endstop hit flags

        current_position[X_AXIS] = destination[X_AXIS];
        current_position[Y_AXIS] = destination[Y_AXIS];
        #if !MECH(SCARA)
          current_position[Z_AXIS] = destination[Z_AXIS];
        #endif

        if (DEBUGGING(INFO)) {
          ECHO_S(INFO);
          print_xyz("> QUICK_HOME > current_position 2", current_position);
        }
      }
    #endif // QUICK_HOME

    #if ENABLED(HOME_Y_BEFORE_X)
      // Home Y
      if (home_all_axis || homeY) HOMEAXIS(Y);
    #endif

    // Home X
    if (home_all_axis || homeX) {
      #if ENABLED(DUAL_X_CARRIAGE)
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
      if (DEBUGGING(INFO)) {
        ECHO_S(INFO);
        print_xyz("> homeX", current_position);
      }
    }

    #if DISABLED(HOME_Y_BEFORE_X)
      // Home Y
      if (home_all_axis || homeY) {
        HOMEAXIS(Y);
        if (DEBUGGING(INFO)) {
          ECHO_S(INFO);
          print_xyz("> homeY", current_position);
        }
      }
    #endif

    // Home Z last if homing towards the bed
    #if Z_HOME_DIR < 0
      #if DISABLED(Z_SAFE_HOMING)
        if (code_seen('M') && !(homeX || homeY)) {
          // Manual G28 bed level
          #if ENABLED(ULTIPANEL)
            ECHO_LM(DB, "--LEVEL PLATE SCRIPT--");
            while(!lcd_clicked()) {
              idle();
            }
            saved_feedrate = feedrate;
            saved_feedrate_multiplier = feedrate_multiplier;
            feedrate_multiplier = 100;
            refresh_cmd_timeout();

            enable_endstops(true);
            for(uint8_t i = 0; i < NUM_AXIS; i++) {
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

            #if ENABLED(ENDSTOPS_ONLY_FOR_HOMING)
              enable_endstops(false);
            #endif

            feedrate = saved_feedrate;
            feedrate_multiplier = saved_feedrate_multiplier;
            refresh_cmd_timeout();
            endstops_hit_on_purpose(); // clear endstop hit flags

            sync_plan_position();

            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], (Z_MIN_POS) + 5);

            // PROBE FIRST POINT
            set_pageShowInfo(1);
            do_blocking_move_to(LEFT_PROBE_BED_POSITION, FRONT_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], Z_MIN_POS);
            while(!lcd_clicked()) {          
              idle();
            }

            // PROBE SECOND POINT
            set_pageShowInfo(2);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], (Z_MIN_POS) + 5);
            do_blocking_move_to(RIGHT_PROBE_BED_POSITION, FRONT_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], Z_MIN_POS);
            while(!lcd_clicked()) {
              idle();
            }

            // PROBE THIRD POINT
            set_pageShowInfo(3);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], (Z_MIN_POS) + 5);
            do_blocking_move_to(RIGHT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], Z_MIN_POS);
            while(!lcd_clicked()) {
              idle();
            }     

            // PROBE FOURTH POINT
            set_pageShowInfo(4);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], (Z_MIN_POS) + 5);
            do_blocking_move_to(LEFT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], Z_MIN_POS);
            while(!lcd_clicked()) {
              idle();
            }

            // PROBE CENTER
            set_pageShowInfo(5);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], (Z_MIN_POS) + 5);
            do_blocking_move_to(((X_MAX_POS) - (X_MIN_POS)) / 2, ((Y_MAX_POS) - (Y_MIN_POS)) / 2, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], Z_MIN_POS);
            while(!lcd_clicked()) {
              idle();
            }

            // FINISH MANUAL BED LEVEL
            set_pageShowInfo(6);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], (Z_MIN_POS) + 5);
            enqueue_and_echo_commands_P(PSTR("G28"));
          #endif // ULTIPANEL
        }
        else if (home_all_axis || homeZ) {
          HOMEAXIS(Z);
          if (DEBUGGING(INFO)) {
            ECHO_S(INFO);
            print_xyz("> (home_all_axis || homeZ) > final", current_position);
          }
        }
      #elif ENABLED(Z_SAFE_HOMING) && ENABLED(AUTO_BED_LEVELING_FEATURE)// Z Safe mode activated.

        if (DEBUGGING(INFO)) ECHO_LM(INFO, "> Z_SAFE_HOMING >>>");

        if (home_all_axis) {

          current_position[Z_AXIS] = 0;
          sync_plan_position();

          //
          // Set the probe (or just the nozzle) destination to the safe homing point
          //
          // NOTE: If current_position[X_AXIS] or current_position[Y_AXIS] were set above
          // then this may not work as expected.
          destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - (X_PROBE_OFFSET_FROM_EXTRUDER));
          destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - (Y_PROBE_OFFSET_FROM_EXTRUDER));
          destination[Z_AXIS] = -(Z_RAISE_BEFORE_HOMING) * home_dir(Z_AXIS);  // Set destination away from bed
          feedrate = xy_travel_speed;

          if (DEBUGGING(INFO)) {
            ECHO_SMV(INFO, "Raise Z (before homing) by ", (float)Z_RAISE_BEFORE_HOMING);
            print_xyz(" > home_all_axis > current_position", current_position, false);
            print_xyz(" > home_all_axis > destination", destination);
          }

          // This could potentially move X, Y, Z all together
          line_to_destination();
          st_synchronize();

          // Set current X, Y is the Z_SAFE_HOMING_POINT minus PROBE_OFFSET_FROM_EXTRUDER
          current_position[X_AXIS] = destination[X_AXIS];
          current_position[Y_AXIS] = destination[Y_AXIS];

          HOMEAXIS(Z);
        }
        else if (homeZ) { // Don't need to Home Z twice

          // Let's see if X and Y are homed
          if (axis_was_homed & (_BV(X_AXIS)|_BV(Y_AXIS)) == (_BV(X_AXIS)|_BV(Y_AXIS))) {

            // Make sure the probe is within the physical limits
            // NOTE: This doesn't necessarily ensure the probe is also within the bed!
            float cpx = current_position[X_AXIS], cpy = current_position[Y_AXIS];
            if (   cpx >= X_MIN_POS - (X_PROBE_OFFSET_FROM_EXTRUDER)
                && cpx <= X_MAX_POS - (X_PROBE_OFFSET_FROM_EXTRUDER)
                && cpy >= Y_MIN_POS - (Y_PROBE_OFFSET_FROM_EXTRUDER)
                && cpy <= Y_MAX_POS - (Y_PROBE_OFFSET_FROM_EXTRUDER)) {
              // Set the plan current position to X, Y, 0
              current_position[Z_AXIS] = 0;
              plan_set_position(cpx, cpy, 0, current_position[E_AXIS]);

              // Set Z destination away from bed and raise the axis
              destination[Z_AXIS] = -(Z_RAISE_BEFORE_HOMING) * home_dir(Z_AXIS);    // Set destination away from bed
              feedrate = max_feedrate[Z_AXIS] * 60;

              if (DEBUGGING(INFO)) {
                ECHO_SMV(INFO, "Raise Z (before homing) by ", (float)Z_RAISE_BEFORE_HOMING);
                print_xyz(" > homeZ > current_position", current_position, false);
                print_xyz(" > homeZ > destination", destination);
              }

              line_to_destination();
              st_synchronize();

              // Home the Z axis
              HOMEAXIS(Z);
            }
            else {
              LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
              ECHO_LM(DB, MSG_ZPROBE_OUT);
            }
          }
          else {
            LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
            ECHO_LM(DB, MSG_POSITION_UNKNOWN);
          }
        }
        if (DEBUGGING(INFO)) ECHO_LM(INFO, "<<< Z_SAFE_HOMING");
      #elif ENABLED(Z_SAFE_HOMING)
        if (home_all_axis || homeZ) {

          // Let's see if X and Y are homed
          if (axis_was_homed & (_BV(X_AXIS)|_BV(Y_AXIS)) == (_BV(X_AXIS)|_BV(Y_AXIS))) {
            current_position[Z_AXIS] = 0;
            sync_plan_position();

            destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT);
            destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT);
            destination[Z_AXIS] = current_position[Z_AXIS] = 0;
            feedrate = xy_travel_speed;

            if (DEBUGGING(INFO)) {
              ECHO_SMV(INFO, "Raise Z (before homing) by ", (float)Z_RAISE_BEFORE_HOMING);
              print_xyz(" > home_all_axis > current_position", current_position, false);
              print_xyz(" > home_all_axis > destination", destination);
            }

            // This could potentially move X, Y, Z all together
            line_to_destination();
            st_synchronize();

            // Set current X, Y is the Z_SAFE_HOMING_POINT minus PROBE_OFFSET_FROM_EXTRUDER
            current_position[X_AXIS] = destination[X_AXIS];
            current_position[Y_AXIS] = destination[Y_AXIS];
            HOMEAXIS(Z);
          }
          else {
            LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
            ECHO_LM(ER, MSG_POSITION_UNKNOWN);
          }
        }
        if (DEBUGGING(INFO)) ECHO_LM(INFO, "<<< Z_SAFE_HOMING");
      #endif // Z_SAFE_HOMING
    #endif // Z_HOME_DIR < 0

    sync_plan_position();

  #endif // !DELTA

  #if MECH(SCARA)
    sync_plan_position_delta();
  #endif

  clean_up_after_endstop_move();

  if(come_back) {
    #if MECH(DELTA)
      feedrate = 1.732 * homing_feedrate[X_AXIS];
      memcpy(destination, lastpos, sizeof(destination));
      prepare_move();
      feedrate = oldfeedrate;
    #else
      if(homeX) {
        feedrate = homing_feedrate[X_AXIS];
        destination[X_AXIS] = lastpos[X_AXIS];
        prepare_move();
      }
      if(homeY) {
        feedrate = homing_feedrate[Y_AXIS];
        destination[Y_AXIS] = lastpos[Y_AXIS];
        prepare_move();
      }
      if(homeZ) {
        feedrate = homing_feedrate[Z_AXIS];
        destination[Z_AXIS] = lastpos[Z_AXIS];
        prepare_move();
      }
      feedrate = oldfeedrate;
    #endif
  }

  #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
    #if MECH(DELTA)
      gfx_clear((X_MAX_POS) * 2, (Y_MAX_POS) * 2, Z_MAX_POS);
      gfx_cursor_to(current_position[X_AXIS] + (X_MAX_POS), current_position[Y_AXIS] + (Y_MAX_POS), current_position[Z_AXIS]);
    #else
      gfx_clear(X_MAX_POS, Y_MAX_POS, Z_MAX_POS);
      gfx_cursor_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
    #endif
  #endif

  if (DEBUGGING(INFO)) ECHO_LM(INFO, "<<< gcode_G28");
}

#if ENABLED(AUTO_BED_LEVELING_FEATURE)

  void out_of_range_error(const char* p_edge) {
    ECHO_M("?Probe ");
    ECHO_T(p_edge);
    ECHO_EM(" position out of range.");
  }

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
    if (DEBUGGING(INFO)) ECHO_LM(INFO, "gcode_G29 >>>");

    // Don't allow auto-leveling without homing first
    if (axis_known_position & (_BV(X_AXIS)|_BV(Y_AXIS)) != (_BV(X_AXIS)|_BV(Y_AXIS))) {
      LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
      ECHO_LM(ER, MSG_POSITION_UNKNOWN);
      return;
    }

    int verbose_level = code_seen('V') ? code_value_short() : 1;
    if (verbose_level < 0 || verbose_level > 4) {
      ECHO_LM(ER,"?(V)erbose Level is implausible (0-4).");
      return;
    }

    bool dryrun = code_seen('D'),
         deploy_probe_for_each_reading = code_seen('E');

    #if ENABLED(AUTO_BED_LEVELING_GRID)

      bool do_topography_map = verbose_level > 2 || code_seen('T');

      if (verbose_level > 0) {
        ECHO_LM(DB, "G29 Auto Bed Leveling");
        if (dryrun) ECHO_LM(DB,"Running in DRY-RUN mode");
      }

      int auto_bed_leveling_grid_points = code_seen('P') ? code_value_short() : AUTO_BED_LEVELING_GRID_POINTS;
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
           left_out = left_out_l || left_probe_bed_position > right_probe_bed_position - (MIN_PROBE_EDGE),
           right_out_r = right_probe_bed_position > MAX_PROBE_X,
           right_out = right_out_r || right_probe_bed_position < left_probe_bed_position + (MIN_PROBE_EDGE),
           front_out_f = front_probe_bed_position < MIN_PROBE_Y,
           front_out = front_out_f || front_probe_bed_position > back_probe_bed_position - (MIN_PROBE_EDGE),
           back_out_b = back_probe_bed_position > MAX_PROBE_Y,
           back_out = back_out_b || back_probe_bed_position < front_probe_bed_position + (MIN_PROBE_EDGE);

      if (left_out || right_out || front_out || back_out) {
        if (left_out) {
          out_of_range_error(PSTR("(L)eft"));
          left_probe_bed_position = left_out_l ? MIN_PROBE_X : right_probe_bed_position - (MIN_PROBE_EDGE);
        }
        if (right_out) {
          out_of_range_error(PSTR("(R)ight"));
          right_probe_bed_position = right_out_r ? MAX_PROBE_X : left_probe_bed_position + (MIN_PROBE_EDGE);
        }
        if (front_out) {
          out_of_range_error(PSTR("(F)ront"));
          front_probe_bed_position = front_out_f ? MIN_PROBE_Y : back_probe_bed_position - (MIN_PROBE_EDGE);
        }
        if (back_out) {
          out_of_range_error(PSTR("(B)ack"));
          back_probe_bed_position = back_out_b ? MAX_PROBE_Y : front_probe_bed_position + (MIN_PROBE_EDGE);
        }
        return;
      }

    #endif // AUTO_BED_LEVELING_GRID

    #if HAS(Z_PROBE_SLED)
      dock_sled(false); // engage (un-dock) the probe
    #endif

    st_synchronize();

    if (!dryrun) {
      // make sure the bed_level_rotation_matrix is identity or the planner will get it wrong
      plan_bed_level_matrix.set_to_identity();

      // vector_3 corrected_position = plan_get_position_mm();
      // corrected_position.debug("position before G29");
      vector_3 uncorrected_position = plan_get_position();
      // uncorrected_position.debug("position during G29");
      current_position[X_AXIS] = uncorrected_position.x;
      current_position[Y_AXIS] = uncorrected_position.y;
      current_position[Z_AXIS] = uncorrected_position.z;
      sync_plan_position();
    }

    setup_for_endstop_move();
    feedrate = homing_feedrate[Z_AXIS];

    #if ENABLED(AUTO_BED_LEVELING_GRID)

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
      int8_t indexIntoAB[auto_bed_leveling_grid_points][auto_bed_leveling_grid_points];

      int probePointCounter = 0;
      bool zig = (auto_bed_leveling_grid_points & 1) ? true : false; //always end at [RIGHT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION]

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

        zig = !zig;

        for (int xCount = xStart; xCount != xStop; xCount += xInc) {
          double xProbe = left_probe_bed_position + xGridSpacing * xCount;

          // raise extruder
          float measured_z,
                z_before = probePointCounter ? Z_RAISE_BETWEEN_PROBINGS + current_position[Z_AXIS] : Z_RAISE_BEFORE_PROBING + current_position[Z_AXIS];

          if (DEBUGGING(INFO)) {
            if (probePointCounter)
              ECHO_LMV(INFO, "z_before = (between) ", (float)(Z_RAISE_BETWEEN_PROBINGS + current_position[Z_AXIS]));
            else
              ECHO_LMV(INFO, "z_before = (before) ", (float)Z_RAISE_BEFORE_PROBING + current_position[Z_AXIS]);
          }

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
          indexIntoAB[xCount][yCount] = probePointCounter;

          probePointCounter++;

          idle();
        } // xProbe
      } // yProbe

      if (DEBUGGING(INFO)) {
        ECHO_S(INFO);
        print_xyz("> probing complete > current_position", current_position);
      }

      clean_up_after_endstop_move();

      // solve lsq problem
      double plane_equation_coefficients[3];
      qr_solve(plane_equation_coefficients, abl2, 3, eqnAMatrix, eqnBVector);

      mean /= abl2;

      if (verbose_level) {
        ECHO_SMV(DB, "Eqn coefficients: a: ", plane_equation_coefficients[0], 8);
        ECHO_MV(" b: ", plane_equation_coefficients[1], 8);
        ECHO_EMV(" d: ", plane_equation_coefficients[2], 8);
        if (verbose_level > 2)
          ECHO_LMV(DB, "Mean of sampled points: ", mean, 8);
      }

      if (!dryrun) set_bed_level_equation_lsq(plane_equation_coefficients);

      // Show the Topography map if enabled
      if (do_topography_map) {
        ECHO_EM(" Bed Height Topography:");
        ECHO_EM("   +--- BACK --+");
        ECHO_EM("   |           |");
        ECHO_EM(" L |    (+)    | R");
        ECHO_EM(" E |           | I");
        ECHO_EM(" F | (-) N (+) | G");
        ECHO_EM(" T |           | H");
        ECHO_EM("   |    (-)    | T");
        ECHO_EM("   |           |");
        ECHO_EM("   O-- FRONT --+");
        ECHO_EM(" (0,0)");

        float min_diff = 999;

        for (int yy = auto_bed_leveling_grid_points - 1; yy >= 0; yy--) {
          ECHO_S(DB);
          for (int xx = 0; xx < auto_bed_leveling_grid_points; xx++) {
            int ind = indexIntoAB[xx][yy];
            float diff = eqnBVector[ind] - mean;

            float x_tmp = eqnAMatrix[ind + 0 * abl2],
                  y_tmp = eqnAMatrix[ind + 1 * abl2],
                  z_tmp = 0;

            apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);

            NOMORE(min_diff, eqnBVector[ind] - z_tmp);

            if (diff >= 0.0)
              ECHO_M(" +");   // Include + for column alignment
            else
              ECHO_M(" ");
            ECHO_V(diff, 5);
          } // xx
          ECHO_E;
        } // yy
        ECHO_E;
        if (verbose_level > 3) {
          ECHO_LM(DB, "Corrected Bed Height vs. Bed Topology:");

          for (int yy = auto_bed_leveling_grid_points - 1; yy >= 0; yy--) {
            ECHO_S(DB);
            for (int xx = 0; xx < auto_bed_leveling_grid_points; xx++) {
              int ind = indexIntoAB[xx][yy];
              float x_tmp = eqnAMatrix[ind + 0 * abl2],
                    y_tmp = eqnAMatrix[ind + 1 * abl2],
                    z_tmp = 0;

              apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);
              float diff = eqnBVector[ind] - min_diff;

              if (diff >= 0.0)
                ECHO_M(" +");   // Include + for column alignment
              else
                ECHO_M(" ");
              ECHO_V(diff, 5);
            } // xx
            ECHO_E;
          } // yy
          ECHO_E;
        }
      } // do_topography_map

    #else // !AUTO_BED_LEVELING_GRID

      if (DEBUGGING(INFO)) ECHO_LM(INFO, "> 3-point Leveling");

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
      plan_bed_level_matrix.debug(" Bed Level Correction Matrix:");

    if (!dryrun) {
      // Correct the Z height difference from Z probe position and nozzle tip position.
      // The Z height on homing is measured by Z probe, but the Z probe is quite far from the nozzle.
      // When the bed is uneven, this height must be corrected.
      float x_tmp = current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER,
            y_tmp = current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER,
            z_tmp = current_position[Z_AXIS],
            real_z = st_get_axis_position_mm(Z_AXIS);  //get the real Z (since plan_get_position is now correcting the plane)

      if (DEBUGGING(INFO)) {
        ECHO_LMV(INFO, "> BEFORE apply_rotation_xyz > z_tmp  = ", z_tmp);
        ECHO_LMV(INFO, "> BEFORE apply_rotation_xyz > real_z = ", real_z);
      }

      apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp); // Apply the correction sending the Z probe offset

      // Get the current Z position and send it to the planner.
      //
      // >> (z_tmp - real_z) : The rotated current Z minus the uncorrected Z (most recent plan_set_position/sync_plan_position)
      //
      // >> zprobe_zoffset : Z distance from nozzle to Z probe (set by default, M851, EEPROM, or Menu)
      //
      // >> Z_RAISE_AFTER_PROBING : The distance the Z probe will have lifted after the last probe
      //
      // >> Should home_offset[Z_AXIS] be included?
      //
      //      Discussion: home_offset[Z_AXIS] was applied in G28 to set the starting Z.
      //      If Z is not tweaked in G29 -and- the Z probe in G29 is not actually "homing" Z...
      //      then perhaps it should not be included here. The purpose of home_offset[] is to
      //      adjust for inaccurate endstops, not for reasonably accurate probes. If it were
      //      added here, it could be seen as a compensating factor for the Z probe.
      //
      if (DEBUGGING(INFO))
        ECHO_LMV(INFO, "> AFTER apply_rotation_xyz > z_tmp  = ", z_tmp);

      current_position[Z_AXIS] = -zprobe_zoffset + (z_tmp - real_z)
        #if HAS(SERVO_ENDSTOPS) || ENABLED(Z_PROBE_SLED)
          + Z_RAISE_AFTER_PROBING
        #endif
        ;
      // current_position[Z_AXIS] += home_offset[Z_AXIS]; // The Z probe determines Z=0, not "Z home"
      sync_plan_position();

      if (DEBUGGING(INFO))
        print_xyz("> corrected Z in G29", current_position);
    }

    // Sled assembly for Cartesian bots
    #if HAS(Z_PROBE_SLED)
      dock_sled(true); // dock the probe
    #elif HASNT(SERVO_ENDSTOPS) && Z_RAISE_AFTER_PROBING > 0
      // Raise Z axis for non servo based probes
      raise_z_after_probing();
    #endif

    #if ENABLED(Z_PROBE_END_SCRIPT)
      if (DEBUGGING(INFO)) {
        ECHO_SM(INFO, "Z Probe End Script: ");
        ECHO_EM(Z_PROBE_END_SCRIPT);
      }
      enqueue_and_echo_commands_P(PSTR(Z_PROBE_END_SCRIPT));
      st_synchronize();
    #endif

    KEEPALIVE_STATE(IN_HANDLER);

    if (DEBUGGING(INFO)) ECHO_LM(INFO, "<<< gcode_G29");
  }

  #if HASNT(Z_PROBE_SLED)
    /**
     * G30: Do a single Z probe at the current XY
     */
    inline void gcode_G30() {
      if (DEBUGGING(INFO)) ECHO_LM(INFO, "gcode_G30 >>>");

      #if HAS(SERVO_ENDSTOPS)
        raise_z_for_servo();
      #endif
      deploy_z_probe(); // Engage Z Servo endstop if available

      st_synchronize();
      // TODO: clear the leveling matrix or the planner will be set incorrectly
      setup_for_endstop_move();

      feedrate = homing_feedrate[Z_AXIS];

      run_z_probe();
      ECHO_SM(DB, "Bed");
      ECHO_MV(" X: ", current_position[X_AXIS] + 0.0001);
      ECHO_MV(" Y: ", current_position[Y_AXIS] + 0.0001);
      ECHO_MV(" Z: ", current_position[Z_AXIS] + 0.0001);
      ECHO_E;

      clean_up_after_endstop_move();

      #if HAS(SERVO_ENDSTOPS)
        raise_z_for_servo();
      #endif

      stow_z_probe(); // Retract Z Servo endstop if available

      if (DEBUGGING(INFO)) ECHO_LM(INFO, "<<< gcode_G30");
    }
  #endif // !Z_PROBE_SLED
#endif // AUTO_BED_LEVELING_FEATURE

#if MECH(DELTA) && ENABLED(Z_PROBE_ENDSTOP)

  /**
   * G29: Delta Z-Probe, probes the bed at more points.
   */
  inline void gcode_G29() {

    if (DEBUGGING(INFO)) ECHO_LM(INFO, "gcode_G29 >>>");

    if (code_seen('D')) {
      print_bed_level();
      return;
    }

    saved_feedrate = feedrate;
    saved_feedrate_multiplier = feedrate_multiplier;
    feedrate_multiplier = 100;

    home_delta_axis();
    deploy_z_probe();
    calibrate_print_surface(z_probe_offset[Z_AXIS] + (code_seen(axis_codes[Z_AXIS]) ? code_value() : 0.0));
    retract_z_probe();
    clean_up_after_endstop_move();

    KEEPALIVE_STATE(IN_HANDLER);

    if (DEBUGGING(INFO)) ECHO_LM(INFO, "<<< gcode_G29");
  }

  /* G30: Delta AutoCalibration
   *
   * Parameters:
   * C  Show Carriage positions
   *
   */
  inline void gcode_G30() {
    if (DEBUGGING(INFO)) ECHO_LM(INFO, "gcode_G30 >>>");

    // Zero the bed level array
    reset_bed_level();

    if (code_seen('C')) {
      // Show carriage positions 
      ECHO_LM(DB, "Carriage Positions for last scan: ");
      for(uint8_t i = 0; i < 7; i++) {
        ECHO_SMV(DB, "[", saved_positions[i][X_AXIS]);
        ECHO_MV(", ", saved_positions[i][Y_AXIS]);
        ECHO_MV(", ", saved_positions[i][Z_AXIS]);
        ECHO_EM("]");
      }
      return;
    }

    if (code_seen('X') and code_seen('Y')) {
      // Probe specified X,Y point
      float x = code_seen('X') ? code_value():0.00;
      float y = code_seen('Y') ? code_value():0.00;
      float probe_value;

      deploy_z_probe();
      probe_value = probe_bed(x, y);
      ECHO_SMV(DB, "Bed Z-Height at X:", x);
      ECHO_MV(" Y:", y);
      ECHO_EMV(" = ", probe_value, 4);

      if (DEBUGGING(INFO)) {
        ECHO_SMV(INFO, "Carriage Positions: [", saved_position[X_AXIS]);
        ECHO_MV(", ", saved_position[Y_AXIS]);
        ECHO_MV(", ", saved_position[Z_AXIS]);
        ECHO_EM("]");
      }
      retract_z_probe();
      return;
    }

    saved_feedrate = feedrate;
    saved_feedrate_multiplier = feedrate_multiplier;
    feedrate_multiplier = 100;

    if (code_seen('A')) {
      ECHO_LM(DB, "Starting Auto Calibration...");
      LCD_MESSAGEPGM("Auto Calibration...");
      if (code_has_value()) ac_prec = code_value();
      ECHO_SMV(DB, "Calibration precision: +/-", ac_prec, 2);
      ECHO_EM(" mm");
    }

    home_delta_axis();
    deploy_z_probe();
    bed_safe_z = current_position[Z_AXIS];

    // Probe all points
    bed_probe_all();

    // Show calibration report      
    calibration_report();

    if (code_seen('E')) {
      int iteration = 0;
      do {
        iteration ++;
        ECHO_LMV(DB, "Iteration: ", iteration);

        ECHO_LM(DB, "Checking/Adjusting endstop offsets");
        adj_endstops();

        bed_probe_all();
        calibration_report();
      } while ((bed_level_x < -ac_prec) or (bed_level_x > ac_prec)
            or (bed_level_y < -ac_prec) or (bed_level_y > ac_prec)
            or (bed_level_z < -ac_prec) or (bed_level_z > ac_prec));

      ECHO_LM(DB, "Endstop adjustment complete");
    }

    if (code_seen('R')) {
      int iteration = 0;
      do {
        iteration ++;
        ECHO_LMV(DB, "Iteration: ", iteration);

        ECHO_LM(DB, "Checking/Adjusting endstop offsets");
        adj_endstops();

        bed_probe_all();
        calibration_report();

        ECHO_LM(DB, "Checking delta radius");
        adj_deltaradius();

      } while ((bed_level_c < -ac_prec) or (bed_level_c > ac_prec)
            or (bed_level_x < -ac_prec) or (bed_level_x > ac_prec)
            or (bed_level_y < -ac_prec) or (bed_level_y > ac_prec)
            or (bed_level_z < -ac_prec) or (bed_level_z > ac_prec));
    }

    if (code_seen('I')) {
      ECHO_LMV(DB, "Adjusting Tower Delta for tower", code_value());
      adj_tower_delta(code_value());
      ECHO_LM(DB, "Tower Delta adjustment complete");
    }

    if (code_seen('D')) {
      ECHO_LM(DB, "Adjusting Diagonal Rod Length");
      adj_diagrod_length();
      ECHO_LM(DB, "Diagonal Rod Length adjustment complete");
    }

    if (code_seen('T')) {
      ECHO_LMV(DB, "Adjusting Tower Radius for tower", code_value());
      adj_tower_radius(code_value());
      ECHO_LM(DB, "Tower Radius adjustment complete");
    }

    if (code_seen('A')) {
      int iteration = 0;
      boolean dr_adjusted;

      do {
        do {
          iteration ++;
          ECHO_LMV(DB, "Iteration: ", iteration);

          ECHO_LM(DB, "Checking/Adjusting endstop offsets");
          adj_endstops();

          bed_probe_all();
          calibration_report();

          if ((bed_level_c < -ac_prec) or (bed_level_c > ac_prec)) {
            ECHO_LM(DB, "Checking delta radius");
            dr_adjusted = adj_deltaradius();
          }
          else
            dr_adjusted = false;

          if (DEBUGGING(DEBUG)) {
            ECHO_LMV(DEB, "bed_level_c=", bed_level_c, 4);
            ECHO_LMV(DEB, "bed_level_x=", bed_level_x, 4);
            ECHO_LMV(DEB, "bed_level_y=", bed_level_y, 4);
            ECHO_LMV(DEB, "bed_level_z=", bed_level_z, 4);
          }

          idle();
        } while ((bed_level_c < -ac_prec) or (bed_level_c > ac_prec)
              or (bed_level_x < -ac_prec) or (bed_level_x > ac_prec)
              or (bed_level_y < -ac_prec) or (bed_level_y > ac_prec)
              or (bed_level_z < -ac_prec) or (bed_level_z > ac_prec)
              or (dr_adjusted));

        if ((bed_level_ox < -ac_prec) or (bed_level_ox > ac_prec) or
            (bed_level_oy < -ac_prec) or (bed_level_oy > ac_prec) or
            (bed_level_oz < -ac_prec) or (bed_level_oz > ac_prec)) {
          ECHO_LM(DB, "Checking for tower geometry errors..");
          if (fix_tower_errors() != 0 ) {
            // Tower positions have been changed .. home to endstops
            ECHO_LM(DB, "Tower Positions changed .. Homing Endstops");
            home_delta_axis();
            bed_safe_z = Z_RAISE_BETWEEN_PROBINGS - z_probe_offset[Z_AXIS];
          }
          else {
            ECHO_LM(DB, "Checking DiagRod Length");
            if (adj_diagrod_length() != 0) { 
              // If diag rod length has been changed .. home to endstops
              ECHO_LM(DB, "Diagonal Rod Length changed .. Homing Endstops");
              home_delta_axis();
              bed_safe_z = Z_RAISE_BETWEEN_PROBINGS - z_probe_offset[Z_AXIS];
            }
          }
          bed_safe_z = Z_RAISE_BETWEEN_PROBINGS - z_probe_offset[Z_AXIS];
          bed_probe_all();
          calibration_report();
        }

        if (DEBUGGING(DEBUG)) {
          ECHO_LMV(DEB, "bed_level_c=", bed_level_c, 4);
          ECHO_LMV(DEB, "bed_level_x=", bed_level_x, 4);
          ECHO_LMV(DEB, "bed_level_y=", bed_level_y, 4);
          ECHO_LMV(DEB, "bed_level_z=", bed_level_z, 4);
          ECHO_LMV(DEB, "bed_level_ox=", bed_level_ox, 4);
          ECHO_LMV(DEB, "bed_level_oy=", bed_level_oy, 4);
          ECHO_LMV(DEB, "bed_level_oz=", bed_level_oz, 4);
        }
      } while((bed_level_c < -ac_prec) or (bed_level_c > ac_prec)
           or (bed_level_x < -ac_prec) or (bed_level_x > ac_prec)
           or (bed_level_y < -ac_prec) or (bed_level_y > ac_prec)
           or (bed_level_z < -ac_prec) or (bed_level_z > ac_prec)
           or (bed_level_ox < -ac_prec) or (bed_level_ox > ac_prec)
           or (bed_level_oy < -ac_prec) or (bed_level_oy > ac_prec)
           or (bed_level_oz < -ac_prec) or (bed_level_oz > ac_prec));

      ECHO_LM(DB, "Autocalibration Complete");
    }

    retract_z_probe();

    // reset LCD alert message
    lcd_reset_alert_level();

    clean_up_after_endstop_move();

    KEEPALIVE_STATE(IN_HANDLER);

    if (DEBUGGING(INFO)) ECHO_LM(INFO, "<<< gcode_G30");
  }
#endif // DELTA && Z_PROBE_ENDSTOP

/**
 * G60:  save current position
 *        S<slot> specifies memory slot # (0-based) to save into (default 0)
 */
inline void gcode_G60() {
  int slot = 0;
  if (code_seen('S')) slot = code_value();

  if (slot < 0 || slot >= NUM_POSITON_SLOTS) {
    ECHO_LMV(ER, MSG_INVALID_POS_SLOT, (int)NUM_POSITON_SLOTS);
    return;
  } 
  memcpy(stored_position[slot], current_position, sizeof(*stored_position));
  pos_saved = true;

  ECHO_SM(DB, MSG_SAVED_POS);
  ECHO_MV(" S", slot);
  ECHO_MV("<-X:", stored_position[slot][X_AXIS]);
  ECHO_MV(" Y:", stored_position[slot][Y_AXIS]);
  ECHO_MV(" Z:", stored_position[slot][Z_AXIS]);
  ECHO_EMV(" E:", stored_position[slot][E_AXIS]);
}

/**
 * G61:  Apply/restore saved coordinates to the active extruder.
 *        X Y Z E - Value to add at stored coordinates.
 *        F<speed> - Set Feedrate.
 *        S<slot> specifies memory slot # (0-based) to save into (default 0).
 */
inline void gcode_G61() {
  if (!pos_saved) return;

  int slot = 0;
  if (code_seen('S')) slot = code_value();

  if (slot < 0 || slot >= NUM_POSITON_SLOTS) {
    ECHO_LMV(ER, MSG_INVALID_POS_SLOT, (int)NUM_POSITON_SLOTS);
    return;
  }

  ECHO_SM(DB, MSG_RESTORING_POS);
  ECHO_MV(" S", slot);
  ECHO_M("->");

  if (code_seen('F')) {
    float next_feedrate = code_value();
    if (next_feedrate > 0.0) feedrate = next_feedrate;
  }

  for(uint8_t i = 0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i])) {
      destination[i] = (float)code_value() + stored_position[slot][i];
    }
    else {
      destination[i] = current_position[i];
    }
    ECHO_MV(" ", axis_codes[i]);
    ECHO_MV(":", destination[i]);
  }
  ECHO_E;

  // finish moves
  prepare_move();
  st_synchronize();
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
  if (didXYZ) {
    #if MECH(DELTA) || MECH(SCARA)
      sync_plan_position_delta();
    #else
      sync_plan_position();
    #endif
  }
}

#if ENABLED(ULTIPANEL)

  /**
   * M0: // M0 - Unconditional stop - Wait for user button press on LCD
   * M1: // M1 - Conditional stop - Wait for user button press on LCD
   */
  inline void gcode_M0_M1() {
    char* args = current_command_args;

    millis_t codenum = 0;
    bool hasP = false, hasS = false;
    if (code_seen('P')) {
      codenum = code_value_short(); // milliseconds to wait
      hasP = codenum > 0;
    }
    if (code_seen('S')) {
      codenum = code_value() * 1000; // seconds to wait
      hasS = codenum > 0;
    }

    if (!hasP && !hasS && *args != '\0')
      lcd_setstatus(args, true);
    else {
      LCD_MESSAGEPGM(MSG_USERWAIT);
      #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
        dontExpireStatus();
      #endif
    }

    lcd_ignore_click();
    st_synchronize();
    refresh_cmd_timeout();
    if (codenum > 0) {
      codenum += previous_cmd_ms;  // wait until this time for a click
      KEEPALIVE_STATE(PAUSED_FOR_USER);
      while (PENDING(millis(), codenum) && !lcd_clicked()) idle();
      KEEPALIVE_STATE(IN_HANDLER);
      lcd_ignore_click(false);
    }
    else {
      if (!lcd_detected()) return;
      KEEPALIVE_STATE(PAUSED_FOR_USER);
      while (!lcd_clicked()) idle();
      KEEPALIVE_STATE(IN_HANDLER);
    }
    if (IS_SD_PRINTING)
      LCD_MESSAGEPGM(MSG_RESUMING);
    else
      LCD_MESSAGEPGM(WELCOME_MSG);
  }
#endif //ULTIPANEL

#if ENABLED(LASERBEAM)
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

/**
 * M11: Start/Stop printing serial mode
 */
inline void gcode_M11() {
  if (Printing) {
    Printing = false;
    ECHO_LM(DB, "Stop Printing");
    #if ENABLED(STOP_GCODE)
      enqueue_and_echo_commands_P(PSTR(STOP_PRINTING_SCRIPT));
    #endif
    #if HAS(FILRUNOUT)
      filrunoutEnqueued = false;
      ECHO_LM(DB, "Filament runout deactivated.");
    #endif
  }
  else {
    Printing = true;
    ECHO_LM(DB, "Start Printing");
    #if ENABLED(START_GCODE)
      enqueue_and_echo_commands_P(PSTR(START_PRINTING_SCRIPT));
    #endif
    #if HAS(FILRUNOUT)
      filrunoutEnqueued = false;
      ECHO_LM(DB, "Filament runout activated.");
      ECHO_S(RESUME);
      ECHO_E;
    #endif
    #if HAS(POWER_CONSUMPTION_SENSOR)
      startpower = power_consumption_hour;
    #endif
  }
}

/**
 * M17: Enable power on all stepper motors
 */
inline void gcode_M17() {
  LCD_MESSAGEPGM(MSG_NO_MOVE);
  enable_all_steppers();
}

#if ENABLED(SDSUPPORT)

  /**
   * M20: List SD card to serial output
   */
  inline void gcode_M20() {
    ECHO_EM(SERIAL_BEGIN_FILE_LIST);
    card.ls();
    ECHO_EM(SERIAL_END_FILE_LIST);
  }

  /**
   * M21: Init SD Card
   */
  inline void gcode_M21() {
    card.mount();
  }

  /**
   * M22: Release SD Card
   */
  inline void gcode_M22() {
    card.unmount();
  }

  /**
   * M23: Select a file
   */
  inline void gcode_M23() {
    card.selectFile(current_command_args);
  }

  /**
   * M24: Start SD Print
   */
  inline void gcode_M24() {
    card.startPrint();
    print_job_timer.start();
    #if HAS(POWER_CONSUMPTION_SENSOR)
      startpower = power_consumption_hour;
    #endif
  }

  /**
   * M25: Pause SD Print
   */
  inline void gcode_M25() {
    card.pausePrint();
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
    card.printStatus();
  }

  /**
   * M28: Start SD Write
   */
  inline void gcode_M28() {
    card.startWrite(current_command_args, false);
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
      card.deleteFile(current_command_args);
    }
  }

  /**
   * M31: Get the time since the start of SD Print (or last M109)
   */
  inline void gcode_M31() {
    millis_t t = print_job_timer.duration();
    int min = t / 60, sec = t % 60;
    char time[30];
    sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
    ECHO_LT(DB, time);
    lcd_setstatus(time);
    autotempShutdown();
  }

  /**
   * M32: Make Directory
   */
  inline void gcode_M32() {
    if (card.cardOK) {
      card.makeDirectory(current_command_args);
      card.mount();
    }
  }

  #if ENABLED(NEXTION)
    /**
     * M35: Upload Firmware to Nextion from SD
     */
    inline void gcode_M35() {
      UploadNewFirmware();
    }
  #endif
#endif

/**
 * M42: Change pin status via GCode
 */
inline void gcode_M42() {
  if (code_seen('S')) {
    int pin_status = code_value_short(),
        pin_number = LED_PIN;

    if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
      pin_number = code_value_short();

    for (uint8_t i = 0; i < COUNT(sensitive_pins); i++) {
      if (sensitive_pins[i] == pin_number) {
        pin_number = -1;
        break;
      }
    }

    #if HAS(FAN)
      if (pin_number == FAN_PIN) fanSpeed = pin_status;
    #endif

    if (pin_number > -1) {
      pinMode(pin_number, OUTPUT);
      digitalWrite(pin_number, pin_status);
      analogWrite(pin_number, pin_status);
    }
  } // code_seen('S')
}

#if ENABLED(AUTO_BED_LEVELING_FEATURE) && ENABLED(Z_PROBE_REPEATABILITY_TEST)
  /**
   * M48: Z-Probe repeatability measurement function.
   *
   * Usage:
   *   M48 <P#> <X#> <Y#> <V#> <E> <L#>
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
  inline void gcode_M48() {
    if (DEBUGGING(INFO)) ECHO_LM(INFO, "gcode_M48 >>>");

    double sum = 0.0, mean = 0.0, sigma = 0.0, sample_set[50];
    uint8_t verbose_level = 1, n_samples = 10, n_legs = 0;

    if (code_seen('V') || code_seen('v')) {
      verbose_level = code_value_short();
      if (verbose_level < 0 || verbose_level > 4 ) {
        ECHO_LM(ER,"?Verbose Level not plausible (0-4).");
        return;
      }
    }

    if (verbose_level > 0)
      ECHO_LM(DB, "M48 Z-Probe Repeatability test");

    if (code_seen('P') || code_seen('p')) {
      n_samples = code_value_short();
      if (n_samples < 4 || n_samples > 50) {
        ECHO_LM(ER, "?Sample size not plausible (4-50).");
        return;
      }
    }

    double X_current = st_get_axis_position_mm(X_AXIS),
           Y_current = st_get_axis_position_mm(Y_AXIS),
           Z_current = st_get_axis_position_mm(Z_AXIS),
           E_current = st_get_axis_position_mm(E_AXIS),
           X_probe_location = X_current, Y_probe_location = Y_current,
           Z_start_location = Z_current + Z_RAISE_BEFORE_PROBING;

    bool deploy_probe_for_each_reading = code_seen('E') || code_seen('e');

    if (code_seen('X') || code_seen('x')) {
      X_probe_location = code_value() - (X_PROBE_OFFSET_FROM_EXTRUDER);
      if (X_probe_location < X_MIN_POS || X_probe_location > X_MAX_POS) {
        out_of_range_error(PSTR("X"));
        return;
      }
    }

    if (code_seen('Y') || code_seen('y')) {
      Y_probe_location = code_value() -  (Y_PROBE_OFFSET_FROM_EXTRUDER);
      if (Y_probe_location < Y_MIN_POS || Y_probe_location > Y_MAX_POS) {
        out_of_range_error(PSTR("Y"));
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
    plan_buffer_line(X_current, Y_current, Z_start_location, E_current, homing_feedrate[Z_AXIS]/60, active_extruder, active_driver);
    st_synchronize();

    //
    // Now get everything to the specified probe point So we can safely do a probe to
    // get us close to the bed.  If the Z-Axis is far from the bed, we don't want to 
    // use that as a starting point for each probe.
    //
    if (verbose_level > 2)
      ECHO_LM(DB, "Positioning the probe...");

    plan_buffer_line(X_probe_location, Y_probe_location, Z_start_location, E_current, homing_feedrate[X_AXIS]/60, active_extruder, active_driver);
    st_synchronize();

    current_position[X_AXIS] = X_current = st_get_axis_position_mm(X_AXIS);
    current_position[Y_AXIS] = Y_current = st_get_axis_position_mm(Y_AXIS);
    current_position[Z_AXIS] = Z_current = st_get_axis_position_mm(Z_AXIS);
    current_position[E_AXIS] = E_current = st_get_axis_position_mm(E_AXIS);

    // 
    // OK, do the initial probe to get us close to the bed.
    // Then retrace the right amount and use that in subsequent probes
    //

    deploy_z_probe();

    setup_for_endstop_move();
    run_z_probe();

    current_position[Z_AXIS] = Z_current = st_get_axis_position_mm(Z_AXIS);
    Z_start_location = st_get_axis_position_mm(Z_AXIS) + Z_RAISE_BEFORE_PROBING;

    plan_buffer_line(X_probe_location, Y_probe_location, Z_start_location, E_current, homing_feedrate[X_AXIS]/60, active_extruder, active_driver);
    st_synchronize();
    current_position[Z_AXIS] = Z_current = st_get_axis_position_mm(Z_AXIS);

    if (deploy_probe_for_each_reading) stow_z_probe();

    for (uint8_t n=0; n < n_samples; n++) {
      // Make sure we are at the probe location
      do_blocking_move_to(X_probe_location, Y_probe_location, Z_start_location); // this also updates current_position

      if (n_legs) {
        millis_t ms = millis();
        double radius = ms % ((X_MAX_LENGTH) / 4),       // limit how far out to go
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
          X_current = constrain(X_current, X_MIN_POS + 10, X_MAX_POS - 10);
          Y_current = Y_probe_location + sin(theta) * radius;
          Y_current = constrain(Y_current, Y_MIN_POS + 10, Y_MAX_POS - 10);

          if (verbose_level > 3) {
            ECHO_SMV(DB, "x: ", X_current);
            ECHO_EMV(" y: ", Y_current);
          }

          do_blocking_move_to(X_current, Y_current, Z_current); // this also updates current_position

        } // n_legs loop

        // Go back to the probe location
        do_blocking_move_to(X_probe_location, Y_probe_location, Z_start_location); // this also updates current_position

      } // n_legs

      if (deploy_probe_for_each_reading) {
        deploy_z_probe(); 
        delay_ms(1000);
      }

      setup_for_endstop_move();
      run_z_probe();

      sample_set[n] = current_position[Z_AXIS];

      // Get the current mean for the data points we have so far
      sum = 0.0;
      for (uint8_t j = 0; j <= n; j++) sum += sample_set[j];
      mean = sum / (n + 1);

      // Now, use that mean to calculate the standard deviation for the
      // data points we have so far
      sum = 0.0;
      for (uint8_t j = 0; j <= n; j++) {
        float ss = sample_set[j] - mean;
        sum += ss * ss;
      }
      sigma = sqrt(sum / (n + 1));

      if (verbose_level > 1) {
        ECHO_SV(DB, n + 1);
        ECHO_MV(" of ", (int)n_samples);
        ECHO_EM(" samples");
        ECHO_SMV(DB, "z: ", current_position[Z_AXIS], 6);
        if (verbose_level > 2) {
          ECHO_MV(" mean: ", mean,6);
          ECHO_MV("   sigma: ", sigma,6);
        }
        ECHO_E;
      }

      if (verbose_level > 0) ECHO_E;

      plan_buffer_line(X_probe_location, Y_probe_location, Z_start_location, E_current, homing_feedrate[Z_AXIS]/60, active_extruder, active_driver);
      st_synchronize();

      if (deploy_probe_for_each_reading) {
        stow_z_probe();
        delay_ms(1000);
      }
    }

    if (!deploy_probe_for_each_reading) {
      stow_z_probe();
      delay_ms(1000);
    }

    clean_up_after_endstop_move();

    if (verbose_level > 0) ECHO_EMV("Mean: ", mean, 6);
    ECHO_EMV("Standard Deviation: ", sigma, 6);

    if (DEBUGGING(INFO)) ECHO_LM(INFO, "<<< gcode_M28");
  }

#endif // AUTO_BED_LEVELING_FEATURE && Z_PROBE_REPEATABILITY_TEST

#if HAS(POWER_CONSUMPTION_SENSOR)
  /**
   * M70 - Power consumption sensor calibration
   *
   * Z - Calibrate zero current offset
   * A - Isert readed DC Current value (Ampere)
   * W - Insert readed AC Wattage value (Watt)
   */
  inline void gcode_M70() {
    if(code_seen('Z')) {
      ECHO_EMV("Actual POWER_ZERO:", POWER_ZERO, 7);
      ECHO_EMV("New POWER_ZERO:", raw_analog2voltage(), 7);
      ECHO_EM("Insert new calculated values into the FW and call \"M70 A\" for the next calibration step.");
    }
    else if(code_seen('A')) {
      ECHO_EMV("Actual POWER_ERROR:", POWER_ERROR, 7);
      ECHO_EMV("New POWER_ERROR:", analog2error(code_value()), 7);
      ECHO_EM("Insert new calculated values into the FW and call \"M70 W\" for the last calibration step.");
    }
    else if(code_seen('W')) {
      ECHO_EMV("Actual POWER_EFFICIENCY:", POWER_EFFICIENCY, 7);
      ECHO_EMV("New POWER_EFFICIENCY:", analog2efficiency(code_value()), 7);
      ECHO_EM("Insert new calculated values into the FW and then ACS712 it should be calibrated correctly.");
    }
  }
#endif

/**
 * M75: Start print timer
 */
inline void gcode_M75() {
  print_job_timer.start();
}

/**
 * M76: Pause print timer
 */
inline void gcode_M76() {
  print_job_timer.pause();
}

/**
 * M77: Stop print timer
 */
inline void gcode_M77() {
  print_job_timer.stop();
}

#if HAS(POWER_SWITCH)
  /**
   * M80: Turn on Power Supply
   */
  inline void gcode_M80() {
    OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE); // GND

    // If you have a switch on suicide pin, this is useful
    // if you want to start another print with suicide feature after
    // a print without suicide...
    #if HAS(SUICIDE)
      OUT_WRITE(SUICIDE_PIN, HIGH);
    #endif

    powersupply = true;

    #if ENABLED(ULTIPANEL) || ENABLED(NEXTION)
      LCD_MESSAGEPGM(WELCOME_MSG);
      lcd_update();
    #endif
  }
#endif // HAS(POWER_SWITCH)

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
  #if ENABLED(LASERBEAM)
    laser_ttl_modulation = 0;
  #endif
  delay_ms(1000); // Wait 1 second before switching off
  #if HAS(SUICIDE)
    st_synchronize();
    suicide();
  #elif HAS(POWER_SWITCH)
    OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    powersupply = false;
  #endif
  #if ENABLED(ULTIPANEL)
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
 * M92: Set axis_steps_per_unit
 */
inline void gcode_M92() {
  if (setTargetedExtruder(92)) return;

  for(uint8_t i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      if (i == E_AXIS)
        axis_steps_per_unit[i + target_extruder] = code_value();
      else
        axis_steps_per_unit[i] = code_value();
    }
  }
  st_synchronize();
  // This recalculates position in steps in case user has changed steps/unit
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}

#if ENABLED(ZWOBBLE)
  /**
   * M96: Print ZWobble value
   */
  inline void gcode_M96() {
    zwobble.ReportToSerial();
  }

  /**
   * M97: Set ZWobble value
   */
  inline void gcode_M97() {
    float zVal = -1, hVal = -1, lVal = -1;

    if (code_seen('A')) zwobble.setAmplitude(code_value());
    if (code_seen('W')) zwobble.setPeriod(code_value());
    if (code_seen('P')) zwobble.setPhase(code_value());
    if (code_seen('Z')) zVal = code_value();
    if (code_seen('H')) hVal = code_value();
    if (code_seen('L')) lVal = code_value();
    if (zVal >= 0 && hVal >= 0) zwobble.setSample(zVal, hVal);
    if (zVal >= 0 && lVal >= 0) zwobble.setScaledSample(zVal, lVal);
    if (lVal >  0 && hVal >  0) zwobble.setScalingFactor(hVal/lVal);
  }
#endif // ZWOBBLE

#if ENABLED(HYSTERESIS)
  /**
   * M98: Print Hysteresis value
   */
  inline void gcode_M98() {
    hysteresis.ReportToSerial();
  }

  /**
   * M99: Set Hysteresis value
   */
  inline void gcode_M99() {
    for(uint8_t i = 0; i < NUM_AXIS; i++) {
      if (code_seen(axis_codes[i]))
        hysteresis.SetAxis(i, code_value());
    }
  }
#endif // HYSTERESIS

/**
 * M100 Free Memory Watcher
 *
 * This code watches the free memory block between the bottom of the heap and the top of the stack.
 * This memory block is initialized and watched via the M100 command.
 *
 * M100 I Initializes the free memory block and prints vitals statistics about the area
 * M100 F Identifies how much of the free memory block remains free and unused. It also
 *        detects and reports any corruption within the free memory block that may have
 *        happened due to errant firmware.
 * M100 D Does a hex display of the free memory block along with a flag for any errant
 *        data that does not match the expected value.
 * M100 C x Corrupts x locations within the free memory block. This is useful to check the
 *        correctness of the M100 F and M100 D commands.
 *
 * Initial version by Roxy-3DPrintBoard
 *
 */
#if ENABLED(M100_FREE_MEMORY_WATCHER)
  inline void gcode_M100() {
    static int m100_not_initialized = 1;
    unsigned char* sp, *ptr;
    int i, j, n;

    // M100 D dumps the free memory block from __brkval to the stack pointer.
    // malloc() eats memory from the start of the block and the stack grows
    // up from the bottom of the block.    Solid 0xE5's indicate nothing has
    // used that memory yet.   There should not be anything but 0xE5's within
    // the block of 0xE5's.  If there is, that would indicate memory corruption
    // probably caused by bad pointers.  Any unexpected values will be flagged in
    // the right hand column to help spotting them.
    #if ENABLED(M100_FREE_MEMORY_DUMPER)      // Comment out to remove Dump sub-command
      if (code_seen('D')) {
        ptr = (unsigned char*) __brkval;

        // We want to start and end the dump on a nice 16 byte boundry even though
        // the values we are using are not 16 byte aligned.
        //
        ECHO_M("\n__brkval : ");
        prt_hex_word((unsigned int) ptr);
        ptr = (unsigned char*) ((unsigned long) ptr & 0xfff0);

        sp = top_of_stack();
        ECHO_M("\nStack Pointer : ");
        prt_hex_word((unsigned int) sp);
        ECHO_M("\n");

        sp = (unsigned char*) ((unsigned long) sp | 0x000f);
        n = sp - ptr;

        // This is the main loop of the Dump command.
        while (ptr < sp) {
          prt_hex_word((unsigned int) ptr);  // Print the address
          ECHO_M(":");
          for(i = 0; i < 16; i++) {     // and 16 data bytes
            prt_hex_byte( *(ptr+i));
            ECHO_M(" ");
            HAL::delayMilliseconds(2);
          }

          ECHO_M("|");        // now show where non 0xE5's are
          for(i = 0; i < 16; i++) {
            HAL::delayMilliseconds(2);
            if ( *(ptr+i)==0xe5)
              ECHO_M(" ");
            else
              ECHO_M("?");
          }
          ECHO_M("\n");

          ptr += 16;
          HAL::delayMilliseconds(2);
        }
        ECHO_M("Done.\n");
        return;
      }
    #endif

    // M100 F   requests the code to return the number of free bytes in the memory pool along with
    // other vital statistics that define the memory pool.
    if (code_seen('F')) {
      int max_addr = (int) __brkval;
      int max_cnt = 0;
      int block_cnt = 0;
      ptr = (unsigned char*) __brkval;
      sp = top_of_stack();
      n = sp - ptr;

      // Scan through the range looking for the biggest block of 0xE5's we can find
      for (i = 0; i < n; i++) {
        if ( *(ptr+i) == (unsigned char) 0xe5) {
          j = how_many_E5s_are_here((unsigned char*) ptr + i);
          if ( j > 8) {
            ECHO_MV("Found ", j );
            ECHO_M(" bytes free at 0x");
            prt_hex_word((int) ptr + i);
            ECHO_M("\n");
            i += j;
            block_cnt++;
          }
          if (j > max_cnt) {  // We don't do anything with this information yet
            max_cnt  = j;     // but we do know where the biggest free memory block is.
            max_addr = (int) ptr + i;
          }
        }
      }
      if (block_cnt > 1)
          ECHO_EM("\nMemory Corruption detected in free memory area.\n");

      ECHO_M("\nDone.\n");
      return;
    }

    // M100 C x  Corrupts x locations in the free memory pool and reports the locations of the corruption.
    // This is useful to check the correctness of the M100 D and the M100 F commands.
    #if ENABLED(M100_FREE_MEMORY_CORRUPTOR)
      if (code_seen('C')) {
        int x;    // x gets the # of locations to corrupt within the memory pool
        x = code_value();
        ECHO_EM("Corrupting free memory block.\n");
        ptr = (unsigned char*) __brkval;
        ECHO_MV("\n__brkval : ",(long) ptr);
        ptr += 8;

        sp = top_of_stack();
        ECHO_MV("\nStack Pointer : ",(long) sp);
        ECHO_EM("\n");

        n = sp - ptr - 64;    // -64 just to keep us from finding interrupt activity that
                              // has altered the stack.
        j = n / (x + 1);
        for(i = 1; i <= x; i++) {
          *(ptr + (i * j)) = i;
          ECHO_M("\nCorrupting address: 0x");
          prt_hex_word((unsigned int) (ptr + (i * j)));
        }
        ECHO_EM("\n");
        return;
      }
    #endif

    // M100 I    Initializes the free memory pool so it can be watched and prints vital
    // statistics that define the free memory pool.
    if (m100_not_initialized || code_seen('I')) {     // If no sub-command is specified, the first time
      ECHO_EM("Initializing free memory block.\n");   // this happens, it will Initialize.
      ptr = (unsigned char*) __brkval;        // Repeated M100 with no sub-command will not destroy the
      ECHO_MV("\n__brkval : ",(long) ptr);    // state of the initialized free memory pool.
      ptr += 8;

      sp = top_of_stack();
      ECHO_MV("\nStack Pointer : ",(long) sp );
      ECHO_EM("\n");

      n = sp - ptr - 64;    // -64 just to keep us from finding interrupt activity that
                            // has altered the stack.

      ECHO_V( n );
      ECHO_EM(" bytes of memory initialized.\n");

      for(i = 0; i < n; i++)
        *(ptr+i) = (unsigned char) 0xe5;

      for(i = 0; i < n; i++) {
        if ( *(ptr + i) != (unsigned char) 0xe5) {
          ECHO_MV("? address : ", (unsigned long) ptr + i);
          ECHO_MV("=", *(ptr + i));
          ECHO_EM("\n");
        }
      }
      m100_not_initialized = 0;
      ECHO_EM("Done.\n");
      return;
    }
    return;
  }
#endif

/**
 * M104: Set hot end temperature
 */
inline void gcode_M104() {
  if (setTargetedExtruder(104)) return;
  if (DEBUGGING(DRYRUN)) return;

  #if HOTENDS == 1
    if (target_extruder != active_extruder) return;
  #endif

  if (code_seen('S')) {
    float temp = code_value();
    setTargetHotend(temp, target_extruder);
    #if ENABLED(DUAL_X_CARRIAGE)
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && target_extruder == 0)
        setTargetHotend1(temp == 0.0 ? 0.0 : temp + duplicate_extruder_temp_offset);
    #endif

    /**
     * We use half EXTRUDE_MINTEMP here to allow nozzles to be put into hot
     * stand by mode, for instance in a dual extruder setup, without affecting
     * the running print timer.
     */
    if (temp <= (EXTRUDE_MINTEMP)/2) {
      print_job_timer.stop();
      LCD_MESSAGEPGM(WELCOME_MSG);
    }
    /**
     * We do not check if the timer is already running because this check will
     * be done for us inside the Stopwatch::start() method thus a running timer
     * will not restart.
     */
    else print_job_timer.start();

    if (temp > degHotend(target_extruder)) LCD_MESSAGEPGM(MSG_HEATING);
  }
}

/**
 * M105: Read hot end and bed temperature
 */
inline void gcode_M105() {
  if (setTargetedExtruder(105)) return;

  #if HAS(TEMP_0) || HAS(TEMP_BED) || ENABLED(HEATER_0_USES_MAX6675)
    ECHO_S(OK);
    print_heaterstates();
  #else // HASNT(TEMP_0) && HASNT(TEMP_BED)
    ECHO_LM(ER, SERIAL_ERR_NO_THERMISTORS);
  #endif

  ECHO_E;
}

#if HAS(FAN)
  /**
   * M106: Set Fan Speed
   */
  inline void gcode_M106() { fanSpeed = code_seen('S') ? constrain(code_value_short(), 0, 255) : 255; }

  /**
   * M107: Fan Off
   */
  inline void gcode_M107() { fanSpeed = 0; }

#endif // HAS(FAN)

/**
 * M109: Sxxx Wait for extruder(s) to reach temperature. Waits only when heating.
 *       Rxxx Wait for extruder(s) to reach temperature. Waits when heating and cooling.
 */
inline void gcode_M109() {
  if (setTargetedExtruder(109)) return;
  if (DEBUGGING(DRYRUN)) return;

  #if HOTENDS == 1
    if (target_extruder != active_extruder) return;
  #endif

  LCD_MESSAGEPGM(MSG_HEATING);

  no_wait_for_cooling = code_seen('S');
  if (no_wait_for_cooling || code_seen('R')) {
    float temp = code_value();
    setTargetHotend(temp, target_extruder);
    #if ENABLED(DUAL_X_CARRIAGE)
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && target_extruder == 0)
        setTargetHotend1(temp == 0.0 ? 0.0 : temp + duplicate_extruder_temp_offset);
    #endif

    /**
     * We use half EXTRUDE_MINTEMP here to allow nozzles to be put into hot
     * stand by mode, for instance in a dual extruder setup, without affecting
     * the running print timer.
     */
    if (temp <= (EXTRUDE_MINTEMP)/2) {
      print_job_timer.stop();
      LCD_MESSAGEPGM(WELCOME_MSG);
    }
    /**
     * We do not check if the timer is already running because this check will
     * be done for us inside the Stopwatch::start() method thus a running timer
     * will not restart.
     */
    else print_job_timer.start();

    if (temp > degHotend(target_extruder)) LCD_MESSAGEPGM(MSG_HEATING);
  }

  #if ENABLED(AUTOTEMP)
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
  mk_debug_flags = code_seen('S') ? code_value_short() : DEBUG_NONE;

  const static char str_debug_1[]   PROGMEM = SERIAL_DEBUG_ECHO;
  const static char str_debug_2[]   PROGMEM = SERIAL_DEBUG_INFO;
  const static char str_debug_4[]   PROGMEM = SERIAL_DEBUG_ERRORS;
  const static char str_debug_8[]   PROGMEM = SERIAL_DEBUG_DRYRUN;
  const static char str_debug_16[]  PROGMEM = SERIAL_DEBUG_COMMUNICATION;
  const static char str_debug_32[]  PROGMEM = SERIAL_DEBUG_DEBUG;

  const static char* const debug_strings[] PROGMEM = {
    str_debug_1, str_debug_2, str_debug_4, str_debug_8, str_debug_16, str_debug_32
  };

  ECHO_M(SERIAL_DEBUG_PREFIX);
  if (mk_debug_flags) {
    uint8_t comma = 0;
    for (uint8_t i = 0; i < COUNT(debug_strings); i++) {
      if (TEST(mk_debug_flags, i)) {
        if (comma++) ECHO_C(',');
        ECHO_T(debug_strings[i]);
      }
    }
  }
  else {
    ECHO_M(SERIAL_DEBUG_OFF);
  }
  ECHO_E;
}

/**
 * M112: Emergency Stop
 */
inline void gcode_M112() { kill(PSTR(MSG_KILLED)); }

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  /**
   * M113: Get or set Host Keepalive interval (0 to disable)
   *
   *   S<seconds> Optional. Set the keepalive interval.
   */
  inline void gcode_M113() {
    if (code_seen('S')) {
      host_keepalive_interval = (uint8_t)code_value_short();
      NOMORE(host_keepalive_interval, 60);
    }
    else {
      ECHO_LMV(DB, "M113 S", (unsigned long)host_keepalive_interval);
    }
  }
#endif

/**
 * M114: Output current position to serial port
 */
inline void gcode_M114() {
  ECHO_MV( "X:", current_position[X_AXIS]);
  ECHO_MV(" Y:", current_position[Y_AXIS]);
  ECHO_MV(" Z:", current_position[Z_AXIS]);
  ECHO_MV(" E:", current_position[E_AXIS]);

  CRITICAL_SECTION_START;
  extern volatile long count_position[NUM_AXIS];
  long  xpos = count_position[X_AXIS],
        ypos = count_position[Y_AXIS],
        zpos = count_position[Z_AXIS];
  CRITICAL_SECTION_END;

  #if MECH(COREXY) || MECH(COREYX) || MECH(COREXZ) || MECH(COREZX)
    ECHO_M(MSG_COUNT_A);
  #elif MECH(DELTA)
    ECHO_M(MSG_COUNT_ALPHA);
  #else
    ECHO_M(MSG_COUNT_X);
  #endif
  ECHO_V(xpos);

  #if MECH(COREXY) || MECH(COREYX)
    ECHO_M(" B:");
  #elif MECH(DELTA)
    ECHO_M(" Beta:");
  #else
    ECHO_M(" Y:");
  #endif
  ECHO_V(ypos);

  #if MECH(COREXZ) || MECH(COREZX)
    ECHO_M(" C:");
  #elif MECH(DELTA)
    ECHO_M(" Teta:");
  #else
    ECHO_M(" Z:");
  #endif
  ECHO_V(zpos);

  ECHO_E;

  #if MECH(SCARA)
    // MESSAGE for Host
    ECHO_SMV(OK, " SCARA Theta:", delta[X_AXIS]);
    ECHO_EMV("   Psi+Theta:", delta[Y_AXIS]);

    ECHO_SMV(DB, "SCARA Cal - Theta:", delta[X_AXIS] + home_offset[X_AXIS]);
    ECHO_EMV("   Psi+Theta (90):", delta[Y_AXIS]-delta[X_AXIS] - 90 + home_offset[Y_AXIS]);

    ECHO_SMV(DB, "SCARA step Cal - Theta:", delta[X_AXIS] / 90 * axis_steps_per_unit[X_AXIS]);
    ECHO_EMV("   Psi+Theta:", (delta[Y_AXIS]-delta[X_AXIS]) / 90 * axis_steps_per_unit[Y_AXIS]);
    ECHO_E;
  #endif
}

/**
 * M115: Capabilities string
 */
inline void gcode_M115() {
  ECHO_M(SERIAL_M115_REPORT);
}

#if ENABLED(ULTIPANEL) || ENABLED(NEXTION)

  /**
   * M117: Set LCD Status Message
   */
  inline void gcode_M117() {
    lcd_setstatus(current_command_args);
  }

#endif

/**
 * M119: Output endstop states to serial output
 */
inline void gcode_M119() {
  ECHO_LM(DB, SERIAL_M119_REPORT);
  #if HAS(X_MIN)
    ECHO_EMT(SERIAL_X_MIN, ((READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(X_MAX)
    ECHO_EMT(SERIAL_X_MAX, ((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(Y_MIN)
    ECHO_EMT(SERIAL_Y_MIN, ((READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(Y_MAX)
    ECHO_EMT(SERIAL_Y_MAX, ((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(Z_MIN)
    ECHO_EMT(SERIAL_Z_MIN, ((READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(Z_MAX)
    ECHO_EMT(SERIAL_Z_MAX, ((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(Z2_MAX)
    ECHO_EMT(SERIAL_Z2_MAX, ((READ(Z2_MAX_PIN)^Z2_MAX_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(Z_PROBE)
    ECHO_EMT(SERIAL_Z_PROBE, ((READ(Z_PROBE_PIN)^Z_PROBE_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(E_MIN)
    ECHO_EMT(SERIAL_E_MIN, ((READ(E_MIN_PIN)^E_MIN_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(FILRUNOUT)
    ECHO_EMT(SERIAL_FILRUNOUT_PIN, ((READ(FILRUNOUT_PIN)^FILRUNOUT_PIN_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  ECHO_E;
}

/**
 * M120: Enable endstops
 */
inline void gcode_M120() { enable_endstops(true); }

/**
 * M121: Disable endstops
 */
inline void gcode_M121() { enable_endstops(false); }

/**
 * M122: Disable or enable software endstops
 */
inline void gcode_M122() {
  if (code_seen('S')) {
    if (code_value() == 0)
      software_endstops = false;
    else
      software_endstops = true;
  }
}

#if ENABLED(BARICUDA)
  #if HAS(HEATER_1)
    /**
     * M126: Heater 1 valve open
     */
    inline void gcode_M126() { baricuda_valve_pressure = code_seen('S') ? constrain(code_value(), 0, 255) : 255; }
    /**
     * M127: Heater 1 valve close
     */
    inline void gcode_M127() { baricuda_valve_pressure = 0; }
  #endif

  #if HAS(HEATER_2)
    /**
     * M128: Heater 2 valve open
     */
    inline void gcode_M128() { baricuda_e_to_p_pressure = code_seen('S') ? constrain(code_value(), 0, 255) : 255; }
    /**
     * M129: Heater 2 valve close
     */
    inline void gcode_M129() { baricuda_e_to_p_pressure = 0; }
  #endif
#endif //BARICUDA

/**
 * M140: Set bed temperature
 */
inline void gcode_M140() {
  if (DEBUGGING(DRYRUN)) return;
  if (code_seen('S')) setTargetBed(code_value());
}

#if ENABLED(ULTIPANEL) && TEMP_SENSOR_0 != 0
  /**
   * M145: Set the heatup state for a material in the LCD menu
   *   S<material> (0=PLA, 1=ABS, 2=GUM)
   *   H<hotend temp>
   *   B<bed temp>
   *   F<fan speed>
   */
  inline void gcode_M145() {
    uint8_t material = code_seen('S') ? code_value_short() : 0;
    if (material < 0 || material > 2) {
      ECHO_SM(DB, SERIAL_ERR_MATERIAL_INDEX);
    }
    else {
      int v;
      switch (material) {
        case 0:
          if (code_seen('H')) {
            v = code_value_short();
            #if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
              plaPreheatHotendTemp = constrain(v, EXTRUDE_MINTEMP, HEATER_0_MAXTEMP);
            #else
              plaPreheatHotendTemp = constrain(v, HEATER_0_MINTEMP, HEATER_0_MAXTEMP);
            #endif
          }
          if (code_seen('F')) {
            v = code_value_short();
            plaPreheatFanSpeed = constrain(v, 0, 255);
          }
          #if TEMP_SENSOR_BED != 0
            if (code_seen('B')) {
              v = code_value_short();
              plaPreheatHPBTemp = constrain(v, BED_MINTEMP, BED_MAXTEMP);
            }
          #endif
          break;
        case 1:
          if (code_seen('H')) {
            v = code_value_short();
            #if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
              absPreheatHotendTemp = constrain(v, EXTRUDE_MINTEMP, HEATER_0_MAXTEMP);
            #else
              absPreheatHotendTemp = constrain(v, HEATER_0_MINTEMP, HEATER_0_MAXTEMP);
            #endif
          }
          if (code_seen('F')) {
            v = code_value_short();
            absPreheatFanSpeed = constrain(v, 0, 255);
          }
          #if TEMP_SENSOR_BED != 0
            if (code_seen('B')) {
              v = code_value_short();
              absPreheatHPBTemp = constrain(v, BED_MINTEMP, BED_MAXTEMP);
            }
          #endif
          break;
        case 2:
          if (code_seen('H')) {
            v = code_value_short();
            #if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
              gumPreheatHotendTemp = constrain(v, EXTRUDE_MINTEMP, HEATER_0_MAXTEMP);
            #else
              gumPreheatHotendTemp = constrain(v, HEATER_0_MINTEMP, HEATER_0_MAXTEMP);
            #endif
          }
          if (code_seen('F')) {
            v = code_value_short();
            gumPreheatFanSpeed = constrain(v, 0, 255);
          }
          #if TEMP_SENSOR_BED != 0
            if (code_seen('B')) {
              v = code_value_short();
              gumPreheatHPBTemp = constrain(v, BED_MINTEMP, BED_MAXTEMP);
            }
          #endif
          break;
      }
    }
  }
#endif

#if ENABLED(BLINKM)
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

#if ENABLED(COLOR_MIXING_EXTRUDER)
  /**
   * M163: Set a single mix factor for a mixing extruder
   *       This is called "weight" by some systems.
   *
   *   S[index]   The channel index to set
   *   P[float]   The mix value
   *
   */
  inline void gcode_M163() {
    int mix_index = code_seen('S') ? code_value_short() : 0;
    float mix_value = code_seen('P') ? code_value() : 0.0;
    if (mix_index < DRIVER_EXTRUDERS) mixing_factor[mix_index] = mix_value;
  }

  #if MIXING_VIRTUAL_TOOLS  > 1
    /**
     * M164: Store the current mix factors as a virtual tools.
     *
     *   S[index]   The virtual tools to store
     *
     */
    inline void gcode_M164() {
      int tool_index = code_seen('S') ? code_value_short() : 0;
      if (tool_index < MIXING_VIRTUAL_TOOLS) {
        normalize_mix();
        for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
          mixing_virtual_tool_mix[tool_index][i] = mixing_factor[i];
        }
      }
    }
  #endif

  /**
   * M165: Set multiple mix factors for a mixing extruder.
   *       Factors that are left out will be set to 0.
   *       All factors together must add up to 1.0.
   *
   *   A[factor] Mix factor for extruder stepper 1
   *   B[factor] Mix factor for extruder stepper 2
   *   C[factor] Mix factor for extruder stepper 3
   *   D[factor] Mix factor for extruder stepper 4
   *   H[factor] Mix factor for extruder stepper 5
   *   I[factor] Mix factor for extruder stepper 6
   *
   */
  inline void gcode_M165() { gcode_get_mix(); }
#endif  // COLOR_MIXING_EXTRUDER

#if HAS(TEMP_BED)
  /**
   * M190: Sxxx Wait for bed current temp to reach target temp. Waits only when heating
   *       Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
   */
  inline void gcode_M190() {
    if (DEBUGGING(DRYRUN)) return;

    LCD_MESSAGEPGM(MSG_BED_HEATING);
    no_wait_for_cooling = code_seen('S');
    if (no_wait_for_cooling || code_seen('R')) setTargetBed(code_value());

    wait_bed();
  }
#endif // HAS(TEMP_BED)

/**
 * M200: Set filament diameter and set E axis units to cubic millimetres
 *
 *    T<extruder> - Optional extruder number. Current extruder if omitted.
 *    D<mm> - Diameter of the filament. Use "D0" to set units back to millimetres.
 *    S<0/1> - Deactivate o Activate volumetric
 */
inline void gcode_M200() {

  if (setTargetedExtruder(200)) return;

  if (code_seen('D')) {
    float diameter = code_value();
    // setting any extruder filament size disables volumetric on the assumption that
    // slicers either generate in extruder values as cubic mm or as as filament feeds
    // for all extruders
    volumetric_enabled = (diameter != 0.0);
    filament_size[target_extruder] = diameter;
  }

  if (code_seen('S')) {
    if (code_value())
      volumetric_enabled = true;
    else
      volumetric_enabled = false;
  }

  if (volumetric_enabled) {
    ECHO_LM(INFO, "Volumetric Enabled");
    // make sure all extruders have some sane value for the filament size
    for (int i = 0; i < EXTRUDERS; i++)
      if (!filament_size[i]) filament_size[i] = DEFAULT_NOMINAL_FILAMENT_DIA;
  }
  else
    ECHO_LM(INFO, "Volumetric Disabled");

  calculate_volumetric_multipliers();
}

/**
 * M201: Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 */
inline void gcode_M201() {
  for (uint8_t i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      max_acceleration_units_per_sq_second[i] = code_value();
    }
  }
  // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
  reset_acceleration_rates();
}

#if 0 // Not used for Sprinter/grbl gen6
  inline void gcode_M202() {
    for(uint8_t i = 0; i < NUM_AXIS; i++) {
      if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
    }
  }
#endif


/**
 * M203: Set maximum feedrate that your machine can sustain in mm/sec
 *
 *    X,Y,Z   = AXIS
 *    T* E    = E_AXIS
 *
 */
inline void gcode_M203() {
  if (setTargetedExtruder(203)) return;

  for(uint8_t i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      if (i == E_AXIS)
        max_feedrate[i + target_extruder] = code_value();
      else
        max_feedrate[i] = code_value();
    }
  }
}

/**
 * M204: Set Accelerations in mm/sec^2 (M204 P1200 T0 R3000 V3000)
 *
 *    P     = Printing moves
 *    T* R  = Retract only (no X, Y, Z) moves
 *    V     = Travel (non printing) moves
 *
 *  Also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
 */
inline void gcode_M204() {
  if (setTargetedExtruder(204)) return;

  if (code_seen('S')) {  // Kept for legacy compatibility. Should NOT BE USED for new developments.
    acceleration = code_value();
    travel_acceleration = acceleration;
    ECHO_LMV(DB, "Setting Print and Travel Acceleration: ", acceleration );
  }
  if (code_seen('P')) {
    acceleration = code_value();
    ECHO_LMV(DB, "Setting Print Acceleration: ", acceleration );
  }
  if (code_seen('R')) {
    retract_acceleration[target_extruder] = code_value();
    ECHO_LMV(DB, "Setting Retract Acceleration: ", retract_acceleration[target_extruder]);
  }
  if (code_seen('V')) {
    travel_acceleration = code_value();
    ECHO_LMV(DB, "Setting Travel Acceleration: ", travel_acceleration );
  }
}

/**
 * M205: Set Advanced Settings
 *
 *    S = Min Feed Rate (mm/s)
 *    V = Min Travel Feed Rate (mm/s)
 *    B = Min Segment Time (µs)
 *    X = Max XY Jerk (mm/s/s)
 *    Z = Max Z Jerk (mm/s/s)
 *    E = Max E Jerk (mm/s/s)
 */
inline void gcode_M205() {
  if (setTargetedExtruder(205)) return;

  if (code_seen('S')) minimumfeedrate = code_value();
  if (code_seen('V')) mintravelfeedrate = code_value();
  if (code_seen('B')) minsegmenttime = code_value();
  if (code_seen('X')) max_xy_jerk = code_value();
  if (code_seen('Z')) max_z_jerk = code_value();
  if (code_seen('E')) max_e_jerk[target_extruder] = code_value();
}

/**
 * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
 */
inline void gcode_M206() {
  for (uint8_t i = X_AXIS; i <= Z_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      home_offset[i] = code_value();
    }
  }
  #if MECH(SCARA)
    if (code_seen('T')) home_offset[X_AXIS] = code_value(); // Theta
    if (code_seen('P')) home_offset[Y_AXIS] = code_value(); // Psi
  #endif
}

#if ENABLED(FWRETRACT)
  /**
   * M207: Set firmware retraction values
   *
   *   S[+mm]    retract_length
   *   W[+mm]    retract_length_swap (multi-extruder)
   *   F[mm/min] retract_feedrate
   *   Z[mm]     retract_zlift
   */
  inline void gcode_M207() {
    if (code_seen('S')) retract_length = code_value();
    if (code_seen('F')) retract_feedrate = code_value() / 60;
    if (code_seen('Z')) retract_zlift = code_value();
    #if EXTRUDERS > 1
      if (code_seen('W')) retract_length_swap = code_value();
    #endif
  }

  /**
   * M208: Set firmware un-retraction values
   *
   *   S[+mm]    retract_recover_length (in addition to M207 S*)
   *   W[+mm]    retract_recover_length_swap (multi-extruder)
   *   F[mm/min] retract_recover_feedrate
   */
  inline void gcode_M208() {
    if (code_seen('S')) retract_recover_length = code_value();
    if (code_seen('F')) retract_recover_feedrate = code_value() / 60;
    #if EXTRUDERS > 1
      if (code_seen('W')) retract_recover_length_swap = code_value();
    #endif
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
          unknown_command_error();
          return;
      }
      for (int i=0; i < EXTRUDERS; i++) retracted[i] = false;
    }
  }
#endif // FWRETRACT

/**
 * M218 - set hotend offset (in mm), H<hotend_number> X<offset_on_X> Y<offset_on_Y> Z<offset_on_Z>
 */
inline void gcode_M218() {
  if (setTargetedHotend(218)) return;

  if (code_seen('X')) hotend_offset[X_AXIS][target_extruder] = code_value();
  if (code_seen('Y')) hotend_offset[Y_AXIS][target_extruder] = code_value();
  if (code_seen('Z')) hotend_offset[Z_AXIS][target_extruder] = code_value();

  ECHO_SM(DB, SERIAL_HOTEND_OFFSET);
  for (uint8_t h = 0; h < HOTENDS; h++) {
    ECHO_MV(" ", hotend_offset[X_AXIS][h]);
    ECHO_MV(",", hotend_offset[Y_AXIS][h]);
    ECHO_MV(",", hotend_offset[Z_AXIS][h]);
  }
  ECHO_E;
}

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
  if (setTargetedExtruder(221)) return;

  if (code_seen('S')) extruder_multiplier[target_extruder] = code_value();
}

/**
 * M222: Set density extrusion percentage (M222 T0 S95)
 */
inline void gcode_M222() {
  if (setTargetedExtruder(222)) return;

  if (code_seen('S')) {
    density_multiplier[target_extruder] = code_value();
    #if ENABLED(RFID_MODULE)
      RFID522.RfidData[target_extruder].data.density = density_multiplier[target_extruder];
    #endif
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

      for (uint8_t i = 0; i < COUNT(sensitive_pins); i++) {
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

        while(digitalRead(pin_number) != target) idle();

      } // pin_number > -1
    } // pin_state -1 0 1
  } // code_seen('P')
}

#if HAS(CHDK) || HAS(PHOTOGRAPH)
  /**
   * M240: Trigger a camera
   */
  inline void gcode_M240() {
    #if HAS(CHDK)
       OUT_WRITE(CHDK_PIN, HIGH);
       chdkHigh = millis();
       chdkActive = true;
    #elif HAS(PHOTOGRAPH)
      const uint8_t NUM_PULSES = 16;
      const float PULSE_LENGTH = 0.01524;
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        HAL::delayMilliseconds(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        HAL::delayMilliseconds(PULSE_LENGTH);
      }
      HAL::delayMilliseconds(7.33);
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        HAL::delayMilliseconds(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        HAL::delayMilliseconds(PULSE_LENGTH);
      }
    #endif // HASNT(CHDK) && HAS(PHOTOGRAPH)
  }
#endif // HAS(CHDK) || PHOTOGRAPH_PIN

#if HAS(LCD_CONTRAST)
  /**
   * M250: Read and optionally set the LCD contrast
   */
  inline void gcode_M250() {
    if (code_seen('C')) lcd_setcontrast(code_value_short() & 0x3F);
    ECHO_LMV(DB, "lcd contrast value: ", lcd_contrast);
  }

#endif // DOGLCD

#if HAS(SERVOS)
  /**
   * M280: Get or set servo position. P<index> S<angle>
   */
  inline void gcode_M280() {
    int servo_index = code_seen('P') ? code_value_short() : -1;
    int servo_position = 0;
    #if ENABLED(DONDOLO)
      if (code_seen('S')) {
        servo_position = code_value_short();
        if (servo_index >= 0 && servo_index < NUM_SERVOS && servo_index != DONDOLO_SERVO_INDEX) {
          servo[servo_index].move(servo_position);
        }
        else if(servo_index == DONDOLO_SERVO_INDEX) {
          Servo *srv = &servo[servo_index];
          srv->attach(0);
          srv->write(servo_position);
          delay_ms(DONDOLO_SERVO_DELAY);
          srv->detach();
        }
        else {
          ECHO_SM(ER, "Servo ");
          ECHO_EVM(servo_index, " out of range");
        }
      }
    #else
      if (code_seen('S')) {
        servo_position = code_value_short();
        if (servo_index >= 0 && servo_index < NUM_SERVOS) {
          servo[servo_index].move(servo_position);
        }
        else {
          ECHO_SM(ER, "Servo ");
          ECHO_EVM(servo_index, " out of range");
        }
      }
      else if (servo_index >= 0) {
        ECHO_SMV(OK, " Servo ", servo_index);
        ECHO_EMV(": ", servo[servo_index].read());
      }
    #endif
  }
#endif // NUM_SERVOS > 0

#if HAS(BUZZER)

  /**
   * M300: Play beep sound S<frequency Hz> P<duration ms>
   */
  inline void gcode_M300() {
    uint16_t beepS = code_seen('S') ? code_value_short() : 100;
    uint32_t beepP = code_seen('P') ? code_value_long() : 1000;
    if (beepP > 5000) beepP = 5000; // limit to 5 seconds
    buzz(beepP, beepS);
  }

#endif // HAS(BUZZER)

#if ENABLED(PIDTEMP)
  /**
   * M301: Set PID parameters P I D (and optionally C, L)
   *
   *   P[float] Kp term
   *   I[float] Ki term (unscaled)
   *   D[float] Kd term (unscaled)
   *
   * With PID_ADD_EXTRUSION_RATE:
   *
   *   C[float] Kc term
   *   L[float] LPQ length
   */
  inline void gcode_M301() {

    // multi-hotend PID patch: M301 updates or prints a single hotend's PID values
    // default behaviour (omitting E parameter) is to update for hotend 0 only
    int h = code_seen('H') ? code_value() : 0; // hotend being updated

    if (h < HOTENDS) { // catch bad input value
      if (code_seen('P')) PID_PARAM(Kp, h) = code_value();
      if (code_seen('I')) PID_PARAM(Ki, h) = scalePID_i(code_value());
      if (code_seen('D')) PID_PARAM(Kd, h) = scalePID_d(code_value());
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        if (code_seen('C')) PID_PARAM(Kc, h) = code_value();
        if (code_seen('L')) lpq_len = code_value();
        NOMORE(lpq_len, LPQ_MAX_LEN);
      #endif

      updatePID();
      ECHO_SMV(DB, "H:", h);
      ECHO_MV(" p:", PID_PARAM(Kp, h));
      ECHO_MV(" i:", unscalePID_i(PID_PARAM(Ki, h)));
      ECHO_MV(" d:", unscalePID_d(PID_PARAM(Kd, h)));
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        ECHO_MV(" c:", PID_PARAM(Kc, h));
      #endif
      ECHO_E;
    }
    else {
      ECHO_LM(ER, SERIAL_INVALID_EXTRUDER);
    }
  }
#endif // PIDTEMP

#if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
  void set_extrude_min_temp(float temp) { extrude_min_temp = temp; }

  /**
   * M302: Allow cold extrudes, or set the minimum extrude S<temperature>.
   */
  inline void gcode_M302() {
    set_extrude_min_temp(code_seen('S') ? code_value() : 0);
  }
#endif // PREVENT_DANGEROUS_EXTRUDE

#if ENABLED(PIDTEMP) || ENABLED(PIDTEMPBED)
  /**
   * M303: PID relay autotune
   *       S<temperature> sets the target temperature. (default target temperature = 150C)
   *       H<hotend> (-1 for the bed)
   *       C<cycles>
   */
  inline void gcode_M303() {
    int h = code_seen('H') ? code_value_short() : 0;
    int c = code_seen('C') ? code_value_short() : 5;
    float temp = code_seen('S') ? code_value() : (h < 0 ? 70.0 : 150.0);
    if (h >= 0 && h < HOTENDS) target_extruder = h;

    KEEPALIVE_STATE(NOT_BUSY); // don't send "busy: processing" messages during autotune output

    PID_autotune(temp, h, c);
    
    KEEPALIVE_STATE(IN_HANDLER);
  }
#endif

#if ENABLED(PIDTEMPBED)
  // M304: Set bed PID parameters P I and D
  inline void gcode_M304() {
    if (code_seen('P')) bedKp = code_value();
    if (code_seen('I')) bedKi = scalePID_i(code_value());
    if (code_seen('D')) bedKd = scalePID_d(code_value());

    updatePID();
    ECHO_SMV(DB, "p:", bedKp);
    ECHO_MV(" i:", unscalePID_i(bedKi));
    ECHO_EMV(" d:", unscalePID_d(bedKd));
  }
#endif // PIDTEMPBED

#if HAS(MICROSTEPS)
  // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
  inline void gcode_M350() {
    if(code_seen('S')) for(int i = 0; i <= 4; i++) microstep_mode(i, code_value());
    for(int i = 0; i < NUM_AXIS; i++) if(code_seen(axis_codes[i])) microstep_mode(i, (uint8_t)code_value());
    if(code_seen('B')) microstep_mode(4, code_value());
    microstep_readings();
  }

  /**
   * M351: Toggle MS1 MS2 pins directly with axis codes X Y Z E B
   *       S# determines MS1 or MS2, X# sets the pin high/low.
   */
  inline void gcode_M351() {
    if (code_seen('S')) switch(code_value_short()) {
      case 1:
        for(int i = 0; i < NUM_AXIS; i++) if (code_seen(axis_codes[i])) microstep_ms(i, code_value(), -1);
        if (code_seen('B')) microstep_ms(4, code_value(), -1);
        break;
      case 2:
        for(int i = 0; i < NUM_AXIS; i++) if (code_seen(axis_codes[i])) microstep_ms(i, -1, code_value());
        if (code_seen('B')) microstep_ms(4, -1, code_value());
        break;
    }
    microstep_readings();
  }
#endif // HAS(MICROSTEPS)

#if MECH(SCARA)
  bool SCARA_move_to_cal(uint8_t delta_x, uint8_t delta_y) {
    //SoftEndsEnabled = false;              // Ignore soft endstops during calibration
    //ECHO_LM(DB, " Soft endstops disabled ");
    if (IsRunning()) {
      //gcode_get_destination(); // For X Y Z E F
      delta[X_AXIS] = delta_x;
      delta[Y_AXIS] = delta_y;
      calculate_SCARA_forward_Transform(delta);
      destination[X_AXIS] = delta[X_AXIS]/axis_scaling[X_AXIS];
      destination[Y_AXIS] = delta[Y_AXIS]/axis_scaling[Y_AXIS];
      prepare_move();
      //ok_to_send();
      return true;
    }
    return false;
  }

  /**
   * M360: SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
   */
  inline bool gcode_M360() {
    ECHO_LM(DB, "Cal: Theta 0 ");
    return SCARA_move_to_cal(0, 120);
  }

  /**
   * M361: SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M361() {
    ECHO_LM(DB, "Cal: Theta 90 ");
    return SCARA_move_to_cal(90, 130);
  }

  /**
   * M362: SCARA calibration: Move to cal-position PsiA (0 deg calibration)
   */
  inline bool gcode_M362() {
    ECHO_LM(DB, "Cal: Psi 0 ");
    return SCARA_move_to_cal(60, 180);
  }

  /**
   * M363: SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M363() {
    ECHO_LM(DB,"Cal: Psi 90 ");
    return SCARA_move_to_cal(50, 90);
  }

  /**
   * M364: SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
   */
  inline bool gcode_M364() {
    ECHO_LM(DB, "Cal: Theta-Psi 90 ");
    return SCARA_move_to_cal(45, 135);
  }

  /**
   * M365: SCARA calibration: Scaling factor, X, Y, Z axis
   */
  inline void gcode_M365() {
    for (uint8_t i = X_AXIS; i <= Z_AXIS; i++) {
      if (code_seen(axis_codes[i])) {
        axis_scaling[i] = code_value();
      }
    }
  }
#endif // SCARA

#if ENABLED(EXT_SOLENOID)
  void enable_solenoid(uint8_t num) {
    switch(num) {
      case 0:
        OUT_WRITE(SOL0_PIN, HIGH);
        break;
        #if HAS(SOLENOID_1)
          case 1:
            OUT_WRITE(SOL1_PIN, HIGH);
            break;
        #endif
        #if HAS(SOLENOID_2)
          case 2:
            OUT_WRITE(SOL2_PIN, HIGH);
            break;
        #endif
        #if HAS(SOLENOID_3)
          case 3:
            OUT_WRITE(SOL3_PIN, HIGH);
            break;
        #endif
      default:
        ECHO_LM(ER, SERIAL_INVALID_SOLENOID);
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

#if HAS(SERVO_ENDSTOPS)

  /**
   * M401: Engage Z Servo endstop if available
   */
  inline void gcode_M401() {
    #if ENABLED(AUTO_BED_LEVELING_FEATURE) && HASNT(Z_PROBE_SLED)
      raise_z_for_servo();
    #endif
    deploy_z_probe();
  }

  /**
   * M402: Retract Z Servo endstop if enabled
   */
  inline void gcode_M402() {
    #if ENABLED(AUTO_BED_LEVELING_FEATURE) && HASNT(Z_PROBE_SLED)
      raise_z_for_servo();
    #endif
    #if MECH(DELTA)
      retract_z_probe();
    #else
      stow_z_probe(false);
    #endif
  }

#endif // HAS(SERVO_ENDSTOPS)

#if ENABLED(FILAMENT_SENSOR)

  /**
   * M404: Display or set the nominal filament width (3mm, 1.75mm ) W<3.0>
   */
  inline void gcode_M404() {
    #if HAS(FILWIDTH)
      if (code_seen('D')) {
        filament_width_nominal = code_value();
      }
      else {
        ECHO_LMV(DB, "Filament dia (nominal mm):", filament_width_nominal);
      }
    #endif
  }
    
  /**
   * M405: Turn on filament sensor for control
   */
  inline void gcode_M405() {
    if (code_seen('D')) meas_delay_cm = code_value();
    NOMORE(meas_delay_cm, MAX_MEASUREMENT_DELAY);

    if (delay_index2 == -1) { //initialize the ring buffer if it has not been done since startup
      int temp_ratio = widthFil_to_size_ratio();

      for (delay_index1 = 0; delay_index1 < MAX_MEASUREMENT_DELAY + 1; ++delay_index1)
        measurement_delay[delay_index1] = temp_ratio - 100;  //subtract 100 to scale within a signed byte

      delay_index1 = delay_index2 = 0;
    }

    filament_sensor = true;

    //ECHO_SMV(DB, "Filament dia (measured mm):", filament_width_meas);
    //ECHO_EMV("Extrusion ratio(%):", extruder_multiplier[active_extruder]);
  }

  /**
   * M406: Turn off filament sensor for control
   */
  inline void gcode_M406() { filament_sensor = false; }
  
  /**
   * M407: Get measured filament diameter on serial output
   */
  inline void gcode_M407() {
    ECHO_LMV(DB, "Filament dia (measured mm):", filament_width_meas);
  }

#endif // FILAMENT_SENSOR

#if ENABLED(JSON_OUTPUT)
  /**
   * M408: JSON STATUS OUTPUT
   */
  inline void gcode_M408() {
    bool firstOccurrence;
    uint8_t type = 0;

    if (code_seen('S')) type = code_value();

    ECHO_M("{\"status\":\"");
    #if ENABLED(SDSUPPORT)
      if (!Printing && !card.sdprinting) ECHO_M("I"); // IDLING
      else if (card.sdprinting) ECHO_M("P");          // SD PRINTING
      else ECHO_M("B");                               // SOMETHING ELSE, BUT SOMETHIG
    #else
      if (!Printing) ECHO_M("I");                     // IDLING
      else ECHO_M("B");                               // SOMETHING ELSE, BUT SOMETHIG
    #endif

    ECHO_M("\",\"coords\": {");
    ECHO_M("\"axesHomed\":[");
    if (axis_was_homed & (_BV(X_AXIS)|_BV(Y_AXIS)|_BV(Z_AXIS)) == (_BV(X_AXIS)|_BV(Y_AXIS)|_BV(Z_AXIS)))
      ECHO_M("1, 1, 1");
    else
      ECHO_M("0, 0, 0");

    ECHO_MV("],\"extr\":[", current_position[E_AXIS]);
    ECHO_MV("],\"xyz\":[", current_position[X_AXIS]); // X
    ECHO_MV(",", current_position[Y_AXIS]); // Y
    ECHO_MV(",", current_position[Z_AXIS]); // Z

    ECHO_MV("]},\"currentTool\":", active_extruder);

    #if HAS(POWER_SWITCH)
      ECHO_M(",\"params\": {\"atxPower\":");
      ECHO_M(powersupply ? "1" : "0");
    #else
      ECHO_M(",\"params\": {\"NormPower\":");
    #endif

    ECHO_M(",\"fanPercent\":[");
    ECHO_V(fanSpeed);

    ECHO_MV("],\"speedFactor\":", feedrate_multiplier);

    ECHO_M(",\"extrFactors\":[");
    firstOccurrence = true;
    for (uint8_t i = 0; i < EXTRUDERS; i++) {
      if (!firstOccurrence) ECHO_M(",");
      ECHO_V(extruder_multiplier[i]); // Really *100? 100 is normal
      firstOccurrence = false;
    }
    ECHO_EM("]},");

    ECHO_M("\"temps\": {");
    #if HAS(TEMP_BED)
      ECHO_MV("\"bed\": {\"current\":", degBed(), 1);
      ECHO_MV(",\"active\":", degTargetBed(), 1);
      ECHO_M(",\"state\":");
      ECHO_M(degTargetBed() > 0 ? "2" : "1");
      ECHO_M("},");
    #endif
    ECHO_M("\"heads\": {\"current\":[");
    firstOccurrence = true;
    for (uint8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) ECHO_M(",");
      ECHO_V(degHotend(h), 1);
      firstOccurrence = false;
    }
    ECHO_M("],\"active\":[");
    firstOccurrence = true;
    for (uint8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) ECHO_M(",");
      ECHO_V(degTargetHotend(h), 1);
      firstOccurrence = false;
    }
    ECHO_M("],\"state\":[");
    firstOccurrence = true;
    for (uint8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) ECHO_M(",");
      ECHO_M(degTargetHotend(h) > EXTRUDER_AUTO_FAN_TEMPERATURE ? "2" : "1");
      firstOccurrence = false;
    }

    ECHO_MV("]}},\"time\":", HAL::timeInMilliseconds());

    switch (type) {
      case 0:
      case 1:
        break;
      case 2:
        ECHO_EM(",");
        ECHO_M("\"coldExtrudeTemp\":0,\"coldRetractTemp\":0.0,\"geometry\":\"");
        #if MECH(CARTESIAN)
          ECHO_M("cartesian");
        #elif MECH(COREXY)
          ECHO_M("corexy");
        #elif MECH(COREYX)
          ECHO_M("coreyx");
        #elif MECH(COREXZ)
          ECHO_M("corexz");
        #elif MECH(COREZX)
          ECHO_M("corezx");
        #elif MECH(DELTA)
          ECHO_M("delta");
        #endif
        ECHO_M("\",\"name\":\"");
        ECHO_T(CUSTOM_MACHINE_NAME);
        ECHO_M("\",\"tools\":[");
        firstOccurrence = true;
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          if (!firstOccurrence) ECHO_M(",");
          ECHO_MV("{\"number\":", i + 1);
          #if HOTENDS > 1
            ECHO_MV(",\"heaters\":[", i + 1);
            ECHO_M("],");
          #else
            ECHO_M(",\"heaters\":[1],");
          #endif
          #if DRIVER_EXTRUDERS > 1
            ECHO_MV("\"drives\":[", i);
            ECHO_M("]");
          #else
            ECHO_M("\"drives\":[0]");
          #endif
          ECHO_M("}");
          firstOccurrence = false;
        }
        break;
      case 3:
        ECHO_EM(",");
        ECHO_M("\"currentLayer\":");
        #if ENABLED(SDSUPPORT)
          if (card.sdprinting && card.layerHeight > 0) { // ONLY CAN TELL WHEN SD IS PRINTING
            ECHO_V((int) (current_position[Z_AXIS] / card.layerHeight));
          }
          else ECHO_V(0);
        #else
          ECHO_V(-1);
        #endif
        ECHO_M(",\"extrRaw\":[");
        firstOccurrence = true;
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          if (!firstOccurrence) ECHO_M(",");
          ECHO_V(current_position[E_AXIS] * extruder_multiplier[i]);
          firstOccurrence = false;
        }
        ECHO_M("],");
        #if ENABLED(SDSUPPORT)
          if (card.sdprinting) {
            ECHO_M("\"fractionPrinted\":");
            float fractionprinted;
            if (card.fileSize < 2000000) {
              fractionprinted = (float)card.sdpos / (float)card.fileSize;
            }
            else fractionprinted = (float)(card.sdpos >> 8) / (float)(card.fileSize >> 8);
            ECHO_V((float) floorf(fractionprinted * 1000) / 1000);
            ECHO_M(",");
          }
        #endif
        ECHO_M("\"firstLayerHeight\":");
        #if ENABLED(SDSUPPORT)
          if (card.sdprinting) ECHO_V(card.firstlayerHeight);
          else ECHO_M("0");
        #else
          ECHO_M("0");
        #endif
        break;
      case 4:
      case 5:
        ECHO_EM(",");
        ECHO_M("\"axisMins\":[");
        ECHO_V((int) X_MIN_POS);
        ECHO_M(",");
        ECHO_V((int) Y_MIN_POS);
        ECHO_M(",");
        ECHO_V((int) Z_MIN_POS);
        ECHO_M("],\"axisMaxes\":[");
        ECHO_V((int) X_MAX_POS);
        ECHO_M(",");
        ECHO_V((int) Y_MAX_POS);
        ECHO_M(",");
        ECHO_V((int) Z_MAX_POS);
        ECHO_M("],\"accelerations\":[");
        ECHO_V(max_acceleration_units_per_sq_second[X_AXIS]);
        ECHO_M(",");
        ECHO_V(max_acceleration_units_per_sq_second[Y_AXIS]);
        ECHO_M(",");
        ECHO_V(max_acceleration_units_per_sq_second[Z_AXIS]);
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          ECHO_M(",");
          ECHO_V(max_acceleration_units_per_sq_second[E_AXIS + i]);
        }
        ECHO_M("],");

        #if MB(ALLIGATOR)
          ECHO_M("\"currents\":[");
          ECHO_V(motor_current[X_AXIS]);
          ECHO_M(",");
          ECHO_V(motor_current[Y_AXIS]);
          ECHO_M(",");
          ECHO_V(motor_current[Z_AXIS]);
          for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
            ECHO_M(",");
            ECHO_V(motor_current[E_AXIS + i]);
          }
          ECHO_EM("],");
        #endif

        ECHO_M("\"firmwareElectronics\":\"");
        #if MB(RAMPS_13_HFB) || MB(RAMPS_13_HHB) || MB(RAMPS_13_HFF) || MB(RAMPS_13_HHF) || MB(RAMPS_13_HHH)
          ECHO_M("RAMPS");
        #elif MB(ALLIGATOR)
          ECHO_M("ALLIGATOR");
        #elif MB(RADDS) || MB(RAMPS_FD_V1) || MB(RAMPS_FD_V2) || MB(SMART_RAMPS) || MB(RAMPS4DUE)
          ECHO_M("Arduino due");
        #elif MB(ULTRATRONICS)
          ECHO_M("ULTRATRONICS");
        #else
          ECHO_M("AVR");
        #endif
        ECHO_M("\",\"firmwareName\":\"");
        ECHO_M(FIRMWARE_NAME);
        ECHO_M(",\"firmwareVersion\":\"");
        ECHO_M(SHORT_BUILD_VERSION);
        ECHO_M("\",\"firmwareDate\":\"");
        ECHO_M(STRING_DISTRIBUTION_DATE);

        ECHO_M("\",\"minFeedrates\":[0,0,0");
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          ECHO_M(",0");
        }
        ECHO_M("],\"maxFeedrates\":[");
        ECHO_V(max_feedrate[X_AXIS]);
        ECHO_M(",");
        ECHO_V(max_feedrate[Y_AXIS]);
        ECHO_M(",");
        ECHO_V(max_feedrate[Z_AXIS]);
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          ECHO_M(",");
          ECHO_V(max_feedrate[E_AXIS + i]);
        }
        ECHO_M("]");
        break;
    }
    ECHO_EM("}");
  }
#endif // JSON_OUTPUT

/**
 * M410: Quickstop - Abort all planned moves
 *
 * This will stop the carriages mid-move, so most likely they
 * will be out of sync with the stepper position after this.
 */
inline void gcode_M410() { quickStop(); }

/**
 * M428: Set home_offset based on the distance between the
 *       current_position and the nearest "reference point."
 *       If an axis is past center its endstop position
 *       is the reference-point. Otherwise it uses 0. This allows
 *       the Z offset to be set near the bed when using a max endstop.
 *
 *       M428 can't be used more than 2cm away from 0 or an endstop.
 *
 *       Use M206 to set these values directly.
 */
inline void gcode_M428() {
  bool err = false;
  float new_offs[3], new_pos[3];
  memcpy(new_pos, current_position, sizeof(new_pos));
  memcpy(new_offs, home_offset, sizeof(new_offs));
  for (uint8_t i = X_AXIS; i <= Z_AXIS; i++) {
    if (TEST(axis_known_position, i)) {
      #if MECH(DELTA)
        float base = (new_pos[i] > (min_pos[i] + max_pos[i]) / 2) ? base_home_pos[i] : 0,
      #else
        float base = (new_pos[i] > (min_pos[i] + max_pos[i]) / 2) ? base_home_pos(i) : 0,
      #endif
              diff = new_pos[i] - base;
      if (diff > -20 && diff < 20) {
        new_offs[i] -= diff;
        new_pos[i] = base;
      }
      else {
        ECHO_LM(ER, SERIAL_ERR_M428_TOO_FAR);
        LCD_ALERTMESSAGEPGM("Err: Too far!");
        #if HAS(BUZZER)
          enqueue_and_echo_commands_P(PSTR("M300 S40 P200"));
        #endif
        err = true;
        break;
      }
    }
  }

  if (!err) {
    memcpy(current_position, new_pos, sizeof(new_pos));
    memcpy(home_offset, new_offs, sizeof(new_offs));
    #if MECH(DELTA) || MECH(SCARA)
      sync_plan_position_delta();
    #else
      sync_plan_position();
    #endif
    ECHO_LM(DB, "Offset applied.");
    LCD_MESSAGEPGM("Offset applied.");
    #if HAS(BUZZER)
      enqueue_and_echo_commands_P(PSTR("M300 S659 P200\nM300 S698 P200"));
    #endif
  }
}

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

#if ENABLED(RFID_MODULE)
  /**
   * M522: Read or Write on card. M522 T<extruders> R<read> or W<write> L<list>
   */
  inline void gcode_M522() {
    if (setTargetedExtruder(522)) return;
    if (!RFID_ON) return;

    if (code_seen('R')) {
      ECHO_LM(DB, "Put RFID on tag!");
      Spool_must_read[target_extruder] = true;
    }
    if (code_seen('W')) {
      if (Spool_ID[target_extruder] != 0) {
        ECHO_LM(DB, "Put RFID on tag!");
        Spool_must_write[target_extruder] = true;
      }
      else
        ECHO_LM(ER, "You have not read this Spool!");
    }

    if (code_seen('L')) RFID522.printInfo(target_extruder);
  }
#endif // RFID_MODULE

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)

  /**
   * M540: Set whether SD card print should abort on endstop hit (M540 S<0|1>)
   */
  inline void gcode_M540() {
    if (code_seen('S')) abort_on_endstop_hit = (code_value() > 0);
  }

#endif // ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

#if HEATER_USES_AD595
  /**
   * M595 - set Hotend AD595 offset & Gain H<hotend_number> O<offset> S<gain>
   */
  inline void gcode_M595() {
    if (setTargetedHotend(595)) return;

    if (code_seen('O')) ad595_offset[target_extruder] = code_value();
    if (code_seen('S')) ad595_gain[target_extruder] = code_value();

    for (uint8_t h = 0; h < HOTENDS; h++) {
      // if gain == 0 you get MINTEMP!
      if (ad595_gain[h] == 0) ad595_gain[h]= 1;
    }

    ECHO_LM(DB, MSG_AD595);
    for (uint8_t h = 0; h < HOTENDS; h++) {
      ECHO_SMV(DB, "T", h);
      ECHO_MV(" Offset: ", ad595_offset[h]);
      ECHO_EMV(", Gain: ", ad595_gain[h]);
    }
  }
#endif // HEATER_USES_AD595

#if ENABLED(FILAMENTCHANGEENABLE)
  /**
   * M600: Pause for filament change
   *
   *  E[distance] - Retract the filament this far (negative value)
   *  Z[distance] - Move the Z axis by this distance
   *  X[position] - Move to this X position, with Y
   *  Y[position] - Move to this Y position, with X
   *  L[distance] - Retract distance for removal (manual reload)
   *
   *  Default values are used for omitted arguments.
   *
   */
  inline void gcode_M600() {

    if (degHotend(active_extruder) < extrude_min_temp) {
      ECHO_LM(ER, MSG_TOO_COLD_FOR_FILAMENTCHANGE);
      return;
    }

    float lastpos[NUM_AXIS];

    filament_changing = true;
    for (int i = 0; i < NUM_AXIS; i++)
      lastpos[i] = destination[i] = current_position[i];

    #if MECH(DELTA)
			float fr60 = feedrate / 60;
      #define RUNPLAN calculate_delta(destination); \
                      plan_buffer_line(delta[TOWER_1], delta[TOWER_2], delta[TOWER_3], destination[E_AXIS], fr60, active_extruder, active_driver);
    #else
      #define RUNPLAN line_to_destination();
    #endif

    //retract by E
    if (code_seen('E')) destination[E_AXIS] += code_value();
    #if ENABLED(FILAMENTCHANGE_FIRSTRETRACT)
      else destination[E_AXIS] += FILAMENTCHANGE_FIRSTRETRACT;
    #endif

    RUNPLAN

    //lift Z
    if (code_seen('Z')) destination[Z_AXIS] += code_value();
    #if ENABLED(FILAMENTCHANGE_ZADD)
      else destination[Z_AXIS] += FILAMENTCHANGE_ZADD;
    #endif

    RUNPLAN

    //move xy
    if (code_seen('X')) destination[X_AXIS] = code_value();
    #if ENABLED(FILAMENTCHANGE_XPOS)
      else destination[X_AXIS] = FILAMENTCHANGE_XPOS;
    #endif

    if (code_seen('Y')) destination[Y_AXIS] = code_value();
    #if ENABLED(FILAMENTCHANGE_YPOS)
      else destination[Y_AXIS] = FILAMENTCHANGE_YPOS;
    #endif

    RUNPLAN

    if (code_seen('L')) destination[E_AXIS] += code_value();
    #if ENABLED(FILAMENTCHANGE_FINALRETRACT)
      else destination[E_AXIS] += FILAMENTCHANGE_FINALRETRACT;
    #endif

    RUNPLAN

    //finish moves
    st_synchronize();
    //disable extruder steppers so filament can be removed
    disable_e();
    delay_ms(100);
    boolean beep = true;
    boolean sleep = false;
    uint8_t cnt = 0;
    
    int old_target_temperature[HOTENDS] = { 0 };
    for (uint8_t e = 0; e < HOTENDS; e++) {
      old_target_temperature[e] = target_temperature[e];
    }
    int old_target_temperature_bed = target_temperature_bed;
    millis_t last_set = millis();
    
    PRESSBUTTON:
    KEEPALIVE_STATE(PAUSED_FOR_USER);
    LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);
    while (!lcd_clicked()) {
      idle();
      if ((millis() - last_set > 60000) && cnt <= FILAMENTCHANGE_PRINTEROFF) beep = true;
      if (cnt >= FILAMENTCHANGE_PRINTEROFF && !sleep) {
        disable_all_heaters();
        disable_all_steppers();
        sleep = true;
        lcd_reset_alert_level();
        LCD_ALERTMESSAGEPGM("Zzzz Zzzz Zzzz");
      }
      if (beep) {
        #if HAS(BUZZER)
          for(uint8_t i = 0; i < 3; i++) buzz(100, 1000);
        #endif
        last_set = millis();
        beep = false;
        ++cnt;
      }
    } // while(!lcd_clicked)
    KEEPALIVE_STATE(IN_HANDLER);
    lcd_quick_feedback();     // click sound feedback
    lcd_reset_alert_level();  //reset LCD alert message

    if (sleep) {
      enable_all_steppers(); // Enable all stepper
      for(uint8_t e = 0; e < HOTENDS; e++) {
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
    if (code_seen('L')) destination[E_AXIS] -= code_value();
    #if ENABLED(FILAMENTCHANGE_FINALRETRACT)
      else destination[E_AXIS] -= FILAMENTCHANGE_FINALRETRACT;
    #endif

    current_position[E_AXIS] = destination[E_AXIS]; //the long retract of L is compensated by manual filament feeding
    plan_set_e_position(current_position[E_AXIS]);

    RUNPLAN // should do nothing

    lcd_reset_alert_level();

    #if MECH(DELTA)
      // Move XYZ to starting position, then E
      calculate_delta(lastpos);
      plan_buffer_line(delta[TOWER_1], delta[TOWER_2], delta[TOWER_3], destination[E_AXIS], fr60, active_extruder, active_driver);
      plan_buffer_line(delta[TOWER_1], delta[TOWER_2], delta[TOWER_3], lastpos[E_AXIS], fr60, active_extruder, active_driver);
    #else
      // Move XY to starting position, then Z, then E
      destination[X_AXIS] = lastpos[X_AXIS];
      destination[Y_AXIS] = lastpos[Y_AXIS];
      line_to_destination();
      destination[Z_AXIS] = lastpos[Z_AXIS];
      line_to_destination();
      destination[E_AXIS] = lastpos[E_AXIS];
      line_to_destination();
    #endif

    #if HAS(FILRUNOUT)
      filrunoutEnqueued = false;
    #endif

    filament_changing = false;
  }
#endif //FILAMENTCHANGEENABLE

#if ENABLED(DUAL_X_CARRIAGE)
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
        if (code_seen('X')) duplicate_hotend_x_offset = max(code_value(), X2_MIN_POS - x_home_pos(0));
        if (code_seen('R')) duplicate_extruder_temp_offset = code_value();
        ECHO_SM(DB, SERIAL_HOTEND_OFFSET);
        ECHO_MV(" ", hotend_offset[X_AXIS][0]);
        ECHO_MV(",", hotend_offset[Y_AXIS][0]);
        ECHO_MV(" ", duplicate_hotend_x_offset);
        ECHO_EMV(",", hotend_offset[Y_AXIS][1]);
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

#if ENABLED(AUTO_BED_LEVELING_FEATURE)
  //M666: Set Z probe offset
  inline void gcode_M666() {
    if (code_seen('P')) {
      zprobe_zoffset = code_value();
      ECHO_LM(DB, MSG_ZPROBE_ZOFFSET " " OK);
    }
    if (code_seen('L')) {
      ECHO_LMV(DB, "P (Z-Probe Offset):", zprobe_zoffset);
    }
  }
#endif

#if MECH(DELTA)
  //M666: Set delta endstop and geometry adjustment
  inline void gcode_M666() {
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
    if (code_seen('U')) {
      diagrod_adj[0] = code_value();
      set_delta_constants();
    }
    if (code_seen('V')) {
      diagrod_adj[1] = code_value();
      set_delta_constants();
    }
    if (code_seen('W')) {
      diagrod_adj[2] = code_value();
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
      boolean axis_done = false;
      float p_val = code_value();
      for (uint8_t i = 0; i < 3; i++) {
        if (code_seen(axis_codes[i])) {
          z_probe_offset[i] = code_value();
          axis_done = true;
        }
      }
      if (axis_done == false) z_probe_offset[Z_AXIS] = p_val;
    }
    else {
      for(uint8_t i = 0; i < 3; i++) {
        if (code_seen(axis_codes[i])) endstop_adj[i] = code_value();
      }
    }
    if (code_seen('L')) {
      ECHO_LM(CFG, "Current Delta geometry values:");
      ECHO_LMV(CFG, "X (Endstop Adj): ", endstop_adj[0], 3);
      ECHO_LMV(CFG, "Y (Endstop Adj): ", endstop_adj[1], 3);
      ECHO_LMV(CFG, "Z (Endstop Adj): ", endstop_adj[2], 3);
      ECHO_SMV(CFG, "P (Z-Probe Offset): X", z_probe_offset[0]);
      ECHO_MV(" Y", z_probe_offset[1]);
      ECHO_EMV(" Z", z_probe_offset[2]);
      ECHO_LMV(CFG, "A (Tower A Position Correction): ", tower_adj[0], 3);
      ECHO_LMV(CFG, "B (Tower B Position Correction): ", tower_adj[1], 3);
      ECHO_LMV(CFG, "C (Tower C Position Correction): ", tower_adj[2], 3);
      ECHO_LMV(CFG, "I (Tower A Radius Correction): ", tower_adj[3], 3);
      ECHO_LMV(CFG, "J (Tower B Radius Correction): ", tower_adj[4], 3);
      ECHO_LMV(CFG, "K (Tower C Radius Correction): ", tower_adj[5], 3);
      ECHO_LMV(CFG, "U (Tower A Diagonal Rod Correction): ", diagrod_adj[0], 3);
      ECHO_LMV(CFG, "V (Tower B Diagonal Rod Correction): ", diagrod_adj[1], 3);
      ECHO_LMV(CFG, "W (Tower C Diagonal Rod Correction): ", diagrod_adj[2], 3);
      ECHO_LMV(CFG, "R (Delta Radius): ", delta_radius);
      ECHO_LMV(CFG, "D (Diagonal Rod Length): ", delta_diagonal_rod);
      ECHO_LMV(CFG, "H (Z-Height): ", max_pos[Z_AXIS]);
    }
  }
#endif

#if MB(ALLIGATOR)
  /**
   * M906: Set motor currents
   */
  inline void gcode_M906() {
    if (setTargetedExtruder(906)) return;
    for (uint8_t i = 0; i < 3 + DRIVER_EXTRUDERS; i++) {
      if (code_seen(axis_codes[i])) {
        if (i == E_AXIS)
          motor_current[i + target_extruder] = code_value();
        else
          motor_current[i] = code_value();
      }
    }
  }
#endif // ALLIGATOR

/**
 * M907: Set digital trimpot motor current using axis codes X, Y, Z, E, B, S
 */
inline void gcode_M907() {
  #if HAS(DIGIPOTSS)
    for (uint8_t i = 0; i < NUM_AXIS; i++)
      if (code_seen(axis_codes[i])) digipot_current(i, code_value());
    if (code_seen('B')) digipot_current(4, code_value());
    if (code_seen('S')) for (uint8_t i = 0; i <= 4; i++) digipot_current(i, code_value());
  #endif
  #if ENABLED(MOTOR_CURRENT_PWM_XY_PIN)
    if (code_seen('X')) digipot_current(0, code_value());
  #endif
  #if ENABLED(MOTOR_CURRENT_PWM_Z_PIN)
    if (code_seen('Z')) digipot_current(1, code_value());
  #endif
  #if ENABLED(MOTOR_CURRENT_PWM_E_PIN)
    if (code_seen('E')) digipot_current(2, code_value());
  #endif
  #if ENABLED(DIGIPOT_I2C)
    // this one uses actual amps in floating point
    for (uint8_t i = 0; i < NUM_AXIS; i++) if(code_seen(axis_codes[i])) digipot_i2c_set_current(i, code_value());
    // for each additional extruder (named B,C,D,E..., channels 4,5,6,7...)
    for (uint8_t i = NUM_AXIS; i < DIGIPOT_I2C_NUM_CHANNELS; i++) if(code_seen('B' + i - (NUM_AXIS))) digipot_i2c_set_current(i, code_value());
  #endif
}

#if HAS(DIGIPOTSS)
  /**
   * M908: Control digital trimpot directly (M908 P<pin> S<current>)
   */
  inline void gcode_M908() {
    digitalPotWrite(
      code_seen('P') ? code_value() : 0,
      code_seen('S') ? code_value() : 0
    );
  }
#endif // HAS(DIGIPOTSS)

#if ENABLED(NPR2)
  /**
   * M997: Cxx Move Carter xx gradi
   */
  inline void gcode_M997() {
    long csteps;
    if (code_seen('C')) {
      csteps = code_value() * color_step_moltiplicator;
      ECHO_LMV(DB, "csteps: ", csteps);
      if (csteps < 0) colorstep(-csteps, false);
      if (csteps > 0) colorstep(csteps, true);
    }
  }
#endif

/**
 * M999: Restart after being stopped
 */
inline void gcode_M999() {
  Running = true;
  Printing = false;
  lcd_reset_alert_level();
  FlushSerialRequestResend();
}

/**
 * T0-T3: Switch tool, usually switching extruders
 *
 *   F[mm/min] Set the movement feedrate
 */
inline void gcode_T(uint8_t tmp_extruder) {
  bool good_extruder = false;

  #if ENABLED(COLOR_MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1

    // T0-T15: Switch virtual tool by changing the mix
    if (tmp_extruder < MIXING_VIRTUAL_TOOLS) {
      good_extruder = true;
      for (uint8_t j = 0; j < DRIVER_EXTRUDERS; j++) {
        mixing_factor[j] = mixing_virtual_tool_mix[tmp_extruder][j];
      }
      ECHO_LMV(DB, SERIAL_ACTIVE_COLOR, (int)tmp_extruder);
    }

  #else // !COLOR_MIXING_EXTRUDER && MIXING_VIRTUAL_TOOLS

    if (tmp_extruder < EXTRUDERS) {
      good_extruder = true;
      target_extruder = tmp_extruder;

      #if ENABLED(DONDOLO)
        bool make_move = true;
      #else
        bool make_move = false;
      #endif

      if (code_seen('F')) {

        #if EXTRUDERS > 1
          make_move = true;
        #endif

        float next_feedrate = code_value();
        if (next_feedrate > 0.0) feedrate = next_feedrate;
      }
      #if EXTRUDERS > 1
        #if ENABLED(NPR2)
          if(target_extruder != old_color)
        #else
          if(target_extruder != active_extruder)
        #endif // NPR2
        {
          // Save current position to return to after applying extruder offset
          set_destination_to_current();
          #if ENABLED(DUAL_X_CARRIAGE)
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

            active_extruder = target_extruder;

            // This function resets the max/min values - the current position may be overwritten below.
            set_axis_is_at_home(X_AXIS);

            if (dual_x_carriage_mode == DXC_FULL_CONTROL_MODE) {
              current_position[X_AXIS] = inactive_extruder_x_pos;
              inactive_extruder_x_pos = destination[X_AXIS];
            }
            else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE) {
              active_extruder_parked = (active_extruder == 0); // this triggers the second extruder to move into the duplication position
              if (active_extruder == 0 || active_extruder_parked)
                current_position[X_AXIS] = inactive_extruder_x_pos;
              else
                current_position[X_AXIS] = destination[X_AXIS] + duplicate_hotend_x_offset;
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

            #if ENABLED(MKR4)
              #if (EXTRUDERS == 4) && HAS(E0E2) && HAS(E1E3) && (DRIVER_EXTRUDERS == 2)
                st_synchronize(); // Finish all movement
                disable_e();
                switch(target_extruder)
                {
                case 0:
                  WRITE_RELE(E0E2_CHOICE_PIN, LOW);
                  WRITE_RELE(E1E3_CHOICE_PIN, LOW);
                  active_driver = 0;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e0();
                  break;
                case 1:
                  WRITE_RELE(E0E2_CHOICE_PIN, LOW);
                  WRITE_RELE(E1E3_CHOICE_PIN, LOW);
                  active_driver = 1;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e1();
                  break;
                case 2:
                  WRITE_RELE(E0E2_CHOICE_PIN, HIGH);
                  WRITE_RELE(E1E3_CHOICE_PIN, LOW);
                  active_driver = 0;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e2();
                  break;
                case 3:
                  WRITE_RELE(E0E2_CHOICE_PIN, LOW);
                  WRITE_RELE(E1E3_CHOICE_PIN, HIGH);
                  active_driver = 1;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e3();
                  break;
                }
              #elif (EXTRUDERS == 4) && HAS(E0E1) && HAS(E0E2) && HAS(E0E3) && (DRIVER_EXTRUDERS == 1)
                st_synchronize(); // Finish all movement
                disable_e();
                switch(target_extruder)
                {
                case 0:
                  WRITE_RELE(E0E1_CHOICE_PIN, LOW);
                  WRITE_RELE(E0E2_CHOICE_PIN, LOW);
                  WRITE_RELE(E0E3_CHOICE_PIN, LOW);
                  active_driver = 0;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e0();
                  break;
                case 1:
                  WRITE_RELE(E0E1_CHOICE_PIN, HIGH);
                  WRITE_RELE(E0E2_CHOICE_PIN, LOW);
                  WRITE_RELE(E0E3_CHOICE_PIN, LOW);
                  active_driver = 0;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e0();
                  break;
                case 2:
                  WRITE_RELE(E0E1_CHOICE_PIN, HIGH);
                  WRITE_RELE(E0E2_CHOICE_PIN, HIGH);
                  WRITE_RELE(E0E3_CHOICE_PIN, LOW);
                  active_driver = 0;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e0();
                  break;
                case 3:
                  WRITE_RELE(E0E1_CHOICE_PIN, HIGH);
                  WRITE_RELE(E0E2_CHOICE_PIN, HIGH);
                  WRITE_RELE(E0E3_CHOICE_PIN, HIGH);
                  active_driver = 0;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e0();
                  break;
                }
              #elif (EXTRUDERS == 3) && HAS(E0E2) && (DRIVER_EXTRUDERS == 2)
                st_synchronize(); // Finish all movement
                disable_e();
                switch(target_extruder)
                {
                case 0:
                  WRITE_RELE(E0E2_CHOICE_PIN, LOW);
                  active_driver = 0;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e0();
                  break;
                case 1:
                  WRITE_RELE(E0E2_CHOICE_PIN, LOW);
                  active_driver = 1;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e1();
                  break;
                case 2:
                  WRITE_RELE(E0E2_CHOICE_PIN, HIGH);
                  active_driver = 0;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e0();
                  break;
                }
              #elif (EXTRUDERS == 3) && HAS(E0E1) && HAS(E0E2) && (DRIVER_EXTRUDERS == 1)
                st_synchronize(); // Finish all movement
                disable_e();
                switch(target_extruder)
                {
                case 0:
                  WRITE_RELE(E0E1_CHOICE_PIN, LOW);
                  WRITE_RELE(E0E2_CHOICE_PIN, LOW);
                  active_driver = 0;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e0();
                  break;
                case 1:
                  WRITE_RELE(E0E1_CHOICE_PIN, HIGH);
                  WRITE_RELE(E0E2_CHOICE_PIN, LOW);
                  active_driver = 0;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e0();
                  break;
                case 2:
                  WRITE_RELE(E0E1_CHOICE_PIN, HIGH);
                  WRITE_RELE(E0E2_CHOICE_PIN, HIGH);
                  active_driver = 0;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e0();
                  break;
                }
              #elif (EXTRUDERS == 2) && HAS(E0E1) && (DRIVER_EXTRUDERS == 1)
                st_synchronize(); // Finish all movement
                disable_e();
                switch(target_extruder)
                {
                case 0:
                  WRITE_RELE(E0E1_CHOICE_PIN, LOW);
                  active_driver = 0;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e0();
                  break;
                case 1:
                  WRITE_RELE(E0E1_CHOICE_PIN, HIGH);
                  active_driver = 0;
                  delay_ms(500); // 500 microseconds delay for relay
                  enable_e0();
                  break;
                }
              #endif // E0E1_CHOICE_PIN E0E2_CHOICE_PIN E1E3_CHOICE_PIN
              previous_extruder = active_extruder;
              active_extruder = target_extruder;
              ECHO_LMV(DB, SERIAL_ACTIVE_DRIVER, (int)active_driver);
              ECHO_LMV(DB, SERIAL_ACTIVE_EXTRUDER, (int)active_extruder);
            #elif ENABLED(NPR2)
              long csteps;
              st_synchronize(); // Finish all movement
              if (old_color == 99) {
                csteps = (color_position[target_extruder]) * color_step_moltiplicator;
              }
              else {
                csteps = (color_position[target_extruder] - color_position[old_color]) * color_step_moltiplicator;
              }
              if (csteps < 0) colorstep(-csteps, false);
              if (csteps > 0) colorstep(csteps, true);
              previous_extruder = active_extruder;
              old_color = active_extruder = target_extruder;
              active_driver = 0;
              ECHO_LMV(DB, SERIAL_ACTIVE_COLOR, (int)active_extruder);
            #elif ENABLED(DONDOLO)
              st_synchronize();
              servo[DONDOLO_SERVO_INDEX].attach(0);
              if (target_extruder == 0) {
                servo[DONDOLO_SERVO_INDEX].write(DONDOLO_SERVOPOS_E0);
              }
              else if (target_extruder == 1) {
                servo[DONDOLO_SERVO_INDEX].write(DONDOLO_SERVOPOS_E1);
              }
              delay_ms(DONDOLO_SERVO_DELAY);
              servo[DONDOLO_SERVO_INDEX].detach();
              previous_extruder = active_extruder;
              active_extruder = target_extruder;
              active_driver = 0;
              set_stepper_direction(true);
              ECHO_LMV(DB, SERIAL_ACTIVE_DRIVER, (int)active_driver);
              ECHO_LMV(DB, SERIAL_ACTIVE_EXTRUDER, (int)active_extruder);
            #else
              previous_extruder = active_extruder;
              active_driver = active_extruder = target_extruder;
              ECHO_LMV(DB, SERIAL_ACTIVE_EXTRUDER, (int)active_extruder);
            #endif // end MKR4 || NPR2 || DONDOLO
          #endif // end no DUAL_X_CARRIAGE

          #if MECH(DELTA) || MECH(SCARA)
            sync_plan_position_delta();
          #else // NO DELTA
            sync_plan_position();
          #endif // !DELTA
          // Move to the old position if 'F' was in the parameters
          if (make_move && IsRunning()) prepare_move();
        }

        #if ENABLED(EXT_SOLENOID)
          st_synchronize();
          disable_all_solenoids();
          enable_solenoid_on_active_extruder();
        #endif // EXT_SOLENOID

      #endif // EXTRUDERS > 1
    }
  #endif // !COLOR_MIXING_EXTRUDER

  if (!good_extruder) {
    ECHO_SMV(DB, "T", (int)tmp_extruder);
    ECHO_EM(" " SERIAL_INVALID_EXTRUDER);
  }
}

/**
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void process_next_command() {
  current_command = command_queue[cmd_queue_index_r];

  if (DEBUGGING(ECHO)) {
    ECHO_LT(DB, current_command);
  }

  // Sanitize the current command:
  //  - Skip leading spaces
  //  - Bypass N[-0-9][0-9]*[ ]*
  //  - Overwrite * with nul to mark the end
  while (*current_command == ' ') ++current_command;
  if (*current_command == 'N' && NUMERIC_SIGNED(current_command[1])) {
    current_command += 2; // skip N[-0-9]
    while (NUMERIC(*current_command)) ++current_command; // skip [0-9]*
    while (*current_command == ' ') ++current_command; // skip [ ]*
  }
  char* starpos = strchr(current_command, '*');  // * should always be the last parameter
  if (starpos) while (*starpos == ' ' || *starpos == '*') *starpos-- = '\0'; // nullify '*' and ' '

  char *cmd_ptr = current_command;

  // Get the command code, which must be G, M, or T
  char command_code = *cmd_ptr++;

  // Skip spaces to get the numeric part
  while (*cmd_ptr == ' ') cmd_ptr++;

  uint16_t codenum = 0; // define ahead of goto

  // Bail early if there's no code
  bool code_is_good = NUMERIC(*cmd_ptr);
  if (!code_is_good) goto ExitUnknownCommand;

  // Get and skip the code number
  do {
    codenum = (codenum * 10) + (*cmd_ptr - '0');
    cmd_ptr++;
  } while (NUMERIC(*cmd_ptr));

  // Skip all spaces to get to the first argument, or nul
  while (*cmd_ptr == ' ') cmd_ptr++;

  // The command's arguments (if any) start here, for sure!
  current_command_args = cmd_ptr;

  KEEPALIVE_STATE(IN_HANDLER);

  // Handle a known G, M, or T
  switch(command_code) {
    case 'G': switch (codenum) {

      // G0 -> G1
      case 0:
      case 1:
        gcode_G0_G1(); break;

      // G2, G3
      #if !MECH(SCARA)
        case 2: // G2  - CW ARC
        case 3: // G3  - CCW ARC
          gcode_G2_G3(codenum == 2); break;
      #endif

      // G4 Dwell
      case 4:
        gcode_G4(); break;

      #if ENABLED(FWRETRACT)
        case 10: // G10: retract
        case 11: // G11: retract_recover
          gcode_G10_G11(codenum == 10); break;
      #endif //FWRETRACT

      case 28: //G28: Home all axes, one at a time
        gcode_G28(); gcode_M114(); break;

      #if ENABLED(AUTO_BED_LEVELING_FEATURE)
        case 29: // G29 Detailed Z-Probe, probes the bed at 3 or more points.
          gcode_G29(); gcode_M114(); break;
        #if HASNT(Z_PROBE_SLED)
          case 30: // G30 Single Z Probe
            gcode_G30(); break;
        #else // Z_PROBE_SLED
          case 31: // G31: dock the sled
          case 32: // G32: undock the sled
            dock_sled(codenum == 31); break;
        #endif // Z_PROBE_SLED
      #endif // AUTO_BED_LEVELING_FEATURE

      #if MECH(DELTA) && ENABLED(Z_PROBE_ENDSTOP)
        case 29: // G29 Detailed Z-Probe, probes the bed at more points.
          gcode_G29(); gcode_M114(); break;
        case 30:  // G30 Delta AutoCalibration
          gcode_G30(); break;
      #endif // DELTA && Z_PROBE_ENDSTOP

      case 60: // G60 Saved Coordinates
        gcode_G60(); break;
      case 61: // G61 Restore Coordinates
        gcode_G61(); break;
      case 90: // G90
        relative_mode = false; break;
      case 91: // G91
        relative_mode = true; break;
      case 92: // G92
        gcode_G92(); break;
    }
    break;

    case 'M': switch (codenum) {

      #if ENABLED(ULTIPANEL)
        case 0: // M0 - Unconditional stop - Wait for user button press on LCD
        case 1: // M1 - Conditional stop - Wait for user button press on LCD
          gcode_M0_M1(); break;
      #endif //ULTIPANEL

      #if ENABLED(LASERBEAM)
        case 3: // M03 S - Setting laser beam
          gcode_M3(); break;
        case 4: // M04 - Turn on laser beam
          gcode_M4(); break;
        case 5: // M05 - Turn off laser beam
          gcode_M5(); break;
      #endif //LASERBEAM

      case 11: // M11 - Start/Stop printing serial mode
        gcode_M11(); break;
      case 17: // M17 - Enable/Power all stepper motors
        gcode_M17(); break;

      #if ENABLED(SDSUPPORT)
        case 20: // M20 - list SD card
          gcode_M20(); break;
        case 21: // M21 - init SD card
          gcode_M21(); break;
        case 22: // M22 - release SD card
          gcode_M22(); break;
        case 23: // M23 - Select file
          gcode_M23(); break;
        case 24: // M24 - Start SD print
          gcode_M24(); break;
        case 25: // M25 - Pause SD print
          gcode_M25(); break;
        case 26: // M26 - Set SD index
          gcode_M26(); break;
        case 27: // M27 - Get SD status
          gcode_M27(); break;
        case 28: // M28 - Start SD write
          gcode_M28(); break;
        case 29: // M29 - Stop SD write
          gcode_M29(); break;
        case 30: // M30 <filename> Delete File
          gcode_M30(); break;
        case 31: // M31 take time since the start of the SD print or an M109 command
          gcode_M31(); break;
        case 32: // M32 - Make directory
          gcode_M32(); break;
        #if ENABLED(NEXTION)
          case 35: // M35 - Upload Firmware to Nextion from SD
            gcode_M35(); break;
        #endif
      #endif //SDSUPPORT

      case 42: // M42 -Change pin status via gcode
        gcode_M42(); break;

      #if ENABLED(AUTO_BED_LEVELING_FEATURE) && ENABLED(Z_PROBE_REPEATABILITY_TEST)
        case 48: // M48 Z-Probe repeatability
          gcode_M48(); break;
      #endif
      
      #if HAS(POWER_CONSUMPTION_SENSOR)
        case 70: // M70 - Power consumption sensor calibration
          gcode_M70(); break;
      #endif
      
      case 75: // Start print timer
        gcode_M75(); break;

      case 76: // Pause print timer
        gcode_M76(); break;

      case 77: // Stop print timer
        gcode_M77(); break;

      #if HAS(POWER_SWITCH)
        case 80: // M80 - Turn on Power Supply
          gcode_M80(); break;
      #endif

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
      case 92: // M92 Set the steps-per-unit for one or more axes
        gcode_M92(); break;

      #if ENABLED(ZWOBBLE)
        case 96: // M96 Print ZWobble value
          gcode_M96(); break;
        case 97: // M97 Set ZWobble parameter
          gcode_M97(); break;
      #endif

      #if ENABLED(HYSTERESIS)
        case 98: // M98 Print Hysteresis value
          gcode_M98(); break;
        case 99: // M99 Set Hysteresis parameter
          gcode_M99(); break;
      #endif

      #if ENABLED(M100_FREE_MEMORY_WATCHER)
        case 100:
          gcode_M100(); break;
      #endif

      case 104: // M104
        gcode_M104(); break;

      case 105: // M105 Read current temperature
        gcode_M105();
        KEEPALIVE_STATE(NOT_BUSY);
        return; // "ok" already printed

      #if HAS(FAN)
        case 106: //M106 Fan On
          gcode_M106(); break;
        case 107: //M107 Fan Off
          gcode_M107(); break;
      #endif // HAS(FAN)

      case 109: // M109 Wait for temperature
        gcode_M109(); break;

      case 110: break; // M110: Set line number - don't show "unknown command"

      case 111: // M111 Set debug level
        gcode_M111(); break;

      case 112: //  M112 Emergency Stop
        gcode_M112(); break;

      case 114: // M114 Report current position
        gcode_M114(); break;

      #if ENABLED(HOST_KEEPALIVE_FEATURE)
        case 113: // M113: Set Host Keepalive interval
          gcode_M113(); break;
      #endif

      case 115: // M115 Report capabilities
        gcode_M115(); break;

      #if ENABLED(ULTIPANEL) || ENABLED(NEXTION)
        case 117: // M117 display message
          gcode_M117(); break;
      #endif

      case 119: // M119 Report endstop states
        gcode_M119(); break;
      case 120: // M120 Enable endstops
        gcode_M120(); break;
      case 121: // M121 Disable endstops
        gcode_M121(); break;
      case 122: // M122 Disable or enable software endstops
        gcode_M122(); break;

      #if ENABLED(BARICUDA)
        // PWM for HEATER_1_PIN
        #if HAS(HEATER_1)
          case 126: // M126 valve open
            gcode_M126(); break;
          case 127: // M127 valve closed
            gcode_M127(); break;
        #endif // HAS(HEATER_1)

        // PWM for HEATER_2_PIN
        #if HAS(HEATER_2)
          case 128: // M128 valve open
            gcode_M128(); break;
          case 129: // M129 valve closed
            gcode_M129(); break;
        #endif // HAS(HEATER_2)
      #endif // BARICUDA

      case 140: // M140 Set bed temp
        gcode_M140(); break;

      #if ENABLED(BLINKM)
        case 150: // M150
          gcode_M150(); break;
      #endif //BLINKM

      #if ENABLED(COLOR_MIXING_EXTRUDER)
        case 163: // M163 S<int> P<float> set weight for a mixing extruder
          gcode_M163(); break;
        #if MIXING_VIRTUAL_TOOLS > 1
          case 164: // M164 S<int> save current mix as a virtual tools
            gcode_M164(); break;
        #endif
        case 165: // M165 [ABCDHI]<float> set multiple mix weights
          gcode_M165(); break;
      #endif

      #if HAS(TEMP_BED)
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

      #if ENABLED(FWRETRACT)
        case 207: //M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop]
          gcode_M207(); break;
        case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
          gcode_M208(); break;
        case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
          gcode_M209(); break;
      #endif // FWRETRACT

      case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
        gcode_M218(); break;
      case 220: // M220 S<factor in percent> - set speed factor override percentage
        gcode_M220(); break;
      case 221: // M221 T<extruder> S<factor in percent> - set extrude factor override percentage
        gcode_M221(); break;
      case 222: // M222 T<extruder> S<factor in percent> - set density extrude factor percentage for purge
        gcode_M222(); break;
      case 226: // M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
        gcode_M226(); break;

      #if HAS(CHDK) || HAS(PHOTOGRAPH)
        case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
          gcode_M240(); break;
      #endif // HAS(CHDK) || HAS(PHOTOGRAPH)

      #if ENABLED(DOGLCD) && LCD_CONTRAST >= 0
        case 250: // M250  Set LCD contrast value: C<value> (value 0..63)
          gcode_M250(); break;
      #endif // DOGLCD

      #if HAS(SERVOS)
        case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
          gcode_M280(); break;
      #endif // NUM_SERVOS > 0

      #if HAS(BUZZER)
        case 300: // M300 - Play beep tone
          gcode_M300(); break;
      #endif // HAS(BUZZER)

      #if ENABLED(PIDTEMP)
        case 301: // M301
          gcode_M301(); break;
      #endif // PIDTEMP

      #if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
        case 302: // allow cold extrudes, or set the minimum extrude temperature
          gcode_M302(); break;
      #endif // PREVENT_DANGEROUS_EXTRUDE

      #if ENABLED(PIDTEMP) || ENABLED(PIDTEMPBED)
        case 303: // M303 PID autotune
          gcode_M303(); break;
      #endif

      #if ENABLED(PIDTEMPBED)
        case 304: // M304
          gcode_M304(); break;
      #endif // PIDTEMPBED

      #if HAS(MICROSTEPS)
        case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
          gcode_M350(); break;
        case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
          gcode_M351(); break;
      #endif // HAS(MICROSTEPS)

      #if MECH(SCARA)
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

      #if HAS(SERVO_ENDSTOPS)
        case 401:
          gcode_M401(); break;
        case 402:
          gcode_M402(); break;
      #endif

      #if ENABLED(FILAMENT_SENSOR)
        case 404:  //M404 Enter the nominal filament width (3mm, 1.75mm ) N<3.0> or display nominal filament width
          gcode_M404(); break;
        case 405:  //M405 Turn on filament sensor for control
          gcode_M405(); break;
        case 406:  //M406 Turn off filament sensor for control
          gcode_M406(); break;
        case 407:   //M407 Display measured filament diameter
          gcode_M407(); break;
      #endif // FILAMENT_SENSOR

      #if ENABLED(JSON_OUTPUT)
        case 408: // M408 JSON STATUS OUTPUT
          gcode_M408(); break;
      #endif // JSON_OUTPUT

      case 410: // M410 quickstop - Abort all the planned moves.
        gcode_M410(); break;

      case 428: // M428 Apply current_position to home_offset
        gcode_M428(); break;

      case 500: // M500 Store settings in EEPROM
        gcode_M500(); break;
      case 501: // M501 Read settings from EEPROM
        gcode_M501(); break;
      case 502: // M502 Revert to default settings
        gcode_M502(); break;
      case 503: // M503 print settings currently in memory
        gcode_M503(); break;

      #if ENABLED(RFID_MODULE)
        case 522: // M422 Read or Write on card. M522 T<extruders> R<read> or W<write>
          gcode_M522(); break;
      #endif

      #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
        case 540:
          gcode_M540(); break;
      #endif

      #if HEATER_USES_AD595
        case 595: // M595 set Hotends AD595 offset & gain
          gcode_M595(); break;
      #endif

      #if ENABLED(FILAMENTCHANGEENABLE)
        case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
          gcode_M600(); break;
      #endif

      #if ENABLED(DUAL_X_CARRIAGE)
        case 605:
          gcode_M605(); break;
      #endif

      #if ENABLED(AUTO_BED_LEVELING_FEATURE) || MECH(DELTA)
        case 666: // M666 Set Z probe offset or set delta endstop and geometry adjustment
          gcode_M666(); break;
      #endif

      #if MB(ALLIGATOR)
        case 906: // M906 Set motor currents XYZ T0-4 E
          gcode_M906(); break;
      #endif

      case 907: // M907 Set digital trimpot motor current using axis codes.
        gcode_M907(); break;

      #if HAS(DIGIPOTSS)
        case 908: // M908 Control digital trimpot directly.
          gcode_M908(); break;
      #endif // HAS(DIGIPOTSS)

      #if ENABLED(NPR2)
        case 997: // M997 Cxx Move Carter xx gradi
          gcode_M997(); break;
      #endif // NPR2

      case 999: // M999: Restart after being Stopped
        gcode_M999(); break;
    }
    break;

    case 'T':
      gcode_T(codenum);
    break;

    default: code_is_good = false;
  }

  KEEPALIVE_STATE(NOT_BUSY);

ExitUnknownCommand:

  // Still unknown command? Throw an error
  if (!code_is_good) unknown_command_error();

  ok_to_send();
}

void FlushSerialRequestResend() {
  //char command_queue[cmd_queue_index_r][100]="Resend:";
  MKSERIAL.flush();
  ECHO_LV(RESEND, (long)(gcode_LastN + 1));
  ok_to_send();
}

void ok_to_send() {
  refresh_cmd_timeout();
  if (!send_ok[cmd_queue_index_r]) return;
  ECHO_S(OK);
  #if ENABLED(ADVANCED_OK)
    char* p = command_queue[cmd_queue_index_r];
    if (*p == 'N') {
      ECHO_C(' ');
      ECHO_C(*p++);
      while (NUMERIC_SIGNED(*p))
        ECHO_C(*p++);
    }
    ECHO_MV(" P", (int)(BLOCK_BUFFER_SIZE - movesplanned() - 1));
    ECHO_MV(" B", BUFSIZE - commands_in_queue);
  #endif
  ECHO_E;
}

void clamp_to_software_endstops(float target[3]) {
  if (SOFTWARE_MIN_ENDSTOPS && software_endstops) {
    NOLESS(target[X_AXIS], min_pos[X_AXIS]);
    NOLESS(target[Y_AXIS], min_pos[Y_AXIS]);
    
    float negative_z_offset = 0;
    #if ENABLED(AUTO_BED_LEVELING_FEATURE)
      if (zprobe_zoffset < 0) negative_z_offset += zprobe_zoffset;
      if (home_offset[Z_AXIS] < 0) negative_z_offset += home_offset[Z_AXIS];
    #endif
    NOLESS(target[Z_AXIS], min_pos[Z_AXIS] + negative_z_offset);
  }

  if (SOFTWARE_MAX_ENDSTOPS && software_endstops) {
    NOMORE(target[X_AXIS], max_pos[X_AXIS]);
    NOMORE(target[Y_AXIS], max_pos[Y_AXIS]);
    NOMORE(target[Z_AXIS], max_pos[Z_AXIS]);
  }
}

#if ENABLED(PREVENT_DANGEROUS_EXTRUDE)

  FORCE_INLINE void prevent_dangerous_extrude(float &curr_e, float &dest_e) {
    float de = dest_e - curr_e;
    if (DEBUGGING(DRYRUN)) return;
    if (de) {
      if (degHotend(active_extruder) < extrude_min_temp) {
        curr_e = dest_e; // Behave as if the move really took place, but ignore E part
        ECHO_LM(ER, SERIAL_ERR_COLD_EXTRUDE_STOP);
      }
      #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
        if (labs(de) > EXTRUDE_MAXLENGTH) {
          curr_e = dest_e; // Behave as if the move really took place, but ignore E part
          ECHO_LM(ER, SERIAL_ERR_LONG_EXTRUDE_STOP);
        }
      #endif
    }
  }

#endif // PREVENT_DANGEROUS_EXTRUDE

#if MECH(DELTA) || MECH(SCARA)

  inline bool prepare_move_delta(float target[NUM_AXIS]) {
    float difference[NUM_AXIS];
    float addDistance[NUM_AXIS];
    float fractions[NUM_AXIS];
    float frfm = feedrate / 60 * feedrate_multiplier / 100.0;

    for (uint8_t i = 0; i < NUM_AXIS; i++) difference[i] = target[i] - current_position[i];

    float cartesian_mm = sqrt(sq(difference[X_AXIS]) + sq(difference[Y_AXIS]) + sq(difference[Z_AXIS]));
    if (cartesian_mm < 0.000001) cartesian_mm = abs(difference[E_AXIS]);
    if (cartesian_mm < 0.000001) return false;

    #if ENABLED(DELTA_SEGMENTS_PER_SECOND)
      float seconds = 6000 * cartesian_mm / feedrate / feedrate_multiplier;
      int steps = max(1, int(DELTA_SEGMENTS_PER_SECOND * seconds));

      if (DEBUGGING(DEBUG)) {
        ECHO_SMV(DEB, "mm=", cartesian_mm);
        ECHO_MV(" seconds=", seconds);
        ECHO_EMV(" steps=", steps);
      }

    #else
      float fTemp = cartesian_mm * 5;
      int steps = (int)fTemp;

      if (steps == 0) {
        steps = 1;
        for (uint8_t i = 0; i < NUM_AXIS; i++) fractions[i] = difference[i];
      }
      else {
        fTemp = 1 / float(steps);
        for (uint8_t i = 0; i < NUM_AXIS; i++) fractions[i] = difference[i] * fTemp;
      }

      // For number of steps, for each step add one fraction
      // First, set initial target to current position
      for (uint8_t i = 0; i < NUM_AXIS; i++) addDistance[i] = 0.0;
    #endif

    for (int s = 1; s <= steps; s++) {

      #if ENABLED(DELTA_SEGMENTS_PER_SECOND)
        float fraction = float(s) / float(steps);
        for (uint8_t i = 0; i < NUM_AXIS; i++)
          target[i] = current_position[i] + difference[i] * fraction;
      #else
        for (uint8_t i = 0; i < NUM_AXIS; i++) {
          addDistance[i] += fractions[i];
          target[i] = current_position[i] + addDistance[i];
        }
      #endif

      calculate_delta(target);
      adjust_delta(target);

      if (DEBUGGING(DEBUG)) {
        ECHO_LMV(DEB, "target[X_AXIS]=", target[X_AXIS]);
        ECHO_LMV(DEB, "target[Y_AXIS]=", target[Y_AXIS]);
        ECHO_LMV(DEB, "target[Z_AXIS]=", target[Z_AXIS]);
        ECHO_LMV(DEB, "delta[TOWER_1]=", delta[TOWER_1]);
        ECHO_LMV(DEB, "delta[TOWER_2]=", delta[TOWER_2]);
        ECHO_LMV(DEB, "delta[TOWER_3]=", delta[TOWER_3]);
      }

      plan_buffer_line(delta[TOWER_1], delta[TOWER_2], delta[TOWER_3], target[E_AXIS], frfm, active_extruder, active_driver);
    }
    return true;
  }

#endif // DELTA || SCARA

#if MECH(SCARA)
  inline bool prepare_move_scara(float target[NUM_AXIS]) { return prepare_move_delta(target); }
#endif

#if ENABLED(DUAL_X_CARRIAGE)

  inline bool prepare_move_dual_x_carriage() {
    if (active_extruder_parked) {
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && active_extruder == 0) {
        // move duplicate extruder into correct duplication position.
        plan_set_position(inactive_extruder_x_pos, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        plan_buffer_line(current_position[X_AXIS] + duplicate_hotend_x_offset, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS], 1, active_driver);
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
            return false;
          }
        }
        delayed_move_time = 0;
        // unpark extruder: 1) raise, 2) move into starting XY position, 3) lower
        plan_buffer_line(raised_parked_position[X_AXIS], raised_parked_position[Y_AXIS], raised_parked_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder, active_driver);
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], raised_parked_position[Z_AXIS], current_position[E_AXIS], min(max_feedrate[X_AXIS], max_feedrate[Y_AXIS]), active_extruder, active_driver);
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder, active_driver);
        active_extruder_parked = false;
      }
    }
    return true;
  }

#endif // DUAL_X_CARRIAGE

#if MECH(CARTESIAN) || MECH(COREXY) || MECH(COREYX) || MECH(COREXZ) || MECH(COREZX)

  inline bool prepare_move_cartesian() {
    // Do not use feedrate_multiplier for E or Z only moves
    if (current_position[X_AXIS] == destination[X_AXIS] && current_position[Y_AXIS] == destination[Y_AXIS]) {
      line_to_destination();
    }
    else {
      line_to_destination(feedrate * feedrate_multiplier / 100.0);
    }
    return true;
  }

#endif // CARTESIAN || COREXY || COREYX || COREXZ || COREZX

/**
 * Prepare a single move and get ready for the next one
 *
 * (This may call plan_buffer_line several times to put
 *  smaller moves into the planner for DELTA or SCARA.)
 */
void prepare_move() {
  clamp_to_software_endstops(destination);
  refresh_cmd_timeout();

  #if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
    prevent_dangerous_extrude(current_position[E_AXIS], destination[E_AXIS]);
  #endif

  #if MECH(SCARA)
    if (!prepare_move_scara(destination)) return;
  #elif MECH(DELTA)
    if (!prepare_move_delta(destination)) return;
  #endif

  #if ENABLED(DUAL_X_CARRIAGE)
    if (!prepare_move_dual_x_carriage()) return;
  #endif

  #if MECH(CARTESIAN) || MECH(COREXY) || MECH(COREYX) || MECH(COREXZ) || MECH(COREZX)
    if (!prepare_move_cartesian()) return;
  #endif

  set_current_to_destination();
}

/**
 * Plan an arc in 2 dimensions
 *
 * The arc is approximated by generating many small linear segments.
 * The length of each segment is configured in MM_PER_ARC_SEGMENT (Default 1mm)
 * Arcs should only be made relatively large (over 5mm), as larger arcs with
 * larger segments will tend to be more efficient. Your slicer should have
 * options for G2/G3 arc generation. In future these options may be GCode tunable.
 */
void plan_arc(
  float target[NUM_AXIS], // Destination position
  float *offset,          // Center of rotation relative to current_position
  uint8_t clockwise       // Clockwise?
) {

  float radius = hypot(offset[X_AXIS], offset[Y_AXIS]),
        center_axis0 = current_position[X_AXIS] + offset[X_AXIS],
        center_axis1 = current_position[Y_AXIS] + offset[Y_AXIS],
        linear_travel = target[Z_AXIS] - current_position[Z_AXIS],
        extruder_travel = target[E_AXIS] - current_position[E_AXIS],
        r_axis0 = -offset[X_AXIS],  // Radius vector from center to current location
        r_axis1 = -offset[Y_AXIS],
        rt_axis0 = target[X_AXIS] - center_axis0,
        rt_axis1 = target[Y_AXIS] - center_axis1;
  
  // CCW angle of rotation between position and target from the circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
  if (angular_travel < 0) { angular_travel += RADIANS(360); }
  if (clockwise) { angular_travel -= RADIANS(360); }
  
  // Make a circle if the angular rotation is 0
  if (current_position[X_AXIS] == target[X_AXIS] && current_position[Y_AXIS] == target[Y_AXIS] && angular_travel == 0)
    angular_travel += RADIANS(360);
  
  float mm_of_travel = hypot(angular_travel*radius, fabs(linear_travel));
  if (mm_of_travel < 0.001) { return; }
  uint16_t segments = floor(mm_of_travel / (MM_PER_ARC_SEGMENT));
  if (segments == 0) segments = 1;
  
  float theta_per_segment = angular_travel/segments;
  float linear_per_segment = linear_travel/segments;
  float extruder_per_segment = extruder_travel/segments;
  
  /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
     and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
         r_T = [cos(phi) -sin(phi);
                sin(phi)  cos(phi] * r ;
     
     For arc generation, the center of the circle is the axis of rotation and the radius vector is 
     defined from the circle center to the initial position. Each line segment is formed by successive
     vector rotations. This requires only two cos() and sin() computations to form the rotation
     matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
     all double numbers are single precision on the Arduino. (True double precision will not have
     round off issues for CNC applications.) Single precision error can accumulate to be greater than
     tool precision in some cases. Therefore, arc path correction is implemented. 

     Small angle approximation may be used to reduce computation overhead further. This approximation
     holds for everything, but very small circles and large MM_PER_ARC_SEGMENT values. In other words,
     theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
     to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for 
     numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
     issue for CNC machines with the single precision Arduino calculations.
     
     This approximation also allows plan_arc to immediately insert a line segment into the planner 
     without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
     a correction, the planner should have caught up to the lag caused by the initial plan_arc overhead. 
     This is important when there are successive arc motions. 
  */
  // Vector rotation matrix values
  float cos_T = 1-0.5*theta_per_segment*theta_per_segment; // Small angle approximation
  float sin_T = theta_per_segment;
  
  float arc_target[NUM_AXIS];
  float sin_Ti;
  float cos_Ti;
  float r_axisi;
  uint16_t i;
  int8_t count = 0;

  // Initialize the linear axis
  arc_target[Z_AXIS] = current_position[Z_AXIS];
  
  // Initialize the extruder axis
  arc_target[E_AXIS] = current_position[E_AXIS];

  float feed_rate = feedrate * feedrate_multiplier / 60 / 100.0;

  for (i = 1; i < segments; i++) { // Increment (segments-1)

    if (count < N_ARC_CORRECTION) {
      // Apply vector rotation matrix to previous r_axis0 / 1
      r_axisi = r_axis0*sin_T + r_axis1*cos_T;
      r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
      r_axis1 = r_axisi;
      count++;
    }
    else {
      // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
      // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
      cos_Ti = cos(i*theta_per_segment);
      sin_Ti = sin(i*theta_per_segment);
      r_axis0 = -offset[X_AXIS]*cos_Ti + offset[Y_AXIS]*sin_Ti;
      r_axis1 = -offset[X_AXIS]*sin_Ti - offset[Y_AXIS]*cos_Ti;
      count = 0;
    }

    // Update arc_target location
    arc_target[X_AXIS] = center_axis0 + r_axis0;
    arc_target[Y_AXIS] = center_axis1 + r_axis1;
    arc_target[Z_AXIS] += linear_per_segment;
    arc_target[E_AXIS] += extruder_per_segment;

    clamp_to_software_endstops(arc_target);
    #if MECH(DELTA) || MECH(SCARA)
      calculate_delta(arc_target);
      adjust_delta(arc_target);
      plan_buffer_line(delta[TOWER_1], delta[TOWER_2], delta[TOWER_3], arc_target[E_AXIS], feed_rate, active_extruder, active_driver);
    #else
      plan_buffer_line(arc_target[X_AXIS], arc_target[Y_AXIS], arc_target[Z_AXIS], arc_target[E_AXIS], feed_rate, active_extruder, active_driver);
    #endif
  }

  // Ensure last segment arrives at target location.
  #if MECH(DELTA) || MECH(SCARA)
    calculate_delta(target);
    adjust_delta(target);
    plan_buffer_line(delta[TOWER_1], delta[TOWER_2], delta[TOWER_3], target[E_AXIS], feed_rate, active_extruder, active_driver);
  #else
    plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feed_rate, active_extruder, active_driver);
  #endif

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  set_current_to_destination();
}

#if HAS(CONTROLLERFAN)

  void controllerFan() {
    static millis_t lastMotor = 0;      // Last time a motor was turned on
    static millis_t lastMotorCheck = 0; // Last time the state was checked
    millis_t ms = millis();
    if (ms >= lastMotorCheck + 2500) { // Not a time critical function, so we only check every 2500ms
      lastMotorCheck = ms;
      if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON || soft_pwm_bed > 0
        || E0_ENABLE_READ == E_ENABLE_ON // If any of the drivers are enabled...
        #if EXTRUDERS > 1
          || E1_ENABLE_READ == E_ENABLE_ON
          #if HAS(X2_ENABLE)
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
      
  #if ENABLED(INVERTED_HEATER_PINS)
      uint8_t speed = (lastMotor == 0 || ms >= lastMotor + (CONTROLLERFAN_SECS * 1000UL)) ? 255 - CONTROLLERFAN_MIN_SPEED : (255 - CONTROLLERFAN_SPEED);
  #else
      uint8_t speed = (lastMotor == 0 || ms >= lastMotor + (CONTROLLERFAN_SECS * 1000UL)) ? CONTROLLERFAN_MIN_SPEED : CONTROLLERFAN_SPEED;
  #endif

      // allows digital or PWM fan output to be used (see M42 handling)
      #if ENABLED(FAN_SOFT_PWM)
        fanSpeedSoftPwm_controller = speed;
      #else
        digitalWrite(CONTROLLERFAN_PIN, speed);
        analogWrite(CONTROLLERFAN_PIN, speed);
      #endif
    }
  }

#endif // HAS(CONTROLLERFAN)

#if MECH(SCARA)

  void calculate_SCARA_forward_Transform(float f_scara[3]) {
    // Perform forward kinematics, and place results in delta[3]
    // The maths and first version has been done by QHARLEY . Integrated into masterbranch 06/2014 and slightly restructured by Joachim Cerny in June 2014

    float x_sin, x_cos, y_sin, y_cos;

      //ECHO_SMV(DB, "f_delta x=", f_scara[X_AXIS]);
      //ECHO_MV(" y=", f_scara[Y_AXIS]);

      x_sin = sin(f_scara[X_AXIS]/SCARA_RAD2DEG) * LINKAGE_1;
      x_cos = cos(f_scara[X_AXIS]/SCARA_RAD2DEG) * LINKAGE_1;
      y_sin = sin(f_scara[Y_AXIS]/SCARA_RAD2DEG) * LINKAGE_2;
      y_cos = cos(f_scara[Y_AXIS]/SCARA_RAD2DEG) * LINKAGE_2;

      //ECHO_MV(" x_sin=", x_sin);
      //ECHO_MV(" x_cos=", x_cos);
      //ECHO_MV(" y_sin=", y_sin);
      //ECHO_MV(" y_cos=", y_cos);

      delta[X_AXIS] = x_cos + y_cos + SCARA_OFFSET_X;  //theta
      delta[Y_AXIS] = x_sin + y_sin + SCARA_OFFSET_Y;  //theta+phi

      //ECHO_MV(" delta[X_AXIS]=", delta[X_AXIS]);
      //ECHO_EMV(" delta[Y_AXIS]=", delta[Y_AXIS]);
  }

  void calculate_delta(float cartesian[3]) {
    //reverse kinematics.
    // Perform reversed kinematics, and place results in delta[3]
    // The maths and first version has been done by QHARLEY . Integrated into masterbranch 06/2014 and slightly restructured by Joachim Cerny in June 2014

    float SCARA_pos[2];
    static float SCARA_C2, SCARA_S2, SCARA_K1, SCARA_K2, SCARA_theta, SCARA_psi; 

    SCARA_pos[X_AXIS] = cartesian[X_AXIS] * axis_scaling[X_AXIS] - SCARA_OFFSET_X;  //Translate SCARA to standard X Y
    SCARA_pos[Y_AXIS] = cartesian[Y_AXIS] * axis_scaling[Y_AXIS] - SCARA_OFFSET_Y;  // With scaling factor.

    #if (LINKAGE_1 == LINKAGE_2)
      SCARA_C2 = ( ( sq(SCARA_pos[X_AXIS]) + sq(SCARA_pos[Y_AXIS]) ) / (2 * (float)sq(LINKAGE_1)) ) - 1;
    #else
      SCARA_C2 =   ( sq(SCARA_pos[X_AXIS]) + sq(SCARA_pos[Y_AXIS]) - (float)sq(LINKAGE_1) - (float)sq(LINKAGE_2) ) / 45000; 
    #endif

    SCARA_S2 = sqrt( 1 - sq(SCARA_C2) );

    SCARA_K1 = LINKAGE_1 + LINKAGE_2 * SCARA_C2;
    SCARA_K2 = LINKAGE_2 * SCARA_S2;

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

#endif // SCARA

#if ENABLED(TEMP_STAT_LEDS)

  static bool red_led = false;
  static millis_t next_status_led_update_ms = 0;

  void handle_status_leds(void) {
    float max_temp = 0.0;
    if (millis() > next_status_led_update_ms) {
      next_status_led_update_ms += 500; // Update every 0.5s
      for (uint8_t h = 0; h < HOTENDS; ++h)
         max_temp = max(max(max_temp, degHotend(h)), degTargetHotend(h));
      #if HAS(TEMP_BED)
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

/**
 * Standard idle routine keeps the machine alive
 */
void idle(
  #if ENABLED(FILAMENTCHANGEENABLE)
    bool no_stepper_sleep/*=false*/
  #endif
) {
  manage_heater();
  manage_inactivity(
    #if ENABLED(FILAMENTCHANGEENABLE)
      no_stepper_sleep
    #endif
  );
  host_keepalive();
  lcd_update();
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
 *  - Check oozing prevent
 *  - Read o Write Rfid
 */
void manage_inactivity(bool ignore_stepper_queue/*=false*/) {

  #if HAS(FILRUNOUT)
    if ((Printing || IS_SD_PRINTING) && (READ(FILRUNOUT_PIN) ^ FILRUNOUT_PIN_INVERTING))
      filrunout();
  #endif

  if (commands_in_queue < BUFSIZE - 1) get_available_commands();

  millis_t ms = millis();

  if (max_inactive_time && ELAPSED(ms, previous_cmd_ms + max_inactive_time)) kill(PSTR(MSG_KILLED));

  if (stepper_inactive_time && ELAPSED(ms, previous_cmd_ms + stepper_inactive_time)
      && !ignore_stepper_queue && !blocks_queued()) {
    #if DISABLE_X == true
      disable_x();
    #endif
    #if DISABLE_Y == true
      disable_y();
    #endif
    #if DISABLE_Z == true
      disable_z();
    #endif
    #if DISABLE_E == true
      disable_e();
    #endif
  }

  #if HAS(CHDK) // Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive && PENDING(ms, chdkHigh + CHDK_DELAY)) {
      chdkActive = false;
      WRITE(CHDK_PIN, LOW);
    }
  #endif

  #if HAS(KILL)
    
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
    if (killCount >= KILL_DELAY) kill(PSTR(MSG_KILLED));
  #endif

  #if HAS(HOME)
    // Check to see if we have to home, use poor man's debouncer
    // ---------------------------------------------------------
    static int homeDebounceCount = 0;   // poor man's debouncing count
    const int HOME_DEBOUNCE_DELAY = 750;
    if (!READ(HOME_PIN)) {
      if (!homeDebounceCount) {
        enqueue_and_echo_commands_P(PSTR("G28"));
        LCD_MESSAGEPGM(MSG_AUTO_HOME);
      }
      if (homeDebounceCount < HOME_DEBOUNCE_DELAY)
        homeDebounceCount++;
      else
        homeDebounceCount = 0;
    }
  #endif
    
  #if HAS(CONTROLLERFAN)
    controllerFan(); // Check if fan should be turned on to cool stepper drivers down
  #endif

  #if ENABLED(EXTRUDER_RUNOUT_PREVENT)
    if (ELAPSED(ms, previous_cmd_ms + (EXTRUDER_RUNOUT_SECONDS) * 1000UL))
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
                        destination[E_AXIS] + (EXTRUDER_RUNOUT_EXTRUDE) * (EXTRUDER_RUNOUT_ESTEPS) / axis_steps_per_unit[E_AXIS],
                        (EXTRUDER_RUNOUT_SPEED) / 60. * (EXTRUDER_RUNOUT_ESTEPS) / axis_steps_per_unit[E_AXIS], active_extruder, active_driver);
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
    }
  #endif

  #if ENABLED(DUAL_X_CARRIAGE)
    // handle delayed move timeout
    if (delayed_move_time && ms > delayed_move_time + 1000 && IsRunning()) {
      // travel moves have been received so enact them
      delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      set_destination_to_current();
      prepare_move();
    }
  #endif

  #if ENABLED(IDLE_OOZING_PREVENT)
    if (blocks_queued()) axis_last_activity = millis();
    if (degHotend(active_extruder) > IDLE_OOZING_MINTEMP && !(DEBUGGING(DRYRUN)) && IDLE_OOZING_enabled) {
      #if ENABLED(FILAMENTCHANGEENABLE)
        if (!filament_changing)
      #endif
      {
        if (degTargetHotend(active_extruder) < IDLE_OOZING_MINTEMP) {
          IDLE_OOZING_retract(false);
        }
        else if ((millis() - axis_last_activity) >  IDLE_OOZING_SECONDS * 1000UL) {
          IDLE_OOZING_retract(true);
        }
      }
    }
  #endif

  #if ENABLED(RFID_MODULE)
    for (uint8_t e = 0; e < EXTRUDERS; e++) {
      if (Spool_must_read[e]) {
        if (RFID522.getID(e)) {
          Spool_ID[e] = RFID522.RfidDataID[e].Spool_ID;
          HAL::delayMilliseconds(200);
          if (RFID522.readBlock(e)) {
            Spool_must_read[e] = false;
            density_multiplier[e] = RFID522.RfidData[e].data.density;
            filament_size[e] = RFID522.RfidData[e].data.size;
            calculate_volumetric_multipliers();
            RFID522.printInfo(e);
          }
        }
      }

      if (Spool_must_write[e]) {
        if (RFID522.getID(e)) {
          if (Spool_ID[e] == RFID522.RfidDataID[e].Spool_ID) {
            HAL::delayMilliseconds(200);
            if (RFID522.writeBlock(e)) {
              Spool_must_write[e] = false;
              ECHO_SMV(INFO, "Spool on E", e);
              ECHO_EM(" writed!");
              RFID522.printInfo(e);
            }
          }
        }
      }
    }
  #endif

  #if ENABLED(SDSUPPORT) && ENABLED(SD_SETTINGS)
    if(IS_SD_INSERTED && !IS_SD_PRINTING) {
      if (!config_readed) {
        ConfigSD_RetrieveSettings(true);
        ConfigSD_StoreSettings();
      }
      else if((millis() - config_last_update) >  SD_CFG_SECONDS * 1000UL) {
        ConfigSD_StoreSettings();
      }
    }
  #endif

  #if ENABLED(TEMP_STAT_LEDS)
    handle_status_leds();
  #endif

  #if ENABLED(TEMP_STAT_LEDS)
    handle_status_leds();
  #endif

  check_axes_activity();
}

void kill(const char* lcd_msg) {
  #if ENABLED(KILL_METHOD) && KILL_METHOD == 1
    HAL::resetHardware();
  #endif

  #if ENABLED(ULTRA_LCD)
    lcd_setalertstatuspgm(lcd_msg);
  #endif

  cli(); // Stop interrupts
  disable_all_heaters();
  disable_all_steppers();

  #if HAS(POWER_SWITCH)
    SET_INPUT(PS_ON_PIN);
  #endif

  ECHO_LM(ER, SERIAL_ERR_KILLED);

  // FMC small patch to update the LCD before ending
  sei();   // enable interrupts
  for (int i = 5; i--; lcd_update()) HAL::delayMilliseconds(200); // Wait a short time
  cli();   // disable interrupts
  #if HAS(SUICIDE)
    suicide();
  #endif
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

#if HAS(FILRUNOUT)
  void filrunout() {
    if (!filrunoutEnqueued) {
      filrunoutEnqueued = true;
      enqueue_and_echo_commands_P(PSTR(FILAMENT_RUNOUT_SCRIPT));
      st_synchronize();
    }
  }
#endif

#if ENABLED(FAST_PWM_FAN)

  void setPwmFrequency(uint8_t pin, int val) {
    val &= 0x07;
    switch(digitalPinToTimer(pin)) {

      #if defined(TCCR0A)
        case TIMER0A:
        case TIMER0B:
             // TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
             // TCCR0B |= val;
             break;
      #endif

      #if defined(TCCR1A)
        case TIMER1A:
        case TIMER1B:
             // TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
             // TCCR1B |= val;
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
#endif // FAST_PWM_FAN

void stop() {
  disable_all_heaters();
  if (IsRunning()) {
    Running = false;
    ECHO_LM(ER, SERIAL_ERR_STOPPED);
    ECHO_S(PAUSE);
    ECHO_E;
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

float calculate_volumetric_multiplier(float diameter) {
  if (!volumetric_enabled || diameter == 0) return 1.0;
  float d2 = diameter * 0.5;
  return 1.0 / (M_PI * d2 * d2);
}

void calculate_volumetric_multipliers() {
  for (uint8_t e = 0; e < EXTRUDERS; e++)
    volumetric_multiplier[e] = calculate_volumetric_multiplier(filament_size[e]);
}
