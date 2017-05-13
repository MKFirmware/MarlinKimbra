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
 */

#ifndef MK_H
#define MK_H

#include <math.h>
#include <stdint.h>

void get_command();

void idle(
  #if ENABLED(FILAMENT_CHANGE_FEATURE)
    bool no_stepper_sleep=false  // pass true to keep steppers from disabling on timeout
  #endif
);

void manage_inactivity(bool ignore_stepper_queue = false);

#if ENABLED(DUAL_X_CARRIAGE)
  extern bool hotend_duplication_enabled;
#endif

void FlushSerialRequestResend();
void ok_to_send();

#if MECH(DELTA)
  void set_delta_constants();
  void inverse_kinematics(const float in_cartesian[ABC]);
  extern float delta[ABC];
  extern float endstop_adj[ABC];
  extern float diagrod_adj[ABC];
  extern float tower_adj[6];
  extern float delta_radius;
  extern float delta_diagonal_rod;
  extern float delta_segments_per_second;
#endif

#if MECH(SCARA)
  extern float delta[ABC];
  extern float axis_scaling[ABC];  // Build size scaling
  void inverse_kinematics(const float in_cartesian[ABC]);
  void forward_kinematics_SCARA(float f_scara[ABC]);
#endif

void kill(const char *);
void Stop();
void quickstop_stepper();

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  void handle_filament_runout();
#endif

extern uint8_t mk_debug_flags;

extern bool Running;
inline bool IsRunning() { return  Running; }
inline bool IsStopped() { return !Running; }
extern bool Printing;

bool enqueue_and_echo_command(const char* cmd, bool say_ok = false); // put a single ASCII command at the end of the current buffer or return false when it is full
void enqueue_and_echo_command_now(const char* cmd); // enqueue now, only return when the command has been enqueued
void enqueue_and_echo_commands_P(const char* cmd);  // put one or many ASCII commands at the end of the current buffer, read from flash
void clear_command_queue();

void prepare_arc_move(char isclockwise);

extern millis_t previous_cmd_ms;
inline void refresh_cmd_timeout() { previous_cmd_ms = millis(); }

extern void safe_delay(millis_t ms);

#if ENABLED(FAST_PWM_FAN) || ENABLED(FAST_PWM_COOLER) || ENABLED(FAST_PWM_CNCROUTER)
  void setPwmFrequency(uint8_t pin, uint8_t val);
#endif

#if ENABLED(LASERBEAM) && ENABLED(CNCROUTER)
  #define PRINTER_MODE_FFF 0
  #define PRINTER_MODE_LASER 1
  #define PRINTER_MODE_CNCROUTER 2
#endif

/**
 * Feedrate scaling and conversion
 */
extern int feedrate_percentage;

extern bool axis_relative_modes[];
extern bool volumetric_enabled;
extern int flow_percentage[EXTRUDERS];          // Extrusion factor for each extruder
extern int density_percentage[EXTRUDERS];       // Extrusion density factor for each extruder
extern float filament_size[EXTRUDERS];          // cross-sectional area of filament (in millimeters), typically around 1.75 or 2.85, 0 disables the volumetric calculations for the extruder.
extern float volumetric_multiplier[EXTRUDERS];  // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner
extern bool axis_known_position[XYZ];           // axis[n].is_known
extern bool axis_homed[XYZ];                    // axis[n].is_homed
extern volatile bool wait_for_heatup;

extern float current_position[NUM_AXIS];
extern float destination[NUM_AXIS];
extern float position_shift[XYZ];
extern float home_offset[XYZ];
extern float hotend_offset[XYZ][HOTENDS];

// Software Endstops
void update_software_endstops(AxisEnum axis);
#if ENABLED(SOFTWARE_MIN_ENDSTOPS) || ENABLED(SOFTWARE_MAX_ENDSTOPS)
  extern bool soft_endstops_enabled;
  void clamp_to_software_endstops(float target[XYZ]);
#else
  #define soft_endstops_enabled false
  #define clamp_to_software_endstops(x) NOOP
#endif
extern float soft_endstop_min[XYZ];
extern float soft_endstop_max[XYZ];

#define LOGICAL_POSITION(POS, AXIS) (POS + home_offset[AXIS] + position_shift[AXIS])
#define RAW_POSITION(POS, AXIS)     (POS - home_offset[AXIS] - position_shift[AXIS])
#define LOGICAL_X_POSITION(POS)     LOGICAL_POSITION(POS, X_AXIS)
#define LOGICAL_Y_POSITION(POS)     LOGICAL_POSITION(POS, Y_AXIS)
#define LOGICAL_Z_POSITION(POS)     LOGICAL_POSITION(POS, Z_AXIS)
#define RAW_X_POSITION(POS)         RAW_POSITION(POS, X_AXIS)
#define RAW_Y_POSITION(POS)         RAW_POSITION(POS, Y_AXIS)
#define RAW_Z_POSITION(POS)         RAW_POSITION(POS, Z_AXIS)
#define RAW_CURRENT_POSITION(AXIS)  RAW_POSITION(current_position[AXIS], AXIS)

// GCode support for external objects
bool code_seen(char);
int code_value_int();
float code_value_temp_abs();
float code_value_temp_diff();

#if ENABLED(LIN_ADVANCE)
  extern int extruder_advance_k;
#endif

#if HEATER_USES_AD595
  extern float ad595_offset[HOTENDS];
  extern float ad595_gain[HOTENDS];
#endif

#if ENABLED(NPR2)
  extern uint8_t old_color; // old color for system NPR2
#endif

#if ENABLED(Z_DUAL_ENDSTOPS)
  extern float z_endstop_adj;
#endif

#if HAS(BED_PROBE)
  extern float zprobe_zoffset;
#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  extern uint8_t host_keepalive_interval;
#endif

extern int fanSpeed;

#if ENABLED(BARICUDA)
  extern int baricuda_valve_pressure;
  extern int baricuda_e_to_p_pressure;
#endif

#if ENABLED(FAN_SOFT_PWM)
  extern unsigned char fanSpeedSoftPwm;
  #if HAS(CONTROLLERFAN)
    extern unsigned char fanSpeedSoftPwm_controller;
  #endif
#endif

#if ENABLED(FILAMENT_SENSOR)
  extern float filament_width_nominal;    // holds the theoretical filament diameter ie., 3.00 or 1.75
  extern bool filament_sensor;            // indicates that filament sensor readings should control extrusion
  extern float filament_width_meas;       // holds the filament diameter as accurately measured
  extern int8_t measurement_delay[];      // ring buffer to delay measurement
  extern int  filwidth_delay_index1,
              filwidth_delay_index2;      // ring buffer index. used by planner, temperature, and main code
  extern float delay_dist;                // delay distance counter
  extern int meas_delay_cm;               // delay distance
#endif

#if ENABLED(FILAMENT_CHANGE_FEATURE)
  extern FilamentChangeMenuResponse filament_change_menu_response;
#endif

#if HAS(POWER_CONSUMPTION_SENSOR)
  extern float power_consumption_meas;          //holds the power consumption as accurately measured
  extern unsigned long power_consumption_hour;  //holds the power consumption per hour as accurately measured
  extern unsigned long startpower;
  extern unsigned long stoppower;
  extern float raw_analog2voltage();
  extern float analog2error(float current);
  extern float analog2efficiency(float watt);
#endif

#if ENABLED(IDLE_OOZING_PREVENT)
  extern bool IDLE_OOZING_enabled;
  extern void IDLE_OOZING_retract(bool retracting);
#endif

#if ENABLED(PIDTEMP) && ENABLED(PID_ADD_EXTRUSION_RATE)
  extern int lpq_len;
#endif

#if ENABLED(FWRETRACT)
  extern bool autoretract_enabled;
  extern bool retracted[EXTRUDERS]; // extruder[n].retracted
  extern float retract_length, retract_length_swap, retract_feedrate, retract_zlift;
  extern float retract_recover_length, retract_recover_length_swap, retract_recover_feedrate;
#endif

#if ENABLED(EASY_LOAD)
  extern bool allow_lengthy_extrude_once; // for load/unload
#endif

// Print job timer
extern PrintCounter print_job_counter;

// Handling multiple extruders pins
extern uint8_t active_extruder;
extern uint8_t previous_extruder;
extern uint8_t active_driver;

#if MB(ALLIGATOR)
  extern float motor_current[3 + DRIVER_EXTRUDERS];
#endif

#if ENABLED(DIGIPOT_I2C)
  extern void digipot_i2c_set_current( int channel, float current );
  extern void digipot_i2c_init();
#endif

#if HAS(TEMP_0) || HAS(TEMP_BED) || ENABLED(HEATER_0_USES_MAX6675)
  void print_heaterstates();
#endif

#if HAS(TEMP_CHAMBER)
  void print_chamberstate();
#endif

#if HAS(TEMP_COOLER)
  void print_coolerstate();
#endif

#if ENABLED(FLOWMETER_SENSOR)
  void print_flowratestate();
#endif

#if ENABLED(FIRMWARE_TEST)
  void FirmwareTest();
#endif

#if ENABLED(COLOR_MIXING_EXTRUDER)
  extern float mixing_factor[E_STEPPERS];
#endif

void calculate_volumetric_multipliers();

/**
 * Blocking movement and shorthand functions
 */
void do_blocking_move_to(const float &x, const float &y, const float &z, const float &fr_mm_s = 0.0);
void do_blocking_move_to_x(const float &x, const float &fr_mm_s = 0.0);
void do_blocking_move_to_z(const float &z, const float &fr_mm_s = 0.0);
void do_blocking_move_to_xy(const float &x, const float &y, const float &fr_mm_s = 0.0);

#if ENABLED(M100_FREE_MEMORY_WATCHER)
  extern void *__brkval;
  extern size_t  __heap_start, __heap_end, __flp;

  //
  // Utility functions used by M100 to get its work done.
  //
  unsigned char *top_of_stack();
  void prt_hex_nibble( unsigned int );
  void prt_hex_byte(unsigned int );
  void prt_hex_word(unsigned int );
  int how_many_E5s_are_here( unsigned char *);

#endif

#endif // MK_H
