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
  #if ENABLED(FILAMENTCHANGEENABLE)
    bool no_stepper_sleep=false  // pass true to keep steppers from disabling on timeout
  #endif
);

void manage_inactivity(bool ignore_stepper_queue = false);

void FlushSerialRequestResend();
void ok_to_send();

#if MECH(DELTA)
  float probe_bed(float x, float y);
  void set_delta_constants();
  void adj_tower_delta(int tower);
  void adj_tower_radius(int tower);
  void home_delta_axis();
  void calibration_report();
  void bed_probe_all();
  void set_delta_constants();
  void save_carriage_positions(int position_num);
  void calculate_delta(float cartesian[3]);
  void adjust_delta(float cartesian[3]);
  void adj_endstops();
  void reset_bed_level();
  void prepare_move_raw();
  extern float delta[3];
  extern float delta_tmp[3];
  extern float delta_tower1_x, delta_tower1_y;
  extern float delta_tower2_x, delta_tower2_y;
  extern float delta_tower3_x, delta_tower3_y;
  extern float z_probe_offset[3];
  extern float endstop_adj[3];
  extern float tower_adj[6];
  extern float diagrod_adj[3];
  extern float delta_radius;
  extern float delta_diagonal_rod;
#endif

#if MECH(SCARA)
  void calculate_delta(float cartesian[3]);
  void calculate_SCARA_forward_Transform(float f_scara[3]);
#endif
void prepare_move();
void kill(const char *);
void Stop();

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  void filrunout();
#endif

/**
 * Debug flags - with repetier
 */
enum DebugFlags {
  DEBUG_NONE          = 0,
  DEBUG_ECHO          = _BV(0), ///< Echo commands in order as they are processed
  DEBUG_INFO          = _BV(1), ///< Print messages for code that has debug output
  DEBUG_ERRORS        = _BV(2), ///< Not implemented
  DEBUG_DRYRUN        = _BV(3), ///< Ignore temperature setting
  DEBUG_COMMUNICATION = _BV(4), ///< Not implemented
  DEBUG_DEBUG         = _BV(5)  ///< Print Debug
};
extern uint8_t mk_debug_flags;

void clamp_to_software_endstops(float target[3]);

extern bool Running;
inline bool IsRunning() { return  Running; }
inline bool IsStopped() { return !Running; }
extern bool Printing;

bool enqueue_and_echo_command(const char* cmd, bool say_ok = false); // put a single ASCII command at the end of the current buffer or return false when it is full
void enqueue_and_echo_command_now(const char* cmd); // enqueue now, only return when the command has been enqueued
void enqueue_and_echo_commands_P(const char* cmd);  // put one or many ASCII commands at the end of the current buffer, read from flash

void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);

extern millis_t previous_cmd_ms;
void refresh_cmd_timeout();

extern void delay_ms(millis_t ms);

#if ENABLED(FAST_PWM_FAN)
  void setPwmFrequency(uint8_t pin, uint8_t val);
#endif

extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedrate_multiplier;
extern bool volumetric_enabled;
extern int extruder_multiplier[EXTRUDERS];      // sets extrude multiply factor (in percent) for each extruder individually
extern int density_multiplier[EXTRUDERS];       // sets density multiply factor (in percent) for each extruder individually
extern float filament_size[EXTRUDERS];          // cross-sectional area of filament (in millimeters), typically around 1.75 or 2.85, 0 disables the volumetric calculations for the extruder.
extern float volumetric_multiplier[EXTRUDERS];  // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner
extern float current_position[NUM_AXIS];
extern float destination[NUM_AXIS];
extern float home_offset[3];
extern float hotend_offset[3][HOTENDS];
extern float min_pos[3];
extern float max_pos[3];
extern float zprobe_zoffset;
extern uint8_t axis_known_position;
extern uint8_t axis_was_homed;

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

#if MECH(SCARA)
  extern float axis_scaling[3];  // Build size scaling
#endif

// Lifetime stats
extern unsigned long printer_usage_seconds;  // this can old about 136 year before go overflow. If you belive that you can live more than this please contact me.
// Filament stats
extern double printer_usage_filament;

#if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
  extern float extrude_min_temp;
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
  extern float filament_width_nominal;    //holds the theoretical filament diameter ie., 3.00 or 1.75
  extern bool filament_sensor;            //indicates that filament sensor readings should control extrusion
  extern float filament_width_meas;       //holds the filament diameter as accurately measured
  extern signed char measurement_delay[]; //ring buffer to delay measurement
  extern int delay_index1, delay_index2;  //ring buffer index. used by planner, temperature, and main code
  extern float delay_dist;                //delay distance counter
  extern int meas_delay_cm;               //delay distance
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

#if ENABLED(LASERBEAM)
  extern int laser_ttl_modulation;
#endif

#if ENABLED(SDSUPPORT) && ENABLED(SD_SETTINGS)
  extern millis_t config_last_update;
  extern bool config_readed;
#endif

// Print job timer
extern Stopwatch print_job_timer;

// Handling multiple extruders pins
extern uint8_t active_extruder;
extern uint8_t previous_extruder;
extern uint8_t active_driver;

#if MB(ALLIGATOR)
  extern float motor_current[DRIVER_EXTRUDERS + 3];
#endif

#if ENABLED(DIGIPOT_I2C)
  extern void digipot_i2c_set_current( int channel, float current );
  extern void digipot_i2c_init();
#endif

#if HAS(TEMP_0) || HAS(TEMP_BED) || ENABLED(HEATER_0_USES_MAX6675)
  void print_heaterstates();
#endif

#if ENABLED(FIRMWARE_TEST)
  void FirmwareTest();
#endif

#if ENABLED(COLOR_MIXING_EXTRUDER)
  extern float mixing_factor[DRIVER_EXTRUDERS];
#endif

extern void calculate_volumetric_multipliers();

#if ENABLED(M100_FREE_MEMORY_WATCHER)
  extern void *__brkval;
  extern size_t  __heap_start, __heap_end, __flp;

  //
  // Declare all the functions we need from Marlin_Main.cpp to do the work!
  //
  float code_value();
  long code_value_long();
  bool code_seen(char );

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
