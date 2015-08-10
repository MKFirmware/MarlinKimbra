// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// License: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#ifdef __SAM3X8E__
  #include "HAL.h"
#else
  #include <util/delay.h>
  #include <avr/eeprom.h>
  #include "fastio.h"
#endif

#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "Configuration.h"
#include "pins.h"

#ifndef SANITYCHECK_H
  #error Your Configuration.h and Configuration_adv.h files are outdated!
#endif

#include "Arduino.h"

typedef unsigned long millis_t;

// Arduino < 1.0.0 does not define this, so we need to do it ourselves
#ifndef analogInputToDigitalPin
  #define analogInputToDigitalPin(p) ((p) + 0xA0)
#endif

#include "comunication.h"

void get_command();

void idle(bool ignore_stepper_queue = false);

void manage_inactivity(bool ignore_stepper_queue=false);

#if ENABLED(DUAL_X_CARRIAGE) && HAS_X_ENABLE && HAS_X2_ENABLE
  #define  enable_x() do { X_ENABLE_WRITE( X_ENABLE_ON); X2_ENABLE_WRITE( X_ENABLE_ON); } while (0)
  #define disable_x() do { X_ENABLE_WRITE(!X_ENABLE_ON); X2_ENABLE_WRITE(!X_ENABLE_ON); axis_known_position[X_AXIS] = false; } while (0)
#elif HAS_X_ENABLE
  #define  enable_x() X_ENABLE_WRITE( X_ENABLE_ON)
  #define disable_x() { X_ENABLE_WRITE(!X_ENABLE_ON); axis_known_position[X_AXIS] = false; }
#else
  #define enable_x() ;
  #define disable_x() ;
#endif

#if HAS_Y_ENABLE
  #if ENABLED(Y_DUAL_STEPPER_DRIVERS)
    #define  enable_y() { Y_ENABLE_WRITE( Y_ENABLE_ON); Y2_ENABLE_WRITE(Y_ENABLE_ON); }
    #define disable_y() { Y_ENABLE_WRITE(!Y_ENABLE_ON); Y2_ENABLE_WRITE(!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }
  #else
    #define  enable_y() Y_ENABLE_WRITE( Y_ENABLE_ON)
    #define disable_y() { Y_ENABLE_WRITE(!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }
  #endif
#else
  #define enable_y() ;
  #define disable_y() ;
#endif

#if HAS_Z_ENABLE
  #if ENABLED(Z_DUAL_STEPPER_DRIVERS)
    #define  enable_z() { Z_ENABLE_WRITE( Z_ENABLE_ON); Z2_ENABLE_WRITE(Z_ENABLE_ON); }
    #define disable_z() { Z_ENABLE_WRITE(!Z_ENABLE_ON); Z2_ENABLE_WRITE(!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }
  #else
    #define  enable_z() Z_ENABLE_WRITE( Z_ENABLE_ON)
    #define disable_z() { Z_ENABLE_WRITE(!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }
  #endif
#else
  #define enable_z() ;
  #define disable_z() ;
#endif

#if HAS_E0_ENABLE
  #define enable_e0()  E0_ENABLE_WRITE( E_ENABLE_ON)
  #define disable_e0() E0_ENABLE_WRITE(!E_ENABLE_ON)
#else
  #define enable_e0()  /* nothing */
  #define disable_e0() /* nothing */
#endif

#if (DRIVER_EXTRUDERS > 1) && HAS_E1_ENABLE
  #define enable_e1()  E1_ENABLE_WRITE( E_ENABLE_ON)
  #define disable_e1() E1_ENABLE_WRITE(!E_ENABLE_ON)
#else
  #define enable_e1()  /* nothing */
  #define disable_e1() /* nothing */
#endif

#if (DRIVER_EXTRUDERS > 2) && HAS_E2_ENABLE
  #define enable_e2()  E2_ENABLE_WRITE( E_ENABLE_ON)
  #define disable_e2() E2_ENABLE_WRITE(!E_ENABLE_ON)
#else
  #define enable_e2()  /* nothing */
  #define disable_e2() /* nothing */
#endif

#if (DRIVER_EXTRUDERS > 3) && HAS_E3_ENABLE
  #define enable_e3()  E3_ENABLE_WRITE( E_ENABLE_ON)
  #define disable_e3() E3_ENABLE_WRITE(!E_ENABLE_ON)
#else
  #define enable_e3()  /* nothing */
  #define disable_e3() /* nothing */
#endif

#define disable_e() {disable_e0(); disable_e1(); disable_e2(); disable_e3();}

/**
 * The axis order in all axis related arrays is X, Y, Z, E
 */
#define NUM_AXIS 4

/**
 * Axis indices as enumerated constants
 *
 * A_AXIS and B_AXIS are used by COREXY printers
 * X_HEAD and Y_HEAD is used for systems that don't have a 1:1 relationship between X_AXIS and X Head movement, like CoreXY bots.
 */
enum AxisEnum {X_AXIS=0, A_AXIS=0, Y_AXIS=1, B_AXIS=1, Z_AXIS=2, C_AXIS=2, E_AXIS=3, X_HEAD=4, Y_HEAD=5, Z_HEAD=5};

enum EndstopEnum {X_MIN=0, Y_MIN=1, Z_MIN=2, Z_PROBE=3, X_MAX=4, Y_MAX=5, Z_MAX=6, Z2_MIN=7, Z2_MAX=8};

void enable_all_steppers();
void disable_all_steppers();

void FlushSerialRequestResend();
void ok_to_send();

#if ENABLED(DELTA)
  float probe_bed(float x, float y);
  void set_delta_constants();
  void adj_tower_delta(int tower);
  void adj_tower_radius(int tower);
  void home_delta_axis();
  void calibration_report();
  void bed_probe_all();
  void set_default_z_probe_offset();
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
#if ENABLED(SCARA)
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
  DEBUG_ECHO          = BIT(0),
  DEBUG_INFO          = BIT(1),
  DEBUG_ERRORS        = BIT(2),
  DEBUG_DRYRUN        = BIT(3),
  DEBUG_COMMUNICATION = BIT(4)
};
extern uint8_t debugLevel;

extern bool Running;
inline bool IsRunning() { return  Running; }
inline bool IsStopped() { return !Running; }

bool enqueuecommand(const char *cmd); //put a single ASCII command at the end of the current buffer or return false when it is full
void enqueuecommands_P(const char *cmd); //put one or many ASCII commands at the end of the current buffer, read from flash

void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);

extern millis_t previous_cmd_ms;
inline void refresh_cmd_timeout() { previous_cmd_ms = millis(); }

#if ENABLED(FAST_PWM_FAN)
  void setPwmFrequency(uint8_t pin, int val);
#endif

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif

extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedrate_multiplier;
extern bool volumetric_enabled;
extern int extruder_multiplier[EXTRUDERS]; // sets extrude multiply factor (in percent) for each extruder individually
extern float filament_size[EXTRUDERS]; // cross-sectional area of filament (in millimeters), typically around 1.75 or 2.85, 0 disables the volumetric calculations for the extruder.
extern float volumetric_multiplier[EXTRUDERS]; // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner
extern float current_position[NUM_AXIS];
extern float destination[NUM_AXIS];
extern float home_offset[3];

// Hotend offset
#if HOTENDS > 1
  #ifndef DUAL_X_CARRIAGE
    #define NUM_HOTEND_OFFSETS 2 // only in XY plane
  #else
    #define NUM_HOTEND_OFFSETS 3 // supports offsets in XYZ plane
  #endif
  extern float hotend_offset[NUM_HOTEND_OFFSETS][HOTENDS];
#endif // HOTENDS > 1

#if ENABLED(NPR2)
  extern int old_color; // old color for system NPR2
#endif

#if ENABLED(DELTA)
  extern float z_probe_offset[3];
  extern float endstop_adj[3];
  extern float tower_adj[6];
  extern float delta_radius;
  extern float delta_diagonal_rod;
  extern float delta_segments_per_second;
#elif ENABLED(Z_DUAL_ENDSTOPS)
  extern float z_endstop_adj;
#endif

#if ENABLED(SCARA)
  extern float axis_scaling[3];  // Build size scaling
#endif

extern float min_pos[3];
extern float max_pos[3];
extern bool axis_known_position[3];
extern float zprobe_zoffset;

// Lifetime stats
extern unsigned long printer_usage_seconds;  //this can old about 136 year before go overflow. If you belive that you can live more than this please contact me.

#if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
  extern float extrude_min_temp;
#endif

extern int fanSpeed;

#if ENABLED(BARICUDA)
  extern int ValvePressure;
  extern int EtoPPressure;
#endif

#if ENABLED(FAN_SOFT_PWM)
  extern unsigned char fanSpeedSoftPwm;
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

#if HAS_POWER_CONSUMPTION_SENSOR
  extern float power_consumption_meas;          //holds the power consumption as accurately measured
  extern unsigned long power_consumption_hour;  //holds the power consumption per hour as accurately measured
  extern unsigned long startpower;
  extern unsigned long stoppower;
#endif

#if ENABLED(IDLE_OOZING_PREVENT)
  extern bool IDLE_OOZING_enabled;
  extern void IDLE_OOZING_retract(bool retracting);
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

extern millis_t print_job_start_ms;
extern millis_t print_job_stop_ms;

// Handling multiple extruders pins
extern uint8_t active_extruder;
extern uint8_t active_driver;

#if ENABLED(DIGIPOT_I2C)
  extern void digipot_i2c_set_current( int channel, float current );
  extern void digipot_i2c_init();
#endif

#if ENABLED(FIRMWARE_TEST)
  void FirmwareTest();
#endif

extern void calculate_volumetric_multipliers();

#endif //MARLIN_H
