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
  #include "Fastio_sam.h"
#else
  #include <util/delay.h>
  #include <avr/eeprom.h>
  #include "fastio.h"
#endif

#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "Configuration.h"

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#define BIT(b) (1<<(b))
#define TEST(n,b) (((n)&BIT(b))!=0)
#define RADIANS(d) ((d)*M_PI/180.0)
#define DEGREES(r) ((d)*180.0/M_PI)
#define NOLESS(v,n) do{ if (v < n) v = n; }while(0)
#define NOMORE(v,n) do{ if (v > n) v = n; }while(0)

typedef unsigned long millis_t;

// Arduino < 1.0.0 does not define this, so we need to do it ourselves
#ifndef analogInputToDigitalPin
  #define analogInputToDigitalPin(p) ((p) + 0xA0)
#endif

#include "comunication.h"

void get_command();
void process_commands();

void manage_inactivity(bool ignore_stepper_queue=false);

#if defined(DUAL_X_CARRIAGE) && HAS_X_ENABLE && HAS_X2_ENABLE
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
  #ifdef Y_DUAL_STEPPER_DRIVERS
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
  #ifdef Z_DUAL_STEPPER_DRIVERS
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
enum AxisEnum {X_AXIS=0, Y_AXIS=1, A_AXIS=0, B_AXIS=1, Z_AXIS=2, E_AXIS=3, X_HEAD=4, Y_HEAD=5};

void enable_all_steppers();
void disable_all_steppers();

void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
#ifdef DELTA
float probe_bed(float x, float y);
void set_delta_constants();
void home_delta_axis();
void calibration_report();
void bed_probe_all();
void set_default_z_probe_offset();
void set_delta_constants();
void save_carriage_positions(int position_num);
void calculate_delta(float cartesian[3]);
void adjust_delta(float cartesian[3]);
void prepare_move_raw();
extern float delta[3];
extern float delta_tmp[3];
extern float delta_tower1_x,delta_tower1_y;
extern float delta_tower2_x,delta_tower2_y;
extern float delta_tower3_x,delta_tower3_y;
#endif
#ifdef SCARA
  void calculate_delta(float cartesian[3]);
  void calculate_SCARA_forward_Transform(float f_scara[3]);
#endif
void prepare_move();
void kill();
void Stop();

#ifdef FILAMENT_RUNOUT_SENSOR
  void filrunout();
#endif

extern bool Running;
inline bool IsRunning() { return  Running; }
inline bool IsStopped() { return !Running; }

bool enqueuecommand(const char *cmd); //put a single ASCII command at the end of the current buffer or return false when it is full
void enqueuecommands_P(const char *cmd); //put one or many ASCII commands at the end of the current buffer, read from flash

void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);

extern millis_t previous_cmd_ms;
inline void refresh_cmd_timeout() { previous_cmd_ms = millis(); }

#ifdef FAST_PWM_FAN
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
extern int extruder_multiply[EXTRUDERS]; // sets extrude multiply factor (in percent) for each extruder individually
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

#ifdef NPR2
  extern int old_color; // old color for system NPR2
#endif

#ifdef DELTA
  extern float z_probe_offset[3];
  extern float endstop_adj[3];
  extern float tower_adj[6];
  extern float delta_radius;
  extern float delta_diagonal_rod;
#elif defined(Z_DUAL_ENDSTOPS)
  extern float z_endstop_adj;
#endif

#ifdef SCARA
  extern float axis_scaling[3];  // Build size scaling
#endif

extern float min_pos[3];
extern float max_pos[3];
extern bool axis_known_position[3];
extern float lastpos[4];
extern float zprobe_zoffset;

// Lifetime stats
extern unsigned long printer_usage_seconds;  //this can old about 136 year before go overflow. If you belive that you can live more than this please contact me.
extern millis_t config_last_update;

#ifdef PREVENT_DANGEROUS_EXTRUDE
  extern float extrude_min_temp;
#endif

extern int fanSpeed;

#ifdef BARICUDA
  extern int ValvePressure;
  extern int EtoPPressure;
#endif

#ifdef FAN_SOFT_PWM
  extern unsigned char fanSpeedSoftPwm;
#endif

#if HAS_FILAMENT_SENSOR
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

#ifdef IDLE_OOZING_PREVENT
  extern bool idleoozing_enabled;
#endif

#ifdef FWRETRACT
  extern bool autoretract_enabled;
  extern bool retracted[EXTRUDERS];
  extern float retract_length, retract_length_swap, retract_feedrate, retract_zlift;
  extern float retract_recover_length, retract_recover_length_swap, retract_recover_feedrate;
#endif

#ifdef EASY_LOAD
  extern bool allow_lengthy_extrude_once; // for load/unload
#endif

#ifdef LASERBEAM
  extern int laser_ttl_modulation;
#endif

extern millis_t print_job_start_ms;
extern millis_t print_job_stop_ms;

// Handling multiple extruders pins
extern uint8_t active_extruder;
extern uint8_t active_driver;

#ifdef DIGIPOT_I2C
  extern void digipot_i2c_set_current( int channel, float current );
  extern void digipot_i2c_init();
#endif

/**
 * Debug with repetier
 */
enum DebugFlags {
  DEBUG_ECHO          = BIT(0),
  DEBUG_INFO          = BIT(1),
  DEBUG_ERRORS        = BIT(2),
  DEBUG_DRYRUN        = BIT(3),
  DEBUG_COMMUNICATION = BIT(4)
};
extern uint8_t debugLevel;
extern inline bool debugDryrun() {
  return ((debugLevel & 8) != 0);
}

#ifdef FIRMWARE_TEST
  void FirmwareTest();
#endif

extern void calculate_volumetric_multipliers();

#endif //MARLIN_H
