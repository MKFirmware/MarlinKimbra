// Define this to set a custom name for your generic Delta
#define CUSTOM_MENDEL_NAME "Delta"

//===========================================================================
//============================== Delta Settings =============================
//===========================================================================

// Make delta curves from many straight lines (linear interpolation).
// This is a trade-off between visible corners (not enough segments)
// and processor overload (too many expensive sqrt calls).
#define DELTA_SEGMENTS_PER_SECOND 200

// Center-to-center distance of the holes in the diagonal push rods.
#define DEFAULT_DELTA_DIAGONAL_ROD 217.0      // mm

// Horizontal offset from middle of printer to smooth rod center.
#define DELTA_SMOOTH_ROD_OFFSET 145.0      // mm

// Horizontal offset of the universal joints on the end effector.
#define DELTA_EFFECTOR_OFFSET 20.0      // mm

// Horizontal offset of the universal joints on the carriages.
#define DELTA_CARRIAGE_OFFSET 20.0      // mm

// Bed Printer radius
#define PRINTER_RADIUS 70     // mm

//Uncomment to enable autocalibration debug messages
#define DEBUG_MESSAGES

// Precision for G30 delta autocalibration function
#define AUTOCALIBRATION_PRECISION 0.1      // mm

// Effective horizontal distance bridged by diagonal push rods.
#define DEFAULT_DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET-DELTA_EFFECTOR_OFFSET-DELTA_CARRIAGE_OFFSET)

// Z-Probe variables
// Start and end location values are used to deploy/retract the probe (will move from start to end and back again)
#define PROBING_FEEDRATE 500                     // Speed for individual probe Use: G30 A F600
#define Z_PROBE_OFFSET {0,0,-1,0}                // X, Y, Z, E distance between hotend nozzle and deployed bed leveling probe.
#define Z_PROBE_DEPLOY_START_LOCATION {0,0,30,0} // X, Y, Z, E start location for z-probe deployment sequence
#define Z_PROBE_DEPLOY_END_LOCATION {0,0,30,0}   // X, Y, Z, E end location for z-probe deployment sequence
#define Z_PROBE_RETRACT_START_LOCATION {0,0,30,0}// X, Y, Z, E start location for z-probe retract sequence
#define Z_PROBE_RETRACT_END_LOCATION {0,0,30,0}  // X, Y, Z, E end location for z-probe retract sequence
#define AUTOLEVEL_GRID 20                        // Distance between autolevel Z probing points, should be less than print surface radius/3.

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// coarse Endstop Settings
#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

#ifndef ENDSTOPPULLUPS
  // fine endstop settings: Individual pullups. will be ignored if ENDSTOPPULLUPS is defined
  // #define ENDSTOPPULLUP_XMAX
  // #define ENDSTOPPULLUP_YMAX
  // #define ENDSTOPPULLUP_ZMAX
  // #define ENDSTOPPULLUP_XMIN
  // #define ENDSTOPPULLUP_YMIN
  // #define ENDSTOPPULLUP_ZMIN
  // #define ENDSTOPPULLUP_EMIN
#endif

#ifdef ENDSTOPPULLUPS
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
  #define ENDSTOPPULLUP_EMIN
#endif

// The pullups are needed if you directly connect a mechanical end switch between the signal and ground pins.
const bool X_MIN_ENDSTOP_INVERTING = false;     // set to true to invert the logic of the endstop.
const bool Y_MIN_ENDSTOP_INVERTING = false;     // set to true to invert the logic of the endstop.
const bool Z_MIN_ENDSTOP_INVERTING = false;     // set to true to invert the logic of the endstop.
const bool E_MIN_ENDSTOP_INVERTING = false;     // set to true to invert the logic of the endstop.
const bool X_MAX_ENDSTOP_INVERTING = false;     // set to true to invert the logic of the endstop.
const bool Y_MAX_ENDSTOP_INVERTING = false;     // set to true to invert the logic of the endstop.
const bool Z_MAX_ENDSTOP_INVERTING = false;     // set to true to invert the logic of the endstop.

// ENDSTOP SETTINGS:
// Sets direction of endstop when homing; 1=MAX, -1=MIN
#define X_HOME_DIR 1 //DELTA MUST HAVE MAX ENDSTOP
#define Y_HOME_DIR 1 //DELTA MUST HAVE MAX ENDSTOP
#define Z_HOME_DIR 1 //DELTA MUST HAVE MAX ENDSTOP

#define min_software_endstops true  // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  // If true, axis won't move to coordinates greater than the defined lengths below.


// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0      // For all extruder

// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false      // For all extruder
#define DISABLE_INACTIVE_EXTRUDER false //disable only inactive extruder and keep active extruder enabled

// If you motor turns to wrong direction, you can invert it here:
#define INVERT_X_DIR false
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false
#define INVERT_E0_DIR false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false

// The position of the homing switches
#define MANUAL_HOME_POSITIONS  // If defined, MANUAL_*_HOME_POS below will be used
#define BED_CENTER_AT_0_0      // If defined, the center of the bed is at (X=0, Y=0)

//Manual homing switch locations:
#define MANUAL_X_HOME_POS 0
#define MANUAL_Y_HOME_POS 0
#define MANUAL_Z_HOME_POS 205      // Distance between nozzle and print surface after homing.

// Travel limits after homing
#define X_MAX_POS PRINTER_RADIUS
#define X_MIN_POS -PRINTER_RADIUS
#define Y_MAX_POS PRINTER_RADIUS
#define Y_MIN_POS -PRINTER_RADIUS
#define Z_MAX_POS MANUAL_Z_HOME_POS
#define Z_MIN_POS 0
#define E_MIN_POS 0

#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)

//// MOVEMENT SETTINGS
#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E
#define HOMING_FEEDRATE {100*60,100*60,100*60,0}      // set the homing speeds (mm/min)

// default settings
// delta speeds must be the same on xyz
#define DEFAULT_AXIS_STEPS_PER_UNIT {80,80,80,451,625,625,625}        // X, Y, Z, E0, E1, E2, E3
#define DEFAULT_MAX_FEEDRATE {300,300,300,45,100,100,100}             // X, Y, Z, E0, E1, E2, E3 (mm/sec)
#define DEFAULT_RETRACTION_MAX_FEEDRATE {150,150,150,150}             // E0, E1, E2, E3 (mm/sec)
#define DEFAULT_MAX_ACCELERATION {2000,2000,2000,1000,1000,1000,1000} // X, Y, Z, E0, E1, E2, E3 maximum start speed for accelerated moves.

#define DEFAULT_ACCELERATION          1000      // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  2500      // X, Y, Z and E max acceleration in mm/s^2 for retracts
#define DEFAULT_TRAVEL_ACCELERATION   3000      // X, Y, Z acceleration in mm/s^2 for travel (non printing) moves

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK 20  // (mm/sec)
#define DEFAULT_ZJERK  20  // (mm/sec)
#define DEFAULT_EJERK   5  // (mm/sec)
