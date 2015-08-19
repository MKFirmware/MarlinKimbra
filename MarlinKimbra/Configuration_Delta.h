// Define this to set a custom name for your generic Delta
// Displayed in the LCD "Ready" message
#define CUSTOM_MACHINE_NAME "Delta"

//===========================================================================
//============================== Delta Settings =============================
//===========================================================================

// Make delta curves from many straight lines (linear interpolation).
// This is a trade-off between visible corners (not enough segments)
// and processor overload (too many expensive sqrt calls).
// The new function do not use segments per second but segments per mm
// if you want use new function comment this (using // at the start of the line)
#define DELTA_SEGMENTS_PER_SECOND 150

// Center-to-center distance of the holes in the diagonal push rods.
#define DEFAULT_DELTA_DIAGONAL_ROD 217.0    // mm

// Horizontal offset from middle of printer to smooth rod center.
#define DELTA_SMOOTH_ROD_OFFSET 145.0       // mm

// Horizontal offset of the universal joints on the end effector.
#define DELTA_EFFECTOR_OFFSET 20.0          // mm

// Horizontal offset of the universal joints on the carriages.
#define DELTA_CARRIAGE_OFFSET 20.0          // mm

// Bed Printer radius
#define PRINTER_RADIUS 75                   // mm

// Radius for probe
#define DELTA_PROBABLE_RADIUS (PRINTER_RADIUS)

// Effective horizontal distance bridged by diagonal push rods.
#define DEFAULT_DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET-DELTA_EFFECTOR_OFFSET-DELTA_CARRIAGE_OFFSET)

// Uncomment to enable autocalibration debug messages
#define DEBUG_MESSAGES

// Speed for autocalibration travel and probing moves
#define AUTOCAL_TRAVELRATE 100  // mm/sec
#define AUTOCAL_PROBERATE   50  // mm/sec

// Precision for G30 delta autocalibration function
#define AUTOCALIBRATION_PRECISION 0.1 // mm

//Endstop Offset Adjustment - All values are in mm and must be negative (to move down away from endstop switches) 
#define TOWER_A_ENDSTOP_ADJ 0 // Front Left Tower
#define TOWER_B_ENDSTOP_ADJ 0 // Front Right Tower
#define TOWER_C_ENDSTOP_ADJ 0 // Rear Tower

//Tower Position Adjustment - Adj x Degrees around delta radius (- move clockwise / + move anticlockwise)
#define TOWER_A_POSITION_ADJ 0 //Front Left Tower
#define TOWER_B_POSITION_ADJ 0 //Front Right Tower
#define TOWER_C_POSITION_ADJ 0 //Rear Tower

//Tower Radius Adjustment - Adj x mm in/out from centre of printer (- move in / + move out)
#define TOWER_A_RADIUS_ADJ 0 //Front Left Tower
#define TOWER_B_RADIUS_ADJ 0 //Front Right Tower
#define TOWER_C_RADIUS_ADJ 0 //Rear Tower

//Diagonal Rod Adjustment - Adj diag rod for Tower by x mm from DEFAULT_DELTA_DIAGONAL_ROD value
#define TOWER_A_DIAGROD_ADJ 0 //Front Left Tower
#define TOWER_B_DIAGROD_ADJ 0 //Front Right Tower
#define TOWER_C_DIAGROD_ADJ 0 //Rear Tower

// Z-Probe variables
// Start and end location values are used to deploy/retract the probe (will move from start to end and back again)
#define Z_PROBE_OFFSET {0, 0, -1}                  // X, Y, Z, E distance between hotend nozzle and deployed bed leveling probe.
#define Z_PROBE_DEPLOY_START_LOCATION {0, 0, 20}   // X, Y, Z, E start location for z-probe deployment sequence
#define Z_PROBE_DEPLOY_END_LOCATION {0, 0, 20}     // X, Y, Z, E end location for z-probe deployment sequence
#define Z_PROBE_RETRACT_START_LOCATION {0, 0, 20}  // X, Y, Z, E start location for z-probe retract sequence
#define Z_PROBE_RETRACT_END_LOCATION {0, 0, 20}    // X, Y, Z, E end location for z-probe retract sequence
#define Z_RAISE_BETWEEN_PROBINGS 5                 // How much the nozzle will be raised when travelling from between next probing points
#define AUTO_BED_LEVELING_GRID_POINTS 9            // Works best with ACCURATE_BED_LEVELING_POINTS 5 or higher.

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// coarse Endstop Settings
#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

#if DISABLED(ENDSTOPPULLUPS)
  // fine endstop settings: Individual pullups. will be ignored if ENDSTOPPULLUPS is defined
  //#define ENDSTOPPULLUP_XMIN
  //#define ENDSTOPPULLUP_YMIN
  //#define ENDSTOPPULLUP_ZMIN
  //#define ENDSTOPPULLUP_XMAX
  //#define ENDSTOPPULLUP_YMAX
  //#define ENDSTOPPULLUP_ZMAX
  //#define ENDSTOPPULLUP_ZPROBE
  //#define ENDSTOPPULLUP_EMIN
#endif

// Mechanical endstop with COM to ground and NC to Signal uses "false" here (most common setup).
#define X_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Y_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define E_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define X_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Y_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z_PROBE_ENDSTOP_LOGIC false   // set to true to invert the logic of the endstop.
// If you want to enable the Z Probe pin, but disable its use, uncomment the line below.
// Z_PROBE_ENDSTOP must are active if you want Autocalibration
#define Z_PROBE_ENDSTOP

// ENDSTOP SETTINGS:
// Sets direction of endstop when homing; 1=MAX, -1=MIN
#define X_HOME_DIR 1 // DELTA MUST HAVE MAX ENDSTOP
#define Y_HOME_DIR 1 // DELTA MUST HAVE MAX ENDSTOP
#define Z_HOME_DIR 1 // DELTA MUST HAVE MAX ENDSTOP

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
#define MANUAL_HOME_POSITIONS   // If defined, MANUAL_*_HOME_POS below will be used
#define BED_CENTER_AT_0_0       // If defined, the center of the bed is at (X=0, Y=0)

//Manual homing switch locations:
#define MANUAL_X_HOME_POS 0
#define MANUAL_Y_HOME_POS 0
#define MANUAL_Z_HOME_POS 200      // Distance between nozzle and print surface after homing.

// Travel limits after homing (units are in mm)
#define X_MAX_POS PRINTER_RADIUS
#define X_MIN_POS -PRINTER_RADIUS
#define Y_MAX_POS PRINTER_RADIUS
#define Y_MIN_POS -PRINTER_RADIUS
#define Z_MAX_POS MANUAL_Z_HOME_POS
#define Z_MIN_POS 0
#define E_MIN_POS 0

#define DELTA_DIAGONAL_X_CORRECTION 1.0 // front left tower
#define DELTA_DIAGONAL_Y_CORRECTION 1.0 // front right tower
#define DELTA_DIAGONAL_Z_CORRECTION 1.0 // back middle tower

// MOVEMENT SETTINGS
#define HOMING_FEEDRATE {100*60, 100*60, 100*60, 0}      // set the homing speeds (mm/min)

// default settings
// delta speeds must be the same on xyz
#define DEFAULT_AXIS_STEPS_PER_UNIT   {80,80,80,451,625,625,625}            // X, Y, Z, E0...(per extruder). Default steps per unit
#define DEFAULT_MAX_FEEDRATE          {500,500,500,45,45,45,45}             // X, Y, Z, E0...(per extruder). (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {5000,5000,5000,1000,1000,1000,1000}  // X, Y, Z, E0...(per extruder). Maximum start speed for accelerated moves.
#define DEFAULT_RETRACT_ACCELERATION  {10000,10000,10000,10000}             // E0... (per extruder) max acceleration in mm/s^2 for retracts
#define DEFAULT_ACCELERATION            3000                                // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_TRAVEL_ACCELERATION     3000                                // X, Y, Z acceleration in mm/s^2 for travel (non printing) moves

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the hotend 0 (default hotend).
// For the other hotends it is their distance from the hotend 0.
//#define HOTEND_OFFSET_X {0.0, 5.00, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the X axis
//#define HOTEND_OFFSET_Y {0.0, 5.00, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the Y axis

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK 20                   // (mm/sec)
#define DEFAULT_ZJERK  20                   // (mm/sec)
#define DEFAULT_EJERK  {5.0,5.0,5.0,5.0}    // E0... (mm/sec) per extruder, max initial speed for retract moves
