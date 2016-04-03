#ifndef CONFIGURATION_MECHANISM
#define CONFIGURATION_MECHANISM
#define KNOWN_MECH
/*
 * This configuration file contains mechanism settings for cartesian printer.
 *
 * - Machine name
 * - Delta settings
 * - Z probe endstop
 * - Endstop pullup resistors
 * - Endstops logic
 * - Endstops min or max
 * - Stepper enable logic
 * - Stepper step logic
 * - Stepper direction
 * - Disables axis
 * - Manual home positions
 * - Travel limits
 * - Axis relative mode
 * - Axis steps per unit
 * - Axis feedrate
 * - Axis accelleration
 * - Homing feedrate
 * - Hotend offset
 *
 * Basic-settings can be found in Configuration_Basic.h
 * Feature-settings can be found in Configuration_Feature.h
 * Pins-settings can be found in "Configuration_Pins.h"
 */
 
/*****************************************************************************************
 *********************************** Machine name ****************************************
 *****************************************************************************************
 *                                                                                       *
 * This to set a custom name for your generic Mendel.                                    *
 * Displayed in the LCD "Ready" message.                                                 *
 *                                                                                       *
 *****************************************************************************************/
#define CUSTOM_MACHINE_NAME "Prusa"
/*****************************************************************************************/


/*****************************************************************************************
 ******************************** Delta configuration ************************************
 ****************************************************************************************/
// Make delta curves from many straight lines (linear interpolation).
// This is a trade-off between visible corners (not enough segments)
// and processor overload (too many expensive sqrt calls).
// The new function do not use segments per second but segments per mm
// if you want use new function comment this (using // at the start of the line)
#define DELTA_SEGMENTS_PER_SECOND 200

// Center-to-center distance of the holes in the diagonal push rods.
#define DEFAULT_DELTA_DIAGONAL_ROD 220.0    // mm

// Horizontal offset from middle of printer to smooth rod center.
#define DELTA_SMOOTH_ROD_OFFSET 150.0       // mm

// Horizontal offset of the universal joints on the end effector.
#define DELTA_EFFECTOR_OFFSET 20.0          // mm

// Horizontal offset of the universal joints on the carriages.
#define DELTA_CARRIAGE_OFFSET 20.0          // mm

// Bed Printer radius
#define BED_PRINTER_RADIUS 75               // mm

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
/*****************************************************************************************/


/*****************************************************************************************
 ******************************* Z probe endstop *****************************************
 *****************************************************************************************
 *                                                                                       *
 * If you enabled Z_PROBE_ENDSTOP this add the support for autocalibration               *
 * and auto bed level                                                                    *
 * To use Z PROBE endstop, you must have a Z_PROBE_PIN                                   *
 * defined in the Configuration_pins.h file for your control board.                      *
 * If you are using a servo based Z PROBE, you will need to enable                       *
 * NUM_SERVOS, SERVO_ENDSTOPS and SERVO_ENDSTOPS_ANGLES in                               *
 * Configuration_Feature R/C Servo section.                                              *
 *                                                                                       *
 * WARNING: Setting the wrong pin may have unexpected and potentially                    *
 * disastrous outcomes. Use with caution and do your homework.                           *
 *                                                                                       *
 * Uncomment Z_PROBE_ENDSTOP to enable.                                                  *
 *                                                                                       *
 *****************************************************************************************/
//#define Z_PROBE_ENDSTOP

// Speed for autocalibration travel and probing moves
#define AUTOCAL_TRAVELRATE 100  // mm/sec
#define AUTOCAL_PROBERATE   50  // mm/sec

// Precision for G30 delta autocalibration function
#define AUTOCALIBRATION_PRECISION 0.1 // mm

// Precision probe. Number of probe for the mean
#define PROBE_COUNT 3

// Z-Probe variables
// Offsets to the probe relative to the extruder tip (Hotend - Probe)
// X and Y offsets MUST be INTEGERS
//
//    +-- BACK ---+
//    |           |
//  L |    (+) P  | R <-- probe (10,10)
//  E |           | I
//  F | (-) N (+) | G <-- nozzle (0,0)
//  T |           | H
//    |  P (-)    | T <-- probe (-10,-10)
//    |           |
//    O-- FRONT --+
#define X_PROBE_OFFSET_FROM_EXTRUDER  0     // X offset: -left  [of the nozzle] +right
#define Y_PROBE_OFFSET_FROM_EXTRUDER  0     // Y offset: -front [of the nozzle] +behind
#define Z_PROBE_OFFSET_FROM_EXTRUDER -1     // Z offset: -below [of the nozzle] (always negative!)

// Start and end location values are used to deploy/retract the probe (will move from start to end and back again)
#define Z_PROBE_DEPLOY_START_LOCATION {0, 0, 20}   // X, Y, Z, E start location for z-probe deployment sequence
#define Z_PROBE_DEPLOY_END_LOCATION {0, 0, 20}     // X, Y, Z, E end location for z-probe deployment sequence
#define Z_PROBE_RETRACT_START_LOCATION {0, 0, 20}  // X, Y, Z, E start location for z-probe retract sequence
#define Z_PROBE_RETRACT_END_LOCATION {0, 0, 20}    // X, Y, Z, E end location for z-probe retract sequence

// How much the nozzle will be raised when travelling from between next probing points
#define Z_RAISE_BETWEEN_PROBINGS 5

// Define the grid for bed level AUTO BED LEVELING GRID POINTS X AUTO BED LEVELING GRID POINTS.
#define AUTO_BED_LEVELING_GRID_POINTS 9

/*****************************************************************************************/


/*****************************************************************************************
 ************************* Endstop pullup resistors **************************************
 *****************************************************************************************
 *                                                                                       *
 * Comment this out (using // at the start of the line) to                               *
 * disable the endstop pullup resistors                                                  *
 *                                                                                       *
 *****************************************************************************************/
#define ENDSTOPPULLUPS

#if DISABLED(ENDSTOPPULLUPS)
// fine endstop settings: Individual pullups. will be ignored if ENDSTOPPULLUPS is defined
//#define ENDSTOPPULLUP_XMIN
//#define ENDSTOPPULLUP_YMIN
//#define ENDSTOPPULLUP_ZMIN
//#define ENDSTOPPULLUP_Z2MIN
//#define ENDSTOPPULLUP_XMAX
//#define ENDSTOPPULLUP_YMAX
//#define ENDSTOPPULLUP_ZMAX
//#define ENDSTOPPULLUP_Z2MAX
//#define ENDSTOPPULLUP_ZPROBE
//#define ENDSTOPPULLUP_EMIN
#endif
/*****************************************************************************************/


/*****************************************************************************************
 ************************************ Endstops logic *************************************
 *****************************************************************************************
 *                                                                                       *
 * Mechanical endstop with COM to ground and NC to Signal                                *
 * uses "false" here (most common setup).                                                *
 *                                                                                       *
 *****************************************************************************************/
#define X_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Y_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z2_MIN_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define X_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Y_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z_MAX_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
#define Z2_MAX_ENDSTOP_LOGIC  false   // set to true to invert the logic of the endstop.
#define Z_PROBE_ENDSTOP_LOGIC false   // set to true to invert the logic of the endstop.
#define E_MIN_ENDSTOP_LOGIC   false   // set to true to invert the logic of the endstop.
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Endstops min or max **********************************
 *****************************************************************************************
 *                                                                                       *
 * Sets direction of endstop when homing; 1=MAX, -1=MIN                                  *
 *                                                                                       *
 *****************************************************************************************/
#define X_HOME_DIR 1 // DELTA MUST HAVE MAX ENDSTOP
#define Y_HOME_DIR 1 // DELTA MUST HAVE MAX ENDSTOP
#define Z_HOME_DIR 1 // DELTA MUST HAVE MAX ENDSTOP
#define E_HOME_DIR -1
/*****************************************************************************************/


/*****************************************************************************************
 ********************************* Stepper enable logic **********************************
 *****************************************************************************************
 *                                                                                       *
 * For Inverting Stepper Enable Pins                                                     *
 * (Active Low) use 0                                                                    *
 * Non Inverting (Active High) use 1                                                     *
 *                                                                                       *
 *****************************************************************************************/
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0      // For all extruder
/*****************************************************************************************/


/*****************************************************************************************
 ********************************* Stepper step logic **********************************
 *****************************************************************************************
 *                                                                                       *
 * By default pololu step drivers require an active high signal.                         *
 * However, some high power drivers require an active low signal as step.                *
 *                                                                                       *
 *****************************************************************************************/
#define INVERT_X_STEP_PIN false
#define INVERT_Y_STEP_PIN false
#define INVERT_Z_STEP_PIN false
#define INVERT_E_STEP_PIN false
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Stepper direction ************************************
 *****************************************************************************************
 *                                                                                       *
 * Invert the stepper direction.                                                         *
 * Change (or reverse the motor connector) if an axis goes the wrong way.                *
 *                                                                                       *
 *****************************************************************************************/
#define INVERT_X_DIR false
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false
#define INVERT_E0_DIR false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
#define INVERT_E4_DIR false
#define INVERT_E5_DIR false
/*****************************************************************************************/


/*****************************************************************************************
 ************************************* Disables axis *************************************
 *****************************************************************************************
 *                                                                                       *
 * Disables axis when it's not being used.                                               *
 *                                                                                       *
 *****************************************************************************************/
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false      // For all extruder
// Disable only inactive extruder and keep active extruder enabled
#define DISABLE_INACTIVE_EXTRUDER false
/*****************************************************************************************/


/*****************************************************************************************
 ******************************** Manual home positions **********************************
 *****************************************************************************************/
// The position of the homing switches
#define MANUAL_HOME_POSITIONS   // If defined, MANUAL_*_HOME_POS below will be used
#define BED_CENTER_AT_0_0       // If defined, the center of the bed is at (X=0, Y=0)

//Manual homing switch locations:
#define MANUAL_X_HOME_POS 0
#define MANUAL_Y_HOME_POS 0
#define MANUAL_Z_HOME_POS 200      // Distance between nozzle and print surface after homing.
/*****************************************************************************************/


/*****************************************************************************************
 ************************************ Travel limits **************************************
 *****************************************************************************************
 *                                                                                       *
 * Travel limits after homing (units are in mm)                                          *
 *                                                                                       *
 *****************************************************************************************/
#define X_MAX_POS BED_PRINTER_RADIUS
#define X_MIN_POS -BED_PRINTER_RADIUS
#define Y_MAX_POS BED_PRINTER_RADIUS
#define Y_MIN_POS -BED_PRINTER_RADIUS
#define Z_MAX_POS MANUAL_Z_HOME_POS
#define Z_MIN_POS 0
#define E_MIN_POS 0
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Axis relative mode ***********************************
 *****************************************************************************************/
#define AXIS_RELATIVE_MODES {false, false, false, false}
/*****************************************************************************************/


/*****************************************************************************************
 ******************************* Axis steps per unit *************************************
 *****************************************************************************************/
// Default steps per unit               X,  Y,  Z,  E0...(per extruder)
#define DEFAULT_AXIS_STEPS_PER_UNIT   {80, 80, 80, 625, 625, 625, 625}
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Axis feedrate ****************************************
 *****************************************************************************************/
//                                       X,   Y,   Z,  E0...(per extruder). (mm/sec)
#define DEFAULT_MAX_FEEDRATE          {500, 500, 500, 100, 100, 100, 100}
// Feedrates for manual moves along        X,     Y,     Z,  E from panel
#define MANUAL_FEEDRATE               {50*60, 50*60, 50*60, 60}
#define DEFAULT_MINIMUMFEEDRATE       0.0                       // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0
// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED         0.05                      // (mm/sec)
/*****************************************************************************************/


/*****************************************************************************************
 ******************************** Axis accelleration *************************************
 *****************************************************************************************/
//  Maximum start speed for accelerated moves.    X,    Y,    Z,   E0...(per extruder)
#define DEFAULT_MAX_ACCELERATION              {5000, 5000, 5000, 1000, 1000, 1000, 1000}
//  Maximum acceleration in mm/s^2 for retracts   E0... (per extruder)
#define DEFAULT_RETRACT_ACCELERATION          {10000, 10000, 10000, 10000}
//  X, Y, Z and E* maximum acceleration in mm/s^2 for printing moves
#define DEFAULT_ACCELERATION          3000
//  X, Y, Z acceleration in mm/s^2 for travel (non printing) moves
#define DEFAULT_TRAVEL_ACCELERATION   3000
/*****************************************************************************************/


/*****************************************************************************************
 ************************************* Axis jerk *****************************************
 *****************************************************************************************
 *                                                                                       *
 * The speed change that does not require acceleration.                                  *
 * (i.e. the software might assume it can be done instantaneously)                       *
 *                                                                                       *
 *****************************************************************************************/
#define DEFAULT_XYJERK 20.0   // (mm/sec)
#define DEFAULT_ZJERK  20.0   // (mm/sec)
//  max initial speed for retract moves   E0... (mm/sec) per extruder
#define DEFAULT_EJERK                   {5.0, 5.0, 5.0, 5.0}
/*****************************************************************************************/


/*****************************************************************************************
 ************************************ Homing feedrate ************************************
 *****************************************************************************************/
// set the homing speeds (mm/min)       X,      Y,      Z
#define HOMING_FEEDRATE           {100*60, 100*60, 100*60, 0}
// homing hits the endstop, then retracts by this distance, before it tries to slowly bump again:
#define XYZ_HOME_BUMP_MM 5
// Re-Bump Speed Divisor (Divides the Homing Feedrate)
#define XYZ_BUMP_DIVISOR 10
/*****************************************************************************************/


/*****************************************************************************************
 *********************************** Hotend offset ***************************************
 *****************************************************************************************
 *                                                                                       *
 * Offset of the hotends (uncomment if using more than one and relying on firmware       *
 * to position when changing).                                                           *
 * The offset has to be X=0, Y=0, Z=0 for the hotend 0 (default hotend).                 *
 * For the other hotends it is their distance from the hotend 0.                         *
 *                                                                                       *
 *****************************************************************************************/
#define HOTEND_OFFSET_X {0.0, 0.0, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the X axis
#define HOTEND_OFFSET_Y {0.0, 0.0, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the Y axis
#define HOTEND_OFFSET_Z {0.0, 0.0, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the Z axis
/*****************************************************************************************/

#endif