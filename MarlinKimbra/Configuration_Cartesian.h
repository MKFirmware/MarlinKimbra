#ifndef CONFIGURATION_MECHANISM
#define CONFIGURATION_MECHANISM
#define KNOWN_MECH 1
/*
 * This configuration file contains mechanism settings for cartesian printer.
 *
 * - Machine name
 * - Endstop pullup resistors
 * - Endstops logic
 * - Endstops min or max
 * - Stepper enable logic
 * - Stepper step logic
 * - Stepper direction
 * - Disables axis
 * - Travel limits
 * - Axis relative mode
 * - MBL or ABL
 * - Auto bed levelling
 * - Z probe endstop
 * - Safe Z homing
 * - Manual home positions
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
#define CUSTOM_MACHINE_NAME "Prusa I3"
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
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1
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
 ************************************ Travel limits **************************************
 *****************************************************************************************
 *                                                                                       *
 * Travel limits after homing (units are in mm)                                          *
 *                                                                                       *
 *****************************************************************************************/
#define X_MAX_POS 200
#define X_MIN_POS 0
#define Y_MAX_POS 200
#define Y_MIN_POS 0
#define Z_MAX_POS 200
#define Z_MIN_POS 0
#define E_MIN_POS 0
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Axis relative mode ***********************************
 *****************************************************************************************/
#define AXIS_RELATIVE_MODES {false, false, false, false}
/*****************************************************************************************/


/*****************************************************************************************
 *********************************** Safe Z homing ***************************************
 *****************************************************************************************
 *                                                                                       *
 * If you have enabled the auto bed levelling feature or are using                       *
 * Z Probe for Z Homing, it is highly recommended you let                                *
 * this Z_SAFE_HOMING enabled!!!                                                         *
 *                                                                                       *
 * X point for Z homing when homing all axis (G28)                                       *
 * Y point for Z homing when homing all axis (G28)                                       *
 *                                                                                       *
 * Uncomment Z_SAFE_HOMING to enable                                                     *
 *                                                                                       *
 *****************************************************************************************/
//#define Z_SAFE_HOMING
#define Z_SAFE_HOMING_X_POINT ((X_MIN_POS + X_MAX_POS) / 2)
#define Z_SAFE_HOMING_Y_POINT ((Y_MIN_POS + Y_MAX_POS) / 2)
/*****************************************************************************************/


/*****************************************************************************************
 ************************************** MBL or ABL ***************************************
/*****************************************************************************************
 *                                                                                       *
 * Manual Bed Leveling (MBL) or Auto Bed Leveling (ABL) settings                         *
 * Set the rectangle in which to probe in MBL or ABL.                                    *
 *                                                                                       *
 *****************************************************************************************/
#define LEFT_PROBE_BED_POSITION 20
#define RIGHT_PROBE_BED_POSITION 180
#define FRONT_PROBE_BED_POSITION 20
#define BACK_PROBE_BED_POSITION 180

#define XY_TRAVEL_SPEED 10000     // X and Y axis travel speed between probes, in mm/min
/*****************************************************************************************/


/*****************************************************************************************
 ******************************* Auto bed levelling **************************************
 *****************************************************************************************
 *                                                                                       *
 * There are 2 different ways to specify probing locations                               *
 *                                                                                       *
 * - "grid" mode                                                                         *
 *   Probe several points in a rectangular grid.                                         *
 *   You specify the rectangle and the density of sample points.                         *
 *   This mode is preferred because there are more measurements.                         *
 *                                                                                       *
 * - "3-point" mode                                                                      *
 *   Probe 3 arbitrary points on the bed (that aren't colinear)                          *
 *   You specify the XY coordinates of all 3 points.                                     *
 *                                                                                       *
 *                                                                                       *
 * Uncomment AUTO_BED_LEVELING_FEATURE to enable                                         *
 *                                                                                       *
 *****************************************************************************************/
//#define AUTO_BED_LEVELING_FEATURE
//#define Z_PROBE_REPEATABILITY_TEST  // If not commented out, Z-Probe Repeatability test will be included if Auto Bed Leveling is Enabled.

// Enable this to sample the bed in a grid (least squares solution)
// Note: this feature generates 10KB extra code size
#define AUTO_BED_LEVELING_GRID

// yes AUTO_BED_LEVELING_GRID
#define MIN_PROBE_EDGE 10 // The probe square sides can be no smaller than this
// Set the number of grid points per dimension
// You probably don't need more than 3 (squared=9)
#define AUTO_BED_LEVELING_GRID_POINTS 2
// yes AUTO_BED_LEVELING_GRID

// no AUTO_BED_LEVELING_GRID
// Arbitrary points to probe. A simple cross-product
// is used to estimate the plane of the bed.
#define ABL_PROBE_PT_1_X 15
#define ABL_PROBE_PT_1_Y 180
#define ABL_PROBE_PT_2_X 15
#define ABL_PROBE_PT_2_Y 20
#define ABL_PROBE_PT_3_X 170
#define ABL_PROBE_PT_3_Y 20
// no AUTO_BED_LEVELING_GRID

// Offsets to the probe relative to the extruder tip (Hotend - Probe)
// X and Y offsets MUST be INTEGERS
#define X_PROBE_OFFSET_FROM_EXTRUDER 0      // Probe on: -left  +right
#define Y_PROBE_OFFSET_FROM_EXTRUDER 0      // Probe on: -front +behind
#define Z_PROBE_OFFSET_FROM_EXTRUDER -1     // -below (always!)

#define Z_RAISE_BEFORE_HOMING       10      // (in mm) Raise Z before homing (G28) for Probe Clearance.
                                            // Be sure you have this distance over your Z_MAX_POS in case

#define Z_RAISE_BEFORE_PROBING      10      //How much the extruder will be raised before travelling to the first probing point.
#define Z_RAISE_BETWEEN_PROBINGS     5      //How much the extruder will be raised when travelling from between next probing points
#define Z_RAISE_AFTER_PROBING        5      //How much the extruder will be raised after the last probing point.

//#define Z_PROBE_SLED                // turn on if you have a z-probe mounted on a sled like those designed by Charles Bell
//#define SLED_DOCKING_OFFSET 5       // the extra distance the X axis must travel to pick up the sled. 0 should be fine but you can push it further if you'd like.
/*****************************************************************************************/


/*****************************************************************************************
 ******************************* Z probe endstop *****************************************
 *****************************************************************************************
 *                                                                                       *
 * If you have enabled the Auto bed levelling this add the Support for                   *
 * a dedicated Z PROBE endstop separate from the Z MIN endstop.                          *
 * If you would like to use both a Z PROBE and a Z MIN endstop together                  *
 * or just a Z PROBE with a custom pin, uncomment #define Z_PROBE_ENDSTOP                *
 * and read the instructions below.                                                      *
 *                                                                                       *
 * If you want to still use the Z min endstop for homing,                                *
 * disable Z_SAFE_HOMING.                                                                *
 * Eg: to park the head outside the bed area when homing with G28.                       *
 *                                                                                       *
 * WARNING: The Z MIN endstop will need to set properly as it would                      *
 * without a Z PROBE to prevent head crashes and premature stopping                      *
 * during a print.                                                                       *
 * To use a separte Z PROBE endstop, you must have a Z_PROBE_PIN                         *
 * defined in the pins.h file for your control board.                                    *
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
/*****************************************************************************************/


/*****************************************************************************************
 ******************************** Manual home positions **********************************
/*****************************************************************************************/
// The position of the homing switches
//#define MANUAL_HOME_POSITIONS  // If defined, MANUAL_*_HOME_POS below will be used
//#define BED_CENTER_AT_0_0  // If defined, the center of the bed is at (X=0, Y=0)

//Manual homing switch locations:
#define MANUAL_X_HOME_POS 0
#define MANUAL_Y_HOME_POS 0
#define MANUAL_Z_HOME_POS 0
/*****************************************************************************************/


/*****************************************************************************************
 ******************************* Axis steps per unit *************************************
 *****************************************************************************************/
// Default steps per unit               X,  Y,    Z,  E0...(per extruder)
#define DEFAULT_AXIS_STEPS_PER_UNIT   {80, 80, 3200, 625, 625, 625, 625}
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Axis feedrate ****************************************
 *****************************************************************************************/
//                                       X,   Y, Z,  E0...(per extruder). (mm/sec)
#define DEFAULT_MAX_FEEDRATE          {300, 300, 2, 100, 100, 100, 100}
#define MANUAL_FEEDRATE               {50*60, 50*60, 4*60, 60}  // Feedrates for manual moves along X, Y, Z, E from panel
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
//  Maximum start speed for accelerated moves.    X,    Y,  Z,   E0...(per extruder)
#define DEFAULT_MAX_ACCELERATION              {3000, 3000, 50, 1000, 1000, 1000, 1000}
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
#define DEFAULT_XYJERK 10.0                 // (mm/sec)
#define DEFAULT_ZJERK   0.4                 // (mm/sec)
//  max initial speed for retract moves   E0... (mm/sec) per extruder
#define DEFAULT_EJERK                   {5.0, 5.0, 5.0, 5.0}
/*****************************************************************************************/


/*****************************************************************************************
 ************************************ Homing feedrate ************************************
 *****************************************************************************************/
#define HOMING_FEEDRATE {100*60, 100*60, 2*60, 0} // set the homing speeds (mm/min)

// homing hits the endstop, then retracts by this distance, before it tries to slowly bump again:
#define X_HOME_BUMP_MM 5
#define Y_HOME_BUMP_MM 5
#define Z_HOME_BUMP_MM 2
#define HOMING_BUMP_DIVISOR {5, 5, 2}  // Re-Bump Speed Divisor (Divides the Homing Feedrate)
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
//#define HOTEND_OFFSET_X {0.0, 5.0, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the X axis
//#define HOTEND_OFFSET_Y {0.0, 5.0, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the Y axis
//#define HOTEND_OFFSET_Z {0.0, 0.0, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the Z axis
/*****************************************************************************************/

#endif