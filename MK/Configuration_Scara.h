#ifndef CONFIGURATION_MECHANISM
#define CONFIGURATION_MECHANISM
#define KNOWN_MECH
/*
 * This configuration file contains mechanism settings for cartesian printer.
 *
 * - Machine name
 * - Scara settings
 * - Endstop pullup resistors
 * - Endstops logic
 * - Endstops min or max
 * - Min Z height for homing
 * - Stepper enable logic
 * - Stepper step logic
 * - Stepper direction
 * - Disables axis
 * - Travel limits
 * - Axis relative mode
 * - MBL or ABL
 * - Manual home positions
 * - Axis steps per unit
 * - Axis feedrate
 * - Axis accelleration
 * - Homing feedrate
 * - Hotend offset
 *
 * Basic-settings can be found in Configuration_Basic.h
 * Temperature-settings can be found in Configuration_Temperature.h
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
#define CUSTOM_MACHINE_NAME "Scara"
/*****************************************************************************************/


/*****************************************************************************************
 ************************************* Scara settings *************************************
/****************************************************************************************/
// SCARA-mode for Marlin has been developed by QHARLEY in ZA in 2012/2013. Implemented
// and slightly reworked by JCERNY in 06/2014 with the goal to bring it into Master-Branch
// QHARLEYS Autobedlevelling has not been ported, because Marlin has now Bed-levelling
// You might need Z-Min endstop on SCARA-Printer to use this feature. Actually untested!
// Uncomment to use Morgan scara mode
#define SCARA_SEGMENTS_PER_SECOND 200 // If movement is choppy try lowering this value
// Length of inner support arm
#define LINKAGE_1 150 //mm      Preprocessor cannot handle decimal point...
// Length of outer support arm     Measure arm lengths precisely and enter 
#define LINKAGE_2 150 //mm    

// SCARA tower offset (position of Tower relative to bed zero position) 
// This needs to be reasonably accurate as it defines the printbed position in the SCARA space.
#define SCARA_OFFSET_X 100 //mm   
#define SCARA_OFFSET_Y -56 //mm
#define SCARA_RAD2DEG 57.2957795  // to convert RAD to degrees

#define THETA_HOMING_OFFSET 0 //calculatated from Calibration Guide and command M360 / M114 see picture in http://reprap.harleystudio.co.za/?page_id=1073
#define PSI_HOMING_OFFSET 0   // calculatated from Calibration Guide and command M364 / M114 see picture in http://reprap.harleystudio.co.za/?page_id=1073
/*****************************************************************************************/


/*****************************************************************************************
 ************************* Endstop pullup resistors **************************************
 *****************************************************************************************
 *                                                                                       *
 * Comment this out (using // at the start of the line) to                               *
 * disable the endstop pullup resistors                                                  *
 *                                                                                       *
 *****************************************************************************************/
//#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

#if DISABLED(ENDSTOPPULLUPS)
// fine endstop settings: Individual pullups. will be ignored if ENDSTOPPULLUPS is defined
#define ENDSTOPPULLUP_XMIN  // open pin, inverted
#define ENDSTOPPULLUP_YMIN  // open pin, inverted
//#define ENDSTOPPULLUP_ZMIN
//#define ENDSTOPPULLUP_XMAX
//#define ENDSTOPPULLUP_YMAX
#define ENDSTOPPULLUP_ZMAX  // open pin, inverted
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
#define X_HOME_DIR 1
#define Y_HOME_DIR 1
#define Z_HOME_DIR -1
#define E_HOME_DIR -1
/*****************************************************************************************/


/*****************************************************************************************
 ***************************** MIN Z HEIGHT FOR HOMING **********************************
 *****************************************************************************************
 *                                                                                       *
 * (in mm) Minimal z height before homing (G28) for Z clearance above the bed, clamps,   *
 * Be sure you have this distance over your Z_MAX_POS in case.                           *
 *                                                                                       *
 *****************************************************************************************/
#define MIN_Z_HEIGHT_FOR_HOMING   0
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
#define Z_MAX_POS 225
#define Z_MIN_POS MANUAL_Z_HOME_POS
#define E_MIN_POS 0
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Axis relative mode ***********************************
 *****************************************************************************************/
#define AXIS_RELATIVE_MODES {false, false, false, false}
/*****************************************************************************************/


/*****************************************************************************************
 ******************************** Manual home positions **********************************
 *****************************************************************************************/
// The position of the homing switches
#define MANUAL_HOME_POSITIONS  // If defined, MANUAL_*_HOME_POS below will be used
//#define BED_CENTER_AT_0_0  // If defined, the center of the bed is at (X=0, Y=0)

//Manual homing switch locations:
#define MANUAL_X_HOME_POS -22
#define MANUAL_Y_HOME_POS -52
#define MANUAL_Z_HOME_POS 0.1  // Distance between nozzle and print surface after homing.
/*****************************************************************************************/


/*****************************************************************************************
 ******************************* Axis steps per unit *************************************
 *****************************************************************************************/
// Default steps per unit               X,    Y,   Z,  E0...(per extruder)
#define DEFAULT_AXIS_STEPS_PER_UNIT   {104, 104, 160, 625, 625, 625, 625}
/*****************************************************************************************/


/*****************************************************************************************
 ********************************** Axis feedrate ****************************************
 *****************************************************************************************/
//                                       X,   Y, Z, E0...(per extruder). (mm/sec)
#define DEFAULT_MAX_FEEDRATE          {300, 300, 4, 45, 45, 45, 45}
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
#define DEFAULT_ACCELERATION          400
//  X, Y, Z acceleration in mm/s^2 for travel (non printing) moves
#define DEFAULT_TRAVEL_ACCELERATION   400
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
#define HOMING_FEEDRATE {40*60, 40*60, 10*60, 0} // set the homing speeds (mm/min)

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
 * Offset of the extruders (uncomment if using more than one and relying on firmware     *
 * to position when changing).                                                           *
 * The offset has to be X=0, Y=0 for the hotend 0 (default hotend).                      *
 * For the other hotends it is their distance from the hotend 0.                         *
 *                                                                                       *
 *****************************************************************************************/
//#define HOTEND_OFFSET_X {0.0, 5.00, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the X axis
//#define HOTEND_OFFSET_Y {0.0, 5.00, 0.0, 0.0} // (in mm) for each hotend, offset of the hotend on the Y axis
/*****************************************************************************************/

#endif