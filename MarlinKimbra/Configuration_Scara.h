// Define this to set a custom name for your generic Mendel,
#define CUSTOM_MENDEL_NAME "Scara"

// SCARA-mode for Marlin has been developed by QHARLEY in ZA in 2012/2013. Implemented
// and slightly reworked by JCERNY in 06/2014 with the goal to bring it into Master-Branch
// QHARLEYS Autobedlevelling has not been ported, because Marlin has now Bed-levelling
// You might need Z-Min endstop on SCARA-Printer to use this feature. Actually untested!
// Uncomment to use Morgan scara mode
#define scara_segments_per_second 200 //careful, two much will decrease performance...
// Length of inner support arm
#define Linkage_1 150 //mm      Preprocessor cannot handle decimal point...
// Length of outer support arm     Measure arm lengths precisely and enter 
#define Linkage_2 150 //mm    

// SCARA tower offset (position of Tower relative to bed zero position) 
// This needs to be reasonably accurate as it defines the printbed position in the SCARA space.
#define SCARA_offset_x 100 //mm   
#define SCARA_offset_y -56 //mm
#define SCARA_RAD2DEG 57.2957795  // to convert RAD to degrees

#define THETA_HOMING_OFFSET 0	//calculatated from Calibration Guide and command M360 / M114 see picture in http://reprap.harleystudio.co.za/?page_id=1073
#define PSI_HOMING_OFFSET 0  // calculatated from Calibration Guide and command M364 / M114 see picture in http://reprap.harleystudio.co.za/?page_id=1073

//some helper variables to make kinematics faster
#define L1_2 sq(Linkage_1) // do not change
#define L2_2 sq(Linkage_2) // do not change


//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// coarse Endstop Settings
//#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

#ifndef ENDSTOPPULLUPS
  // fine endstop settings: Individual pullups. will be ignored if ENDSTOPPULLUPS is defined
  // #define ENDSTOPPULLUP_XMAX
  // #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX  // open pin, inverted
  #define ENDSTOPPULLUP_XMIN  // open pin, inverted
  #define ENDSTOPPULLUP_YMIN  // open pin, inverted
  // #define ENDSTOPPULLUP_ZMIN
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
const bool X_MIN_ENDSTOP_INVERTING = true;       // set to true to invert the logic of the endstop.
const bool Y_MIN_ENDSTOP_INVERTING = true;       // set to true to invert the logic of the endstop.
const bool Z_MIN_ENDSTOP_INVERTING = true;       // set to true to invert the logic of the endstop.
const bool E_MIN_ENDSTOP_INVERTING = false;      // set to true to invert the logic of the endstop.
const bool X_MAX_ENDSTOP_INVERTING = true;       // set to true to invert the logic of the endstop.
const bool Y_MAX_ENDSTOP_INVERTING = true;       // set to true to invert the logic of the endstop.
const bool Z_MAX_ENDSTOP_INVERTING = true;       // set to true to invert the logic of the endstop.

// ENDSTOP SETTINGS:
// Sets direction of endstop when homing; 1=MAX, -1=MIN
#define X_HOME_DIR 1
#define Y_HOME_DIR 1
#define Z_HOME_DIR -1
#define E_HOME_DIR -1

#define min_software_endstops true // If true, axis won't move to coordinates less than HOME_POS.
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

#define INVERT_X_DIR false      // for Mendel set to false, for Orca set to true
#define INVERT_Y_DIR false      // for Mendel set to true, for Orca set to false
#define INVERT_Z_DIR true       // for Mendel set to false, for Orca set to true
#define INVERT_E0_DIR false     // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E1_DIR false     // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E2_DIR false     // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E3_DIR false     // for direct drive extruder v9 set to true, for geared extruder set to false

// Travel limits after homing
#define X_MAX_POS 200
#define X_MIN_POS 0
#define Y_MAX_POS 200
#define Y_MIN_POS 0
#define Z_MAX_POS 225
#define Z_MIN_POS MANUAL_Z_HOME_POS
#define E_MIN_POS 0

#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)

//=====================================================================================
//============================= Bed Manual or Auto Leveling ===========================
//=====================================================================================

// set the rectangle in which to probe in manual or automatic
#define LEFT_PROBE_BED_POSITION 20
#define RIGHT_PROBE_BED_POSITION 180
#define FRONT_PROBE_BED_POSITION 20
#define BACK_PROBE_BED_POSITION 180

#define XY_TRAVEL_SPEED 8000     // X and Y axis travel speed between probes, in mm/min

//If you have enabled the Auto Bed Levelling and are using the same Z Probe for Z Homing,
//it is highly recommended you let this Z_SAFE_HOMING enabled!!!
//#define Z_SAFE_HOMING
#ifdef Z_SAFE_HOMING
  #define Z_SAFE_HOMING_X_POINT (X_MAX_LENGTH/2)    // X point for Z homing when homing all axis (G28) or homing Z
  #define Z_SAFE_HOMING_Y_POINT (Y_MAX_LENGTH/2)    // Y point for Z homing when homing all axis (G28) or homing Z
#endif

//#define ENABLE_AUTO_BED_LEVELING    // Delete the comment to enable (ABL)
//#define Z_PROBE_REPEATABILITY_TEST  // Delete the comment to enable

#ifdef ENABLE_AUTO_BED_LEVELING

  // There are 2 different ways to specify probing locations
  //
  // - "grid" mode
  //   Probe several points in a rectangular grid.
  //   You specify the rectangle and the density of sample points.
  //   This mode is preferred because there are more measurements.
  //
  // - "3-point" mode
  //   Probe 3 arbitrary points on the bed (that aren't colinear)
  //   You specify the XY coordinates of all 3 points.

  // Enable this to sample the bed in a grid (least squares solution)
  // Note: this feature generates 10KB extra code size
  #define AUTO_BED_LEVELING_GRID

  #ifdef AUTO_BED_LEVELING_GRID

    // Use one of these defines to specify the origin
    // for a topographical map to be printed for your bed.
    enum { OriginBackLeft, OriginFrontLeft, OriginBackRight, OriginFrontRight };
    #define TOPO_ORIGIN OriginFrontLeft

    #define MIN_PROBE_EDGE 10 // The probe square sides can be no smaller than this

    // Set the number of grid points per dimension
    // You probably don't need more than 3 (squared=9)
    #define AUTO_BED_LEVELING_GRID_POINTS 2

  #else  // not AUTO_BED_LEVELING_GRID

    // Arbitrary points to probe. A simple cross-product
    // is used to estimate the plane of the bed.
    #define ABL_PROBE_PT_1_X 15
    #define ABL_PROBE_PT_1_Y 180
    #define ABL_PROBE_PT_2_X 15
    #define ABL_PROBE_PT_2_Y 20
    #define ABL_PROBE_PT_3_X 170
    #define ABL_PROBE_PT_3_Y 20

  #endif // AUTO_BED_LEVELING_GRID

  // Offsets to the probe relative to the extruder tip (Hotend - Probe)
  // X and Y offsets must be integers
  #define X_PROBE_OFFSET_FROM_EXTRUDER 0      // -left  +right
  #define Y_PROBE_OFFSET_FROM_EXTRUDER 0      // -front +behind
  #define Z_PROBE_OFFSET_FROM_EXTRUDER -1     // -below (always!)

  #define Z_RAISE_BEFORE_HOMING 10      // (in mm) Raise Z before homing (G28) for Probe Clearance.
                                        // Be sure you have this distance over your Z_MAX_POS in case

  #define Z_RAISE_BEFORE_PROBING 10     //How much the extruder will be raised before travelling to the first probing point.
  #define Z_RAISE_BETWEEN_PROBINGS 5    //How much the extruder will be raised when travelling from between next probing points

  //#define Z_PROBE_SLED                // turn on if you have a z-probe mounted on a sled like those designed by Charles Bell
  //#define SLED_DOCKING_OFFSET 5       // the extra distance the X axis must travel to pick up the sled. 0 should be fine but you can push it further if you'd like.

  //If defined, the Probe servo will be turned on only during movement and then turned off to avoid jerk
  //The value is the delay to turn the servo off after powered on - depends on the servo speed; 300ms is good value, but you can try lower it.
  //You MUST HAVE the SERVO ENDSTOPS defined to use here a value higher than zero otherwise your code will not compile.

  //#define PROBE_SERVO_DEACTIVATION_DELAY 300

#endif // ENABLE_AUTO_BED_LEVELING


// The position of the homing switches
//#define MANUAL_HOME_POSITIONS  // If defined, MANUAL_*_HOME_POS below will be used
//#define BED_CENTER_AT_0_0  // If defined, the center of the bed is at (X=0, Y=0)

//Manual homing switch locations:
// For SCARA: Offset between HomingPosition and Bed X=0 / Y=0
#define MANUAL_X_HOME_POS -22
#define MANUAL_Y_HOME_POS -52
#define MANUAL_Z_HOME_POS 0.1  // Distance between nozzle and print surface after homing.

// MOVEMENT SETTINGS
#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E
#define HOMING_FEEDRATE {40*60, 40*60, 10*60, 0}  // set the homing speeds (mm/min)

// default settings
#define DEFAULT_AXIS_STEPS_PER_UNIT     {103.69,103.69,200/1.25,1000,1000,1000,1000}    // X, Y, Z, E0, E1, E2, E3
#define DEFAULT_MAX_FEEDRATE            {300,300,4,45,45,45,45}                         // X, Y, Z, E0, E1, E2, E3 (mm/sec)
#define DEFAULT_RETRACTION_MAX_FEEDRATE {80,80,80,80}                                   // E0, E1, E2, E3 (mm/sec)
#define DEFAULT_MAX_ACCELERATION        {5000,5000,50,5000,5000,5000,5000}     // X, Y, Z, E0, E1, E2, E3 maximum start speed for accelerated moves.

#define DEFAULT_ACCELERATION           400      // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  2000      // X, Y, Z and E max acceleration in mm/s^2 for retracts
#define DEFAULT_TRAVEL_ACCELERATION    400      // X, Y, Z acceleration in mm/s^2 for travel (non printing) moves

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
//#define HOTEND_OFFSET_X {0.0, 5.00, 0.0, 0.0} // (in mm) for each extruder, offset of the hotend on the X axis
//#define HOTEND_OFFSET_Y {0.0, 5.00, 0.0, 0.0} // (in mm) for each extruder, offset of the hotend on the Y axis

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK 5    // (mm/sec)
#define DEFAULT_ZJERK  0.4  // (mm/sec)
#define DEFAULT_EJERK  3    // (mm/sec)

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

// Custom M code points
//#define CUSTOM_M_CODES
#ifdef CUSTOM_M_CODES
  #define CUSTOM_M_CODE_SET_Z_PROBE_OFFSET 851
  #define Z_PROBE_OFFSET_RANGE_MIN -15
  #define Z_PROBE_OFFSET_RANGE_MAX -5
#endif
