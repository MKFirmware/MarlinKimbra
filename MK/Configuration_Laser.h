#ifndef CONFIGURATION_LASER
#define CONFIGURATION_LASER

//===========================================================================
//============================= Laser Settings ==============================
//===========================================================================
//
// Laser control is used by the Muve1 3D printer and the Buildlog.net laser cutter
//

//// The following define selects how to control the laser.  Please choose the one that matches your setup.
// 1 = Single pin control - LOW when off, HIGH when on, PWM to adjust intensity
// 2 = Two pin control - A firing pin for which LOW = off, HIGH = on, and a seperate intensity pin which carries a constant PWM signal and adjusts duty cycle to control intensity
// mUVe, buildlog.net and K40 chinese machines uses 2, AMRI ablative uses 1, AMRI SLS uses 2
#define LASER_CONTROL 2

// Uncomment the following if your laser firing pin (not the PWM pin) for two pin control requires a HIGH signal to fire rather than a low (eg Red Sail M300 RS 3040)
/// #define HIGH_TO_FIRE

// Uncomment the following to enable the use of the PWM (the one for the extruder 0) to drive a peltier cell or any PWM driven cooler for the laser
#define COOLER
#define COOLER_MAXTEMP 25

// Uncomment the following to enable COOLER PWM instead of bang-bang
#define COOLER_PWM
#define COOLER_PWM_FREQUENCY 1000 // Frequency in Hz

//// The following defines select which G codes tell the laser to fire.  It's OK to uncomment more than one.
#define LASER_FIRE_G1 10 // fire the laser on a G1 move, extinguish when the move ends
#define LASER_FIRE_SPINDLE 11 // fire the laser on M3, extinguish on M5
#define LASER_FIRE_E 12 // fire the laser when the E axis moves

//// Raster mode enables the laser to etch bitmap data at high speeds.  Increases command buffer size substantially.
#define LASER_RASTER
#define LASER_MAX_RASTER_LINE 68 // maximum number of base64 encoded pixels per raster gcode command
#define LASER_RASTER_ASPECT_RATIO 1 // pixels aren't square on most displays, 1.33 == 4:3 aspect ratio. 
#define LASER_RASTER_MM_PER_PULSE 0.2 //Can be overridden by providing an R value in M649 command : M649 S17 B2 D0 R0.1 F4000

//// Uncomment the following if the laser cutter is equipped with a peripheral relay board
//// to control power to an exhaust fan, cooler pump, laser power supply, etc.
//#define LASER_PERIPHERALS
//#define LASER_PERIPHERALS_TIMEOUT 30000  // Number of milliseconds to wait for status signal from peripheral control board

//// Uncomment the following line to enable cubic bezier curve movement with the G5 code
// #define G5_BEZIER

// Uncomment these options for the mUVe 1 3D printer
// #define CUSTOM_MENDEL_NAME "mUVe1 Printer"
// #define LASER_WATTS 0.05
// #define LASER_DIAMETER 0.1 // milimeters
// #define LASER_PWM 8000 // hertz
// #define MUVE_Z_PEEL // The mUVe 1 uses a special peel maneuver between each layer, it requires independent control of each Z motor

// Uncomment these options for the Buildlog.net laser cutter, and other similar models
#define CUSTOM_MENDEL_NAME "Laser Cutter"
#define LASER_WATTS 40.0
#define LASER_DIAMETER 0.1 // milimeters
#define LASER_PWM 50000 // hertz
#define LASER_FOCAL_HEIGHT 74.50 // z axis position at which the laser is focused

//Uncomment for AMRI Ablative or SLS
//#define CUSTOM_MENDEL_NAME "Laser Cutter"
//#define LASER_WATTS 40.0
//#define LASER_DIAMETER 0.1 // milimeters
//#define LASER_PWM 25000 // hertz
//#define LASER_FOCAL_HEIGHT 74.50 // z axis position at which the laser is focused



/***********************************************************************
 ************************ PID Settings - COOLER ************************
 ***********************************************************************
 *                                                                     *
 * PID Tuning Guide here: http://reprap.org/wiki/PID_Tuning            *
 * Select PID or bang-bang with PIDTEMPCOOLER.                         *
 * If bang-bang, COOLER_LIMIT_SWITCHING will enable hysteresis         *
 *                                                                     *
 ***********************************************************************/
// Uncomment this to enable PID on the bed. It uses the same frequency PWM as the extruder.
// If your PID_dT is the default, and correct for your hardware/configuration, that means 7.689Hz,
// which is fine for driving a square wave into a resistive load and does not significantly impact you FET heating.
// This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W heater.
// If your configuration is significantly different than this and you don't understand the issues involved, you probably
// shouldn't use bed PID until someone else verifies your hardware works.
// If this is enabled, find your own PID constants below.
#define PIDTEMPCOOLER

//#define COOLER_LIMIT_SWITCHING
#define COOLER_HYSTERESIS 2 //only disable heating if T<target-COOLER_HYSTERESIS and enable heating if T<target+COOLER_HYSTERESIS (works only if COOLER_LIMIT_SWITCHING is enabled)
#define COOLER_CHECK_INTERVAL 5000 //ms between checks in bang-bang control

// This sets the max power delivered to the bed.
// all forms of bed control obey this (PID, bang-bang, bang-bang with hysteresis)
// setting this to anything other than 255 enables a form of PWM to the bed,
// so you shouldn't use it unless you are OK with PWM on your bed.  (see the comment on enabling PIDTEMPCOOLER)
#define MAX_COOLER_POWER 255 // limits duty cycle to bed; 255=full current

#define PID_COOLER_INTEGRAL_DRIVE_MAX MAX_COOLER_POWER // limit for the integral term
// 120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
// from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, aggressive factor of .15 (vs .1, 1, 10)
#define DEFAULT_coolerKp 10.00
#define DEFAULT_coolerKi .023
#define DEFAULT_coolerKd 305.4

// FIND YOUR OWN: "M303 E-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.

//#define PID_COOLER_DEBUG // Sends debug data to the serial port.
/***********************************************************************/


#endif
