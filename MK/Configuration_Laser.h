#ifndef CONFIGURATION_LASER
#define CONFIGURATION_LASER

//===========================================================================
//============================= Laser Settings ==============================
//===========================================================================

// The following define selects how to control the laser.
// Please choose the one that matches your setup.
// 1 = Single pin control - LOW when off, HIGH when on, PWM to adjust intensity
// 2 = Two pin control - A firing pin for which LOW = off, HIGH = on, and a seperate intensity pin which carries a constant PWM signal and adjusts duty cycle to control intensity
#define LASER_CONTROL 1

// The following define to use the new HakanBasted laser_pulse method to fire laser. It should be more efficient, but it's less tested.
// Thanks for it to HakanBastedt that has implemented it for Marlin at https://github.com/HakanBastedt/Marlin
// Uncomment to enable it *USE AT YOUR OWN RISK*, it should work but it's *NOT WELL TESTED YET*
// Only for MEGA. On DUE processor is automatic.
//#define LASER_PULSE_METHOD

// If your machine has laser focuser, set this to true and it will use Z axis for focus or disable it.
#define LASER_HAS_FOCUS false

//// In the case that the laserdriver need at least a certain level "LASER_REMAP_INTENSITY"
// to give anything, the intensity can be remapped to start at "LASER_REMAP_INTENSITY"
// At least some CO2-drivers need it, not sure about laserdiode drivers.
#define LASER_REMAP_INTENSITY 7

// Uncomment the following if your laser firing pin (not the PWM pin) for two pin control requires a HIGH signal to fire rather than a low (eg Red Sail M300 RS 3040)
// #define HIGH_TO_FIRE

// The following defines select which G codes tell the laser to fire. It's OK to uncomment more than one.
#define LASER_FIRE_G1 10      // fire the laser on a G1 move, extinguish when the move ends
#define LASER_FIRE_SPINDLE 11 // fire the laser on M3, extinguish on M5
#define LASER_FIRE_E 12       // fire the laser when the E axis moves

// Raster mode enables the laser to etch bitmap data at high speeds. Increases command buffer size substantially.
#define LASER_RASTER
#define LASER_MAX_RASTER_LINE 68      // Maximum number of base64 encoded pixels per raster gcode command
#define LASER_RASTER_ASPECT_RATIO 1   // pixels aren't square on most displays, 1.33 == 4:3 aspect ratio. 
#define LASER_RASTER_MM_PER_PULSE 0.2 // Can be overridden by providing an R value in M649 command : M649 S17 B2 D0 R0.1 F4000

// Uncomment the following if the laser cutter is equipped with a peripheral relay board
// to control power to an exhaust fan, cooler pump, laser power supply, etc.
//#define LASER_PERIPHERALS
//#define LASER_PERIPHERALS_TIMEOUT 30000  // Number of milliseconds to wait for status signal from peripheral control board

// Uncomment the following line to enable cubic bezier curve movement with the G5 code
// #define G5_BEZIER

// Uncomment these options for the Buildlog.net laser cutter, and other similar models
#define LASER_WATTS 40.0
#define LASER_DIAMETER 0.1        // milimeters
#define LASER_PWM 50000           // hertz
#define LASER_FOCAL_HEIGHT 74.50  // z axis position at which the laser is focused

#endif
