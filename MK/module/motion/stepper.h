/**
 * MK & MK4due 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
 */

#ifndef STEPPER_H
  #define STEPPER_H 
  
  /**
   * Axis indices as enumerated constants
   *
   * A_AXIS and B_AXIS are used by COREXY or COREYX printers
   * A_AXIS and C_AXIS are used by COREXZ or COREZX printers
   * X_HEAD and Y_HEAD and Z_HEAD is used for systems that don't have a 1:1 relationship between X_AXIS and X Head movement, like CoreXY bots.
   */
  enum AxisEnum {X_AXIS=0, A_AXIS=0, Y_AXIS=1, B_AXIS=1, Z_AXIS=2, C_AXIS=2, E_AXIS=3, X_HEAD=4, Y_HEAD=5, Z_HEAD=5};
  enum EndstopEnum {X_MIN=0, Y_MIN=1, Z_MIN=2, Z_PROBE=3, X_MAX=4, Y_MAX=5, Z_MAX=6, Z2_MIN=7, Z2_MAX=8, E_MIN=9};
  
  #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
    extern bool abort_on_endstop_hit;
  #endif

  // Initialize and start the stepper motor subsystem
  void st_init();

  // Block until all buffered steps are executed
  void st_synchronize();

  // Set the stepper direction of each axis
  void set_stepper_direction(bool onlye = false);

  // Set current position in steps
  void st_set_position(const long &x, const long &y, const long &z, const long &e);
  void st_set_e_position(const long &e);

  // Get current position in steps
  long st_get_position(uint8_t axis);

  // Get current position in mm
  float st_get_axis_position_mm(AxisEnum axis);

  // The stepper subsystem goes to sleep when it runs out of things to execute. Call this
  // to notify the subsystem that it is time to go to work.
  void st_wake_up();

  void checkHitEndstops(); //call from somewhere to create an serial error message with the locations the endstops where hit, in case they were triggered
  void endstops_hit_on_purpose(); //avoid creation of the message, i.e. after homing and before a routine call of checkHitEndstops();

  void enable_endstops(bool check); // Enable/disable endstop checking

  void checkStepperErrors(); //Print errors detected by the stepper

  void enable_all_steppers();
  void disable_all_steppers();
  void finishAndDisableSteppers();

  extern block_t *current_block;  // A pointer to the block currently being traced

  void quickStop();

  void digitalPotWrite(int address, int value);
  void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2);
  void microstep_mode(uint8_t driver, uint8_t stepping);
  void digipot_init();
  void digipot_current(uint8_t driver, int current);
  void microstep_init();
  void microstep_readings();

  #if ENABLED(Z_DUAL_ENDSTOPS)
    void In_Homing_Process(bool state);
    void Lock_z_motor(bool state);
    void Lock_z2_motor(bool state);
  #endif

  #if ENABLED(BABYSTEPPING)
    void babystep(const uint8_t axis, const bool direction); // perform a short step with a single stepper motor, outside of any convention
  #endif

  #if ENABLED(NPR2) // Multiextruder
    void colorstep(long csteps, const bool direction);
  #endif

#endif // STEPPER_H
