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
 * stepper_indirection.h - stepper motor driver indirection macros
 * to allow some stepper functions to be done via SPI/I2c instead of direct pin manipulation
 *
 * Copyright (c) 2015 Dominik Wenger
 *
 */

#ifndef STEPPER_INDIRECTION_H
#define STEPPER_INDIRECTION_H

// X motor
#define X_STEP_INIT SET_OUTPUT(X_STEP_PIN)
#define X_STEP_WRITE(STATE) WRITE(X_STEP_PIN,STATE)
#define X_STEP_READ READ(X_STEP_PIN)

#define X_DIR_INIT SET_OUTPUT(X_DIR_PIN)
#define X_DIR_WRITE(STATE) WRITE(X_DIR_PIN,STATE)
#define X_DIR_READ READ(X_DIR_PIN)

#define X_ENABLE_INIT SET_OUTPUT(X_ENABLE_PIN)
#define X_ENABLE_WRITE(STATE) WRITE(X_ENABLE_PIN,STATE)
#define X_ENABLE_READ READ(X_ENABLE_PIN)

// X2 motor
#define X2_STEP_INIT SET_OUTPUT(X2_STEP_PIN)
#define X2_STEP_WRITE(STATE) WRITE(X2_STEP_PIN,STATE)
#define X2_STEP_READ READ(X2_STEP_PIN)

#define X2_DIR_INIT SET_OUTPUT(X2_DIR_PIN)
#define X2_DIR_WRITE(STATE) WRITE(X2_DIR_PIN,STATE)
#define X2_DIR_READ READ(X_DIR_PIN)

#define X2_ENABLE_INIT SET_OUTPUT(X2_ENABLE_PIN)
#define X2_ENABLE_WRITE(STATE) WRITE(X2_ENABLE_PIN,STATE)
#define X2_ENABLE_READ READ(X_ENABLE_PIN)

// Y motor
#define Y_STEP_INIT SET_OUTPUT(Y_STEP_PIN)
#define Y_STEP_WRITE(STATE) WRITE(Y_STEP_PIN,STATE)
#define Y_STEP_READ READ(Y_STEP_PIN)

#define Y_DIR_INIT SET_OUTPUT(Y_DIR_PIN)
#define Y_DIR_WRITE(STATE) WRITE(Y_DIR_PIN,STATE)
#define Y_DIR_READ READ(Y_DIR_PIN)

#define Y_ENABLE_INIT SET_OUTPUT(Y_ENABLE_PIN)
#define Y_ENABLE_WRITE(STATE) WRITE(Y_ENABLE_PIN,STATE)
#define Y_ENABLE_READ READ(Y_ENABLE_PIN)

// Y2 motor
#define Y2_STEP_INIT SET_OUTPUT(Y2_STEP_PIN)
#define Y2_STEP_WRITE(STATE) WRITE(Y2_STEP_PIN,STATE)
#define Y2_STEP_READ READ(Y2_STEP_PIN)

#define Y2_DIR_INIT SET_OUTPUT(Y2_DIR_PIN)
#define Y2_DIR_WRITE(STATE) WRITE(Y2_DIR_PIN,STATE)
#define Y2_DIR_READ READ(Y2_DIR_PIN)

#define Y2_ENABLE_INIT SET_OUTPUT(Y2_ENABLE_PIN)
#define Y2_ENABLE_WRITE(STATE) WRITE(Y2_ENABLE_PIN,STATE)
#define Y2_ENABLE_READ READ(Y2_ENABLE_PIN)

// Z motor
#define Z_STEP_INIT SET_OUTPUT(Z_STEP_PIN)
#define Z_STEP_WRITE(STATE) WRITE(Z_STEP_PIN,STATE)
#define Z_STEP_READ READ(Z_STEP_PIN)

#define Z_DIR_INIT SET_OUTPUT(Z_DIR_PIN)
#define Z_DIR_WRITE(STATE) WRITE(Z_DIR_PIN,STATE)
#define Z_DIR_READ READ(Z_DIR_PIN)

#define Z_ENABLE_INIT SET_OUTPUT(Z_ENABLE_PIN)
#define Z_ENABLE_WRITE(STATE) WRITE(Z_ENABLE_PIN,STATE)
#define Z_ENABLE_READ READ(Z_ENABLE_PIN)

// Z2 motor
#define Z2_STEP_INIT SET_OUTPUT(Z2_STEP_PIN)
#define Z2_STEP_WRITE(STATE) WRITE(Z2_STEP_PIN,STATE)
#define Z2_STEP_READ READ(Z2_STEP_PIN)

#define Z2_DIR_INIT SET_OUTPUT(Z2_DIR_PIN)
#define Z2_DIR_WRITE(STATE) WRITE(Z2_DIR_PIN,STATE)
#define Z2_DIR_READ READ(Z2_DIR_PIN)

#define Z2_ENABLE_INIT SET_OUTPUT(Z2_ENABLE_PIN)
#define Z2_ENABLE_WRITE(STATE) WRITE(Z2_ENABLE_PIN,STATE)
#define Z2_ENABLE_READ READ(Z2_ENABLE_PIN)

// E0 motor
#define E0_STEP_INIT SET_OUTPUT(E0_STEP_PIN)
#define E0_STEP_WRITE(STATE) WRITE(E0_STEP_PIN,STATE)
#define E0_STEP_READ READ(E0_STEP_PIN)

#define E0_DIR_INIT SET_OUTPUT(E0_DIR_PIN)
#define E0_DIR_WRITE(STATE) WRITE(E0_DIR_PIN,STATE)
#define E0_DIR_READ READ(E0_DIR_PIN)

#define E0_ENABLE_INIT SET_OUTPUT(E0_ENABLE_PIN)
#define E0_ENABLE_WRITE(STATE) WRITE(E0_ENABLE_PIN,STATE)
#define E0_ENABLE_READ READ(E0_ENABLE_PIN)

// E1 motor
#define E1_STEP_INIT SET_OUTPUT(E1_STEP_PIN)
#define E1_STEP_WRITE(STATE) WRITE(E1_STEP_PIN,STATE)
#define E1_STEP_READ READ(E1_STEP_PIN)

#define E1_DIR_INIT SET_OUTPUT(E1_DIR_PIN)
#define E1_DIR_WRITE(STATE) WRITE(E1_DIR_PIN,STATE)
#define E1_DIR_READ READ(E1_DIR_PIN)

#define E1_ENABLE_INIT SET_OUTPUT(E1_ENABLE_PIN)
#define E1_ENABLE_WRITE(STATE) WRITE(E1_ENABLE_PIN,STATE)
#define E1_ENABLE_READ READ(E1_ENABLE_PIN)

// E2 motor
#define E2_STEP_INIT SET_OUTPUT(E2_STEP_PIN)
#define E2_STEP_WRITE(STATE) WRITE(E2_STEP_PIN,STATE)
#define E2_STEP_READ READ(E2_STEP_PIN)

#define E2_DIR_INIT SET_OUTPUT(E2_DIR_PIN)
#define E2_DIR_WRITE(STATE) WRITE(E2_DIR_PIN,STATE)
#define E2_DIR_READ READ(E2_DIR_PIN)

#define E2_ENABLE_INIT SET_OUTPUT(E2_ENABLE_PIN)
#define E2_ENABLE_WRITE(STATE) WRITE(E2_ENABLE_PIN,STATE)
#define E2_ENABLE_READ READ(E2_ENABLE_PIN)

// E3 motor
#define E3_STEP_INIT SET_OUTPUT(E3_STEP_PIN)
#define E3_STEP_WRITE(STATE) WRITE(E3_STEP_PIN,STATE)
#define E3_STEP_READ READ(E3_STEP_PIN)

#define E3_DIR_INIT SET_OUTPUT(E3_DIR_PIN)
#define E3_DIR_WRITE(STATE) WRITE(E3_DIR_PIN,STATE)
#define E3_DIR_READ READ(E3_DIR_PIN)

#define E3_ENABLE_INIT SET_OUTPUT(E3_ENABLE_PIN)
#define E3_ENABLE_WRITE(STATE) WRITE(E3_ENABLE_PIN,STATE)
#define E3_ENABLE_READ READ(E3_ENABLE_PIN)

// E4 motor
#define E4_STEP_INIT SET_OUTPUT(E4_STEP_PIN)
#define E4_STEP_WRITE(STATE) WRITE(E4_STEP_PIN,STATE)
#define E4_STEP_READ READ(E4_STEP_PIN)

#define E4_DIR_INIT SET_OUTPUT(E4_DIR_PIN)
#define E4_DIR_WRITE(STATE) WRITE(E4_DIR_PIN,STATE)
#define E4_DIR_READ READ(E4_DIR_PIN)

#define E4_ENABLE_INIT SET_OUTPUT(E4_ENABLE_PIN)
#define E4_ENABLE_WRITE(STATE) WRITE(E4_ENABLE_PIN,STATE)
#define E4_ENABLE_READ READ(E4_ENABLE_PIN)

// E5 motor
#define E5_STEP_INIT SET_OUTPUT(E5_STEP_PIN)
#define E5_STEP_WRITE(STATE) WRITE(E5_STEP_PIN,STATE)
#define E5_STEP_READ READ(E5_STEP_PIN)

#define E5_DIR_INIT SET_OUTPUT(E5_DIR_PIN)
#define E5_DIR_WRITE(STATE) WRITE(E5_DIR_PIN,STATE)
#define E5_DIR_READ READ(E5_DIR_PIN)

#define E5_ENABLE_INIT SET_OUTPUT(E5_ENABLE_PIN)
#define E5_ENABLE_WRITE(STATE) WRITE(E5_ENABLE_PIN,STATE)
#define E5_ENABLE_READ READ(E5_ENABLE_PIN)

#if ENABLED(COLOR_MIXING_EXTRUDER)
  #define E_STEP_WRITE(v) ; /* not used for mixing extruders! */
  #if DRIVER_EXTRUDERS > 5
    #define En_STEP_WRITE(n,v) { switch (n) { case 0: E0_STEP_WRITE(v); break; case 1: E1_STEP_WRITE(v); break; case 2: E2_STEP_WRITE(v); break; case 3: E3_STEP_WRITE(v); break; case 4: E4_STEP_WRITE(v); break; case 5: E5_STEP_WRITE(v); } }
    #define NORM_E_DIR() { E0_DIR_WRITE(!INVERT_E0_DIR); E1_DIR_WRITE(!INVERT_E1_DIR); E2_DIR_WRITE(!INVERT_E2_DIR); E3_DIR_WRITE(!INVERT_E3_DIR); E4_DIR_WRITE(!INVERT_E4_DIR); E5_DIR_WRITE(!INVERT_E5_DIR); }
    #define REV_E_DIR()  { E0_DIR_WRITE( INVERT_E0_DIR); E1_DIR_WRITE( INVERT_E1_DIR); E2_DIR_WRITE( INVERT_E2_DIR); E3_DIR_WRITE( INVERT_E3_DIR); E4_DIR_WRITE( INVERT_E4_DIR); E5_DIR_WRITE( INVERT_E5_DIR); }
  #elif DRIVER_EXTRUDERS > 4
    #define En_STEP_WRITE(n,v) { switch (n) { case 0: E0_STEP_WRITE(v); break; case 1: E1_STEP_WRITE(v); break; case 2: E2_STEP_WRITE(v); break; case 3: E3_STEP_WRITE(v); break; case 4: E4_STEP_WRITE(v); } }
    #define NORM_E_DIR() { E0_DIR_WRITE(!INVERT_E0_DIR); E1_DIR_WRITE(!INVERT_E1_DIR); E2_DIR_WRITE(!INVERT_E2_DIR); E3_DIR_WRITE(!INVERT_E3_DIR); E4_DIR_WRITE(!INVERT_E4_DIR); }
    #define REV_E_DIR()  { E0_DIR_WRITE( INVERT_E0_DIR); E1_DIR_WRITE( INVERT_E1_DIR); E2_DIR_WRITE( INVERT_E2_DIR); E3_DIR_WRITE( INVERT_E3_DIR); E4_DIR_WRITE( INVERT_E4_DIR); }
  #elif DRIVER_EXTRUDERS > 3
    #define En_STEP_WRITE(n,v) { switch (n) { case 0: E0_STEP_WRITE(v); break; case 1: E1_STEP_WRITE(v); break; case 2: E2_STEP_WRITE(v); break; case 3: E3_STEP_WRITE(v); } }
    #define NORM_E_DIR() { E0_DIR_WRITE(!INVERT_E0_DIR); E1_DIR_WRITE(!INVERT_E1_DIR); E2_DIR_WRITE(!INVERT_E2_DIR); E3_DIR_WRITE(!INVERT_E3_DIR); }
    #define REV_E_DIR()  { E0_DIR_WRITE( INVERT_E0_DIR); E1_DIR_WRITE( INVERT_E1_DIR); E2_DIR_WRITE( INVERT_E2_DIR); E3_DIR_WRITE( INVERT_E3_DIR); }
  #elif DRIVER_EXTRUDERS > 2
    #define En_STEP_WRITE(n,v) { switch (n) { case 0: E0_STEP_WRITE(v); break; case 1: E1_STEP_WRITE(v); break; case 2: E2_STEP_WRITE(v); } }
    #define NORM_E_DIR() { E0_DIR_WRITE(!INVERT_E0_DIR); E1_DIR_WRITE(!INVERT_E1_DIR); E2_DIR_WRITE(!INVERT_E2_DIR); }
    #define REV_E_DIR()  { E0_DIR_WRITE( INVERT_E0_DIR); E1_DIR_WRITE( INVERT_E1_DIR); E2_DIR_WRITE( INVERT_E2_DIR); }
  #else
    #define En_STEP_WRITE(n,v) { switch (n) { case 0: E0_STEP_WRITE(v); break; case 1: E1_STEP_WRITE(v); } }
    #define NORM_E_DIR() { E0_DIR_WRITE(!INVERT_E0_DIR); E1_DIR_WRITE(!INVERT_E1_DIR); }
    #define REV_E_DIR()  { E0_DIR_WRITE( INVERT_E0_DIR); E1_DIR_WRITE( INVERT_E1_DIR); }
  #endif
#elif DRIVER_EXTRUDERS > 5
  #define E_STEP_WRITE(v) { switch(current_block->active_driver) { case 5: E5_STEP_WRITE(v); break; case 4: E4_STEP_WRITE(v); break; case 3: E3_STEP_WRITE(v); break; case 2: E2_STEP_WRITE(v); break; case 1: E1_STEP_WRITE(v); break; case 0: E0_STEP_WRITE(v); break; } }
  #define NORM_E_DIR()    { switch(current_block->active_driver) { case 5: E5_DIR_WRITE(!INVERT_E5_DIR); break; case 4: E4_DIR_WRITE(!INVERT_E4_DIR); break; case 3: E3_DIR_WRITE(!INVERT_E3_DIR); break; case 2: E2_DIR_WRITE(!INVERT_E2_DIR); break; case 1: E1_DIR_WRITE(!INVERT_E1_DIR); break; case 0: E0_DIR_WRITE(!INVERT_E0_DIR); break; } }
  #define REV_E_DIR()     { switch(current_block->active_driver) { case 5: E5_DIR_WRITE( INVERT_E5_DIR); break; case 4: E4_DIR_WRITE( INVERT_E4_DIR); break; case 3: E3_DIR_WRITE( INVERT_E3_DIR); break; case 2: E2_DIR_WRITE( INVERT_E2_DIR); break; case 1: E1_DIR_WRITE( INVERT_E1_DIR); break; case 0: E0_DIR_WRITE( INVERT_E0_DIR); break; } }
#elif DRIVER_EXTRUDERS > 4
  #define E_STEP_WRITE(v) { switch(current_block->active_driver) { case 4: E4_STEP_WRITE(v); break; case 3: E3_STEP_WRITE(v); break; case 2: E2_STEP_WRITE(v); break; case 1: E1_STEP_WRITE(v); break; case 0: E0_STEP_WRITE(v); break; } }
  #define NORM_E_DIR()    { switch(current_block->active_driver) { case 4: E4_DIR_WRITE(!INVERT_E4_DIR); break; case 3: E3_DIR_WRITE(!INVERT_E3_DIR); break; case 2: E2_DIR_WRITE(!INVERT_E2_DIR); break; case 1: E1_DIR_WRITE(!INVERT_E1_DIR); break; case 0: E0_DIR_WRITE(!INVERT_E0_DIR); break; } }
  #define REV_E_DIR()     { switch(current_block->active_driver) { case 4: E4_DIR_WRITE( INVERT_E4_DIR); break; case 3: E3_DIR_WRITE( INVERT_E3_DIR); break; case 2: E2_DIR_WRITE( INVERT_E2_DIR); break; case 1: E1_DIR_WRITE( INVERT_E1_DIR); break; case 0: E0_DIR_WRITE( INVERT_E0_DIR); break; } }
#elif DRIVER_EXTRUDERS > 3
  #define E_STEP_WRITE(v) { switch(current_block->active_driver) { case 3: E3_STEP_WRITE(v); break; case 2: E2_STEP_WRITE(v); break; case 1: E1_STEP_WRITE(v); break; case 0: E0_STEP_WRITE(v); break; } }
  #define NORM_E_DIR()    { switch(current_block->active_driver) { case 3: E3_DIR_WRITE(!INVERT_E3_DIR); break; case 2: E2_DIR_WRITE(!INVERT_E2_DIR); break; case 1: E1_DIR_WRITE(!INVERT_E1_DIR); break; case 0: E0_DIR_WRITE(!INVERT_E0_DIR); break; } }
  #define REV_E_DIR()     { switch(current_block->active_driver) { case 3: E3_DIR_WRITE( INVERT_E3_DIR); break; case 2: E2_DIR_WRITE( INVERT_E2_DIR); break; case 1: E1_DIR_WRITE( INVERT_E1_DIR); break; case 0: E0_DIR_WRITE( INVERT_E0_DIR); break; } }
#elif DRIVER_EXTRUDERS > 2
  #define E_STEP_WRITE(v) { switch(current_block->active_driver) { case 2: E2_STEP_WRITE(v); break; case 1: E1_STEP_WRITE(v); break; case 0: E0_STEP_WRITE(v); break; } }
  #define NORM_E_DIR()    { switch(current_block->active_driver) { case 2: E2_DIR_WRITE(!INVERT_E2_DIR); break; case 1: E1_DIR_WRITE(!INVERT_E1_DIR); break; case 0: E0_DIR_WRITE(!INVERT_E0_DIR); break; } }
  #define REV_E_DIR()     { switch(current_block->active_driver) { case 2: E2_DIR_WRITE( INVERT_E2_DIR); break; case 1: E1_DIR_WRITE( INVERT_E1_DIR); break; case 0: E0_DIR_WRITE( INVERT_E0_DIR); break; } }
#elif DRIVER_EXTRUDERS > 1
  #if DISABLED(DUAL_X_CARRIAGE)
    #define E_STEP_WRITE(v) { switch(current_block->active_driver) { case 1: E1_STEP_WRITE(v); break; case 0: E0_STEP_WRITE(v); break; }}
    #define NORM_E_DIR()    { switch(current_block->active_driver) { case 1: E1_DIR_WRITE(!INVERT_E1_DIR); break; case 0: E0_DIR_WRITE(!INVERT_E0_DIR); break; }}
    #define REV_E_DIR()     { switch(current_block->active_driver) { case 1: E1_DIR_WRITE( INVERT_E1_DIR); break; case 0: E0_DIR_WRITE( INVERT_E0_DIR); break; }}
  #else
    extern bool extruder_duplication_enabled;
    #define E_STEP_WRITE(v) { if(extruder_duplication_enabled) { E0_STEP_WRITE(v); E1_STEP_WRITE(v); } else if(current_block->active_driver == 1) { E1_STEP_WRITE(v); } else { E0_STEP_WRITE(v); }}
    #define NORM_E_DIR()    { if(extruder_duplication_enabled) { E0_DIR_WRITE(!INVERT_E0_DIR); E1_DIR_WRITE(!INVERT_E1_DIR); } else if(current_block->active_driver == 1) { E1_DIR_WRITE(!INVERT_E1_DIR); } else { E0_DIR_WRITE(!INVERT_E0_DIR); }}
    #define REV_E_DIR()     { if(extruder_duplication_enabled) { E0_DIR_WRITE( INVERT_E0_DIR); E1_DIR_WRITE( INVERT_E1_DIR); } else if(current_block->active_driver == 1) { E1_DIR_WRITE( INVERT_E1_DIR); } else { E0_DIR_WRITE( INVERT_E0_DIR); }}
  #endif
#else
  #if ENABLED(DONDOLO)
    #define E_STEP_WRITE(v) E0_STEP_WRITE(v)
    #define NORM_E_DIR()  { switch(active_extruder) { case 1: E0_DIR_WRITE( INVERT_E0_DIR); break; case 0: E0_DIR_WRITE(!INVERT_E0_DIR); break; }}
    #define REV_E_DIR()   { switch(active_extruder) { case 1: E0_DIR_WRITE(!INVERT_E0_DIR); break; case 0: E0_DIR_WRITE( INVERT_E0_DIR); break; }}
  #else
    #define E_STEP_WRITE(v) E0_STEP_WRITE(v)
    #define NORM_E_DIR() E0_DIR_WRITE(!INVERT_E0_DIR)
    #define REV_E_DIR()  E0_DIR_WRITE( INVERT_E0_DIR)
  #endif
#endif // DRIVER_EXTRUDERS

#if ENABLED(DUAL_X_CARRIAGE) && HAS(X_ENABLE) && HAS(X2_ENABLE)
  #define  enable_x() do { X_ENABLE_WRITE( X_ENABLE_ON); X2_ENABLE_WRITE( X_ENABLE_ON); } while (0)
  #define disable_x() do { X_ENABLE_WRITE(!X_ENABLE_ON); X2_ENABLE_WRITE(!X_ENABLE_ON); CBI(axis_known_position, X_AXIS); } while (0)
#elif HAS(X_ENABLE)
  #define  enable_x() X_ENABLE_WRITE( X_ENABLE_ON)
  #define disable_x() { X_ENABLE_WRITE(!X_ENABLE_ON); CBI(axis_known_position, X_AXIS); }
#else
  #define  enable_x() ;
  #define disable_x() ;
#endif

#if HAS(Y_ENABLE)
  #if ENABLED(Y_DUAL_STEPPER_DRIVERS)
    #define  enable_y() { Y_ENABLE_WRITE( Y_ENABLE_ON); Y2_ENABLE_WRITE(Y_ENABLE_ON); }
    #define disable_y() { Y_ENABLE_WRITE(!Y_ENABLE_ON); Y2_ENABLE_WRITE(!Y_ENABLE_ON); CBI(axis_known_position, Y_AXIS); }
  #else
    #define  enable_y() Y_ENABLE_WRITE( Y_ENABLE_ON)
    #define disable_y() { Y_ENABLE_WRITE(!Y_ENABLE_ON); CBI(axis_known_position, Y_AXIS); }
  #endif
#else
  #define  enable_y() ;
  #define disable_y() ;
#endif

#if HAS(Z_ENABLE)
  #if ENABLED(Z_DUAL_STEPPER_DRIVERS)
    #define  enable_z() { Z_ENABLE_WRITE( Z_ENABLE_ON); Z2_ENABLE_WRITE(Z_ENABLE_ON); }
    #define disable_z() { Z_ENABLE_WRITE(!Z_ENABLE_ON); Z2_ENABLE_WRITE(!Z_ENABLE_ON); CBI(axis_known_position, Z_AXIS); }
  #else
    #define  enable_z() Z_ENABLE_WRITE( Z_ENABLE_ON)
    #define disable_z() { Z_ENABLE_WRITE(!Z_ENABLE_ON); CBI(axis_known_position, Z_AXIS); }
  #endif
#else
  #define  enable_z() ;
  #define disable_z() ;
#endif

#if ENABLED(COLOR_MIXING_EXTRUDER)

  #if DRIVER_EXTRUDERS > 5
    #define  enable_e0() { E0_ENABLE_WRITE( E_ENABLE_ON); E1_ENABLE_WRITE( E_ENABLE_ON); E2_ENABLE_WRITE( E_ENABLE_ON); E3_ENABLE_WRITE( E_ENABLE_ON); E4_ENABLE_WRITE( E_ENABLE_ON); E5_ENABLE_WRITE( E_ENABLE_ON); }
    #define disable_e0() { E0_ENABLE_WRITE(!E_ENABLE_ON); E1_ENABLE_WRITE(!E_ENABLE_ON); E2_ENABLE_WRITE(!E_ENABLE_ON); E3_ENABLE_WRITE(!E_ENABLE_ON); E4_ENABLE_WRITE(!E_ENABLE_ON); E5_ENABLE_WRITE(!E_ENABLE_ON); }
  #elif DRIVER_EXTRUDERS > 4
    #define  enable_e0() { E0_ENABLE_WRITE( E_ENABLE_ON); E1_ENABLE_WRITE( E_ENABLE_ON); E2_ENABLE_WRITE( E_ENABLE_ON); E3_ENABLE_WRITE( E_ENABLE_ON); E4_ENABLE_WRITE( E_ENABLE_ON); }
    #define disable_e0() { E0_ENABLE_WRITE(!E_ENABLE_ON); E1_ENABLE_WRITE(!E_ENABLE_ON); E2_ENABLE_WRITE(!E_ENABLE_ON); E3_ENABLE_WRITE(!E_ENABLE_ON); E4_ENABLE_WRITE(!E_ENABLE_ON); }
  #elif DRIVER_EXTRUDERS > 3
    #define  enable_e0() { E0_ENABLE_WRITE( E_ENABLE_ON); E1_ENABLE_WRITE( E_ENABLE_ON); E2_ENABLE_WRITE( E_ENABLE_ON); E3_ENABLE_WRITE( E_ENABLE_ON); }
    #define disable_e0() { E0_ENABLE_WRITE(!E_ENABLE_ON); E1_ENABLE_WRITE(!E_ENABLE_ON); E2_ENABLE_WRITE(!E_ENABLE_ON); E3_ENABLE_WRITE(!E_ENABLE_ON); }
  #elif DRIVER_EXTRUDERS > 2
    #define  enable_e0() { E0_ENABLE_WRITE( E_ENABLE_ON); E1_ENABLE_WRITE( E_ENABLE_ON); E2_ENABLE_WRITE( E_ENABLE_ON); }
    #define disable_e0() { E0_ENABLE_WRITE(!E_ENABLE_ON); E1_ENABLE_WRITE(!E_ENABLE_ON); E2_ENABLE_WRITE(!E_ENABLE_ON); }
  #else
    #define  enable_e0() { E0_ENABLE_WRITE( E_ENABLE_ON); E1_ENABLE_WRITE( E_ENABLE_ON); }
    #define disable_e0() { E0_ENABLE_WRITE(!E_ENABLE_ON); E1_ENABLE_WRITE(!E_ENABLE_ON); }
  #endif

  #define  enable_e1() ;
  #define disable_e1() ;
  #define  enable_e2() ;
  #define disable_e2() ;
  #define  enable_e3() ;
  #define disable_e3() ;
  #define  enable_e4() ;
  #define disable_e4() ;
  #define  enable_e5() ;
  #define disable_e5() ;

#else // !COLOR_MIXING_EXTRUDER

  #if HAS(E0_ENABLE)
    #define  enable_e0() E0_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_e0() E0_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_e0() /* nothing */
    #define disable_e0() /* nothing */
  #endif

  #if (DRIVER_EXTRUDERS > 1) && HAS(E1_ENABLE)
    #define  enable_e1() E1_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_e1() E1_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_e1() /* nothing */
    #define disable_e1() /* nothing */
  #endif

  #if (DRIVER_EXTRUDERS > 2) && HAS(E2_ENABLE)
    #define  enable_e2() E2_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_e2() E2_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_e2() /* nothing */
    #define disable_e2() /* nothing */
  #endif

  #if (DRIVER_EXTRUDERS > 3) && HAS(E3_ENABLE)
    #define  enable_e3() E3_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_e3() E3_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_e3() /* nothing */
    #define disable_e3() /* nothing */
  #endif

  #if (DRIVER_EXTRUDERS > 4) && HAS(E4_ENABLE)
    #define  enable_e4() E4_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_e4() E4_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_e4() /* nothing */
    #define disable_e4() /* nothing */
  #endif

  #if (DRIVER_EXTRUDERS > 5) && HAS(E5_ENABLE)
    #define  enable_e5() E5_ENABLE_WRITE( E_ENABLE_ON)
    #define disable_e5() E5_ENABLE_WRITE(!E_ENABLE_ON)
  #else
    #define  enable_e5() /* nothing */
    #define disable_e5() /* nothing */
  #endif

#endif

#define disable_e() { disable_e0(); disable_e1(); disable_e2(); disable_e3(); disable_e4(); disable_e5(); }

//////////////////////////////////
// Pin redefines for TMC drivers. 
// TMC26X drivers have step and dir on normal pins, but everything else via SPI
//////////////////////////////////
#if ENABLED(HAVE_TMCDRIVER)
#include <SPI.h>
#include <TMC26XStepper.h>

  void tmc_init();
#if ENABLED(X_IS_TMC)
   extern TMC26XStepper stepperX;
   #undef X_ENABLE_INIT 
   #define X_ENABLE_INIT ((void)0)
   
   #undef X_ENABLE_WRITE
   #define X_ENABLE_WRITE(STATE) stepperX.setEnabled(STATE)
   
   #undef X_ENABLE_READ
   #define X_ENABLE_READ stepperX.isEnabled()
   
#endif
#if ENABLED(X2_IS_TMC)
   extern TMC26XStepper stepperX2;
   #undef X2_ENABLE_INIT
   #define X2_ENABLE_INIT ((void)0)
   
   #undef X2_ENABLE_WRITE
   #define X2_ENABLE_WRITE(STATE) stepperX2.setEnabled(STATE)
   
   #undef X2_ENABLE_READ
   #define X2_ENABLE_READ stepperX2.isEnabled()   
#endif
#if ENABLED(Y_IS_TMC)
   extern TMC26XStepper stepperY;
   #undef Y_ENABLE_INIT
   #define Y_ENABLE_INIT ((void)0)
   
   #undef Y_ENABLE_WRITE
   #define Y_ENABLE_WRITE(STATE) stepperY.setEnabled(STATE)
   
   #undef Y_ENABLE_READ
   #define Y_ENABLE_READ stepperY.isEnabled()   
#endif
#if ENABLED(Y2_IS_TMC)
   extern TMC26XStepper stepperY2;
   #undef Y2_ENABLE_INIT
   #define Y2_ENABLE_INIT ((void)0)
   
   #undef Y2_ENABLE_WRITE
   #define Y2_ENABLE_WRITE(STATE) stepperY2.setEnabled(STATE)
   
   #undef Y2_ENABLE_READ
   #define Y2_ENABLE_READ stepperY2.isEnabled()     
#endif
#if ENABLED(Z_IS_TMC)
   extern TMC26XStepper stepperZ;
   #undef Z_ENABLE_INIT
   #define Z_ENABLE_INIT ((void)0)
   
   #undef Z_ENABLE_WRITE
   #define Z_ENABLE_WRITE(STATE) stepperZ.setEnabled(STATE)
   
   #undef Z_ENABLE_READ
   #define Z_ENABLE_READ stepperZ.isEnabled()       
#endif
#if ENABLED(Z2_IS_TMC)
   extern TMC26XStepper stepperZ2;
   #undef Z2_ENABLE_INIT
   #define Z2_ENABLE_INIT ((void)0)
   
   #undef Z2_ENABLE_WRITE
   #define Z2_ENABLE_WRITE(STATE) stepperZ2.setEnabled(STATE)
   
   #undef Z2_ENABLE_READ
   #define Z2_ENABLE_READ stepperZ2.isEnabled()   
#endif
#if ENABLED(E0_IS_TMC)
   extern TMC26XStepper stepperE0;
   #undef E0_ENABLE_INIT
   #define E0_ENABLE_INIT ((void)0)
   
   #undef E0_ENABLE_WRITE
   #define E0_ENABLE_WRITE(STATE) stepperE0.setEnabled(STATE)
   
   #undef E0_ENABLE_READ
   #define E0_ENABLE_READ stepperE0.isEnabled()   
#endif
#if ENABLED(E1_IS_TMC)
   extern TMC26XStepper stepperE1;
   #undef E1_ENABLE_INIT
   #define E1_ENABLE_INIT ((void)0)
   
   #undef E1_ENABLE_WRITE
   #define E1_ENABLE_WRITE(STATE) stepperE1.setEnabled(STATE)
   
   #undef E1_ENABLE_READ
   #define E1_ENABLE_READ stepperE1.isEnabled()   
#endif
#if ENABLED(E2_IS_TMC)
   extern TMC26XStepper stepperE2;
   #undef E2_ENABLE_INIT
   #define E2_ENABLE_INIT ((void)0)
   
   #undef E2_ENABLE_WRITE
   #define E2_ENABLE_WRITE(STATE) stepperE2.setEnabled(STATE)
   
   #undef E2_ENABLE_READ
   #define E2_ENABLE_READ stepperE2.isEnabled()   
#endif
#if ENABLED(E3_IS_TMC)
   extern TMC26XStepper stepperE3;
   #undef E3_ENABLE_INIT
   #define E3_ENABLE_INIT ((void)0)
   
   #undef E3_ENABLE_WRITE
   #define E3_ENABLE_WRITE(STATE) stepperE3.setEnabled(STATE)
   
   #undef E3_ENABLE_READ
   #define E3_ENABLE_READ stepperE3.isEnabled()   
#endif

#endif  // HAVE_TMCDRIVER

//////////////////////////////////
// Pin redefines for L6470 drivers. 
// L640 drivers have step on normal pins, but dir and everything else via SPI
//////////////////////////////////
#if ENABLED(HAVE_L6470DRIVER)

#include <SPI.h>
#include <L6470.h>

  void L6470_init();
#if ENABLED(X_IS_L6470)
   extern L6470 stepperX;
   #undef X_ENABLE_INIT 
   #define X_ENABLE_INIT ((void)0)
   
   #undef X_ENABLE_WRITE
   #define X_ENABLE_WRITE(STATE) {if(STATE) stepperX.Step_Clock(stepperX.getStatus() & STATUS_HIZ); else stepperX.softFree();}
   
   #undef X_ENABLE_READ
   #define X_ENABLE_READ (stepperX.getStatus() & STATUS_HIZ)
   
   #undef X_DIR_INIT 
   #define X_DIR_INIT ((void)0)
   
   #undef X_DIR_WRITE
   #define X_DIR_WRITE(STATE) stepperX.Step_Clock(STATE)
   
   #undef X_DIR_READ
   #define X_DIR_READ (stepperX.getStatus() & STATUS_DIR)
   
#endif
#if ENABLED(X2_IS_L6470)
   extern L6470 stepperX2;
   #undef X2_ENABLE_INIT
   #define X2_ENABLE_INIT ((void)0)
   
   #undef X2_ENABLE_WRITE
   #define X2_ENABLE_WRITE(STATE) (if(STATE) stepperX2.Step_Clock(stepperX2.getStatus() & STATUS_HIZ); else stepperX2.softFree();)
   
   #undef X2_ENABLE_READ
   #define X2_ENABLE_READ (stepperX2.getStatus() & STATUS_HIZ)
   
   #undef X2_DIR_INIT 
   #define X2_DIR_INIT ((void)0)
   
   #undef X2_DIR_WRITE
   #define X2_DIR_WRITE(STATE) stepperX2.Step_Clock(STATE)
   
   #undef X2_DIR_READ
   #define X2_DIR_READ (stepperX2.getStatus() & STATUS_DIR)
#endif
#if ENABLED(Y_IS_L6470)
   extern L6470 stepperY;
   #undef Y_ENABLE_INIT
   #define Y_ENABLE_INIT ((void)0)
   
   #undef Y_ENABLE_WRITE
   #define Y_ENABLE_WRITE(STATE) (if(STATE) stepperY.Step_Clock(stepperY.getStatus() & STATUS_HIZ); else stepperY.softFree();)
   
   #undef Y_ENABLE_READ
   #define Y_ENABLE_READ (stepperY.getStatus() & STATUS_HIZ)
   
   #undef Y_DIR_INIT 
   #define Y_DIR_INIT ((void)0)
   
   #undef Y_DIR_WRITE
   #define Y_DIR_WRITE(STATE) stepperY.Step_Clock(STATE)
   
   #undef Y_DIR_READ
   #define Y_DIR_READ (stepperY.getStatus() & STATUS_DIR)  
#endif
#if ENABLED(Y2_IS_L6470)
   extern L6470 stepperY2;
   #undef Y2_ENABLE_INIT
   #define Y2_ENABLE_INIT ((void)0)
   
   #undef Y2_ENABLE_WRITE
   #define Y2_ENABLE_WRITE(STATE) (if(STATE) stepperY2.Step_Clock(stepperY2.getStatus() & STATUS_HIZ); else stepperY2.softFree();)
   
   #undef Y2_ENABLE_READ
   #define Y2_ENABLE_READ (stepperY2.getStatus() & STATUS_HIZ)
   
   #undef Y2_DIR_INIT 
   #define Y2_DIR_INIT ((void)0)
   
   #undef Y2_DIR_WRITE
   #define Y2_DIR_WRITE(STATE) stepperY2.Step_Clock(STATE)
   
   #undef Y2_DIR_READ
   #define Y2_DIR_READ (stepperY2.getStatus() & STATUS_DIR)   
#endif
#if ENABLED(Z_IS_L6470)
   extern L6470 stepperZ;
   #undef Z_ENABLE_INIT
   #define Z_ENABLE_INIT ((void)0)
   
   #undef Z_ENABLE_WRITE
   #define Z_ENABLE_WRITE(STATE) (if(STATE) stepperZ.Step_Clock(stepperZ.getStatus() & STATUS_HIZ); else stepperZ.softFree();)
   
   #undef Z_ENABLE_READ
   #define Z_ENABLE_READ (stepperZ.getStatus() & STATUS_HIZ)
   
   #undef Z_DIR_INIT 
   #define Z_DIR_INIT ((void)0)
   
   #undef Z_DIR_WRITE
   #define Z_DIR_WRITE(STATE) stepperZ.Step_Clock(STATE)
   
   #undef Y_DIR_READ
   #define Y_DIR_READ (stepperZ.getStatus() & STATUS_DIR)      
#endif
#if ENABLED(Z2_IS_L6470)
   extern L6470 stepperZ2;
   #undef Z2_ENABLE_INIT
   #define Z2_ENABLE_INIT ((void)0)
   
   #undef Z2_ENABLE_WRITE
   #define Z2_ENABLE_WRITE(STATE) (if(STATE) stepperZ2.Step_Clock(stepperZ2.getStatus() & STATUS_HIZ); else stepperZ2.softFree();)
   
   #undef Z2_ENABLE_READ
   #define Z2_ENABLE_READ (stepperZ2.getStatus() & STATUS_HIZ)
   
   #undef Z2_DIR_INIT 
   #define Z2_DIR_INIT ((void)0)
   
   #undef Z2_DIR_WRITE
   #define Z2_DIR_WRITE(STATE) stepperZ2.Step_Clock(STATE)
   
   #undef Y2_DIR_READ
   #define Y2_DIR_READ (stepperZ2.getStatus() & STATUS_DIR)       
#endif
#if ENABLED(E0_IS_L6470)
   extern L6470 stepperE0;
   #undef E0_ENABLE_INIT
   #define E0_ENABLE_INIT ((void)0)
   
   #undef E0_ENABLE_WRITE
   #define E0_ENABLE_WRITE(STATE) (if(STATE) stepperE0.Step_Clock(stepperE0.getStatus() & STATUS_HIZ); else stepperE0.softFree();)
   
   #undef E0_ENABLE_READ
   #define E0_ENABLE_READ (stepperE0.getStatus() & STATUS_HIZ)
   
   #undef E0_DIR_INIT 
   #define E0_DIR_INIT ((void)0)
   
   #undef E0_DIR_WRITE
   #define E0_DIR_WRITE(STATE) stepperE0.Step_Clock(STATE)
   
   #undef E0_DIR_READ
   #define E0_DIR_READ (stepperE0.getStatus() & STATUS_DIR)    
#endif
#if ENABLED(E1_IS_L6470)
   extern L6470 stepperE1;
   #undef E1_ENABLE_INIT
   #define E1_ENABLE_INIT ((void)0)
   
   #undef E1_ENABLE_WRITE
   #define E1_ENABLE_WRITE(STATE) (if(STATE) stepperE1.Step_Clock(stepperE1.getStatus() & STATUS_HIZ); else stepperE1.softFree();)
   
   #undef E1_ENABLE_READ
   #define E1_ENABLE_READ (stepperE1.getStatus() & STATUS_HIZ)
   
   #undef E1_DIR_INIT 
   #define E1_DIR_INIT ((void)0)
   
   #undef E1_DIR_WRITE
   #define E1_DIR_WRITE(STATE) stepperE1.Step_Clock(STATE)
   
   #undef E1_DIR_READ
   #define E1_DIR_READ (stepperE1.getStatus() & STATUS_DIR)  
#endif
#if ENABLED(E2_IS_L6470)
   extern L6470 stepperE2;
   #undef E2_ENABLE_INIT
   #define E2_ENABLE_INIT ((void)0)
   
   #undef E2_ENABLE_WRITE
   #define E2_ENABLE_WRITE(STATE) (if(STATE) stepperE2.Step_Clock(stepperE2.getStatus() & STATUS_HIZ); else stepperE2.softFree();)
   
   #undef E2_ENABLE_READ
   #define E2_ENABLE_READ (stepperE2.getStatus() & STATUS_HIZ)
   
   #undef E2_DIR_INIT 
   #define E2_DIR_INIT ((void)0)
   
   #undef E2_DIR_WRITE
   #define E2_DIR_WRITE(STATE) stepperE2.Step_Clock(STATE)
   
   #undef E2_DIR_READ
   #define E2_DIR_READ (stepperE2.getStatus() & STATUS_DIR)  
#endif
#if ENABLED(E3_IS_L6470)
   extern L6470 stepperE3;
   #undef E3_ENABLE_INIT
   #define E3_ENABLE_INIT ((void)0)
   
   #undef E3_ENABLE_WRITE
   #define E3_ENABLE_WRITE(STATE) (if(STATE) stepperE3.Step_Clock(stepperE3.getStatus() & STATUS_HIZ); else stepperE3.softFree();)
   
   #undef E3_ENABLE_READ
   #define E3_ENABLE_READ (stepperE3.getStatus() & STATUS_HIZ)
   
   #undef E3_DIR_INIT 
   #define E3_DIR_INIT ((void)0)
   
   #undef E3_DIR_WRITE
   #define E3_DIR_WRITE(STATE) stepperE3.Step_Clock(STATE)
   
   #undef E3_DIR_READ
   #define E3_DIR_READ (stepperE3.getStatus() & STATUS_DIR)  
#endif

#endif  //HAVE_L6470DRIVER

#endif // STEPPER_INDIRECTION_H