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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * endstops.cpp - A singleton object to manage endstops
 */

#include "../../base.h"
#include "endstops.h"

// TEST_ENDSTOP: test the old and the current status of an endstop
#define TEST_ENDSTOP(ENDSTOP) (TEST(current_endstop_bits & old_endstop_bits, ENDSTOP))

Endstops endstops;

Endstops::Endstops() {
  enable_globally(
    #if ENABLED(ENDSTOPS_ONLY_FOR_HOMING)
      false
    #else
      true
    #endif
  );
  enable(true);
  #if ENABLED(HAS_Z_PROBE)
    enable_z_probe(false);
  #endif
} // Endstops::Endstops

void Endstops::init() {

  #if HAS(X_MIN)
    SET_INPUT(X_MIN_PIN);
    #if ENABLED(ENDSTOPPULLUP_XMIN)
      PULLUP(X_MIN_PIN, HIGH);
    #endif
  #endif

  #if HAS(Y_MIN)
    SET_INPUT(Y_MIN_PIN);
    #if ENABLED(ENDSTOPPULLUP_YMIN)
      PULLUP(Y_MIN_PIN, HIGH);
    #endif
  #endif

  #if HAS(Z_MIN)
    SET_INPUT(Z_MIN_PIN);
    #if ENABLED(ENDSTOPPULLUP_ZMIN)
      PULLUP(Z_MIN_PIN, HIGH);
    #endif
  #endif

  #if HAS(Z2_MIN)
    SET_INPUT(Z2_MIN_PIN);
    #if ENABLED(ENDSTOPPULLUP_Z2MIN)
      PULLUP(Z2_MIN_PIN, HIGH);
    #endif
  #endif

  #if HAS(E_MIN)
    SET_INPUT(E_MIN_PIN);
    #if ENABLED(ENDSTOPPULLUP_EMIN)
      PULLUP(E_MIN_PIN, HIGH);
    #endif
  #endif

  #if HAS(X_MAX)
    SET_INPUT(X_MAX_PIN);
    #if ENABLED(ENDSTOPPULLUP_XMAX)
      PULLUP(X_MAX_PIN, HIGH);
    #endif
  #endif

  #if HAS(Y_MAX)
    SET_INPUT(Y_MAX_PIN);
    #if ENABLED(ENDSTOPPULLUP_YMAX)
      PULLUP(Y_MAX_PIN, HIGH);
    #endif
  #endif

  #if HAS(Z_MAX)
    SET_INPUT(Z_MAX_PIN);
    #if ENABLED(ENDSTOPPULLUP_ZMAX)
      PULLUP(Z_MAX_PIN, HIGH);
    #endif
  #endif

  #if HAS(Z2_MAX)
    SET_INPUT(Z2_MAX_PIN);
    #if ENABLED(ENDSTOPPULLUP_Z2MAX)
      PULLUP(Z2_MAX_PIN, HIGH);
    #endif
  #endif

  #if HAS(Z_PROBE) // Check for Z_PROBE_ENDSTOP so we don't pull a pin high unless it's to be used.
    SET_INPUT(Z_PROBE_PIN);
    #if ENABLED(ENDSTOPPULLUP_ZPROBE)
      PULLUP(Z_PROBE_PIN, HIGH);
    #endif
  #endif

} // Endstops::init

void Endstops::report_state() {
  if (endstop_hit_bits) {
    #if ENABLED(ULTRA_LCD)
      char chrX = ' ', chrY = ' ', chrZ = ' ', chrP = ' ';
      #define _SET_STOP_CHAR(A,C) (chr## A = C)
    #else
      #define _SET_STOP_CHAR(A,C) ;
    #endif

    #define _ENDSTOP_HIT_ECHO(A,C) do{ \
      ECHO_MV(" " STRINGIFY(A) ":", triggered_position_mm(A ##_AXIS)); \
      _SET_STOP_CHAR(A,C); }while(0)

    #define _ENDSTOP_HIT_TEST(A,C) \
      if (TEST(endstop_hit_bits, A ##_MIN) || TEST(endstop_hit_bits, A ##_MAX)) \
        _ENDSTOP_HIT_ECHO(A,C)

    ECHO_SM(ER, SERIAL_ENDSTOPS_HIT);
    _ENDSTOP_HIT_TEST(X, 'X');
    _ENDSTOP_HIT_TEST(Y, 'Y');
    _ENDSTOP_HIT_TEST(Z, 'Z');

    #if ENABLED(Z_PROBE_ENDSTOP)
      #define P_AXIS Z_AXIS
      if (TEST(endstop_hit_bits, Z_PROBE)) _ENDSTOP_HIT_ECHO(P, 'P');
    #endif
    ECHO_E;

    #if ENABLED(ULTRA_LCD)
      char msg[3 * strlen(MSG_ENDSTOPS_HIT) + 8 + 1]; // Room for a UTF 8 string
      sprintf_P(msg, PSTR(MSG_ENDSTOPS_HIT " %c %c %c %c"), chrX, chrY, chrZ, chrP);
      lcd_setstatus(msg);
    #endif

    hit_on_purpose();

    #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED) && ENABLED(SDSUPPORT)
      if (abort_on_endstop_hit) {
        card.sdprinting = false;
        card.closeFile();
        quickStop();
        disable_all_heaters(); // switch off all heaters.
        disable_all_coolers();
      }
    #endif
  }
} // Endstops::report_state

void Endstops::M119() {
  ECHO_LM(DB, SERIAL_M119_REPORT);
  #if HAS(X_MIN)
    ECHO_EMT(SERIAL_X_MIN, ((READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(X_MAX)
    ECHO_EMT(SERIAL_X_MAX, ((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(Y_MIN)
    ECHO_EMT(SERIAL_Y_MIN, ((READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(Y_MAX)
    ECHO_EMT(SERIAL_Y_MAX, ((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(Z_MIN)
    ECHO_EMT(SERIAL_Z_MIN, ((READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(Z_MAX)
    ECHO_EMT(SERIAL_Z_MAX, ((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(Z2_MAX)
    ECHO_EMT(SERIAL_Z2_MAX, ((READ(Z2_MAX_PIN)^Z2_MAX_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(Z_PROBE)
    ECHO_EMT(SERIAL_Z_PROBE, ((READ(Z_PROBE_PIN)^Z_PROBE_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(E_MIN)
    ECHO_EMT(SERIAL_E_MIN, ((READ(E_MIN_PIN)^E_MIN_ENDSTOP_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
  #if HAS(FILRUNOUT)
    ECHO_EMT(SERIAL_FILRUNOUT_PIN, ((READ(FILRUNOUT_PIN)^FILRUNOUT_PIN_INVERTING)?SERIAL_ENDSTOP_HIT:SERIAL_ENDSTOP_OPEN));
  #endif
} // Endstops::M119

#if ENABLED(Z_DUAL_ENDSTOPS)

  // Pass the result of the endstop test
  void Endstops::test_dual_z_endstops(EndstopEnum es1, EndstopEnum es2) {
    byte z_test = TEST_ENDSTOP(es1) | (TEST_ENDSTOP(es2) << 1); // bit 0 for Z, bit 1 for Z2
    if (current_block->steps[Z_AXIS] > 0) {
      endstop_triggered(Z_AXIS);
      SBI(endstop_hit_bits, Z_MIN);
      if (!performing_homing || (z_test == 0x3))  //if not performing home or if both endstops were trigged during homing...
        kill_current_block();
    }
  }

#endif

// Check endstops - Called from ISR!
void Endstops::update() {

  #define _ENDSTOP_PIN(AXIS, MINMAX) AXIS ##_## MINMAX ##_PIN
  #define _ENDSTOP_INVERTING(AXIS, MINMAX) AXIS ##_## MINMAX ##_ENDSTOP_INVERTING
  #define _ENDSTOP_HIT(AXIS) SBI(endstop_hit_bits, _ENDSTOP(AXIS, MIN))
  #define _ENDSTOP(AXIS, MINMAX) AXIS ##_## MINMAX

  // UPDATE_ENDSTOP_BIT: set the current endstop bits for an endstop to its status
  #define UPDATE_ENDSTOP_BIT(AXIS, MINMAX) SET_BIT(current_endstop_bits, _ENDSTOP(AXIS, MINMAX), (READ(_ENDSTOP_PIN(AXIS, MINMAX)) != _ENDSTOP_INVERTING(AXIS, MINMAX)))
  // COPY_BIT: copy the value of COPY_BIT to BIT in bits
  #define COPY_BIT(bits, COPY_BIT, BIT) SET_BIT(bits, BIT, TEST(bits, COPY_BIT))

  #define UPDATE_ENDSTOP(AXIS,MINMAX) do { \
      UPDATE_ENDSTOP_BIT(AXIS, MINMAX); \
      if (TEST_ENDSTOP(_ENDSTOP(AXIS, MINMAX)) && current_block->steps[_AXIS(AXIS)] > 0) { \
        _ENDSTOP_HIT(AXIS); \
        endstop_triggered(_AXIS(AXIS)); \
      } \
    } while(0)

  #if MECH(COREXY) || MECH(COREYX)|| MECH(COREXZ) || MECH(COREZX)
    // Head direction in -X axis for CoreXY and CoreXZ bots.
    // If Delta1 == -Delta2, the movement is only in Y or Z axis
    if ((current_block->steps[A_AXIS] != current_block->steps[CORE_AXIS_2]) || (motor_direction(A_AXIS) == motor_direction(CORE_AXIS_2))) {
      if (motor_direction(X_HEAD))
  #else
    if (motor_direction(X_AXIS))   // stepping along -X axis (regular Cartesian bot)
  #endif
      { // -direction
        #if ENABLED(DUAL_X_CARRIAGE)
          // with 2 x-carriages, endstops are only checked in the homing direction for the active extruder
          if ((current_block->active_driver == 0 && X_HOME_DIR == -1) || (current_block->active_driver != 0 && X2_HOME_DIR == -1))
        #endif
          {
            #if HAS(X_MIN)
              UPDATE_ENDSTOP(X, MIN);
            #endif
          }
      }
      else { // +direction
        #if ENABLED(DUAL_X_CARRIAGE)
          // with 2 x-carriages, endstops are only checked in the homing direction for the active extruder
          if ((current_block->active_driver == 0 && X_HOME_DIR == 1) || (current_block->active_driver != 0 && X2_HOME_DIR == 1))
        #endif
          {
            #if HAS(X_MAX)
              UPDATE_ENDSTOP(X, MAX);
            #endif
          }
      }
  #if MECH(COREXY) || MECH(COREYX) || MECH(COREXZ) || MECH(COREZX)
    }
  #endif

  #if MECH(COREXY) || MECH(COREYX)
    // Head direction in -Y axis for CoreXY bots.
    // If DeltaX == DeltaY, the movement is only in X axis
    if ((current_block->steps[A_AXIS] != current_block->steps[B_AXIS]) || (motor_direction(A_AXIS) != motor_direction(B_AXIS))) {
      if (motor_direction(Y_HEAD))
  #else
      if (motor_direction(Y_AXIS))   // -direction
  #endif
      { // -direction
        #if HAS(Y_MIN)
          UPDATE_ENDSTOP(Y, MIN);
        #endif
      }
      else { // +direction
        #if HAS(Y_MAX)
          UPDATE_ENDSTOP(Y, MAX);
        #endif
      }
  #if MECH(COREXY) || MECH(COREYX)
    }
  #endif

  #if MECH(COREXZ) || MECH(COREZX)
    // Head direction in -Z axis for CoreXZ bots.
    // If DeltaX == DeltaZ, the movement is only in X axis
    if ((current_block->steps[A_AXIS] != current_block->steps[C_AXIS]) || (motor_direction(A_AXIS) != motor_direction(C_AXIS))) {
      if (motor_direction(Z_HEAD))
  #else
      if (motor_direction(Z_AXIS))
  #endif
      { // z -direction
        #if HAS(Z_MIN)

          #if ENABLED(Z_DUAL_ENDSTOPS)
            UPDATE_ENDSTOP_BIT(Z, MIN);
            #if HAS_Z2_MIN
              UPDATE_ENDSTOP_BIT(Z2, MIN);
            #else
              COPY_BIT(current_endstop_bits, Z_MIN, Z2_MIN);
            #endif

            test_dual_z_endstops(Z_MIN, Z2_MIN);

          #else // !Z_DUAL_ENDSTOPS

            UPDATE_ENDSTOP(Z, MIN);

          #endif // !Z_DUAL_ENDSTOPS
        #endif // HAS_Z_MIN

        #if HAS(Z_PROBE)
          if (z_probe_enabled) {
            UPDATE_ENDSTOP(Z, PROBE);
            if (TEST_ENDSTOP(Z_PROBE)) SBI(endstop_hit_bits, Z_PROBE);
          }
        #endif
      }
      else { // z +direction
        #if HAS(Z_MAX)

          #if ENABLED(Z_DUAL_ENDSTOPS)

            UPDATE_ENDSTOP_BIT(Z, MAX);
            #if HAS_Z2_MAX
              UPDATE_ENDSTOP_BIT(Z2, MAX);
            #else
              COPY_BIT(current_endstop_bits, Z_MAX, Z2_MAX);
            #endif

            test_dual_z_endstops(Z_MAX, Z2_MAX);

          #else // !Z_DUAL_ENDSTOPS

            UPDATE_ENDSTOP(Z, MAX);

          #endif // !Z_DUAL_ENDSTOPS
        #endif // Z_MAX_PIN
      }
  #if MECH(COREXZ) || MECH(COREZX)
    }
  #endif

  #if ENABLED(NPR2)
    UPDATE_ENDSTOP(E, MIN);
  #endif

  old_endstop_bits = current_endstop_bits;

} // Endstops::update()
