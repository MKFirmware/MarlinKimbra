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

#include "../../base.h"

#if ENABLED(FIRMWARE_TEST)

#include "firmware_test.h"

static char serial_answer;

void FirmwareTest() {
  ECHO_EM("---------- FIRMWARE TEST --------------");
  ECHO_EM("--------------- MK --------------------");
  ECHO_EV(MSG_FWTEST_01);
  ECHO_EV(MSG_FWTEST_02);
  ECHO_EV(MSG_FWTEST_YES_NO);
  serial_answer = ' ';
  while (serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N') {
    serial_answer = MKSERIAL.read();
  }
  if (serial_answer=='y' || serial_answer=='Y') {
    ECHO_EV(MSG_FWTEST_03);

    ECHO_EM(" ");
    ECHO_EM("***** ENDSTOP X *****");
    #if PIN_EXISTS(X_MIN) && (X_HOME_DIR == -1)
      if (!READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING) {
        ECHO_M("MIN ENDSTOP X: ");
        ECHO_EV(SERIAL_ENDSTOP_OPEN);
      }
      else {
        ECHO_M("X ENDSTOP ");
        ECHO_EM(MSG_FWTEST_ERROR);
        ECHO_M(MSG_FWTEST_INVERT);
        ECHO_M("#define X_MIN_ENDSTOP_LOGIC ");
        ECHO_M(MSG_FWTEST_INTO);
        #if MECH(CARTESIAN)
          ECHO_EM("Configuration_Cartesian.h");
        #elif MECH(COREXY)
          ECHO_EM("Configuration_Core.h");
        #elif MECH(COREXZ)
          ECHO_EM("Configuration_Core.h");
        #elif MECH(DELTA)
          ECHO_EM("Configuration_Delta.h");
        #elif MECH(SCARA)
          ECHO_EM("Configuration_Scara.h");
        #endif
        return;
      }
      ECHO_V(MSG_FWTEST_PRESS);
      ECHO_EM("X");
      ECHO_EV(MSG_FWTEST_YES);
      serial_answer = ' ';
      while (serial_answer!='y' && serial_answer!='Y' && !(READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)) {
        serial_answer = MKSERIAL.read();
      }
      if (READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING) {
        ECHO_M("MIN ENDSTOP X: ");
        ECHO_EV(SERIAL_ENDSTOP_HIT);
      }
      else {
        ECHO_M("X ");
        ECHO_EV(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif PIN_EXISTS(X_MAX) && X_HOME_DIR == 1
      if (!READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING) {
        ECHO_M("MAX ENDSTOP X: ");
        ECHO_EV(SERIAL_ENDSTOP_OPEN);
      }
      else {
        ECHO_M("X ENDSTOP ");
        ECHO_EM(MSG_FWTEST_ERROR);
        ECHO_M(MSG_FWTEST_INVERT);
        ECHO_M("#define X_MAX_ENDSTOP_LOGIC ");
        ECHO_M(MSG_FWTEST_INTO);
        #if MECH(CARTESIAN)
          ECHO_EM("Configuration_Cartesian.h");
        #elif MECH(COREXY)
          ECHO_EM("Configuration_Core.h");
        #elif MECH(COREXZ)
          ECHO_EM("Configuration_Core.h");
        #elif MECH(DELTA)
          ECHO_EM("Configuration_Delta.h");
        #elif MECH(SCARA)
          ECHO_EM("Configuration_Scara.h");
        #endif
        return;
      }
      ECHO_V(MSG_FWTEST_PRESS);
      ECHO_EM("X");
      ECHO_EV(MSG_FWTEST_YES);
      serial_answer = ' ';
      while (serial_answer!='y' && serial_answer!='Y' && !(READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)) {
        serial_answer = MKSERIAL.read();
      }
      if (READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING) {
        ECHO_M("MAX ENDSTOP X: ");
        ECHO_EV(SERIAL_ENDSTOP_HIT);
      }
      else {
        ECHO_M("X ");
        ECHO_EV(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif X_HOME_DIR == -1
      ECHO_M(MSG_FWTEST_ERROR);
      ECHO_M("!!! X_MIN_PIN ");
      ECHO_EM(MSG_FWTEST_NDEF);
      return;
    #elif X_HOME_DIR == 1
      ECHO_M(MSG_FWTEST_ERROR);
      ECHO_M("!!! X_MAX_PIN ");
      ECHO_EM(MSG_FWTEST_NDEF);
      return;
    #endif

    ECHO_EM(" ");
    ECHO_EM("***** ENDSTOP Y *****");
    #if PIN_EXISTS(Y_MIN) && Y_HOME_DIR == -1
      if (!READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING) {
        ECHO_M("MIN ENDSTOP Y: ");
        ECHO_EV(SERIAL_ENDSTOP_OPEN);
      }
      else {
        ECHO_M("Y ENDSTOP ");
        ECHO_EM(MSG_FWTEST_ERROR);
        ECHO_M(MSG_FWTEST_INVERT);
        ECHO_M("#define Y_MIN_ENDSTOP_LOGIC ");
        ECHO_M(MSG_FWTEST_INTO);
        #if MECH(CARTESIAN)
          ECHO_EM("Configuration_Cartesian.h");
        #elif MECH(COREXY)
          ECHO_EM("Configuration_Core.h");
        #elif MECH(COREXZ)
          ECHO_EM("Configuration_Core.h");
        #elif MECH(DELTA)
          ECHO_EM("Configuration_Delta.h");
        #elif MECH(SCARA)
          ECHO_EM("Configuration_Scara.h");
        #endif
        return;
      }
      ECHO_V(MSG_FWTEST_PRESS);
      ECHO_EM("Y");
      ECHO_EV(MSG_FWTEST_YES);
      serial_answer = ' ';
      while (serial_answer!='y' && serial_answer!='Y' && !(READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)) {
        serial_answer = MKSERIAL.read();
      }
      if (READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING) {
        ECHO_M("MIN ENDSTOP Y: ");
        ECHO_EV(SERIAL_ENDSTOP_HIT);
      }
      else {
        ECHO_M("Y ");
        ECHO_EV(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif PIN_EXISTS(Y_MAX) && Y_HOME_DIR == 1
      if (!READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING) {
        ECHO_M("MAX ENDSTOP Y: ");
        ECHO_EV(SERIAL_ENDSTOP_OPEN);
      }
      else {
        ECHO_M("Y ENDSTOP ");
        ECHO_EM(MSG_FWTEST_ERROR);
        ECHO_M(MSG_FWTEST_INVERT);
        ECHO_M("#define Y_MAX_ENDSTOP_LOGIC ");
        ECHO_M(MSG_FWTEST_INTO);
        #if MECH(CARTESIAN)
          ECHO_EM("Configuration_Cartesian.h");
        #elif MECH(COREXY)
          ECHO_EM("Configuration_Core.h");
        #elif MECH(COREXZ)
          ECHO_EM("Configuration_Core.h");
        #elif MECH(DELTA)
          ECHO_EM("Configuration_Delta.h");
        #elif MECH(SCARA)
          ECHO_EM("Configuration_Scara.h");
        #endif
        return;
      }
      ECHO_V(MSG_FWTEST_PRESS);
      ECHO_EM("Y");
      ECHO_EV(MSG_FWTEST_YES);
      serial_answer = ' ';
      while (serial_answer!='y' && serial_answer!='Y' && !(READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)) {
        serial_answer = MKSERIAL.read();
      }
      if (READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING) {
        ECHO_M("MAX ENDSTOP Y: ");
        ECHO_EV(SERIAL_ENDSTOP_HIT);
      }
      else {
        ECHO_M("Y ");
        ECHO_EV(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif Y_HOME_DIR == -1
      ECHO_M(MSG_FWTEST_ERROR);
      ECHO_M("!!! Y_MIN_PIN ");
      ECHO_EM(MSG_FWTEST_NDEF);
      return;
    #elif Y_HOME_DIR == 1
      ECHO_M(MSG_FWTEST_ERROR);
      ECHO_M("!!! Y_MAX_PIN ");
      ECHO_EM(MSG_FWTEST_NDEF);
      return;
    #endif

    ECHO_EM(" ");
    ECHO_EM("***** ENDSTOP Z *****");
    #if PIN_EXISTS(Z_MIN) && Z_HOME_DIR == -1
      if (!READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING) {
        ECHO_M("MIN ENDSTOP Z: ");
        ECHO_EV(SERIAL_ENDSTOP_OPEN);
      }
      else {
        ECHO_M("Z ENDSTOP ");
        ECHO_EM(MSG_FWTEST_ERROR);
        ECHO_M(MSG_FWTEST_INVERT);
        ECHO_M("#define Z_MIN_ENDSTOP_LOGIC ");
        ECHO_M(MSG_FWTEST_INTO);
        #if MECH(CARTESIAN)
          ECHO_EM("Configuration_Cartesian.h");
        #elif MECH(COREXY)
          ECHO_EM("Configuration_Core.h");
        #elif MECH(COREXZ)
          ECHO_EM("Configuration_Core.h");
        #elif MECH(DELTA)
          ECHO_EM("Configuration_Delta.h");
        #elif MECH(SCARA)
          ECHO_EM("Configuration_Scara.h");
        #endif
        return;
      }
      ECHO_V(MSG_FWTEST_PRESS);
      ECHO_EM("Z");
      ECHO_EV(MSG_FWTEST_YES);
      serial_answer = ' ';
      while (serial_answer!='y' && serial_answer!='Y' && !(READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)) {
        serial_answer = MKSERIAL.read();
      }
      if (READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING) {
        ECHO_M("MIN ENDSTOP Z: ");
        ECHO_EV(SERIAL_ENDSTOP_HIT);
      }
      else {
        ECHO_M("Z ");
        ECHO_EV(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif PIN_EXISTS(Z_MAX) && Z_HOME_DIR == 1
      if (!READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING) {
        ECHO_M("MAX ENDSTOP Z: ");
        ECHO_EV(SERIAL_ENDSTOP_OPEN);
      }
      else {
        ECHO_M("Z ENDSTOP ");
        ECHO_EM(MSG_FWTEST_ERROR);
        ECHO_M(MSG_FWTEST_INVERT);
        ECHO_M("#define Z_MAX_ENDSTOP_LOGIC ");
        ECHO_M(MSG_FWTEST_INTO);
        #if MECH(CARTESIAN)
          ECHO_EM("Configuration_Cartesian.h");
        #elif MECH(COREXY)
          ECHO_EM("Configuration_Core.h");
        #elif MECH(COREXZ)
          ECHO_EM("Configuration_Core.h");
        #elif MECH(DELTA)
          ECHO_EM("Configuration_Delta.h");
        #elif MECH(SCARA)
          ECHO_EM("Configuration_Scara.h");
        #endif
        return;
      }
      ECHO_V(MSG_FWTEST_PRESS);
      ECHO_EM("Z");
      ECHO_EV(MSG_FWTEST_YES);
      serial_answer = ' ';
      while (serial_answer!='y' && serial_answer!='Y' && !(READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)) {
        serial_answer = MKSERIAL.read();
      }
      if (READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING) {
        ECHO_M("MAX ENDSTOP Z: ");
        ECHO_EV(SERIAL_ENDSTOP_HIT);
      }
      else {
        ECHO_M("Z ");
        ECHO_EV(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif Z_HOME_DIR == -1
      ECHO_M(MSG_FWTEST_ERROR);
      ECHO_M("!!! Z_MIN_PIN ");
      ECHO_EM(MSG_FWTEST_NDEF);
      return;
    #elif Z_HOME_DIR == 1
      ECHO_M(MSG_FWTEST_ERROR);
      ECHO_M("!!! Z_MAX_PIN ");
      ECHO_EM(MSG_FWTEST_NDEF);
      return;
    #endif

    ECHO_EM("ENDSTOP ");
    ECHO_M(MSG_FWTEST_OK);
    ECHO_EM(" ");
  }

  #if HAS(POWER_SWITCH)
    SET_OUTPUT(PS_ON_PIN);
    WRITE(PS_ON_PIN, PS_ON_AWAKE);
  #endif

  // Reset position to 0
  st_synchronize();
  for (int8_t i = 0; i < NUM_AXIS; i++) current_position[i] = 0;
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

  ECHO_EM("***** TEST MOTOR  *****");
  ECHO_EV(MSG_FWTEST_ATTENTION);
  ECHO_EV(MSG_FWTEST_YES);
  serial_answer = ' ';
  while (serial_answer!='y' && serial_answer!='Y') {
    serial_answer = MKSERIAL.read();
  }
  ECHO_EV(MSG_FWTEST_04);
  ECHO_EM(" ");
  ECHO_EM("***** MOTOR X *****");
  destination[X_AXIS] = 10;
  prepare_move();
  st_synchronize();

  ECHO_EV(MSG_FWTEST_XAXIS);
  ECHO_EV(MSG_FWTEST_YES_NO);
  serial_answer = ' ';
  while (serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N') {
    serial_answer = MKSERIAL.read();
  }
  if (serial_answer=='y' || serial_answer=='Y') {
    ECHO_EM("MOTOR X ");
    ECHO_M(MSG_FWTEST_OK);
  }
  else {
    ECHO_M(MSG_FWTEST_INVERT);
    ECHO_M("#define INVERT_X_DIR ");
    ECHO_M(MSG_FWTEST_INTO);
    #if MECH(CARTESIAN)
      ECHO_EM("Configuration_Cartesian.h");
    #elif MECH(COREXY)
      ECHO_EM("Configuration_Core.h");
    #elif MECH(COREXZ)
      ECHO_EM("Configuration_Core.h");
    #elif MECH(DELTA)
      ECHO_EM("Configuration_Delta.h");
    #elif MECH(SCARA)
      ECHO_EM("Configuration_Scara.h");
    #endif
    return;
  }
  ECHO_EM(" ");
  ECHO_EM("***** MOTOR Y *****");
  destination[Y_AXIS] = 10;
  prepare_move();
  st_synchronize();
  ECHO_EV(MSG_FWTEST_YAXIS);
  ECHO_EV(MSG_FWTEST_YES_NO);
  serial_answer = ' ';
  while (serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N') {
    serial_answer = MKSERIAL.read();
  }
  if (serial_answer=='y' || serial_answer=='Y') {
    ECHO_EM("MOTOR Y ");
    ECHO_M(MSG_FWTEST_OK);
  }
  else {
    ECHO_M(MSG_FWTEST_INVERT);
    ECHO_M("#define INVERT_Y_DIR ");
    ECHO_M(MSG_FWTEST_INTO);
    #if MECH(CARTESIAN)
      ECHO_EM("Configuration_Cartesian.h");
    #elif MECH(COREXY)
      ECHO_EM("Configuration_Core.h");
    #elif MECH(COREXZ)
      ECHO_EM("Configuration_Core.h");
    #elif MECH(DELTA)
      ECHO_EM("Configuration_Delta.h");
    #elif MECH(SCARA)
      ECHO_EM("Configuration_Scara.h");
    #endif
    return;
  }
  ECHO_EM(" ");
  ECHO_EM("***** MOTOR Z *****");
  destination[Z_AXIS] = 10;
  prepare_move();
  st_synchronize();
  ECHO_EV(MSG_FWTEST_ZAXIS);
  ECHO_EV(MSG_FWTEST_YES_NO);
  serial_answer = ' ';
  while (serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N') {
    serial_answer = MKSERIAL.read();
  }
  if (serial_answer=='y' || serial_answer=='Y') {
    ECHO_EM("MOTOR Z ");
    ECHO_M(MSG_FWTEST_OK);
  }
  else {
    ECHO_M(MSG_FWTEST_INVERT);
    ECHO_M("#define INVERT_Z_DIR ");
    ECHO_M(MSG_FWTEST_INTO);
    #if MECH(CARTESIAN)
      ECHO_EM("Configuration_Cartesian.h");
    #elif MECH(COREXY)
      ECHO_EM("Configuration_Core.h");
    #elif MECH(COREXZ)
      ECHO_EM("Configuration_Core.h");
    #elif MECH(DELTA)
      ECHO_EM("Configuration_Delta.h");
    #elif MECH(SCARA)
      ECHO_EM("Configuration_Scara.h");
    #endif
    return;
  }
  ECHO_EM("MOTOR ");
  ECHO_M(MSG_FWTEST_OK);
  ECHO_EM(" ");
  ECHO_V(MSG_FWTEST_END);
}
#endif
