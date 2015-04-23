/*
  Test.h
  Tools for firmware test
  By MagoKimbra
 */

#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "temperature.h"
#include "language.h"

static char serial_answer;

void FirmwareTest()
{
  ECHO_EM("---------- FIRMWARE TEST --------------");
  ECHO_EM("--------- by MarlinKimbra -------------");
  ECHO_EV(MSG_FWTEST_01);
  ECHO_EV(MSG_FWTEST_02);
  ECHO_EV(MSG_FWTEST_YES_NO);
  serial_answer = ' ';
  while(serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N') {
    serial_answer = MYSERIAL.read();
  }
  if (serial_answer=='y' || serial_answer=='Y') {
    ECHO_EV(MSG_FWTEST_03);

    ECHO_EM(" ");
    ECHO_EM("***** ENDSTOP X *****");
    #if defined(X_MIN_PIN) && X_MIN_PIN > -1 && X_HOME_DIR == -1
      if (!READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING) {
        ECHO_M("MIN ENDSTOP X: ");
        ECHO_EV(MSG_ENDSTOP_OPEN);
      }
      else
      {
        ECHO_EM("X ENDSTOP ERROR");
        ECHO_V(MSG_FWTEST_INVERT);
        ECHO_EM("#define X_MIN_ENDSTOP_INVERTING");
        return;
      }
      ECHO_V(MSG_FWTEST_PRESS);
      ECHO_EM("X");
      ECHO_EV(MSG_FWTEST_YES);
      serial_answer = ' ';
      while(serial_answer!='y' && serial_answer!='Y' && !(READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)){
        serial_answer = MYSERIAL.read();
      }
      if (READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING) {
        ECHO_M("MIN ENDSTOP X: ");
        ECHO_EV(MSG_ENDSTOP_HIT);
      }
      else
      {
        ECHO_M("X ");
        ECHO_EV(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif defined(X_MAX_PIN) && X_MAX_PIN > -1 && X_HOME_DIR == 1
      if (!READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING) {
        ECHO_M("MAX ENDSTOP X: ");
        ECHO_EV(MSG_ENDSTOP_OPEN);
      }
      else
      {
        ECHO_EM("X ENDSTOP ERROR");
        ECHO_V(MSG_FWTEST_INVERT);
        ECHO_EM("#define X_MAX_ENDSTOP_INVERTING");
        return;
      }
      ECHO_V(MSG_FWTEST_PRESS);
      ECHO_EM("X");
      ECHO_EV(MSG_FWTEST_YES);
      serial_answer = ' ';
      while(serial_answer!='y' && serial_answer!='Y' && !(READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)) {
        serial_answer = MYSERIAL.read();
      }
      if (READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING) {
        ECHO_M("MAX ENDSTOP X: ");
        ECHO_EV(MSG_ENDSTOP_HIT);
      }
      else
      {
        ECHO_M("X ");
        ECHO_EV(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif X_HOME_DIR == -1
      ECHO_EM("ERROR!!! X_MIN_PIN not defined");
      return;
    #elif X_HOME_DIR == 1
      ECHO_EM("ERROR!!! X_MAX_PIN not defined");
      return;
    #endif

    ECHO_EM(" ");
    ECHO_EM("***** ENDSTOP Y *****");
    #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1 && Y_HOME_DIR == -1
      if (!READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING){
        ECHO_M("MIN ENDSTOP Y: ");
        ECHO_EV(MSG_ENDSTOP_OPEN);
      }
      else
      {
        ECHO_EM("Y ENDSTOP ERROR");
        ECHO_V(MSG_FWTEST_INVERT);
        ECHO_EM("#define Y_MIN_ENDSTOP_INVERTING");
        return;
      }
      ECHO_V(MSG_FWTEST_PRESS);
      ECHO_EM("Y");
      ECHO_EV(MSG_FWTEST_YES);
      serial_answer = ' ';
      while(serial_answer!='y' && serial_answer!='Y' && !(READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)){
        serial_answer = MYSERIAL.read();
      }
      if (READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING){
        ECHO_M("MIN ENDSTOP Y: ");
        ECHO_EV(MSG_ENDSTOP_HIT);
      }
      else
      {
        ECHO_M("Y ");
        ECHO_EV(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif defined(Y_MAX_PIN) && Y_MAX_PIN > -1 && Y_HOME_DIR == 1
      if (!READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING){
        ECHO_M("MAX ENDSTOP Y: ");
        ECHO_EV(MSG_ENDSTOP_OPEN);
      }
      else
      {
        ECHO_EM("Y ENDSTOP ERROR");
        ECHO_V(MSG_FWTEST_INVERT);
        ECHO_EM("#define Y_MAX_ENDSTOP_INVERTING");
        return;
      }
      ECHO_V(MSG_FWTEST_PRESS);
      ECHO_EM("Y");
      ECHO_EV(MSG_FWTEST_YES);
      serial_answer = ' ';
      while(serial_answer!='y' && serial_answer!='Y' && !(READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)){
        serial_answer = MYSERIAL.read();
      }
      if (READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING){
        ECHO_M("MAX ENDSTOP Y: ");
        ECHO_EV(MSG_ENDSTOP_HIT);
      }
      else
      {
        ECHO_M("Y ");
        ECHO_EV(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif Y_HOME_DIR == -1
      ECHO_EM("ERROR!!! Y_MIN_PIN not defined");
      return;
    #elif Y_HOME_DIR == 1
      ECHO_EM("ERROR!!! Y_MAX_PIN not defined");
      return;
    #endif

    ECHO_EM(" ");
    ECHO_EM("***** ENDSTOP Z *****");
    #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1 && Z_HOME_DIR == -1
      if (!READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING){
        ECHO_M("MIN ENDSTOP Z: ");
        ECHO_EV(MSG_ENDSTOP_OPEN);
      }
      else
      {
        ECHO_EM("Z ENDSTOP ERROR");
        ECHO_V(MSG_FWTEST_INVERT);
        ECHO_EM("#define Z_MIN_ENDSTOP_INVERTING");
        return;
      }
      ECHO_V(MSG_FWTEST_PRESS);
      ECHO_EM("Z");
      ECHO_EV(MSG_FWTEST_YES);
      serial_answer = ' ';
      while(serial_answer!='y' && serial_answer!='Y' && !(READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)){
        serial_answer = MYSERIAL.read();
      }
      if (READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING){
        ECHO_M("MIN ENDSTOP Z: ");
        ECHO_EV(MSG_ENDSTOP_HIT);
      }
      else
      {
        ECHO_M("Z ");
        ECHO_EV(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif defined(Z_MAX_PIN) && Z_MAX_PIN > -1 && Z_HOME_DIR == 1
      if (!READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING){
        ECHO_M("MAX ENDSTOP Z: ");
        ECHO_EV(MSG_ENDSTOP_OPEN);
      }
      else
      {
        ECHO_EM("Z ENDSTOP ERROR");
        ECHO_V(MSG_FWTEST_INVERT);
        ECHO_EM("#define Z_MAX_ENDSTOP_INVERTING");
        return;
      }
      ECHO_V(MSG_FWTEST_PRESS);
      ECHO_EM("Z");
      ECHO_EV(MSG_FWTEST_YES);
      serial_answer = ' ';
      while(serial_answer!='y' && serial_answer!='Y' && !(READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)){
        serial_answer = MYSERIAL.read();
      }
      if (READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING){
        ECHO_M("MAX ENDSTOP Z: ");
        ECHO_EV(MSG_ENDSTOP_HIT);
      }
      else
      {
        ECHO_M("Z ");
        ECHO_EV(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif Z_HOME_DIR == -1
      ECHO_EM("ERROR!!! Z_MIN_PIN not defined");
      return;
    #elif Z_HOME_DIR == 1
      ECHO_EM("ERROR!!! Z_MAX_PIN not defined");
      return;
    #endif

    ECHO_EM("ENDSTOP OK");
    ECHO_EM(" ");
  }

  #if HAS_POWER_SWITCH
    SET_OUTPUT(PS_ON_PIN);
    WRITE(PS_ON_PIN, PS_ON_AWAKE);
  #endif

  // Reset position to 0
  st_synchronize();
  for(int8_t i=0; i < NUM_AXIS; i++) current_position[i] = 0;
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

  ECHO_EM("***** TEST MOTOR  *****");
  ECHO_EV(MSG_FWTEST_ATTENTION);
  ECHO_EV(MSG_FWTEST_YES);
  serial_answer = ' ';
  while(serial_answer!='y' && serial_answer!='Y'){
    serial_answer = MYSERIAL.read();
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
  while(serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N'){
    serial_answer = MYSERIAL.read();
  }
  if(serial_answer=='y' || serial_answer=='Y'){
    ECHO_EM("MOTOR X OK");
  }
  else
  {
    ECHO_V(MSG_FWTEST_INVERT);
    ECHO_EM("#define INVERT_X_DIR");
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
  while(serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N'){
    serial_answer = MYSERIAL.read();
  }
  if(serial_answer=='y' || serial_answer=='Y'){
    ECHO_EM("MOTOR Y OK");
  }
  else
  {
    ECHO_V(MSG_FWTEST_INVERT);
    ECHO_EM("#define INVERT_Y_DIR");
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
  while(serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N'){
    serial_answer = MYSERIAL.read();
  }
  if(serial_answer=='y' || serial_answer=='Y'){
    ECHO_EM("MOTOR Z OK");
  }
  else
  {
    ECHO_V(MSG_FWTEST_INVERT);
    ECHO_EM("#define INVERT_Z_DIR");
    return;
  }
  ECHO_EM("MOTOR OK");
  ECHO_EM(" ");
  ECHO_V(MSG_FWTEST_END);
}
