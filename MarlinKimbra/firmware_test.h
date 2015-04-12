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
  SERIAL_ECHOLN("---------- FIRMWARE TEST --------------");
  SERIAL_ECHOLN("--------- by MarlinKimbra -------------");
  SERIAL_ECHOLN(" ");
  SERIAL_ECHOLN(MSG_FWTEST_01);
  SERIAL_ECHOLN(MSG_FWTEST_02);
  SERIAL_ECHOLN(MSG_FWTEST_YES_NO);
  serial_answer = ' ';
  while(serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N') {
    serial_answer = MYSERIAL.read();
  }
  if (serial_answer=='y' || serial_answer=='Y') {
    SERIAL_ECHOLN(MSG_FWTEST_03);

    SERIAL_ECHOLN(" ");
    SERIAL_ECHOLN("***** ENDSTOP X *****");
    #if defined(X_MIN_PIN) && X_MIN_PIN > -1 && X_HOME_DIR == -1
      if (!READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING) {
        SERIAL_ECHO("MIN ENDSTOP X: ");
        SERIAL_ECHOLN(MSG_ENDSTOP_OPEN);
      }
      else
      {
        SERIAL_ECHOLN("X ENDSTOP ERROR");
        SERIAL_ECHO(MSG_FWTEST_INVERT);
        SERIAL_ECHOLN("#define X_MIN_ENDSTOP_INVERTING");
        return;
      }
      SERIAL_ECHO(MSG_FWTEST_PRESS);
      SERIAL_ECHOLN("X");
      SERIAL_ECHOLN(MSG_FWTEST_YES);
      serial_answer = ' ';
      while(serial_answer!='y' && serial_answer!='Y' && !(READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)){
        serial_answer = MYSERIAL.read();
      }
      if (READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING) {
        SERIAL_ECHO("MIN ENDSTOP X: ");
        SERIAL_ECHOLN(MSG_ENDSTOP_HIT);
      }
      else
      {
        SERIAL_ECHO("X ");
        SERIAL_ECHOLN(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif defined(X_MAX_PIN) && X_MAX_PIN > -1 && X_HOME_DIR == 1
      if (!READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING) {
        SERIAL_ECHO("MAX ENDSTOP X: ");
        SERIAL_ECHOLN(MSG_ENDSTOP_OPEN);
      }
      else
      {
        SERIAL_ECHOLN("X ENDSTOP ERROR");
        SERIAL_ECHO(MSG_FWTEST_INVERT);
        SERIAL_ECHOLN("#define X_MAX_ENDSTOP_INVERTING");
        return;
      }
      SERIAL_ECHO(MSG_FWTEST_PRESS);
      SERIAL_ECHOLN("X");
      SERIAL_ECHOLN(MSG_FWTEST_YES);
      serial_answer = ' ';
      while(serial_answer!='y' && serial_answer!='Y' && !(READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)) {
        serial_answer = MYSERIAL.read();
      }
      if (READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING) {
        SERIAL_ECHO("MAX ENDSTOP X: ");
        SERIAL_ECHOLN(MSG_ENDSTOP_HIT);
      }
      else
      {
        SERIAL_ECHO("X ");
        SERIAL_ECHOLN(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif X_HOME_DIR == -1
      SERIAL_ECHOLN("ERROR!!! X_MIN_PIN not defined");
      return;
    #elif X_HOME_DIR == 1
      SERIAL_ECHOLN("ERROR!!! X_MAX_PIN not defined");
      return;
    #endif

    SERIAL_ECHOLN(" ");
    SERIAL_ECHOLN("***** ENDSTOP Y *****");
    #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1 && Y_HOME_DIR == -1
      if (!READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING){
        SERIAL_ECHO("MIN ENDSTOP Y: ");
        SERIAL_ECHOLN(MSG_ENDSTOP_OPEN);
      }
      else
      {
        SERIAL_ECHOLN("Y ENDSTOP ERROR");
        SERIAL_ECHO(MSG_FWTEST_INVERT);
        SERIAL_ECHOLN("#define Y_MIN_ENDSTOP_INVERTING");
        return;
      }
      SERIAL_ECHO(MSG_FWTEST_PRESS);
      SERIAL_ECHOLN("Y");
      SERIAL_ECHOLN(MSG_FWTEST_YES);
      serial_answer = ' ';
      while(serial_answer!='y' && serial_answer!='Y' && !(READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)){
        serial_answer = MYSERIAL.read();
      }
      if (READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING){
        SERIAL_ECHO("MIN ENDSTOP Y: ");
        SERIAL_ECHOLN(MSG_ENDSTOP_HIT);
      }
      else
      {
        SERIAL_ECHO("Y ");
        SERIAL_ECHOLN(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif defined(Y_MAX_PIN) && Y_MAX_PIN > -1 && Y_HOME_DIR == 1
      if (!READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING){
        SERIAL_ECHO("MAX ENDSTOP Y: ");
        SERIAL_ECHOLN(MSG_ENDSTOP_OPEN);
      }
      else
      {
        SERIAL_ECHOLN("Y ENDSTOP ERROR");
        SERIAL_ECHO(MSG_FWTEST_INVERT);
        SERIAL_ECHOLN("#define Y_MAX_ENDSTOP_INVERTING");
        return;
      }
      SERIAL_ECHO(MSG_FWTEST_PRESS);
      SERIAL_ECHOLN("Y");
      SERIAL_ECHOLN(MSG_FWTEST_YES);
      serial_answer = ' ';
      while(serial_answer!='y' && serial_answer!='Y' && !(READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)){
        serial_answer = MYSERIAL.read();
      }
      if (READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING){
        SERIAL_ECHO("MAX ENDSTOP Y: ");
        SERIAL_ECHOLN(MSG_ENDSTOP_HIT);
      }
      else
      {
        SERIAL_ECHO("Y ");
        SERIAL_ECHOLN(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif Y_HOME_DIR == -1
      SERIAL_ECHOLN("ERROR!!! Y_MIN_PIN not defined");
      return;
    #elif Y_HOME_DIR == 1
      SERIAL_ECHOLN("ERROR!!! Y_MAX_PIN not defined");
      return;
    #endif

    SERIAL_ECHOLN(" ");
    SERIAL_ECHOLN("***** ENDSTOP Z *****");
    #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1 && Z_HOME_DIR == -1
      if (!READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING){
        SERIAL_ECHO("MIN ENDSTOP Z: ");
        SERIAL_ECHOLN(MSG_ENDSTOP_OPEN);
      }
      else
      {
        SERIAL_ECHOLN("Z ENDSTOP ERROR");
        SERIAL_ECHO(MSG_FWTEST_INVERT);
        SERIAL_ECHOLN("#define Z_MIN_ENDSTOP_INVERTING");
        return;
      }
      SERIAL_ECHO(MSG_FWTEST_PRESS);
      SERIAL_ECHOLN("Z");
      SERIAL_ECHOLN(MSG_FWTEST_YES);
      serial_answer = ' ';
      while(serial_answer!='y' && serial_answer!='Y' && !(READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)){
        serial_answer = MYSERIAL.read();
      }
      if (READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING){
        SERIAL_ECHO("MIN ENDSTOP Z: ");
        SERIAL_ECHOLN(MSG_ENDSTOP_HIT);
      }
      else
      {
        SERIAL_ECHO("Z ");
        SERIAL_ECHOLN(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif defined(Z_MAX_PIN) && Z_MAX_PIN > -1 && Z_HOME_DIR == 1
      if (!READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING){
        SERIAL_ECHO("MAX ENDSTOP Z: ");
        SERIAL_ECHOLN(MSG_ENDSTOP_OPEN);
      }
      else
      {
        SERIAL_ECHOLN("Z ENDSTOP ERROR");
        SERIAL_ECHO(MSG_FWTEST_INVERT);
        SERIAL_ECHOLN("#define Z_MAX_ENDSTOP_INVERTING");
        return;
      }
      SERIAL_ECHO(MSG_FWTEST_PRESS);
      SERIAL_ECHOLN("Z");
      SERIAL_ECHOLN(MSG_FWTEST_YES);
      serial_answer = ' ';
      while(serial_answer!='y' && serial_answer!='Y' && !(READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)){
        serial_answer = MYSERIAL.read();
      }
      if (READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING){
        SERIAL_ECHO("MAX ENDSTOP Z: ");
        SERIAL_ECHOLN(MSG_ENDSTOP_HIT);
      }
      else
      {
        SERIAL_ECHO("Z ");
        SERIAL_ECHOLN(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif Z_HOME_DIR == -1
      SERIAL_ECHOLN("ERROR!!! Z_MIN_PIN not defined");
      return;
    #elif Z_HOME_DIR == 1
      SERIAL_ECHOLN("ERROR!!! Z_MAX_PIN not defined");
      return;
    #endif

    SERIAL_ECHOLN("ENDSTOP OK");
    SERIAL_ECHOLN(" ");
  }

  #if HAS_POWER_SWITCH
    SET_OUTPUT(PS_ON_PIN);
    WRITE(PS_ON_PIN, PS_ON_AWAKE);
  #endif

  // Reset position to 0
  st_synchronize();
  for(int8_t i=0; i < NUM_AXIS; i++) current_position[i] = 0;
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

  SERIAL_ECHOLN("***** TEST MOTOR  *****");
  SERIAL_ECHOLN(MSG_FWTEST_ATTENTION);
  SERIAL_ECHOLN(MSG_FWTEST_YES);
  serial_answer = ' ';
  while(serial_answer!='y' && serial_answer!='Y'){
    serial_answer = MYSERIAL.read();
  }
  SERIAL_ECHOLN(MSG_FWTEST_04);
  SERIAL_ECHOLN(" ");
  SERIAL_ECHOLN("***** MOTOR X *****");
  destination[X_AXIS] = 10;
  prepare_move();
  st_synchronize();

  SERIAL_ECHOLN(MSG_FWTEST_XAXIS);
  SERIAL_ECHOLN(MSG_FWTEST_YES_NO);
  serial_answer = ' ';
  while(serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N'){
    serial_answer = MYSERIAL.read();
  }
  if(serial_answer=='y' || serial_answer=='Y'){
    SERIAL_ECHOLN("MOTOR X OK");
  }
  else
  {
    SERIAL_ECHO(MSG_FWTEST_INVERT);
    SERIAL_ECHOLN("#define INVERT_X_DIR");
    return;
  }
  SERIAL_ECHOLN(" ");
  SERIAL_ECHOLN("***** MOTOR Y *****");
  destination[Y_AXIS] = 10;
  prepare_move();
  st_synchronize();
  SERIAL_ECHOLN(MSG_FWTEST_YAXIS);
  SERIAL_ECHOLN(MSG_FWTEST_YES_NO);
  serial_answer = ' ';
  while(serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N'){
    serial_answer = MYSERIAL.read();
  }
  if(serial_answer=='y' || serial_answer=='Y'){
    SERIAL_ECHOLN("MOTOR Y OK");
  }
  else
  {
    SERIAL_ECHO(MSG_FWTEST_INVERT);
    SERIAL_ECHOLN("#define INVERT_Y_DIR");
    return;
  }
  SERIAL_ECHOLN(" ");
  SERIAL_ECHOLN("***** MOTOR Z *****");
  destination[Z_AXIS] = 10;
  prepare_move();
  st_synchronize();
  SERIAL_ECHOLN(MSG_FWTEST_ZAXIS);
  SERIAL_ECHOLN(MSG_FWTEST_YES_NO);
  serial_answer = ' ';
  while(serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N'){
    serial_answer = MYSERIAL.read();
  }
  if(serial_answer=='y' || serial_answer=='Y'){
    SERIAL_ECHOLN("MOTOR Z OK");
  }
  else
  {
    SERIAL_ECHO(MSG_FWTEST_INVERT);
    SERIAL_ECHOLN("#define INVERT_Z_DIR");
    return;
  }
  SERIAL_ECHOLN("MOTOR OK");
  SERIAL_ECHOLN(" ");
  SERIAL_ECHO(MSG_FWTEST_END);
}
