#ifndef ELEMENTS_H
#define ELEMENTS_H
#warning called
#include "Arduino.h"
#include "pins_arduino.h"

// Arduino < 1.0.0 does not define this, so we need to do it ourselves
#ifndef analogInputToDigitalPin
  #define analogInputToDigitalPin(p) ((p) + 0xA0)
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#ifdef __SAM3X8E__
  #include "HAL.h"
#else
  #include <util/delay.h>
  #include <avr/eeprom.h>
  #include "fastio.h"
#endif
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "macros.h"
#include "boards.h"
#include "mechanics.h"
#include "configurations.h"
#include "language.h"
#include "dependencies.h"
#include "conditionals.h"
#include "conflicts.h"

#include "comunication.h"

typedef unsigned long millis_t;

#endif