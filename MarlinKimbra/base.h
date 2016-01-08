#ifndef BASE_H
#define BASE_H

#include "Arduino.h"
#include "pins_arduino.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#ifdef __SAM3X8E__
  #include "module/HAL.h"
#else
  // Arduino < 1.0.0 does not define this, so we need to do it ourselves
  #ifndef analogInputToDigitalPin
    #define analogInputToDigitalPin(p) ((p) + 0xA0)
  #endif
  #include <util/delay.h>
  #include <avr/eeprom.h>
  #include "module/fastio.h"
#endif
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "module/macros.h"
#include "Boards.h"
#include "module/mechanics.h"

#include "Configuration_Version.h"
#include "Configuration_Overall.h"

#ifndef CONFIGURATION_OVERALL_ON
  #include "Configuration_Basic.h"

  #if MECH(CARTESIAN)
    #include "Configuration_Cartesian.h"
  #elif MECH(COREXY)
    #include "Configuration_Core.h"
  #elif MECH(COREXZ)
    #include "Configuration_Core.h"
  #elif MECH(DELTA)
    #include "Configuration_Delta.h"
  #elif MECH(SCARA)
    #include "Configuration_Scara.h"
  #endif

  #include "Configuration_Feature.h"
#endif

#include "language/language.h"
#include "module/conditionals.h"
#include "module/sanitycheck.h"
#include "module/thermistortables.h"
#include "module/comunication.h"

typedef unsigned long millis_t;

#endif
