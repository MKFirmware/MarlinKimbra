#ifndef BASE_H
#define BASE_H

#include "Arduino.h"
#include "pins_arduino.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#define ENABLED defined
#define DISABLED !defined

#include "Boards.h"
#include "module/mechanics.h"

#include "Configuration_Version.h"
#include "Configuration_Basic.h"
#include "Configuration_Overall.h"

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
#include "Configuration_Overall.h"

#include "module/HAL/HAL.h"
#include "module/communication/communication.h"

#include "Configuration_Store.h"

#include "module/language/language.h"
#include "module/conditionals.h"
#include "module/sanitycheck.h"
#include "module/MK_Main.h"
#include "module/motion/stepper.h"
#include "module/motion/stepper_indirection.h"
#include "module/motion/planner.h"
#include "module/temperature/temperature.h"
#include "module/temperature/thermistortables.h"
#include "module/lcd/ultralcd.h"
#include "module/nextion/nextion_lcd.h"
#include "module/sd/cardreader.h"

#if ENABLED(AUTO_BED_LEVELING_FEATURE)
  #include "module/motion/vector_3.h"
  #if ENABLED(AUTO_BED_LEVELING_GRID)
    #include "module/motion/qr_solve.h"
  #endif
#endif // AUTO_BED_LEVELING_FEATURE

#if MB(ALLIGATOR)
  #include "module/alligator/external_dac.h"
#endif

#if ENABLED(USE_WATCHDOG)
  #include "module/watchdog/watchdog.h"
#endif

#if HAS(BUZZER)
  #include "module/lcd/buzzer.h"
#endif

#if ENABLED(BLINKM)
  #include "module/blinkm/blinkm.h"
#endif

#if HAS(SERVOS)
  #include "module/servo/servo.h"
#endif

#if HAS(DIGIPOTSS)
  #include <SPI.h>
#endif

#if ENABLED(FIRMWARE_TEST)
  #include "module/fwtest/firmware_test.h"
#endif

#endif
