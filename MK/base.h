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

#include "module/macros.h"
#include "Boards.h"
#include "module/mechanics.h"

#include "Configuration_Version.h"
#include "Configuration_Basic.h"
#include "Configuration_Overall.h"

#if MECH(CARTESIAN)
  #include "Configuration_Cartesian.h"
#elif MECH(COREXY)
  #include "Configuration_Core.h"
#elif MECH(COREYX)
  #include "Configuration_Core.h"
#elif MECH(COREXZ)
  #include "Configuration_Core.h"
#elif MECH(COREZX)
  #include "Configuration_Core.h"
#elif MECH(DELTA)
  #include "Configuration_Delta.h"
#elif MECH(SCARA)
  #include "Configuration_Scara.h"
#endif

#include "Configuration_Temperature.h"
#include "Configuration_Feature.h"
#include "Configuration_Overall.h"

#if ENABLED(LASERBEAM)
  #include "Configuration_Laser.h"
  #if ENABLED(LASER_RASTER)
    #include "module/laser/base64/base64.h"
  #endif
  #include "module/laser/laser.h"
#endif

#include "module/conditionals.h"
#include "module/sanitycheck.h"
#include "module/HAL/HAL.h"
#include "module/communication/communication.h"

#include "Configuration_Store.h"

#include "module/language/language.h"
#include "module/printcounter/printcounter.h"
#include "module/MK_Main.h"
#include "module/motion/planner.h"
#include "module/motion/stepper_indirection.h"
#include "module/motion/stepper.h"
#include "module/motion/endstops.h"
#include "module/motion/vector_3.h"
#include "module/motion/qr_solve.h"
#include "module/motion/cartesian_correction.h"
#include "module/temperature/temperature.h"
#include "module/sensor/flowmeter.h"
#include "module/temperature/thermistortables.h"
#include "module/lcd/ultralcd.h"
#include "module/lcd/buzzer.h"
#include "module/nextion/Nextion_lcd.h"
#include "module/sd/cardreader.h"
#include "module/servo/servo.h"
#include "module/watchdog/watchdog.h"
#include "module/blinkm/blinkm.h"

#if MB(ALLIGATOR)
  #include "module/alligator/external_dac.h"
#endif

#if HAS(DIGIPOTSS)
  #include <SPI.h>
#endif

#if ENABLED(FIRMWARE_TEST)
  #include "module/fwtest/firmware_test.h"
#endif

#if ENABLED(RFID_MODULE)
  #include "module/mfrc522/MFRC522_serial.h"
#endif

#endif
