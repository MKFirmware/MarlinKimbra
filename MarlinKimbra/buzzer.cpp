/*
  buzzer.cpp - Library for controlling a buzzer
  Created by Mago Kimbra, 10/05/2015
*/
#include "Marlin.h"
#include "buzzer.h"
#include "ultralcd.h"

#if HAS_BUZZER
  void buzz(long duration, uint16_t freq) {
    if (freq > 0) {
      #ifdef LCD_USE_I2C_BUZZER
        lcd_buzz(duration, freq);
      #elif defined(BEEPER) && BEEPER >= 0 // on-board buzzers have no further condition
        SET_OUTPUT(BEEPER);
        tone(BEEPER, freq);
        delay(duration);
        noTone(BEEPER);
      #else
        delay(duration);
      #endif
    }
    else {
      delay(duration);
    }
  }
#endif
