#ifndef BUZZER_H
  #define BUZZER_H

  #if HAS(BUZZER)
    void buzz(uint16_t freq, long duration);
  #endif

#endif // BUZZER_H
