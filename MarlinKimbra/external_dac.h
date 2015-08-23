#ifndef _EXTERNAL_DAC_H
#define _EXTERNAL_DAC_H

class ExternalDac {
  public:
    ExternalDac();
    static void begin(void);
    static void setValue(uint8_t channel, uint8_t value);
};

#endif //_EXTERNAL_DAC_H
