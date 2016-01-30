#ifndef COMMUNICATION_H
  #define COMMUNICATION_H

  class Com {
    public:
      #define START       "start"               // start for host
      #define OK          "ok "                 // ok answer for host
      #define ER          "Error: "             // error for host
      #define WT          "Wait"                // wait for host
      #define DB          "Echo: "              // message for user
      #define CFG         "Config: "            // config for host
      #define INFO        "Info: "              // info for host
      #define RESEND      "Resend: "            // resend for host
      #define WARNING     "Warning: "           // warning for host
      #define TNAN        "NAN"                 // NAN for host
      #define TINF        "INF"                 // INF for host
      #define PAUSE       "//action:pause"      // command for host that support action
      #define RESUME      "//action:resume"     // command for host that support action
      #define DISCONNECT  "//action:disconnect" // command for host that support action

      static void printFloat(float number, uint8_t digits);
      static void printVal(int value);
      static void printVal(int8_t value);
      static void printVal(uint8_t value);
      static void printVal(int32_t value);
      static void printVal(uint32_t value);
      static void printVal(float value, uint8_t digits = 2);
      static void printVal(double value, uint8_t digits = 2);
      static void printArray(float *arr, uint8_t n = 4, uint8_t digits = 2);
      static void printArray(long *arr, uint8_t n = 4);
      static void printNumber(uint32_t n);
      static void print(long value);
      static void print(uint16_t value);
      static void print(uint32_t value);
      static void print(int value);
      static void print(float number);
      static void print(const char *text);
      static void print(char c);
      static void println() { HAL::serialWriteByte('\r'); HAL::serialWriteByte('\n'); }
      static void printF(FSTRINGPARAM(ptr));
    protected:
    private:
  };

  #define SERIAL_WRITE(x)                   HAL::serialWriteByte(x)

  #define ECHO_S(srt)                       Com::printF(PSTR(srt))
  #define ECHO_M(msg)                       Com::printF(PSTR(msg))
  #define ECHO_T(txt)                       Com::print(txt)
  #define ECHO_V(val, args...)              Com::printVal(val, ##args)
  #define ECHO_C(x)                         Com::print(x)
  #define ECHO_E                            Com::println()

  #define ECHO_MV(msg, val, args...)        ECHO_M(msg),ECHO_V(val, ##args)
  #define ECHO_VM(val, msg, args...)        ECHO_V(val, ##args),ECHO_M(msg)
  #define ECHO_MT(msg, txt)                 ECHO_M(msg),ECHO_T(txt)
  #define ECHO_TM(txt, msg)                 ECHO_T(txt),ECHO_M(msg)

  #define ECHO_SM(srt, msg)                 ECHO_S(srt),ECHO_M(msg)
  #define ECHO_ST(srt, txt)                 ECHO_S(srt),ECHO_T(txt)
  #define ECHO_SV(srt, val, args...)        ECHO_S(srt),ECHO_V(val, ##args)
  #define ECHO_SMV(srt, msg, val, args...)  ECHO_S(srt),ECHO_MV(msg, val, ##args)
  #define ECHO_SMT(srt, msg, txt)           ECHO_S(srt),ECHO_MT(msg, txt)

  #define ECHO_EM(msg)                      ECHO_M(msg),ECHO_E
  #define ECHO_ET(txt)                      ECHO_T(txt),ECHO_E
  #define ECHO_EV(val, args...)             ECHO_V(val, ##args),ECHO_E
  #define ECHO_EMV(msg, val, args...)       ECHO_MV(msg, val, ##args),ECHO_E
  #define ECHO_EVM(val, msg, args...)       ECHO_VM(val, msg, ##args),ECHO_E
  #define ECHO_EMT(msg, txt)                ECHO_MT(msg, txt),ECHO_E

  #define ECHO_L(srt)                       ECHO_S(srt),ECHO_E
  #define ECHO_LM(srt, msg)                 ECHO_S(srt),ECHO_M(msg),ECHO_E
  #define ECHO_LT(srt, txt)                 ECHO_S(srt),ECHO_T(txt),ECHO_E
  #define ECHO_LV(srt, val, args...)        ECHO_S(srt),ECHO_V(val, ##args),ECHO_E
  #define ECHO_LMV(srt, msg, val, args...)  ECHO_S(srt),ECHO_MV(msg, val, ##args),ECHO_E
  #define ECHO_LVM(srt, val, msg, args...)  ECHO_S(srt),ECHO_VM(val, msg, ##args),ECHO_E
  #define ECHO_LMT(srt, msg, txt)           ECHO_S(srt),ECHO_MT(msg, txt),ECHO_E

#endif
