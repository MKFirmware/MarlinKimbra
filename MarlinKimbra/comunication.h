/**
 *  Comunication.h - serial messages functions
 *  Part of MarlinKimbra
 *  
 *  Author: Simone Primarosa
*/

#ifndef COMUNICATION_H
#define COMUNICATION_H

#ifdef AT90USB
  #include "HardwareSerial.h"
#endif

#ifndef __SAM3X8E__
  #include "MarlinSerial.h"
#endif

#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "WString.h"

#ifdef AT90USB
  #ifdef BTENABLED
    #define MYSERIAL bt
  #else
    #define MYSERIAL Serial
  #endif // BTENABLED
#else
  #ifdef __SAM3X8E__
    #define MYSERIAL Serial
  #else
    #define MYSERIAL MSerial
  #endif
#endif

#define START       "start"               //start for host
#define OK          "ok "                 //ok answer for host
#define ER          "Error: "             //error for host
#define WT          "wait"                //wait for host
#define DB          "echo: "              //message for user
#define RS          "Resend: "            //resend for host
#define PAUSE       "//action:pause"      //command for host that support action
#define RESUME      "//action:resume"     //command for host that support action
#define DISCONNECT  "//action:disconnect" //command for host that support action

#define SERIAL_INIT(baud) MYSERIAL.begin(baud), delay(1)
#define SERIAL_WRITE(x) MYSERIAL.write(x)
#define SERIAL_PRINT(msg, args...) MYSERIAL.print(msg, ##args)
#define SERIAL_ENDL MYSERIAL.println()

FORCE_INLINE void PS_PGM(const char *str) {
  char ch;
  while ((ch = pgm_read_byte(str))) {
    MYSERIAL.write(ch);
    str++;
  }
}

#define ECHO_ENDL SERIAL_ENDL
#define ECHO_PGM(message) PS_PGM(PSTR(message))

#define ECHO_MV(msg, val, args...) ECHO_PGM(msg),ECHO_V(val, ##args)
#define ECHO_VM(val, msg, args...) ECHO_V(val, ##args),ECHO_PGM(msg)
#define ECHO_M(msg) ECHO_PGM(msg)
#define ECHO_V SERIAL_PRINT
#define ECHO_C SERIAL_WRITE
#define ECHO_S(srt) ECHO_PGM(srt)

#define ECHO_SM(srt, msg) ECHO_S(srt),ECHO_M(msg)
#define ECHO_SV(srt, val, args...) ECHO_S(srt),ECHO_V(val, ##args)
#define ECHO_SMV(srt, msg, val, args...) ECHO_S(srt),ECHO_MV(msg, val, ##args)
#define ECHO_SVM(srt, val, msg, args...) ECHO_S(srt),ECHO_VM(val, msg, ##args)

#define ECHO_E ECHO_ENDL
#define ECHO_EM(msg) ECHO_M(msg),ECHO_E
#define ECHO_EV(val, args...) ECHO_V(val, ##args),ECHO_E
#define ECHO_EMV(msg, val, args...) ECHO_MV(msg, val, ##args),ECHO_E
#define ECHO_EVM(val, msg, args...) ECHO_VM(val, msg, ##args),ECHO_E

#define ECHO_L(srt) ECHO_S(srt),ECHO_E
#define ECHO_LM(srt, msg) ECHO_S(srt),ECHO_M(msg),ECHO_E
#define ECHO_LV(srt, val) ECHO_S(srt),ECHO_V(val),ECHO_E
#define ECHO_LMV(srt, msg, val, args...) ECHO_S(srt),ECHO_MV(msg, val, ##args),ECHO_E
#define ECHO_LVM(srt, val, msg, args...) ECHO_S(srt),ECHO_VM(val, msg, ##args),ECHO_E

#endif
