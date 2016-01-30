/**
 * This is the main Hardware Abstraction Layer (HAL).
 * To make the firmware work with different processors and toolchains,
 * all hardware related code should be packed into the hal files.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Description:          *** HAL for Arduino Due ***
 *
 * ARDUINO_ARCH_SAM
 */

#ifndef HAL_H
  #define HAL_H

  #include <avr/pgmspace.h>
  //#include <avr/io.h>

  // Arduino < 1.0.0 does not define this, so we need to do it ourselves
  #ifndef analogInputToDigitalPin
    #define analogInputToDigitalPin(p) ((p) + 0xA0)
  #endif

  /**
   * Defines & Macros
   */
  // Compiler warning on unused varable.
  #define UNUSED(x) (void) (x)

  // Macros for bit
  #define BIT(b)                (1<<(b))
  #define TEST(n, b)            (((n)&BIT(b))!=0)
  #define SET_BIT(n, b, value)  (n) ^= ((-value)^(n)) & (BIT(b))
  #define bit_clear(x, y)       x&= ~(1<<y)
  #define bit_set(x, y)         x|= (1<<y)

  // Macros for maths shortcuts
  #ifndef M_PI 
    #define M_PI 3.1415926536
  #endif
  #define RADIANS(d) ((d)*M_PI/180.0)
  #define DEGREES(r) ((r)*180.0/M_PI)
  #define SIN_60 0.8660254037844386
  #define COS_60 0.5

  // Macros to support option testing
  #define ENABLED defined
  #define DISABLED !defined
  #define PIN_EXISTS(PN) (defined(PN##_PIN) && PN##_PIN >= 0)
  #define HAS(FE) (HAS_##FE)
  #define HASNT(FE) (!(HAS_##FE))

  // Macros to contrain values
  #define NOLESS(v,n) do{ if (v < n) v = n; }while(0)
  #define NOMORE(v,n) do{ if (v > n) v = n; }while(0)
  #define COUNT(a) (sizeof(a)/sizeof(*a))

  #define  FORCE_INLINE __attribute__((always_inline)) inline
  #ifndef CRITICAL_SECTION_START
    #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
    #define CRITICAL_SECTION_END    SREG = _sreg;
  #endif

  #if CPU_ARCH == ARCH_AVR
    #include <avr/io.h>
  #else
    #define PROGMEM
    #define PGM_P const char *
  
    #define PSTR(s) s
    #define pgm_read_byte_near(x) (*(uint8_t*)x)
    #define pgm_read_byte(x) (*(uint8_t*)x)
  #endif

  #define PACK

  #define FSTRINGVALUE(var,value) const char var[] PROGMEM = value;
  #define FSTRINGVAR(var) static const char var[] PROGMEM;
  #define FSTRINGPARAM(var) PGM_P var

  #include <avr/eeprom.h>
  #include <avr/wdt.h>

  //#define EXTERNALSERIAL  // Force using arduino serial
  #ifndef EXTERNALSERIAL
    #define  HardwareSerial_h // Don't use standard serial console
  #endif

  #include <inttypes.h>
  #include "Print.h"

  #ifdef EXTERNALSERIAL
    #define SERIAL_RX_BUFFER_SIZE 128
  #endif
  #if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
  #else
    #include "WProgram.h"
    #define COMPAT_PRE1
  #endif

  /**
   * Types
   */
  typedef uint32_t millis_t;

  #if CPU_ARCH == ARCH_AVR
    #include "fastio.h"
  #else
    #define	READ(IO)  digitalRead(IO)
    #define	WRITE(IO, v)  digitalWrite(IO, v)
    #define	SET_INPUT(IO)  pinMode(IO, INPUT)
    #define	SET_OUTPUT(IO)  pinMode(IO, OUTPUT)
  #endif
  
  class InterruptProtectedBlock {
    uint8_t sreg;
    public:
      inline void protect() {
        cli();
      }
  
      inline void unprotect() {
        SREG = sreg;
      }
  
      inline InterruptProtectedBlock(bool later = false) {
        sreg = SREG;
        if (!later) cli();
      }
  
      inline ~InterruptProtectedBlock() {
        SREG = sreg;
      }
  };

  #ifndef EXTERNALSERIAL
    // Implement serial communication for one stream only!
    /*
     * HardwareSerial.h - Hardware serial library for Wiring
     * Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
     *
     * This library is free software; you can redistribute it and/or
     * modify it under the terms of the GNU Lesser General Public
     * License as published by the Free Software Foundation; either
     * version 2.1 of the License, or (at your option) any later version.
     *
     * This library is distributed in the hope that it will be useful,
     * but WITHOUT ANY WARRANTY; without even the implied warranty of
     * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
     * Lesser General Public License for more details.
     *
     * You should have received a copy of the GNU Lesser General Public
     * License along with this library; if not, write to the Free Software
     * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
     *
     * Modified 28 September 2010 by Mark Sproul
     *
     * Modified to use only 1 queue with fixed length by Repetier
     */

    #define SERIAL_BUFFER_SIZE 128
    #define SERIAL_BUFFER_MASK 127
    #undef SERIAL_TX_BUFFER_SIZE
    #undef SERIAL_TX_BUFFER_MASK
    #ifdef BIG_OUTPUT_BUFFER
      #define SERIAL_TX_BUFFER_SIZE 128
      #define SERIAL_TX_BUFFER_MASK 127
    #else
      #define SERIAL_TX_BUFFER_SIZE 64
      #define SERIAL_TX_BUFFER_MASK 63
    #endif

    struct ring_buffer {
      uint8_t buffer[SERIAL_BUFFER_SIZE];
      volatile uint8_t head;
      volatile uint8_t tail;
    };

    struct ring_buffer_tx {
      uint8_t buffer[SERIAL_TX_BUFFER_SIZE];
      volatile uint8_t head;
      volatile uint8_t tail;
    };

    class MKHardwareSerial : public Print {
      public:
        ring_buffer *_rx_buffer;
        ring_buffer_tx *_tx_buffer;
        volatile uint8_t *_ubrrh;
        volatile uint8_t *_ubrrl;
        volatile uint8_t *_ucsra;
        volatile uint8_t *_ucsrb;
        volatile uint8_t *_udr;
        uint8_t _rxen;
        uint8_t _txen;
        uint8_t _rxcie;
        uint8_t _udrie;
        uint8_t _u2x;
      public:
        MKHardwareSerial(ring_buffer *rx_buffer, ring_buffer_tx *tx_buffer,
                         volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
                         volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
                         volatile uint8_t *udr,
                         uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udrie, uint8_t u2x);
        void begin(unsigned long);
        void end();
        virtual int available(void);
        virtual int peek(void);
        virtual int read(void);
        virtual void flush(void);

      #ifdef COMPAT_PRE1
        virtual void write(uint8_t);
      #else
        virtual size_t write(uint8_t);
      #endif

      using Print::write; // pull in write(str) and write(buf, size) from Print
      operator bool();
      int outputUnused(void); // Used for output in interrupts
    };

    extern MKHardwareSerial MKSerial;

    #define MKSERIAL MKSerial

    //extern ring_buffer x_buffer;
    #define WAIT_OUT_EMPTY while(tx_buffer.head != tx_buffer.tail) {}
  #else
    #define MKSERIAL Serial
  #endif

  class HAL {
    public:

      HAL();

      virtual ~HAL();

      static inline char readFlashByte(PGM_P ptr) { return pgm_read_byte(ptr); }
      static inline void serialSetBaudrate(long baud) { MKSERIAL.begin(baud); }
      static inline bool serialByteAvailable() { return MKSERIAL.available() > 0; }
      static inline uint8_t serialReadByte() { return MKSERIAL.read(); }
      static inline void serialWriteByte(char b) { MKSERIAL.write(b); }
      static inline void serialFlush() { MKSERIAL.flush(); }
      static void showStartReason();
      static int getFreeRam();
      static void resetHardware();
    protected:
    private:
  };

#endif // HAL_H
