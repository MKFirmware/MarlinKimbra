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
 * Description:          *** HAL for Arduino ***
 *
 * ARDUINO_ARCH_ARM
 */

#ifndef HAL_H
  #define HAL_H

  #include <avr/io.h>
  #include "fastio.h"

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

  // Macros for maths shortcuts
  #ifndef M_PI 
    #define M_PI 3.1415926536
  #endif
  #define RADIANS(d) ((d)*M_PI/180.0)
  #define DEGREES(r) ((r)*180.0/M_PI)
  #define SIN_60 0.8660254037844386
  #define COS_60 0.5

  // Macros to support option testing
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

  //#define EXTERNALSERIAL  // Force using arduino serial
  #ifndef EXTERNALSERIAL
    #include "HardwareSerial.h"
    #define MKSERIAL MKSerial
  #else
    #define MKSERIAL Serial
  #endif

  #if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif

  /**
   * Types
   */
  typedef uint32_t millis_t;

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

  class HAL {
    public:

      HAL();

      virtual ~HAL();

      static void showStartReason();
      static int getFreeRam();
      static void resetHardware();
    protected:
    private:
  };

#endif // HAL_H
