/**
 * MK & MK4due 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef MACROS_H
#define MACROS_H

// Compiler warning on unused varable.
#define UNUSED(x) (void) (x)

// Macros for bit masks
#ifndef _BV
  #define _BV(b) (1<<(b))
#endif
#define TEST(n,b) (((n)&_BV(b))!=0)
#define SBI(n,b) (n |= _BV(b))
#define CBI(n,b) (n &= ~_BV(b))
#define SET_BIT(n,b,value) (n) ^= ((-value)^(n)) & (_BV(b))

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

#define PENDING(NOW,SOON) ((long)(NOW-(SOON))<0)
#define ELAPSED(NOW,SOON) (!PENDING(NOW,SOON))

// Macros to contrain values
#define NUMERIC(a) ((a) >= '0' && '9' >= (a))
#define NUMERIC_SIGNED(a) (NUMERIC(a) || (a) == '-')
#define NOLESS(v,n) do{ if (v < n) v = n; }while(0)
#define NOMORE(v,n) do{ if (v > n) v = n; }while(0)
#define COUNT(a) (sizeof(a)/sizeof(*a))

// Function macro
#define  FORCE_INLINE __attribute__((always_inline)) inline

// Macro for debugging
#define DEBUGGING(F) (mk_debug_flags & (DEBUG_## F))

#endif //__MACROS_H
