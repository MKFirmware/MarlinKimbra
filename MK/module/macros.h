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

#endif //__MACROS_H
