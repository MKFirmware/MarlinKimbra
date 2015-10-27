#ifndef MACROS_H
#define MACROS_H

// Compiler warning on unused varable.
#define UNUSED(x) (void) (x)

// Macros for bit masks
#define BIT(b) (1<<(b))
#define TEST(n,b) (((n)&BIT(b))!=0)
#define SET_BIT(n,b,value) (n) ^= ((-value)^(n)) & (BIT(b))

// Macros for maths shortcuts
#define M_PI 3.1415926536
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

// Function macro
#define  FORCE_INLINE __attribute__((always_inline)) inline

#if DISABLED(CRITICAL_SECTION_START)
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif

#endif //__MACROS_H
