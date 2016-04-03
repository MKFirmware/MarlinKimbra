#ifndef MECHANICS_H
#define MECHANICS_H

// Macros for mechanics type
#define MECH_UNKNOWN    -1
#define MECH_CARTESIAN   0
#define MECH_COREXY      1
#define MECH_COREYX      2
#define MECH_COREXZ      8
#define MECH_COREZX      9
#define MECH_DELTA       3
#define MECH_SCARA       4

#define MECH(mech)  (MECHANISM == MECH_##mech)

#endif