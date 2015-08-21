#ifndef MECHANISM_H
#define MECHANISM_H

#define MECH_UNKNOWN    -1
#define MECH_CARTESIAN   0
#define MECH_COREXY      1
#define MECH_COREXZ      2
#define MECH_DELTA       3
#define MECH_SCARA       4

//Keept for legacy support
#define CARTESIAN             MECH_CARTESIAN
#define COREXY                MECH_COREXY
#define COREXZ                MECH_COREXZ
#define DELTA                 MECH_DELTA
#define SCARA                 MECH_SCARA

#define MECHANISM             MECH_UNKNOWN
#define MECH_TYPE(mechanism)  (MEACHANISM == mechanism)

#endif //__MECHANISM_H
