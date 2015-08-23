#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H
#include "Configuration_Version.h"
#include "Configuration_Basic.h"

#if MECH(CARTESIAN)
  #include "Configuration_Cartesian.h"
#elif MECH(COREXY)
  #include "Configuration_Core.h"
#elif MECH(COREXZ)
  #include "Configuration_Core.h"
#elif MECH(DELTA)
  #include "Configuration_Delta.h"
#elif MECH(SCARA)
  #include "Configuration_Scara.h"
#endif

#include "Configuration_Feature.h"
#include "Configuration_Overall.h"
#endif