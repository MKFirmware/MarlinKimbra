#ifndef CONFIGURATIONSTORE_H
#define CONFIGURATIONSTORE_H

#include "Configuration.h"

void Config_ResetDefault();
void ConfigSD_ResetDefault();

#ifndef DISABLE_M503
  void Config_PrintSettings(bool forReplay=false);
  void ConfigSD_PrintSettings(bool forReplay=false);
#else
  FORCE_INLINE void Config_PrintSettings(bool forReplay=false) {}
  FORCE_INLINE void ConfigSD_PrintSettings(bool forReplay=false) {}
#endif

#ifdef EEPROM_SETTINGS
  void Config_StoreSettings();
  void Config_RetrieveSettings();
#else
  FORCE_INLINE void Config_StoreSettings() {}
  FORCE_INLINE void Config_RetrieveSettings() { Config_ResetDefault(); Config_PrintSettings(); }
#endif

#if defined(SDSUPPORT) && defined(SD_SETTINGS)
  static const char *cfgSD_KEY[] = { //Keep this in lexicographical order for better search performance(O(Nlog2(N)) insted of O(N*N)) (if you don't keep this sorted, the algorithm for find the key index won't work, keep attention.)
    #ifdef POWER_CONSUMPTION
      "PWR",
    #endif
    "TME",
  };
  
  enum cfgSD_ENUM {	//This need to be in the same order as cfgSD_KEY
    #ifdef POWER_CONSUMPTION
      SD_CFG_PWR,
    #endif
    SD_CFG_TME,
    SD_CFG_END //Leave this always as the last
  };
  
  void ConfigSD_StoreSettings();
  void ConfigSD_RetrieveSettings(bool addValue = false);
  int ConfigSD_KeyIndex(char *key);
#else
  FORCE_INLINE void ConfigSD_RetrieveSettings() {  ConfigSD_ResetDefault(); ConfigSD_PrintSettings();  }
#endif

#endif //CONFIGURATIONSTORE_H
