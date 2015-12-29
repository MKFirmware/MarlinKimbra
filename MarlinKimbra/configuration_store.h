#ifndef CONFIGURATION_STORE_H
#define CONFIGURATION_STORE_H

#include "base.h"

void Config_ResetDefault();
void ConfigSD_ResetDefault();

#if DISABLED(DISABLE_M503)
void Config_PrintSettings(bool forReplay = false);
void ConfigSD_PrintSettings(bool forReplay = false);
#else
FORCE_INLINE void Config_PrintSettings(bool forReplay = false) {}
FORCE_INLINE void ConfigSD_PrintSettings(bool forReplay = false) {}
#endif

#if ENABLED(EEPROM_SETTINGS)
void Config_StoreSettings();
void Config_RetrieveSettings();
#else
FORCE_INLINE void Config_StoreSettings() {}
FORCE_INLINE void Config_RetrieveSettings() { Config_ResetDefault(); Config_PrintSettings(); }
#endif

#if ENABLED(SDSUPPORT) && ENABLED(SD_SETTINGS)
void ConfigSD_StoreSettings();
void ConfigSD_RetrieveSettings(bool addValue = false);
int ConfigSD_KeyIndex(char *key);
#else
FORCE_INLINE void ConfigSD_RetrieveSettings() {  ConfigSD_ResetDefault(); }
#endif

#endif //CONFIGURATION_STORE_H
