#ifndef CONFIGURATION_VERSION_H
  #define CONFIGURATION_VERSION_H

  #define SHORT_BUILD_VERSION "4.2.6_dev"
  #define BUILD_VERSION "MK_" SHORT_BUILD_VERSION
  #define STRING_DISTRIBUTION_DATE __DATE__ " " __TIME__    // build date and time
  // It might also be appropriate to define a location where additional information can be found
  #define FIRMWARE_URL  "https://github.com/MagoKimbra/MarlinKimbra"
#endif