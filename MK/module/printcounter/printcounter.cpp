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

#include "../../base.h"
#include "printcounter.h"

PrintCounter::PrintCounter(): super() {
  this->initStats();
}

uint16_t PrintCounter::deltaDuration() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("deltaDuration"));
  #endif

  uint16_t tmp = this->lastDuration;
  this->lastDuration = this->duration();
  return this->lastDuration - tmp;
}

void PrintCounter::initStats() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("initStats"));
  #endif

  this->data = { 0, 0, 0, 0, 0.0 };
}

void PrintCounter::loadStats() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("loadStats"));
  #endif

  #if ENABLED(SDSUPPORT) && ENABLED(SD_SETTINGS)
    // Checks if the SDCARD is inserted
    if(IS_SD_INSERTED && !IS_SD_PRINTING) {
      ConfigSD_RetrieveSettings(true);
    }
  #endif
}

void PrintCounter::saveStats() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("saveStats"));
  #endif

  // Refuses to save data is object is not loaded
  if (!this->loaded) return;

  #if ENABLED(SDSUPPORT) && ENABLED(SD_SETTINGS)
    ConfigSD_StoreSettings();
  #endif
}

void PrintCounter::showStats() {
  char temp[30];
  uint32_t day, hours, minutes;

  ECHO_MV("Print statistics: Total: ", this->data.numberPrints);
  ECHO_MV(", Finished: ", this->data.completePrints);
  ECHO_M(", Failed: ");
  ECHO_EV (this->data.numberPrints - this->data.completePrints -
          ((this->isRunning() || this->isPaused()) ? 1 : 0)); // Removes 1 from failures with an active counter

  day     = this->data.printTime / 60 / 60 / 24;
  hours   = (this->data.printTime / 60 / 60) % 24;
  minutes = (this->data.printTime / 60) % 60;

  sprintf_P(temp, PSTR("  %i " MSG_END_DAY " %i " MSG_END_HOUR " %i " MSG_END_MINUTE), day, hours, minutes);
  ECHO_EMT("Total print time: ", temp);

  day     = this->data.printer_usage_seconds / 60 / 60 / 24;
  hours   = (this->data.printer_usage_seconds / 60 / 60) % 24;
  minutes = (this->data.printer_usage_seconds / 60) % 60;

  sprintf_P(temp, PSTR("  %i " MSG_END_DAY " %i " MSG_END_HOUR " %i " MSG_END_MINUTE), day, hours, minutes);
  ECHO_EMT("Power on time: ", temp);

  uint32_t  kmeter = (long)this->data.printer_usage_filament / 1000 / 1000,
            meter = ((long)this->data.printer_usage_filament / 1000) % 1000,
            centimeter = ((long)this->data.printer_usage_filament / 10) % 100,
            millimeter = ((long)this->data.printer_usage_filament) % 10;
  sprintf_P(temp, PSTR("  %i Km %i m %i cm %i mm"), kmeter, meter, centimeter, millimeter);

  ECHO_EMT("Filament printed: ", temp);
}

void PrintCounter::tick() {

  static uint32_t update_before = millis(),
                  config_last_update = millis();

  uint32_t now = millis();

  // Trying to get the amount of calculations down to the bare min
  const static uint16_t i = this->updateInterval * 1000;

  if (now - update_before >= i) {
    this->data.printer_usage_seconds += this->updateInterval;

    if (this->isRunning())
      this->data.printTime += this->deltaDuration();

    update_before = now;

    #if ENABLED(DEBUG_PRINTCOUNTER)
      PrintCounter::debug(PSTR("tick"));
    #endif
  }

  #if ENABLED(SDSUPPORT) && ENABLED(SD_SETTINGS)
    if (!this->loaded) {
      this->loadStats();
      this->saveStats();
    }
    else if (now - config_last_update >= this->saveInterval) {
      config_last_update = now;
      this->saveStats();
    }
  #endif
}

bool PrintCounter::start() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("start"));
  #endif

  bool paused = this->isPaused();

  if (super::start()) {
    if (!paused) {
      this->data.numberPrints++;
      this->lastDuration = 0;
    }
    return true;
  }
  else return false;
}

bool PrintCounter::stop() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("stop"));
  #endif

  if (super::stop()) {
    this->data.completePrints++;
    this->data.printTime += this->deltaDuration();
    this->saveStats();
  }
  else return false;
}

void PrintCounter::reset() {
  #if ENABLED(DEBUG_PRINTCOUNTER)
    PrintCounter::debug(PSTR("stop"));
  #endif

  super::reset();
  this->lastDuration = 0;
}

#if ENABLED(DEBUG_PRINTCOUNTER)

  void PrintCounter::debug(const char func[]) {
    if (DEBUGGING(INFO)) {
      SERIAL_ECHOPGM("PrintCounter::");
      serialprintPGM(func);
      SERIAL_ECHOLNPGM("()");
    }
  }

#endif
