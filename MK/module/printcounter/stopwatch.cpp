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
#include "stopwatch.h"

Stopwatch::Stopwatch() {
  this->reset();
}

bool Stopwatch::stop() {
  #if ENABLED(DEBUG_STOPWATCH)
    Stopwatch::debug(PSTR("stop"));
  #endif

  if (this->isRunning() || this->isPaused()) {
    this->state = STOPWATCH_STOPPED;
    this->stopTimestamp = millis();
    return true;
  }
  else return false;
}

bool Stopwatch::pause() {
  #if ENABLED(DEBUG_STOPWATCH)
    Stopwatch::debug(PSTR("pause"));
  #endif

  if (this->isRunning()) {
    this->state = STOPWATCH_PAUSED;
    this->stopTimestamp = millis();
    return true;
  }
  else return false;
}

bool Stopwatch::start() {
  #if ENABLED(DEBUG_STOPWATCH)
    Stopwatch::debug(PSTR("start"));
  #endif

  if (!this->isRunning()) {
    if (this->isPaused()) this->accumulator = this->duration();
    else this->reset();

    this->state = STOPWATCH_RUNNING;
    this->startTimestamp = millis();
    return true;
  }
  else return false;
}

void Stopwatch::reset() {
  #if ENABLED(DEBUG_STOPWATCH)
    Stopwatch::debug(PSTR("reset"));
  #endif

  this->state = STOPWATCH_STOPPED;
  this->startTimestamp = 0;
  this->stopTimestamp = 0;
  this->accumulator = 0;
}

bool Stopwatch::isRunning() {
  return (this->state == STOPWATCH_RUNNING) ? true : false;
}

bool Stopwatch::isPaused() {
  return (this->state == STOPWATCH_PAUSED) ? true : false;
}

uint16_t Stopwatch::duration() {
  return (((this->isRunning()) ? millis() : this->stopTimestamp)
          - this->startTimestamp) / 1000 + this->accumulator;
}

#if ENABLED(DEBUG_STOPWATCH)
  void Stopwatch::debug(const char func[]) {
    if (DEBUGGING(INFO)) {
      ECHO_MT("Stopwatch::", func);
      ECHO_EM("()");
    }
  }
#endif
