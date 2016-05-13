/*
  flowmeter.cpp - Flowmeter control library for Arduino - Version 1
  Copyright (c) 2016 Franco (nextime) Lanza.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "../../base.h"
#include <Arduino.h>


#if ENABLED(FLOWMETER_SENSOR)

volatile byte flowrate_pulsecount;  
float flowrate;
unsigned int flowml;
static millis_t flowmeter_timer = 0;

void flowrate_pulsecounter();

void flow_init() {

   flowrate = 0;
   flowrate_pulsecount = 0;
   flowml = 0;
   pinMode(FLOWMETER_PIN, INPUT);
   
   attachInterrupt(digitalPinToInterrupt(FLOWMETER_PIN), flowrate_pulsecounter, FALLING);
}

void flowrate_manage() {
   millis_t  now;
   now = millis();
   if(ELAPSED(now, flowmeter_timer)) {
      detachInterrupt(digitalPinToInterrupt(FLOWMETER_PIN));
      flowrate  = ((1000.0 / (now - flowmeter_timer)) * flowrate_pulsecount) / FLOWMETER_CALIBRATION;
      flowmeter_timer = now + 1000UL;
      flowml = (flowrate / 60.0) * 1000;

      flowrate_pulsecount = 0;
      attachInterrupt(digitalPinToInterrupt(FLOWMETER_PIN), flowrate_pulsecounter, FALLING);
   }

}

int get_flowrate() {
   return flowml;
}

void flowrate_pulsecounter()
{
  // Increment the pulse counter
  flowrate_pulsecount++;
}

#endif // FLOWMETER_SENSOR

