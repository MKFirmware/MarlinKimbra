/* 
  flowmeter.h - Flowmeter control library for Arduino - Version 1
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
#ifndef FLOWMETER_H
#define FLOWMETER_H

#include <inttypes.h>
#include "../../base.h"


#define FLOWMETER_CALIBRATION (FLOWMETER_MAXFREQ/FLOWETER_MAXFLOW)

#if ENABLED(FLOWMETER_SENSOR)

void flowrate_manage();
void flow_init();
int get_flowrate();

#endif // FLOWMETER_SENSOR

#endif // FLOWMETER_H
