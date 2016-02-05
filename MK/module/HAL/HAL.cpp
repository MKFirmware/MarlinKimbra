/*
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// **************************************************************************
//
// Description:          *** HAL for Arduino ***
//
// **************************************************************************

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../../base.h"
#include "HAL.h"

HAL::HAL() {
  // ctor
}

HAL::~HAL() {
  // dtor
}

// Print apparent cause of start/restart
void HAL::showStartReason() {
  byte mcu = MCUSR;
  if (mcu & 1) ECHO_EM(SERIAL_POWERUP);
  if (mcu & 2) ECHO_EM(SERIAL_EXTERNAL_RESET);
  if (mcu & 4) ECHO_EM(SERIAL_BROWNOUT_RESET);
  if (mcu & 8) ECHO_EM(SERIAL_WATCHDOG_RESET);
  if (mcu & 32) ECHO_EM(SERIAL_SOFTWARE_RESET);
  MCUSR = 0;
}

// Return available memory
int HAL::getFreeRam() {
  int freeram = 0;
  InterruptProtectedBlock noInts;
  uint8_t * heapptr, * stackptr;
  heapptr = (uint8_t *)malloc(4);          // get heap pointer
  free(heapptr);      // free up the memory again (sets heapptr to 0)
  stackptr =  (uint8_t *)(SP);           // save value of stack pointer
  freeram = (int)stackptr-(int)heapptr;
  return freeram;
}

// Reset peripherals and cpu
void HAL::resetHardware() {}
