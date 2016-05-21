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

#ifndef PRINTCOUNTER_H
  #define PRINTCOUNTER_H

  #include "stopwatch.h"

  // Print debug messages with M111 S2
  //#define DEBUG_PRINTCOUNTER

  struct printStatistics {
    uint16_t numberPrints;          // Number of prints
    uint16_t completePrints;        // Number of complete prints
    uint32_t printTime;             // Total printing time
    uint32_t printer_usage_seconds; // Longest print job
    double printer_usage_filament;  // Filament usage
  };

  class PrintCounter: public Stopwatch {
    private:
      typedef Stopwatch super;

      /**
       * @brief Interval in seconds between counter updates
       * @details This const value defines what will be the time between each
       * accumulator update.
       */
      const uint16_t updateInterval = 10;

      /**
       * @brief Timestamp of the last call to deltaDuration()
       * @details Stores the timestamp of the last deltaDuration(), this is
       * required due to the updateInterval cycle.
       */
      uint16_t lastDuration;

    protected:
      /**
       * @brief dT since the last call
       * @details Returns the elapsed time in seconds since the last call, this is
       * used internally for print statistics accounting is not intended to be a
       * user callable function.
       */
      uint16_t deltaDuration();

    public:

      printStatistics data;

      /**
       * @brief Class constructor
       */
      PrintCounter();

      /**
       * @brief Checks if Print Statistics has been loaded
       * @details Returns true if the statistical data has been loaded.
       * @return bool
       */
      bool isLoaded();

      /**
       * @brief Resets the Print Statistics
       * @details Resets the statistics to zero
       * also the magic header.
       */
      void initStats();

      /**
       * @brief Serial output the Print Statistics
       * @details This function may change in the future, for now it directly
       * prints the statistical data to serial.
       */
      void showStats();

      /**
       * @brief Loop function
       * @details This function should be called at loop, it will take care of
       * periodically save the statistical data to EEPROM and do time keeping.
       */
      void tick();

      /**
       * The following functions are being overridden
       */
      void start();
      void stop();
      void reset();

      #if ENABLED(DEBUG_PRINTCOUNTER)

        /**
         * @brief Prints a debug message
         * @details Prints a simple debug message "PrintCounter::function"
         */
        static void debug(const char func[]);

      #endif
  };

#endif // PRINTCOUNTER_H
