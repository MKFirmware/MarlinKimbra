/**
  * This file is part of MarlinKimbra Firmware.
  *
  * MarlinKimbra Firmware is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * MarlinKimbra Firmware is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with MarlinKimbra Firmware. If not, see <http://www.gnu.org/licenses/>.
  */
#include "../../base.h"

void Com::printF(FSTRINGPARAM(ptr)) {
  char c;
  while ((c = HAL::readFlashByte(ptr++)) != 0)
    HAL::serialWriteByte(c);
}

void Com::printVal(int value) {
  print(value);
}

void Com::printVal(int8_t value) {
  print(value);
}

void Com::printVal(uint8_t value) {
  print(value);
}

void Com::printVal(int32_t value) {
  print(value);
}

void Com::printVal(uint32_t value) {
  printNumber(value);
}

void Com::printVal(float value, uint8_t digits) {
  printFloat(value, digits);
}

void Com::printVal(double value, uint8_t digits) {
  printFloat(value, digits);
}

void Com::print(const char* text) {
  while(*text) {
    HAL::serialWriteByte(*text++);
  }
}

void Com::print(char c) {
  HAL::serialWriteByte(c);
}

void Com::print(float number) {
  printFloat(number, 6);
}

void Com::print(int value) {
  print((int32_t)value);
}

void Com::print(long value) {
  if(value < 0) {
    HAL::serialWriteByte('-');
    value = -value;
  }
  printNumber(value);
}

void Com::print(uint16_t value) {
  printNumber(value);
}

void Com::print(uint32_t value) {
  printNumber(value);
}

void Com::printNumber(uint32_t n) {
  char buf[11]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[10];
  *str = '\0';
  do {
    unsigned long m = n;
    n /= 10;
    *--str = '0'+(m - 10 * n);
  } while(n);

  print(str);
}

void Com::printArray(float *arr, uint8_t n, uint8_t digits) {
  for (uint8_t i = 0; i < n; i++) {
    print(" ");
    printFloat(arr[i], digits);
  }
}

void Com::printArray(int32_t *arr, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    print(" ");
    printVal(arr[i]);
  }
}

void Com::printFloat(float number, uint8_t digits) {
  if (isnan(number)) {
	print(TNAN);
    return;
  }
  if (isinf(number)) {
	print(TINF);
    return;
  }
  // Handle negative numbers
  if (number < 0.0) {
    print('-');
    number = -number;
  }
  // Round correctly so that print(1.999, 2) prints as "2.00"
  float rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  float remainder = number - (float)int_part;
  printNumber(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    print('.');

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    print(toPrint);
    remainder -= toPrint;
  }
}
