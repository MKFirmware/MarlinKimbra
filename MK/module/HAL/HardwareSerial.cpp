/*
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
  
  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
*/

#include "../../base.h"
#include "HardwareSerial.h"

#if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H) || defined(UBRR2H) || defined(UBRR3H)

  #if UART_PRESENT(SERIAL_PORT)
    ring_buffer rx_buffer  =  { { 0 }, 0, 0 };
  #endif

  FORCE_INLINE void store_char(unsigned char c) {
    int i = (unsigned int)(rx_buffer.head + 1) % RX_BUFFER_SIZE;
    if (i != rx_buffer.tail) {
      rx_buffer.buffer[rx_buffer.head] = c;
      rx_buffer.head = i;
    }
  }

  #if defined(M_USARTx_RX_vect)
    SIGNAL(M_USARTx_RX_vect) {
      unsigned char c  =  M_UDRx;
      store_char(c);
    }
  #endif

  // Constructors
  MKHardwareSerial::MKHardwareSerial() { }

  // Public Methods

  void MKHardwareSerial::begin(long baud) {
    uint16_t baud_setting;
    bool useU2X = true;

    #if F_CPU == 16000000UL && SERIAL_PORT == 0
      if (baud == 57600) {
        useU2X = false;
      }
    #endif

    if (useU2X) {
      M_UCSRxA = BIT(M_U2Xx);
      baud_setting = (F_CPU / 4 / baud - 1) / 2;
    }
    else {
      M_UCSRxA = 0;
      baud_setting = (F_CPU / 8 / baud - 1) / 2;
    }

    M_UBRRxH = baud_setting >> 8;
    M_UBRRxL = baud_setting;

    set_bit(M_UCSRxB, M_RXENx);
    set_bit(M_UCSRxB, M_TXENx);
    set_bit(M_UCSRxB, M_RXCIEx);
  }

  void MKHardwareSerial::end() {
    clear_bit(M_UCSRxB, M_RXENx);
    clear_bit(M_UCSRxB, M_TXENx);
    clear_bit(M_UCSRxB, M_RXCIEx);
  }

  int MKHardwareSerial::peek(void) {
    if (rx_buffer.head == rx_buffer.tail) {
      return -1;
    }
    else {
      return rx_buffer.buffer[rx_buffer.tail];
    }
  }

  int MKHardwareSerial::read(void) {
    if (rx_buffer.head == rx_buffer.tail) {
      return -1;
    }
    else {
      unsigned char c = rx_buffer.buffer[rx_buffer.tail];
      rx_buffer.tail = (unsigned int)(rx_buffer.tail + 1) % RX_BUFFER_SIZE;
      return c;
    }
  }

  void MKHardwareSerial::flush() {
    rx_buffer.head = rx_buffer.tail;
  }

  void MKHardwareSerial::print(char c, int base) {
    print((long) c, base);
  }

  void MKHardwareSerial::print(unsigned char b, int base) {
    print((unsigned long) b, base);
  }

  void MKHardwareSerial::print(int n, int base) {
    print((long) n, base);
  }

  void MKHardwareSerial::print(unsigned int n, int base) {
    print((unsigned long) n, base);
  }

  void MKHardwareSerial::print(long n, int base) {
    if (base == 0) {
      write(n);
    }
    else if (base == 10) {
      if (n < 0) {
        print('-');
        n = -n;
      }
      printNumber(n, 10);
    }
    else {
      printNumber(n, base);
    }
  }

  void MKHardwareSerial::print(unsigned long n, int base) {
    if (base == 0)
      write(n);
    else
      printNumber(n, base);
  }

  void MKHardwareSerial::print(double n, int digits) {
    printFloat(n, digits);
  }

  void MKHardwareSerial::println(void) {
    print('\r');
    print('\n');
  }

  void MKHardwareSerial::println(const String &s) {
    print(s);
    println();
  }

  void MKHardwareSerial::println(const char c[]) {
    print(c);
    println();
  }

  void MKHardwareSerial::println(char c, int base) {
    print(c, base);
    println();
  }

  void MKHardwareSerial::println(unsigned char b, int base) {
    print(b, base);
    println();
  }

  void MKHardwareSerial::println(int n, int base) {
    print(n, base);
    println();
  }

  void MKHardwareSerial::println(unsigned int n, int base) {
    print(n, base);
    println();
  }

  void MKHardwareSerial::println(long n, int base) {
    print(n, base);
    println();
  }

  void MKHardwareSerial::println(unsigned long n, int base) {
    print(n, base);
    println();
  }

  void MKHardwareSerial::println(double n, int digits) {
    print(n, digits);
    println();
  }

  // Private Methods

  void MKHardwareSerial::printNumber(unsigned long n, uint8_t base) {
    unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars. 
    unsigned long i = 0;

    if (n == 0) {
      print('0');
      return;
    }

    while (n > 0) {
      buf[i++] = n % base;
      n /= base;
    }

    for (; i > 0; i--)
      print((char) (buf[i - 1] < 10 ?
        '0' + buf[i - 1] :
        'A' + buf[i - 1] - 10));
  }

  void MKHardwareSerial::printFloat(double number, uint8_t digits) {
    // Handle negative numbers
    if (number < 0.0) {
       print('-');
       number = -number;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i)
      rounding /= 10.0;

    number += rounding;

    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;
    print(int_part);

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) print('.');

    // Extract digits from the remainder one at a time
    while (digits-- > 0) {
      remainder *= 10.0;
      int toPrint = int(remainder);
      print(toPrint);
      remainder -= toPrint; 
    }
  }
  
  // Preinstantiate Objects
  MKHardwareSerial MKSerial;

#endif
