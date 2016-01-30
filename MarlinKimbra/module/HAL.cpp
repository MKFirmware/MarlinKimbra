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

#include "../base.h"
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

#ifndef EXTERNALSERIAL
  // Implement serial communication for one stream only!
  /*
    HardwareSerial.h - Hardware serial library for Wiring
    Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

    Modified 28 September 2010 by Mark Sproul

    Modified to use only 1 queue with fixed length by Repetier
  */

  ring_buffer rx_buffer = { { 0 }, 0, 0};
  ring_buffer_tx tx_buffer = { { 0 }, 0, 0};

  inline void rf_store_char(unsigned char c, ring_buffer *buffer) {
    uint8_t i = (buffer->head + 1) & SERIAL_BUFFER_MASK;

    // if we should be storing the received character into the location
    // just before the tail (meaning that the head would advance to the
    // current location of the tail), we're about to overflow the buffer
    // and so we don't write the character or advance the head.
    if (i != buffer->tail) {
        buffer->buffer[buffer->head] = c;
        buffer->head = i;
    }
  }

  #if !defined(USART0_RX_vect) && defined(USART1_RX_vect)
    // do nothing - on the 32u4 the first USART is USART1
  #else
    void rfSerialEvent() __attribute__((weak));
    void rfSerialEvent() {}
    #define serialEvent_implemented
    #if defined(USART_RX_vect)
      SIGNAL(USART_RX_vect)
    #elif defined(USART0_RX_vect)
      SIGNAL(USART0_RX_vect)
    #else
      #if defined(SIG_USART0_RECV)
        SIGNAL(SIG_USART0_RECV)
      #elif defined(SIG_UART0_RECV)
        SIGNAL(SIG_UART0_RECV)
      #elif defined(SIG_UART_RECV)
        SIGNAL(SIG_UART_RECV)
      #else
        #error "Don't know what the Data Received vector is called for the first UART"
      #endif
    #endif
    {
      #if defined(UDR0)
        uint8_t c  =  UDR0;
      #elif defined(UDR)
        uint8_t c  =  UDR;
      #else
        #error UDR not defined
      #endif
      rf_store_char(c, &rx_buffer);
    }
  #endif

  #if !defined(USART0_UDRE_vect) && defined(USART1_UDRE_vect)
    // do nothing - on the 32u4 the first USART is USART1
  #else
    #if !defined(UART0_UDRE_vect) && !defined(UART_UDRE_vect) && !defined(USART0_UDRE_vect) && !defined(USART_UDRE_vect)
      #error "Don't know what the Data Register Empty vector is called for the first UART"
    #else
      #if defined(UART0_UDRE_vect)
        ISR(UART0_UDRE_vect)
      #elif defined(UART_UDRE_vect)
        ISR(UART_UDRE_vect)
      #elif defined(USART0_UDRE_vect)
        ISR(USART0_UDRE_vect)
      #elif defined(USART_UDRE_vect)
        ISR(USART_UDRE_vect)
      #endif
      {
        if (tx_buffer.head == tx_buffer.tail) {
          // Buffer empty, so disable interrupts
          #if defined(UCSR0B)
            bit_clear(UCSR0B, UDRIE0);
          #else
            bit_clear(UCSRB, UDRIE);
          #endif
        }
        else {
          // There is more data in the output buffer. Send the next byte
          uint8_t c = tx_buffer.buffer[tx_buffer.tail];
          #if defined(UDR0)
            UDR0 = c;
          #elif defined(UDR)
            UDR = c;
          #else
            #error UDR not defined
          #endif
          tx_buffer.tail = (tx_buffer.tail + 1) & SERIAL_TX_BUFFER_MASK;
        }
      }
    #endif
  #endif

  #if defined(BLUETOOTH) && BLUETOOTH_SERIAL > 0
    #if !(defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega1281__) || defined (__AVR_ATmega644__) || defined (__AVR_ATmega644P__))
      #error BlueTooth option cannot be used with your mainboard
    #endif
    #if BLUETOOTH_SERIAL > 1 && !(defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__))
      #error BlueTooth serial 2 or 3 can be used only with boards based on ATMega2560 or ATMega1280
    #endif
    #if (BLUETOOTH_SERIAL == 1)
      #if defined(USART1_RX_vect)
        #define SIG_USARTx_RECV   USART1_RX_vect
        #define USARTx_UDRE_vect  USART1_UDRE_vect
      #else
        #define SIG_USARTx_RECV   SIG_USART1_RECV
        #define USARTx_UDRE_vect  SIG_USART1_DATA
      #endif
      #define UDRx              UDR1
      #define UCSRxA            UCSR1A
      #define UCSRxB            UCSR1B
      #define UBRRxH            UBRR1H
      #define UBRRxL            UBRR1L
      #define U2Xx              U2X1
      #define UARTxENABLE       ((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)|(1<<UDRIE1))
      #define UDRIEx            UDRIE1
      #define RXxPIN            19
    #elif (BLUETOOTH_SERIAL == 2)
      #if defined(USART2_RX_vect)
        #define SIG_USARTx_RECV   USART2_RX_vect
        #define USARTx_UDRE_vect  USART2_UDRE_vect
      #else
        #define SIG_USARTx_RECV SIG_USART2_RECV
        #define USARTx_UDRE_vect  SIG_USART2_DATA
      #endif
      #define UDRx              UDR2
      #define UCSRxA            UCSR2A
      #define UCSRxB            UCSR2B
      #define UBRRxH            UBRR2H
      #define UBRRxL            UBRR2L
      #define U2Xx              U2X2
      #define UARTxENABLE       ((1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2)|(1<<UDRIE2))
      #define UDRIEx            UDRIE2
      #define RXxPIN            17
    #elif (BLUETOOTH_SERIAL == 3)
      #if defined(USART3_RX_vect)
        #define SIG_USARTx_RECV   USART3_RX_vect
        #define USARTx_UDRE_vect  USART3_UDRE_vect
      #else
        #define SIG_USARTx_RECV SIG_USART3_RECV
        #define USARTx_UDRE_vect  SIG_USART3_DATA
      #endif
      #define UDRx              UDR3
      #define UCSRxA            UCSR3A
      #define UCSRxB            UCSR3B
      #define UBRRxH            UBRR3H
      #define UBRRxL            UBRR3L
      #define U2Xx              U2X3
      #define UARTxENABLE       ((1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3)|(1<<UDRIE3))
      #define UDRIEx            UDRIE3
      #define RXxPIN            15
    #else
      #error Wrong serial port number for BlueTooth
    #endif

    SIGNAL(SIG_USARTx_RECV) {
      uint8_t c  =  UDRx;
      rf_store_char(c, &rx_buffer);
    }

    volatile uint8_t txx_buffer_tail = 0;

    ISR(USARTx_UDRE_vect) {
      if (tx_buffer.head == txx_buffer_tail) {
        // Buffer empty, so disable interrupts
        bit_clear(UCSRxB, UDRIEx);
      }
      else {
        // There is more data in the output buffer. Send the next byte
        uint8_t c = tx_buffer.buffer[txx_buffer_tail];
        txx_buffer_tail = (txx_buffer_tail + 1) & SERIAL_TX_BUFFER_MASK;
        UDRx = c;
      }
    }
  #endif

  // Constructors

  MKHardwareSerial::MKHardwareSerial(ring_buffer *rx_buffer, ring_buffer_tx *tx_buffer,
                                     volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
                                     volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
                                     volatile uint8_t *udr,
                                     uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udrie, uint8_t u2x) {
    _rx_buffer = rx_buffer;
    _tx_buffer = tx_buffer;
    _ubrrh = ubrrh;
    _ubrrl = ubrrl;
    _ucsra = ucsra;
    _ucsrb = ucsrb;
    _udr = udr;
    _rxen = rxen;
    _txen = txen;
    _rxcie = rxcie;
    _udrie = udrie;
    _u2x = u2x;
  }

  // Public Methods

  void MKHardwareSerial::begin(unsigned long baud) {
    uint16_t baud_setting;
    bool use_u2x = true;

    #if F_CPU == 16000000UL
      // hardcoded exception for compatibility with the bootloader shipped
      // with the Duemilanove and previous boards and the firmware on the 8U2
      // on the Uno and Mega 2560.
      if (baud == 57600) {
        use_u2x = false;
      }
    #endif

  try_again:

    if (use_u2x) {
      *_ucsra = 1 << _u2x;
      baud_setting = (F_CPU / 4 / baud - 1) / 2;
    }
    else {
      *_ucsra = 0;
      baud_setting = (F_CPU / 8 / baud - 1) / 2;
    }

    if ((baud_setting > 4095) && use_u2x) {
      use_u2x = false;
      goto try_again;
    }

    // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
    *_ubrrh = baud_setting >> 8;
    *_ubrrl = baud_setting;

    bit_set(*_ucsrb, _rxen);
    bit_set(*_ucsrb, _txen);
    bit_set(*_ucsrb, _rxcie);
    bit_clear(*_ucsrb, _udrie);
    #if defined(BLUETOOTH) && BLUETOOTH_SERIAL > 0
      WRITE(RXxPIN,1);            // Pullup on RXDx
      UCSRxA  = (1<<U2Xx);
      UBRRxH = (uint8_t)(((F_CPU / 4 / BLUETOOTH_BAUD -1) / 2) >> 8);
      UBRRxL = (uint8_t)(((F_CPU / 4 / BLUETOOTH_BAUD -1) / 2) & 0xFF);
      UCSRxB |= UARTxENABLE;
    #endif
  }

  void MKHardwareSerial::end() {
    // wait for transmission of outgoing data
    while (_tx_buffer->head != _tx_buffer->tail)
        ;

    bit_clear(*_ucsrb, _rxen);
    bit_clear(*_ucsrb, _txen);
    bit_clear(*_ucsrb, _rxcie);
    bit_clear(*_ucsrb, _udrie);

    #if defined(BLUETOOTH_SERIAL) && BLUETOOTH_SERIAL > 0
      UCSRxB = 0;
    #endif
    // clear a  ny received data
    _rx_buffer->head = _rx_buffer->tail;
  }

  int MKHardwareSerial::available(void) {
    return (unsigned int)(SERIAL_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) & SERIAL_BUFFER_MASK;
  }

  int MKHardwareSerial::outputUnused(void) {
    return SERIAL_TX_BUFFER_SIZE - (unsigned int)((SERIAL_TX_BUFFER_SIZE + _tx_buffer->head - _tx_buffer->tail) & SERIAL_TX_BUFFER_MASK);
  }

  int MKHardwareSerial::peek(void) {
    if (_rx_buffer->head == _rx_buffer->tail)
      return -1;
    return _rx_buffer->buffer[_rx_buffer->tail];
  }

  int MKHardwareSerial::read(void) {
    // if the head isn't ahead of the tail, we don't have any characters
    if (_rx_buffer->head == _rx_buffer->tail) {
      return -1;
    }
    unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
    _rx_buffer->tail = (_rx_buffer->tail + 1) & SERIAL_BUFFER_MASK;
    return c;
  }

  void MKHardwareSerial::flush() {
    while (_tx_buffer->head != _tx_buffer->tail);
    #if defined(BLUETOOTH) && BLUETOOTH_SERIAL > 0
      while (_tx_buffer->head != txx_buffer_tail);
    #endif
  }

  #ifdef COMPAT_PRE1
    void
  #else
    size_t
  #endif
    MKHardwareSerial::write(uint8_t c) {
      uint8_t i = (_tx_buffer->head + 1) & SERIAL_TX_BUFFER_MASK;

      // If the output buffer is full, there's nothing for it other than to
      // wait for the interrupt handler to empty it a bit
      while (i == _tx_buffer->tail) {}
      #if defined(BLUETOOTH) && BLUETOOTH_SERIAL > 0
        while (i == txx_buffer_tail) {}
      #endif
      _tx_buffer->buffer[_tx_buffer->head] = c;
      _tx_buffer->head = i;

      bit_set(*_ucsrb, _udrie);
      #if defined(BLUETOOTH) && BLUETOOTH_SERIAL > 0
          bit_set(UCSRxB, UDRIEx);
      #endif
      #ifndef COMPAT_PRE1
          return 1;
      #endif
    }

  // Preinstantiate Object

  #if defined(UBRRH) && defined(UBRRL)
    MKHardwareSerial MKSerial(&rx_buffer, &tx_buffer, &UBRRH, &UBRRL, &UCSRA, &UCSRB, &UDR, RXEN, TXEN, RXCIE, UDRIE, U2X);
  #elif defined(UBRR0H) && defined(UBRR0L)
    MKHardwareSerial MKSerial(&rx_buffer, &tx_buffer, &UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UDR0, RXEN0, TXEN0, RXCIE0, UDRIE0, U2X0);
  #elif defined(USBCON)
    // do nothing - Serial object and buffers are initialized in CDC code
  #else
    #error no serial port defined  (port 0)
  #endif

#endif