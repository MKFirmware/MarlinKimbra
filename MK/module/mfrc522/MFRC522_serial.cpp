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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * MFRC522 Serial
 * Designed for module MFRC522 width UART
 *
 * Copyright (C) 2016 Alberto Cotronei MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "../../base.h"
#include <Arduino.h>

#ifdef RFID_MODULE

  #include "MFRC522_serial.h"

  uint8_t MFRC522Data[20];

  /**
   * Constructor.
   */
  MFRC522::MFRC522() {
    #if RFID_SERIAL > 0
      #if RFID_SERIAL == 1
        #define RFID Serial1
      #elif RFID_SERIAL == 2
        #define RFID Serial2
      #elif RFID_SERIAL == 3
        #define RFID Serial3
      #endif
    #else
      #define RFID Serial1
    #endif
  }

  /**
   * Start Serial
   */
  bool MFRC522::init() {

    RFID.begin(9600);
    HAL::delayMilliseconds(1000);

    write(MFRC522_HEADER);  // Header
    write(0x02);
    write(COMMAND_READ_ID);

    HAL::delayMilliseconds(500);
    if (available()) {
      clear(); // Clear Buffer
      return true;
    }
    else {
      return false;
    }
  }

  bool MFRC522::getID(int8_t e) {
    if (communicate(0x02, COMMAND_READ_ID)) {
      for (int i = 0; i < 4; i++)
        RfidDataID[e].RfidPacketID[i] = MFRC522Data[i];
      return true;
    }
    return false;
  }

  bool MFRC522::readBlock(int8_t e) {

    int8_t Packetdata = 0;
    byte sendData[8] = {
      0x01,         // block
      MIFARE_KEYA,  // key type
      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  // key
    };

    for (uint8_t sector = BLOCK_START; sector < BLOCK_TOTAL; sector += 4) {
      sendData[0] = sector;
      if (communicate(
        0x0A,               // length
        COMMAND_READ_BLOCK, // command
        sendData            // sendData
        )) {
          for (int8_t i = 0; i < 16; i++)
            RfidData[e].RfidPacket[i + Packetdata] = MFRC522Data[i];
        }
      else {
        return false;
      }
      Packetdata += 16;
      HAL::delayMilliseconds(20);
    }
    return true;
  }

  bool MFRC522::writeBlock(int8_t e) {

    int8_t Packetdata = 0;
    byte sendData[24] = {
      0x01,         // block
      MIFARE_KEYA,  // key type
      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
    };

    for (uint8_t sector = BLOCK_START; sector < BLOCK_TOTAL; sector += 4) {
      sendData[0] = sector;
      for (int8_t i = 0; i < 16; i++) {
        sendData[i + 8] = RfidData[e].RfidPacket[i + Packetdata];
      }

      if (!(communicate(
        0x1A,                 // length
        COMMAND_WRITE_BLOCK,  // command
        sendData              // sendData
      ))) {
        return false;
      }
      HAL::delayMilliseconds(20);
      Packetdata += 16;
    }
    return true;
  }

  /**
   * Description： Comunication between MFRC522 and ISO14443.
   * Command Format: Header + Lenght + Command + (Data)
   * Return： Return STATUS_OK if success.
   */
  bool MFRC522::communicate(uint8_t sendDataLength, uint8_t command, uint8_t* sendData) {

    // Send instruction to MFRC522
    write(MFRC522_HEADER);      // Header
    write(sendDataLength);      // Length (Length + Command + Data)
    write(command);             // Command

    if (sendData != NULL) {
      for (uint8_t i = 0; i < sendDataLength - 2; i++) {
        write(sendData[i]);     // Data
      }
    }

    // Read response to MFRC522
    while (!available());
    uint8_t header = read();            // Header
    while (!available());
    uint8_t returnDataLength = read();  // Length (Length + Command + Data)
    while (!available());
    uint8_t commandResult = read();     // Command result

    for (int i = 0; i < returnDataLength - 2; i++) {
      while (!available());
      if (available()) {
        MFRC522Data[i] = read(); // Data
      }
    }

    // Return
    if (command != commandResult) {
      return STATUS_ERROR;
    }

    return STATUS_OK;
  }
  void MFRC522::printInfo(int8_t e) {
    char lung[30];
    ECHO_LMV(INFO, MSG_RFID_SPOOL, e);
    ECHO_LMT(INFO, MSG_RFID_BRAND, RfidData[e].data.brand);
    ECHO_LMT(INFO, MSG_RFID_TYPE, RfidData[e].data.type);
    ECHO_LMT(INFO, MSG_RFID_COLOR, RfidData[e].data.color);
    ECHO_LMV(INFO, MSG_RFID_SIZE, RfidData[e].data.size);
    ECHO_SMV(INFO, MSG_RFID_TEMP_HOTEND, RfidData[e].data.temphotendmin);
    ECHO_EMV(" - ", RfidData[e].data.temphotendmax);
    ECHO_SMV(INFO, MSG_RFID_TEMP_BED, RfidData[e].data.tempbedmin);
    ECHO_EMV(" - ", RfidData[e].data.tempbedmax);
    ECHO_LMV(INFO, MSG_RFID_TEMP_USER_HOTEND, RfidData[e].data.temphotend);
    ECHO_LMV(INFO, MSG_RFID_TEMP_USER_BED, RfidData[e].data.tempbed);
    ECHO_SMV(INFO, MSG_RFID_DENSITY, RfidData[e].data.density); ECHO_EM("%");
    unsigned int  kmeter = (long)RfidData[e].data.lenght / 1000 / 1000,
                  meter = ((long)RfidData[e].data.lenght / 1000) % 1000,
                  centimeter = ((long)RfidData[e].data.lenght / 10) % 100,
                  millimeter = ((long)RfidData[e].data.lenght) % 10;
    sprintf_P(lung, PSTR("%i Km %i m %i cm %i mm"), kmeter, meter, centimeter, millimeter);
    ECHO_LMT(INFO, MSG_RFID_SPOOL_LENGHT, lung);
  }

  // Private
  /*
   * Description：Write a byte data into MFRC522.
   */
  void MFRC522::write(uint8_t value) {
    RFID.write(value);
}

  /*
   * Description：Read a byte data of MFRC522
   * Return：Return the read value.
   */
  byte MFRC522::read() {
    return RFID.read();
  }

  void MFRC522::clear() {
    while (available()) RFID.read();
  }

  /*
   * Description：Returns true if detect card in MFRC522.
   */
  bool MFRC522::available() {
    return (RFID.available() > 0);
  }

#endif // ENABLED(RFID_MODULE)
