/*
  SG112A.cpp - SG112A Sensor Library
  Copyright (c) 2020-2021, Stefan Brand
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <Arduino.h>
#include "SG112A.h"

// Constructor - Inititalize Hardware UART
SG112A::SG112A(void) {
    Serial.begin(9600);
    Serial.setTimeout(READ_TIMEOUT);
}


uint8_t SG112A::getSensorData(char *payload, uint8_t startbyte) {
    write(CMD_GET_PPM);
    delay(50);
    uint8_t readBytes = read();
    
    payload[startbyte]   = 0x00;
    payload[startbyte+1] = 0x00;
    if (readBytes > 0) {
        switch(buffer[2]) {
            case 0x15:
              uint16ToPayload((buffer[3]*256) + buffer[4], payload, startbyte);
              break;
        }
    }
  return startbyte+2;
}

// Write a Command to the Sensor
void SG112A::write(byte cmd) {
    uint8_t _cmd[6] = {0xAA, 0x55, cmd, 0x00, 0x00, 0x00};
    uint16_t crc = crc16(_cmd, 4);
    _cmd[4] = (uint8_t)(crc & 0xFF);
    _cmd[5] = (uint8_t)(crc >> 8);
    while (Serial.available() > 0) Serial.read();
    Serial.write(_cmd, 6);
    Serial.flush();
}

// Read a Sensor Response
uint8_t SG112A::read() {
    // Number of returned Bytes
    uint8_t ret = 0;
    // Clear Internal Buffer
    zeroBuffer();

    // Read Available Bytes
    if (Serial.available() > 0) {
      ret = Serial.readBytes(buffer, SER_BUF_LEN);
    }

    // Check Sync Bytes
    if (buffer[0] != 0xBB || buffer[1] != 0x66)
      return 0;
   
    // Check CRC of the Returned Messages
    uint16_t crc = crc16(buffer, ret-2);
    if (buffer[ret-1] != (uint8_t)(crc >> 8) || buffer[ret-2] != (uint8_t)(crc & 0xFF))
      return 0;

    return ret;
}

// Fill the Internal Buffer with Zeroes
void SG112A::zeroBuffer() {
    for (int i=0; i < SER_BUF_LEN; i++)
        buffer[i] = 0x00;
}

// Calculate 16Bit CRC of Messages and Commands
uint16_t SG112A::crc16(uint8_t *cmd, int len){
  uint16_t ret = 0xffff;
  uint16_t polynomial = 0xa001;
  int shift = 0x0;
  int i = 0;
  for (i = len - 1; i >= 0 ; i-- ){
    
    uint16_t code = ( uint16_t )( cmd [ len -1 - i ]  & 0xff );
    ret = ret^code;
    shift = 0x0;
    while ( shift <= 7 ){           
      if ( ret & 0x1 ) {                     
        ret = ret >> 1;                     
        ret = ret^polynomial ;
      } else {
        ret = ret >> 1;
      }
      shift++;
    }
  }      
  return ret;
}