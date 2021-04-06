/*
  MHZ19C.cpp - MHZ19C Sensor Library
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
#include "MHZ19C.h"

// Constructor - Inititalize Hardware UART
MHZ19C::MHZ19C(void) {
  Serial.begin(9600);
  Serial.setTimeout(MHZ19C_READ_TIMEOUT);
}

MHZ19C::MHZ19C(pin_size_t calpin) {
  this->calpin = calpin;
  
  Serial.begin(9600);
  Serial.setTimeout(MHZ19C_READ_TIMEOUT);
}

void MHZ19C::initialize(void) {
  #ifdef MHZ19C_ENABLE_AUTOCAL
  setSelfCalibration(1);
  #else
  setSelfCalibration(0);
  #endif
}

void MHZ19C::calibrate(void) {
  pinMode(calpin, OUTPUT);
  digitalWrite(calpin, LOW);
  delay(7500);
  digitalWrite(calpin, HIGH);
  pinMode(calpin, INPUT_PULLUP);
}

uint8_t MHZ19C::getSensorData(char * payload, uint8_t startbyte) {
    write(MHZ19C_CMD_GET_PPM, 0x00);
    delay(50);
    uint8_t readBytes = read();
    
    payload[startbyte]   = 0x00;
    payload[startbyte+1] = 0x00;
    if (readBytes > 0) {
        switch(buffer[1]) {
            case 0x86:
              uint16ToPayload((buffer[2]*256) + buffer[3], payload, startbyte);
              break;
        }
    }
    return startbyte+2;
}

// Turn Self Calibration Routine On or Off
void MHZ19C::setSelfCalibration(bool state) {
  if (state) {
    write(0x79, 0xA0);
  } else {
    write(0x79, 0x00);
  }
}

// Write a Command to the Sensor
void MHZ19C::write(uint8_t cmd, uint8_t arg) {
    uint8_t _cmd[9] = {0xFF, 0x01, cmd, arg, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t crc = crc8(_cmd);
    _cmd[8] = crc;
    while (Serial.available() > 0) Serial.read();
    Serial.write(_cmd, 9);
    Serial.flush();
}

// Read a Sensor Response
uint8_t MHZ19C::read() {
    uint8_t ret = 0;
    zeroBuffer();

    // Read Available Bytes
    if (Serial.available() > 0) {
      ret = Serial.readBytes(buffer, MHZ19C_SER_BUF_LEN);
    }
    
    // Check Sync Bit and CRC
    if (buffer[0] != 0xFF || buffer [8] != crc8(buffer))
      return 0;   
    
    // Return Read Bytes
    return ret;
}

// Fill the Internal Buffer with Zeroes
void MHZ19C::zeroBuffer() {
  for (int i=0; i < MHZ19C_SER_BUF_LEN; i++)
    buffer[i] = 0x00;
}

// Calculate 8Bit CRC of Messages and Commands
uint8_t MHZ19C::crc8(uint8_t *paket){
  uint8_t i, checksum = 0x00;
  for( i = 1; i < 8; i++)
    checksum += paket[i];

  checksum  = 0xff - checksum;
  checksum += 1;
  return checksum;
}