/*
  .cpp - SENSAIRS8 Sensor Library
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
#include "SENSAIRS8.h"

// Constructor - Inititalize Hardware UART
SENSAIRS8::SENSAIRS8(void) {
    Serial.begin(9600);
    Serial.setTimeout(READ_TIMEOUT);
}

uint8_t SENSAIRS8::getSensorData(char *payload, uint8_t startbyte) {
  uint8_t _cmd[7] = {0xFE, 0x44, 0x00, 0x08, 0x02, 0x9F, 0x25};
  while (Serial.available() > 0) Serial.read();
    Serial.write(_cmd, 7);

  delay(1000);
  uint8_t readBytes = read();
  
  payload[startbyte]   = 0x00;
  payload[startbyte+1] = 0x00;
  if (readBytes > 0) {
    uint16ToPayload((buffer[3]*256) + buffer[4], payload, startbyte);
  }
  return startbyte+2;
}

void SENSAIRS8::calibrate(void) {
  pinMode(calpin, OUTPUT);
  digitalWrite(calpin, LOW);
  delay(6000);
  digitalWrite(calpin, HIGH);
  pinMode(calpin, INPUT_PULLUP);
}

void SENSAIRS8::initialize(void) {
  // Disable Auto Background Calibration
  uint8_t _cmd[8] = {0xFE, 0x06, 0x00, 0x01F, 0x00, 0x00, 0xAC, 0x03};
  while (Serial.available() > 0) Serial.read();
    Serial.write(_cmd, 8);
    Serial.flush();
}

// Read a Sensor Response
uint8_t SENSAIRS8::read() {
    // Number of returned Bytes
    uint8_t ret = 0;
    // Clear Internal Buffer
    zeroBuffer();

    // Read Available Bytes
    if (Serial.available() > 0) {
      ret = Serial.readBytes(buffer, SER_BUF_LEN);
    }
    return ret;
}

// Fill the Internal Buffer with Zeroes
void SENSAIRS8::zeroBuffer() {
    for (int i=0; i < SER_BUF_LEN; i++)
        buffer[i] = 0x00;
}