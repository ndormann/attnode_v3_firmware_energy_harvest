/*
  SHT21.cpp - SHT21 Sensor Library
  Copyright (c) 2019-2020, Stefan Brand
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
#include <inttypes.h>
#include <Wire.h>
#include "SHT21.h"

SHT21::SHT21() {}

uint16_t SHT21::sensorRead(uint8_t command) {
    
    uint8_t d = 85; // Delay after starting Measurement
    if(command == SHT21_TEMPHOLD || command == SHT21_TEMPNOHOLD) d = 85;
    if(command == SHT21_HUMIHOLD || command == SHT21_HUMINOHOLD) d = 30;
    
    // Trigger Measurement
    Wire.beginTransmission(SHT21_I2CADDR);
    Wire.write(command);
    Wire.endTransmission();

    // Wait for Measurement
    delay(d);

    // Read and Return value
    Wire.requestFrom(SHT21_I2CADDR, 2);
    uint16_t result = (Wire.read() << 8) | Wire.read();
    result &= ~0x0003; // Clear CRC Bits
    return result;
}

uint8_t SHT21::getSensorData(char *payload, uint8_t startbyte) {
  // Temperature
  int32ToPayload((int32_t)((-46.85 + 175.72 / 65536.0 * (float)(sensorRead(SHT21_TEMPHOLD)))*100), payload, startbyte);
  // Humidity
  int32ToPayload((int32_t)((-6.0 + 125.0 / 65536.0 * (float)(sensorRead(SHT21_HUMIHOLD)))*100), payload, startbyte+4);
  
  return startbyte+8;
}