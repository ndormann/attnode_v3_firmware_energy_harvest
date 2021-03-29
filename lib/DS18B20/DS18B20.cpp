/*
  DS18B20.cpp - Dallas 1-Wire Temperature Sensor Library
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

#include "DS18B20.h"

DS18B20::DS18B20(uint8_t owpin, uint8_t resbits = 12, bool para = false) {
  onewire = new OneWire(owpin);
  parasite = para;

  switch(resbits) {
    case 9 :
      resolution = DS18B20_9_BIT;
      convtime = 300;
      break;
    case 10 :
      resolution = DS18B20_10_BIT;
      convtime = 600;
      break;
    case 11 :
      resolution = DS18B20_11_BIT;
      convtime = 900;
      break;
    default :
      resolution = DS18B20_12_BIT;
      convtime = 1200;
  }
}

void DS18B20::initialize(void){
  onewire->reset();
  onewire->reset_search();
  sensorcount = 0;
  
  while (onewire->search(addr)) {
    if (OneWire::crc8( addr, 7) == addr[7] && addr[0] != 0x00) {
      sensorcount++;
      setResolution();
    }
  }
  delay(500);
}

uint8_t DS18B20::numBytes(void){
  return sensorcount * 2;
}

void DS18B20::setResolution(void) {
  // Write Scratchpad
  onewire->reset();
  onewire->select(addr);
  onewire->write(0x4E);
  // Dummy Alarm Values (not used)
  onewire->write(0);
  onewire->write(100);
  // Write Resolution
  onewire->write(resolution);
  onewire->reset();
}

uint8_t DS18B20::getSensorData(char *payload, uint8_t startbyte){
  uint8_t data[9];
  int16_t value;

  // Start Conversation on all Sensors
  onewire->reset();
  onewire->skip();
  onewire->write(0x44,parasite);

  // Wait for Conversion to Finish
  delay(convtime);

  // Read Temperature from all Sensors
  onewire->reset_search();
  while (onewire->search(addr)) {
    if (OneWire::crc8(addr, 7) == addr[7] && addr[0] != 0x00) {
      // Get Data
      onewire->reset();
      onewire->select(addr);
      onewire->write(0xBE);
      for (uint8_t i = 0; i < 9; i++)
        data[i] = onewire->read();

      onewire->reset();
      
      if (OneWire::crc8(data, 8) == data[8]) {
        int16_t rawTemp = (((int16_t)data[1]) << 8) | data[0];
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        switch (cfg) {
          case 0x00 :
            rawTemp = rawTemp & ~7;
            break;
          case 0x20 :
            rawTemp = rawTemp & ~3;
            break;
          case 0x40 :
            rawTemp = rawTemp & ~1;
            break;
        }
        value = ((float)rawTemp/16.0)*100;
      } else {
        value = -85;
      }
      // Add to Payload
      int16ToPayload(value, payload, startbyte);

      // Set next Startbyte
      startbyte += 2;
      delay(50);
    }
  }
  onewire->depower();
  return startbyte;
}