/*
  .cpp - SPS30 Sensor Library
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
#include "SPS30.h"

SPS30::SPS30() {
}


uint8_t SPS30::getSensorData(char *payload, uint8_t startbyte) {
  uint8_t data[30];
  for (uint8_t i = 0; i < 30; i++)
    data[i] = 0xFF;
  
  uint16_t massPM1, massPM25, massPM4, massPM10, typPM;

  // Start Measuring, Integer Output
  write(SPS30_START_MEASUREMENT, 0x0500);
  // Wait for Stable Values (max 30s according to datasheet)
  delay(30000);

  if (hasData()) {
    write(SPS30_READ_MEASUREMENT);
    Wire.requestFrom(SPS30_I2C_ADDRESS, 30);
    if (Wire.available() != 0) {
      for (uint8_t i = 0; i < 30; i++)
        data[i] = 0xFE;
      
      Wire.readBytes(data, 30);

      // DEBUG OUTPUT
      DEBUG_PRINT("SPS30 I2C DATA: ")
      for (uint8_t i = 0; i < 30; i++) {
        DEBUG_PRINT("0x");
        DEBUG_PRINT (data[i]);
        DEBUG_PRINT(", ");
      }
        
      DEBUG_PRINTLN("");

      // PM1.0
      if (data[2] == calcCRC(data, 2))
        massPM1 = data[0] << 8 | data[1];

      // PM2.5
      if (data[5] == calcCRC(data+3, 2))
        massPM25 = data[3] << 8 | data[4];
      
      // PM4
      if (data[8] == calcCRC(data+6, 2))
        massPM4 = data[6] << 8 | data[7];
      
      // PM10
      if (data[11] == calcCRC(data+9, 2))
        massPM10 = data[9] << 8 | data[10];
      
      // Typical Size
      if (data[29] == calcCRC(data+27, 2))
        typPM = data[27] << 8 | data[28];
    }
  }
  write(SPS30_STOP_MEASUREMENT);

  uint16ToPayload(massPM1, payload, startbyte);
  uint16ToPayload(massPM25, payload, startbyte+2);
  uint16ToPayload(massPM4, payload, startbyte+4);
  uint16ToPayload(massPM10, payload, startbyte+6);
  uint16ToPayload(typPM, payload, startbyte+8);
}

// Check if Sensor has Data
bool SPS30::hasData(void) {
  uint16_t resp = readReg(SPS30_GET_DATA_READY);
  return (resp == 1);
}


// Read a 16Bit Register
uint16_t SPS30::readReg(uint16_t regAddr) {
  Wire.beginTransmission(SPS30_I2C_ADDRESS);
  Wire.write(regAddr >> 8);
  Wire.write(regAddr & 0xFF);
  Wire.endTransmission();
  Wire.requestFrom(SPS30_I2C_ADDRESS, 2);
  if (Wire.available() != 0) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    return ((uint16_t)msb << 8 | lsb);
  }
  return(0);
}

bool SPS30::write(uint16_t cmd) {
  Wire.beginTransmission(SPS30_I2C_ADDRESS);
  Wire.write(cmd >> 8);
  Wire.write(cmd & 0xFF);
  if (Wire.endTransmission() != 0)
    return(false);
  return(true);
}

bool SPS30::write(uint16_t cmd, uint16_t arg){
  uint8_t crcdata[2];
  crcdata[0] = arg >> 8;
  crcdata[1] = arg & 0xFF;
  uint8_t csum = calcCRC(crcdata, 2);

  Wire.beginTransmission(SPS30_I2C_ADDRESS);
  Wire.write(cmd >> 8);
  Wire.write(cmd & 0xFF);
  Wire.write(arg >> 8);
  Wire.write(arg & 0xFF);
  Wire.write(csum);
  if (Wire.endTransmission() != 0)
    return(false);
  return(true);
}

// Calculate the Checksum
uint8_t SPS30::calcCRC(uint8_t data[], uint8_t len) {
  uint8_t csum = 0xFF;
  for (uint8_t x = 0; x < len; x++)
  {
    csum ^= data[x];
    for (uint8_t i = 0; i < 8; i++)
    {
      if ((csum & 0x80) != 0)
        csum = (uint8_t)((csum << 1) ^ 0x31);
      else
        csum <<= 1;
    }
  }
  return csum;
}