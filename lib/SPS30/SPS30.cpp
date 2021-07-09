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

// Read Measurements from Sensor and put them into the Paylaod
uint8_t SPS30::getSensorData(char *payload, uint8_t startbyte) {
  // Buffer to hold the raw I2C Return
  uint8_t  data[30]; 
  // Array with calculated Values.
  // Predefined with 0xEEEE to recognize Read Errors in the Payload
  uint16_t values[5] = {0xEEEE, 0xEEEE, 0xEEEE, 0xEEEE, 0xEEEE};

  // Wakeup Sensor from Sleep
  DEBUG_PRINTLN("SPS30::getSensorData - Wake Up Sensor");
  Wire.beginTransmission(SPS30_I2C_ADDRESS);
  Wire.endTransmission();
  delay(50);
  write(SPS30_WAKEUP);
  delay(50);

  // Start Measuring, Integer Output
  DEBUG_PRINTLN("SPS30::getSensorData - StartMeasurement");
  write(SPS30_START_MEASUREMENT, 0x0500);
  // Wait for Stable Values
  DEBUG_PRINTLN("SPS30::getSensorData - WaitForStabilize");
  delay(SPS30_STABILIZE_TIME);

  bool    dataok = false;
  uint8_t tries = 5;

  // Retry up to 5 times to get a Valid Measurement from the Sensor
  while (!dataok && tries > 0) {
    DEBUG_PRINT("SPS30::getSensorData - Reading Sensor Data, Try ");
    DEBUG_PRINTLN(6-tries);
    if (hasData()) {
      write(SPS30_READ_MEASUREMENT);
      Wire.requestFrom(SPS30_I2C_ADDRESS, 30);
      if (Wire.available() != 0) { 
        Wire.readBytes(data, 30);

        DEBUG_PRINT("SPS30::getSensorData - I2C-Data: ");
        DEBUG_PRINTARR(data);
        DEBUG_PRINTLN("");

        // Get Values for PM1.0, 2.5, 4 and 10 in Order
        dataok = true;
        for (uint8_t i=0; i<4; i++)
          if (data[(i*3)+2] == calcCRC(data+(i*3), 2)) {
            values[i] = data[i*3] << 8 | data[(i*3)+1]; // Create uint16_t from Bytes
          } else {
            dataok = false; // Set false on Checksum Error
          }
        
        // Get Typical Particle size
        if (data[29] == calcCRC(data+27, 2)) {
          values[4] = data[27] << 8 | data[28]; // Create uint16_t from Bytes
        } else {
          dataok = false; // Set false on Checksum Error
        }
        
        if (!dataok) {
          // If a Checksum error occured
          DEBUG_PRINTLN("SPS30::getSensorData - CheckSum Error");
          tries--;
          delay(2000);
        }
      } else {
        // No I2C Data Available
        DEBUG_PRINTLN("SPS30::getSensorData - Error, no I2C Data available");
        tries--;
        delay(2000);
      }
    } else {
      // No Data Ready from Sensor
      DEBUG_PRINTLN("SPS30::getSensorData - Error, Sensor Data not Ready");
      tries--;
      delay(2000);
    }
  }
  DEBUG_PRINTLN("SPS30::getSensorData - StopMeasurement");
  write(SPS30_STOP_MEASUREMENT);
  delay(50);
  DEBUG_PRINTLN("SPS30::getSensorData - SensorSleep");
  write(SPS30_SLEEP);
  
  // Put the Values into the Payload Array
  for (uint8_t i=0; i<5; i++)
    uint16ToPayload(values[i], payload, startbyte+(i*2));
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

// Write a 16 Bit Command without Parameters
bool SPS30::write(uint16_t cmd) {
  Wire.beginTransmission(SPS30_I2C_ADDRESS);
  Wire.write(cmd >> 8);
  Wire.write(cmd & 0xFF);
  if (Wire.endTransmission() != 0)
    return(false);
  return(true);
}

// Write a 16Bit Command with 16Bit Parameters
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