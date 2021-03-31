/*
  SCD30.cpp - Sensirion SCD30 CO2-Sensor Library
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
#include <inttypes.h>
#include <Wire.h>
#include "SCD30.h"

// Default Constructor
SCD30::SCD30() {};

// Constructor with Measuring Interval and Autocalibration Status
SCD30::SCD30(uint8_t interval, bool selfcalib) {
    this->interval  = interval;
    this->selfcalib = selfcalib;
}

// Initialize the Sensor, Enable Constant Measurement, Set Autocalibration and Interval
void SCD30::initialize(void) {
  uint8_t fw[3];
  // Soft Reset
  reset();
  // Send Get Firmware, Workaround to Avoid First Real Command to be Ignored
  getBytes(SCD30_GET_FW_VER, fw, 3);
  delay(10);
  // Start Continous Measurement
  sendCmd(SCD30_CONT_MEASURE, 0);
  // Enable/Disable Autocalibration
  if (selfcalib) {
    sendCmd(SCD30_AUTOCAL, 1);
  } else {
    sendCmd(SCD30_AUTOCAL, 0);
  }
  delay(10);
  // Measurement Interval
  sendCmd(SCD30_SET_INTERVAL, interval);
}

// Read Data From Sensor and Put Them Into the Payload Array
uint8_t SCD30::getSensorData(char *payload, uint8_t startbyte) {
  uint8_t ready[2] = {0};
  uint8_t data[18] = {0};
  // Check if Sensor Data is Available
  getBytes(SCD30_DATA_READY, ready, 2);
  if (ready[1] == 1){
    // Get All Measurements (18 Bytes)
    getBytes(SCD30_GET_MEASURE, data, 18);

    // CO2 PPM
    int16_t value = (int16_t)(byteToFloat(data[0], data[1], data[3], data[4]));
    int16ToPayload(value, payload, startbyte);

    // Temperature
    value = (int16_t)(byteToFloat(data[6], data[7], data[9], data[10])*100);
    int16ToPayload(value, payload, startbyte+2);

    // Humidity
    value = (int16_t)(byteToFloat(data[12], data[13], data[15], data[16])*100);
    int16ToPayload(value, payload, startbyte+4);
  }
  return startbyte+6;
}

// Calibrate the Sensor to 400ppm (Outside Level)
void SCD30::calibrate(void) {
  sendCmd(SCD30_SET_RECALIB, 400);
}

// Sensor Soft Reset
void SCD30::reset(void) {
  sendCmd(SCD30_RESET);
  delay(2000); // Bootup Time
}

// Send a Command with Argument
void SCD30::sendCmd(uint16_t cmd, uint16_t arg) {
  uint8_t args[2], crc;
  args[0] = (arg >> 8);
  args[1] = (arg & 0xFF);
  crc = calcCrc(args, 2);

  Wire.beginTransmission(SCD30_ADDR);
  Wire.write(cmd >> 8);
  Wire.write(cmd & 0xFF);
  Wire.write(args[0]);
  Wire.write(args[1]);
  Wire.write(crc);
  Wire.endTransmission();
}

// Send only a Command
void SCD30::sendCmd(uint16_t cmd) {
  Wire.beginTransmission(SCD30_ADDR);
  Wire.write(cmd >> 8);
  Wire.write(cmd & 0xFF);
  Wire.endTransmission();
}

// Read len Number of Bytes from Register reg into Array bytes
void SCD30::getBytes(uint16_t reg, uint8_t bytes[], uint8_t len) {
  sendCmd(reg);
  delay(3);
  Wire.requestFrom((uint8_t)SCD30_ADDR, (uint8_t)len);
  if (Wire.available()) {
    for (uint8_t i = 0; i<len; i++)
      bytes[i] = Wire.read();
  }
}

// Calculate CRC8
uint8_t SCD30::calcCrc(uint8_t bytes[], uint8_t len) {
  uint8_t crc8 = 0xFF;
  for (uint8_t i = 0; i < len; i++)
  {
    crc8 ^= bytes[i];
    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc8 & 0x80)
        crc8 = (uint8_t)((crc8 << 1) ^ 0x31);
      else
        crc8 <<= 1;
    }
  }
  return crc8;
}

// Convert 4 Bytes from the Measurement to a Float Value
float SCD30::byteToFloat(uint8_t mmsb, uint8_t mlsb, uint8_t lmsb, uint8_t llsb) {
  uint32_t iValue = (uint32_t)((((uint32_t)mmsb) << 24) | (((uint32_t)mlsb) << 16) | (((uint32_t)lmsb) << 8)  | ((uint32_t)llsb));
  float    fValue = *(float*)&iValue;
  return fValue;
}