/*
  attsensor.h - Define the Base Sensor Interface Class
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
#ifndef ATTSENSOR_H
#define ATTSENSOR_H

#include <inttypes.h>

class AttSensor  {
  public:
    // Put the Data into the Payload Array starting at <startbyte>
    // Return the next startbyte when done
    virtual uint8_t getSensorData(char *payload, uint8_t startbyte) = 0;
    
    // Called in Setup, Do any Necessary Initialization
    virtual void initialize(void) = 0;

    // Return the number of Bytes added to the Payload
    virtual uint8_t numBytes(void) = 0;

    // Calibrate a Sensor. Needs to be Implemented in the Child Class
    virtual void calibrate(void) = 0;

    // Helper Functions to Put Values Into the Payload Array
    static void int32ToPayload(int32_t value, char *payload, uint8_t startbyte) {
      payload[startbyte]   = (value) & 0XFF;
      payload[startbyte+1] = (value >> 8) & 0XFF;
      payload[startbyte+2] = (value >> 16) & 0XFF;
      payload[startbyte+3] = (value >> 24) & 0XFF;
    }
    static void int16ToPayload(int16_t value, char *payload, uint8_t startbyte) {
      payload[startbyte]   = (value) & 0XFF;
      payload[startbyte+1] = (value >> 8) & 0XFF;
    }
    static void uint16ToPayload(uint16_t value, char *payload, uint8_t startbyte) {
      payload[startbyte]   = (value) & 0XFF;
      payload[startbyte+1] = (value >> 8) & 0XFF;
    }
};

#endif