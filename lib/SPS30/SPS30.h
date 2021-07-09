/*
  SPS30.h - SPS30 Sensor Library
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

#ifndef SPS30_H
#define SPS30_H

#include <Wire.h>
#include "../../include/attsensor.h"
#include "../../include/debug.h"

// I2C Address
#define SPS30_I2C_ADDRESS    0x69
#define SPS30_STABILIZE_TIME 30000

// I2C Commands and Registers
#define SPS30_START_MEASUREMENT   0x0010
#define SPS30_STOP_MEASUREMENT    0x0104
#define SPS30_GET_DATA_READY      0x0202
#define SPS30_READ_MEASUREMENT    0x0300
#define SPS30_AUTO_CLEAN_INTERVAL 0x8004
#define SPS30_START_FAN_CLEANING  0x5607
#define SPS30_READ_ARTICLE_CODE   0xD025
#define SPS30_READ_SERIAL_NUMBER  0xD033
#define SPS30_DEVICE_RESET        0xD304
#define SPS30_SLEEP               0x1001
#define SPS30_WAKEUP              0x1103

class SPS30 : public AttSensor {
  private:
    bool hasData(void);
    uint8_t calcCRC(uint8_t data[], uint8_t len);
    uint16_t readReg(uint16_t regAddr);
    bool write(uint16_t cmd);
    bool write(uint16_t cmd, uint16_t arg);
  
  public:
    SPS30() {};
    uint8_t getSensorData(char *payload, uint8_t startbyte) override;
    void calibrate(void) override {};
    void initialize(void) override {} ;
    uint8_t numBytes(void) override { return 10; };
};

#endif