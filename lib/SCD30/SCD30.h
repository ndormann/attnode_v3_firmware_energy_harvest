/*
  SCD30.h - Sensirion SCD30 CO2-Sensor Library
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

#ifndef SCD30_H
#define SCD30_H

#include "../../include/attsensor.h"

// Sensor I2C Address
#define SCD30_ADDR 0x61

// Commands and Registers
#define SCD30_CONT_MEASURE   0x0010
#define SCD30_SET_INTERVAL   0x4600
#define SCD30_DATA_READY     0x0202
#define SCD30_GET_MEASURE    0x0300
#define SCD30_AUTOCAL        0x5306
#define SCD30_SET_RECALIB    0x5204
#define SCD30_SET_TEMPOFFSET 0x5403
#define SCD30_SET_ALTCOMP    0x5102
#define SCD30_RESET          0xD304
#define SCD30_STOP_MEAS      0x0104
#define SCD30_GET_FW_VER     0xD100

class SCD30 : public AttSensor {
  private:
    bool     selfcalib = false;
    uint16_t interval = 2;

    void    reset(void);
    void    sendCmd(uint16_t cmd, uint16_t arg);
    void    sendCmd(uint16_t cmd);
    void    getBytes(uint16_t register, uint8_t bytes[], uint8_t len);
    uint8_t calcCrc(uint8_t bytes[], uint8_t len);
    float   byteToFloat(uint8_t mmsb, uint8_t mlsb, uint8_t lmsb, uint8_t llsb);

  public:
    SCD30();
    SCD30(uint8_t interval, bool selfcalib);
    void initialize(void);
    void calibrate(void);
    uint8_t numBytes(void) {return 6;};
    uint8_t getSensorData(char *payload, uint8_t startbyte);
    
};

#endif