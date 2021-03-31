/*
  MHZ19C.h - MHZ19C Sensor Library
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

#ifndef MHZ19C_H
#define MHZ19C_H

#include "../../include/attsensor.h"

#define MHZ19C_READ_TIMEOUT 500  // Timeout for Serial Communication
#define MHZ19C_SER_BUF_LEN  9    // Length of the Internal Serial Message Buffer

#define MHZ19C_CMD_SET_AUTOCAL 0x79  // Turn Self Calibration on/off
#define MHZ19C_CMD_GET_PPM     0x86  // Get Current PPM Reading

class MHZ19C : public AttSensor {
  private:
    uint8_t buffer[MHZ19C_SER_BUF_LEN];
    pin_size_t calpin = PIN_PB4; // PB4 is the Calibration Pin on the Addon PCB

    void write(byte cmd, byte arg);
    uint8_t read();
    void zeroBuffer(void);
    uint8_t crc8(uint8_t *paket);
    uint16_t getPPM(void);

  public:
    MHZ19C(void);
    MHZ19C(pin_size_t calpin);
    void initialize(void);
    void calibrate(void);
    uint8_t numBytes(void) {return 2;};
    uint8_t getSensorData(char *payload, uint8_t startbyte);
    void setSelfCalibration(bool state);
};

#endif