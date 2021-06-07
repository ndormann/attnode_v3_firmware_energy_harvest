/*
  .cpp - Brightness Sensor Library
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
#include "Brightness.h"

// Constructor - Inititalize Hardware UART
Brightness::Brightness(uint8_t a, uint8_t c) {
    this->anode = a;
    this->cathode = c;
}

uint8_t Brightness::getSensorData(char *payload, uint8_t startbyte) {
  
  uint16_t counter;

  pinMode(this->anode, OUTPUT);
  digitalWrite(this->anode, LOW);

  pinMode(this->cathode, OUTPUT);
  digitalWrite(this->cathode, HIGH);
  delayMicroseconds(4);
 
  pinMode(this->cathode, INPUT);
  digitalWrite(this->cathode, LOW);

  for ( counter = 0; counter < 65000; counter++) {
    if (digitalRead(this->cathode)==0) break;
     delayMicroseconds(5);
  }
  
  uint16ToPayload(65000-counter, payload, startbyte);
  return startbyte+2;
}