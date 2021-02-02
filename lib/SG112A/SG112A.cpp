#include <Arduino.h>
#include "SG112A.h"

SG112A::SG112A(void) {
    Serial.begin(9600);
    Serial.setTimeout(READ_TIMEOUT);
    delay(5000);
    write(CMD_GET_PPM);
    read();
}

void SG112A::getSensorData(lora_data &loradata) {
    write(CMD_GET_PPM);
    delay(50);
    uint8_t byteLen = read();

    if (byteLen > 0) {
        switch(buffer[3]) {
            case 0x15:
                loradata.ppm = (buffer[5]*256) + buffer[4];
                break;
        }
    }
}

void SG112A::write(byte cmd) {
    uint8_t _cmd[6] = {0xAA, 0x55, cmd, 0x00, 0x00, 0x00};
    uint16_t crc = crc16(_cmd, 4);
    _cmd[4] = (uint8_t)(crc & 0xFF);
    _cmd[5] = (uint8_t)(crc >> 8);
    while (Serial.available() > 0) Serial.read();
    Serial.write(_cmd, 6);
    Serial.flush();
}

uint8_t SG112A::read() {
    uint8_t ret = 0;
    zeroBuffer();
    if (Serial.available() > 0) {
      ret = Serial.readBytes(buffer, SER_BUF_LEN);
    }

    if (buffer[0] != 0xBB && buffer[1] != 0x66)
        ret = 0;

    // TODO: Do CRC Check Here

    return ret;
}

void SG112A::zeroBuffer() {
    for (int i=0; i < SER_BUF_LEN; i++)
        buffer[i] = 0x00;
}

uint16_t SG112A::crc16(uint8_t *cmd, int len){
  uint16_t ret = 0xffff;
  uint16_t polynomial = 0xa001;
  int shift = 0x0;
  int i = 0;
  for (i = len - 1; i >= 0 ; i-- ){
    
    uint16_t code = ( uint16_t )( cmd [ len -1 - i ]  & 0xff );
    ret = ret^code;
    shift = 0x0;
    while ( shift <= 7 ){           
      if ( ret & 0x1 ) {                     
        ret = ret >> 1;                     
        ret = ret^polynomial ;
      } else {
        ret = ret >> 1;
      }
      shift++;
    }
  }      
  return ret;
}
