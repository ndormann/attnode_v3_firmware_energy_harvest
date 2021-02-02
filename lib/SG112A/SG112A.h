#ifndef SG112A_H
#define SG112A_H

struct lora_data {
  uint8_t bat;
  int16_t ppm;
} __attribute__ ((packed));

#define READ_TIMEOUT 500
#define SER_BUF_LEN  16

#define CMD_GET_VER 0x10
#define CMD_GET_SER 0x12
#define CMD_GET_PPM 0x14

class SG112A {
  private:
    uint8_t buffer[SER_BUF_LEN];

    void write(byte cmd);
    uint8_t read();
    void zeroBuffer(void);
    uint16_t crc16(uint8_t *cmd, int len);
    uint16_t getPPM(void);

  public:
    SG112A(void);
    void getSensorData(lora_data &loradata);
};

#endif