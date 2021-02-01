#ifndef SG112A_H
#define SG112A_H

struct lora_data {
  uint8_t bat;
  int16_t ppm;
} __attribute__ ((packed));

class SG112A {
  private:
    void sendCmd(uint8_t *cmd, uint8_t len);
    uint16_t crc16(uint8_t *cmd, uint8_t len);
    uint16_t getPPM(void);
  public:
    SG112A(void);
    void getSensorData(lora_data &loradata);
};

#endif