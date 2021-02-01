#ifndef SG112A_H
#define SG112A_H

class SG112A {
  private:
    void sendCmd(uint8_t *cmd, uint8_t len);
    uint16_t crc16(uint8_t *cmd, uint8_t len);
  public:
    SG112A(void);
    uint16_t getPPM(void);
};

#endif
