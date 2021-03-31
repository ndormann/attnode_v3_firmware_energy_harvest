#ifndef BME280_H
#define BME280_H

#include "../../include/attsensor.h"

#define BME280_I2CADDR 0x76

class BME280 : public AttSensor {
private:
  // Variables for Calibration Values
  uint8_t  dig_H1, dig_H3;
  int8_t   dig_H6;
  uint16_t dig_T1, dig_P1;
  int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2, dig_H4, dig_H5;

  // Helper Variable
  int32_t t_fine;

  // Functions for Calculation compensated Values
  int32_t compensate_t(int32_t);
  int32_t compensate_p(int32_t);
  int32_t compensate_h(int32_t);

  // Helper Functions for Reading / Writing I2C Data
  uint8_t  read8(uint8_t addr);
  uint16_t read16(uint8_t addr);
  uint16_t read16_LE(uint8_t addr);
  int16_t  readS16(uint8_t addr);
  int16_t  readS16_LE(uint8_t addr);
  void     write8(uint8_t addr, uint8_t data);
  void     getCalData(void);

public:
  BME280(void);
  uint8_t getSensorData(char *payload, uint8_t startbyte);
  void calibrate(void) {};
  void initialize(void) {getCalData();};
  uint8_t numBytes(void) {return 12;};
};
#endif
