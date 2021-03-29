#include <Arduino.h>
#include <Wire.h>
#include "BME280.h"

BME280::BME280() {}

void BME280::getCalData() {
  dig_T1 = read16_LE(0x88);
  dig_T2 = readS16_LE(0x8A);
  dig_T3 = readS16_LE(0x8C);

  dig_P1 = read16_LE(0x8E);
  dig_P2 = readS16_LE(0x90);
  dig_P3 = readS16_LE(0x92);
  dig_P4 = readS16_LE(0x94);
  dig_P5 = readS16_LE(0x96);
  dig_P6 = readS16_LE(0x98);
  dig_P7 = readS16_LE(0x9A);
  dig_P8 = readS16_LE(0x9C);
  dig_P9 = readS16_LE(0x9E);

  dig_H1 = read8(0xA1);
  dig_H2 = readS16_LE(0xE1);
  dig_H3 = read8(0xE3);
  dig_H4 = (read8(0xE4) << 4) | (read8(0xE5) & 0xF);
  dig_H5 = (read8(0xE6) << 4) | (read8(0xE5) >> 4);
  dig_H6 = (int8_t)read8(0xE7);
}

int32_t BME280::compensate_t(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1  = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
  var2  = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T  = (t_fine * 5 + 128) >> 8;
  return T;
}

int32_t BME280::compensate_p(int32_t adc_P)
{
  int32_t var1, var2;
  uint32_t p;

	var1 = (((int32_t)t_fine)>>1) -(int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
  var2 = var2 + ((var1*((int32_t)dig_P5))<<1);var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
  var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
	var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
	if (var1 == 0) {
		return 0; 
	}

	p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
	if (p < 0x80000000) {
		p = (p << 1) / ((uint32_t)var1);
	} else {
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
	return p;
}

int32_t BME280::compensate_h(int32_t adc_H)
{
  int32_t v_x1_u32r;
  v_x1_u32r=(t_fine-((int32_t)76800));
  v_x1_u32r=(((((adc_H<<14)-(((int32_t)dig_H4)<<20)-(((int32_t)dig_H5)*v_x1_u32r))+
    ((int32_t)16384))>>15)*(((((((v_x1_u32r*((int32_t)dig_H6))>>10)*
    (((v_x1_u32r*((int32_t)dig_H3))>>11)+((int32_t)32768)))>>10)+
    ((int32_t)2097152))*((int32_t)dig_H2)+8192)>>14));
  v_x1_u32r=(v_x1_u32r-(((((v_x1_u32r>>15)*(v_x1_u32r>>15))>>7)*((int32_t)dig_H1))>>4));
  v_x1_u32r=(v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r=(v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  return (uint32_t)((v_x1_u32r>>12)/10);
}

uint8_t BME280::getSensorData(char *payload, uint8_t startbyte) {

	int32_t UP, UT, UH;
  int32_t rawP, rawT;

  // Trigger Measurement
 	// Set Sensor Config
	write8(0xF2, 0b00000001); // 1x Oversampling for Humidity
  write8(0xF4, 0b00100101); // 1x Oversampling for Temperature, Pressure, Forced Mode

  delay(10);

	// Read Pressure
  rawP   = read16(0xF7);
  rawP <<= 8;
  rawP  |= read8(0xF9);
  UP     = rawP >> 4;

	// Read Temperature
  rawT   = read16(0xFA);
  rawT <<= 8;
  rawT  |= read8(0xFC);
  UT     = rawT >> 4;

	// Read Humidity
  UH = read16(0xFD);

  // Temperature
	int32ToPayload(compensate_t(UT), payload, startbyte);
  // Humidity
  int32ToPayload(compensate_h(UH), payload, startbyte+4);
  // Pressure
  int32ToPayload(compensate_p(UP), payload, startbyte+8);
  
  return startbyte+12;
}

uint8_t BME280::read8(uint8_t addr) {
  Wire.beginTransmission(BME280_I2CADDR);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(BME280_I2CADDR, 1);
  uint8_t ret = Wire.read();
  return ret;
}

uint16_t BME280::read16(uint8_t addr) {
  Wire.beginTransmission(BME280_I2CADDR);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(BME280_I2CADDR, 2);
  uint16_t ret = (Wire.read() << 8) | Wire.read();
  return ret;
}

uint16_t BME280::read16_LE(uint8_t addr) {
  uint16_t temp = read16(addr);
  return (temp >> 8) | (temp << 8);
}

int16_t BME280::readS16(uint8_t addr) {
  return (int16_t)read16(addr);
}

int16_t BME280::readS16_LE(uint8_t addr) {
  return (int16_t)read16_LE(addr);
}

void BME280::write8(uint8_t addr, uint8_t data) {
 Wire.beginTransmission(BME280_I2CADDR);
 Wire.write(addr);
 Wire.write(data);
 Wire.endTransmission();
}