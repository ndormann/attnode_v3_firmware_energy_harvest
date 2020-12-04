
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <SPI.h>
#include <Wire.h>

// Use the local config.h for LMIC Configuration
#define ARDUINO_LMIC_PROJECT_CONFIG_H config.h
#include <lmic.h>
#include <hal/hal.h>
#include "config.h"

#ifdef HAS_BME280
#include "BME280.h"
BME280 sensor;
#endif

#ifdef HAS_SHT21
#include "SHT21.h"
SHT21 sensor;
#endif

// Define some LMIC Callbacks and Variables
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}
static osjob_t sendjob;

// Pin-Mapping for ATTNode v3
const lmic_pinmap lmic_pins = {
  .nss = PIN_PA5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN,
  .dio = {PIN_PA4, PIN_PA6, LMIC_UNUSED_PIN},
};

// List of unused Pins - will be disabled for Power Saving
const int disabledPins[] = {PIN_PB5, PIN_PB4, PIN_PB3, PIN_PB2, PIN_PB1, PIN_PB0, PIN_PC3, PIN_PC2, PIN_PC1, PIN_PC0};

// ISR Routine for Sleep
ISR(RTC_PIT_vect)
{
  /* Clear interrupt flag by writing '1' (required) */
  RTC.PITINTFLAGS = RTC_PI_bm;
}

// Sleep Routine
void sleep_32s() {
  cli();
  while (RTC.PITSTATUS > 0) {}
    RTC.PITINTCTRL = RTC_PI_bm;
  // 32 Sekunden
  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm;
  while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) {}
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei();
  sleep_cpu();
  sleep_disable();
  sei();
}

// LMIC Callback Handling
void onEvent(ev_t ev) {
  switch (ev) {
    case EV_JOINED:
      // Disable LinkCheck
      LMIC_setLinkCheckMode(0);
      break;
    case EV_TXCOMPLETE:
      // Schedule Next Transmit
      do_send(&sendjob);
      // Got to sleep for specified Time
      for (int i = 0; i < int(SLEEP_TIME*2); i++)
        sleep_32s();
      break;
  }
}

// Get Battery Voltage
uint16_t readSupplyVoltage() { //returns value in millivolts to avoid floating point
  uint16_t temp = 0;
  analogReference(VDD);
  VREF.CTRLA = VREF_ADC0REFSEL_1V5_gc;
  ADC0.CTRLD = ADC_INITDLY_DLY256_gc;
  ADC0_CTRLB = ADC_SAMPNUM_ACC64_gc;
  uint16_t reading = analogRead(ADC_INTREF);
  temp = reading / 64;
  uint32_t intermediate = 1534500;
  reading = 0;
  reading = intermediate / temp;
  return reading;
}

// Blink x times
#ifdef LED_PIN
void blink(uint8_t num) {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);
  for (uint8_t i = 0; i < num * 2; i++) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(5);
  }
  digitalWrite(LED_PIN, 0);
}
#endif

// Read Sensors and Send Data
// All Sensor Code and Data Preparation goes here
void do_send(osjob_t* j) {
  // Prepare LoRa Data Packet
  #ifdef HAS_NO_SENSOR
  struct lora_data {
    uint8_t bat;
  } __attribute ((packed)) data;
  #elif defined HAS_SHT21
  struct lora_data {
    uint8_t bat;
    int32_t temperature;
    int32_t humidity;
  } __attribute__ ((packed)) data;
  #elif defined HAS_BME280
  struct lora_data {
    uint8_t bat;
    int32_t temperature;
    int32_t humidity;
    int32_t pressure;
  } __attribute__ ((packed)) data;
  #endif
  
  if (LMIC.opmode & OP_TXRXPEND) {
    delay(1);
  } else {
    // Add Battery Voltage (0.2V Accuracy stored in 1 byte)
    uint32_t batv = readSupplyVoltage();
    data.bat = (uint8_t)(batv / 20);
    if (batv % 20 > 9)
      data.bat += 1;

    // Take Measurements depending on Sensor
    #ifdef HAS_SHT21
    data.temperature = (int32_t)(sensor.getTemperature()*100);
    data.humidity    = (int32_t)(sensor.getHumidity()*100);
    #elif defined HAS_BME280
    sensor.getData(&data.temperature, &data.pressure, &data.humidity);
    #endif

    // Queue Paket for Sending
    LMIC_setTxData2(1, (unsigned char *)&data, sizeof(data), 0);
  }
}

void setup()
{
  // Initialize SPI and I2C
  Wire.begin();
  SPI.begin();
  
  // Disable unused Pins (for power saving)
  for (int i = 0; i < (sizeof(disabledPins) / sizeof(disabledPins[0])) - 1; i++)
    pinMode(disabledPins[i], INPUT_PULLUP);

  // Set RTC
  while (RTC.STATUS > 0) {}
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;
  while (RTC.PITSTATUS > 0) {}

  // Initialize Sensor(s)
  #ifdef HAS_BME280
  sensor.getCalData();
  #endif

  // Setup LMIC
  os_init();
  LMIC_reset();                                  // Reset LMIC state and cancel all queued transmissions
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100); // Compensate for Clock Skew
  LMIC.dn2Dr = DR_SF9;                           // Downlink Band
  LMIC_setDrTxpow(DR_SF7, 14);                   // Default to SF7

  // Schedule First Send (Triggers OTAA Join as well)
  do_send(&sendjob);

#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
  blink(1);
#endif
}

void loop()
{
  // Only Run the LMIC loop here. Actual Sending Code is in do_send()
  os_runloop_once(); 
}
