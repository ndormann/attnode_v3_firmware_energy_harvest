#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

#include <lmic.h>
#include <hal/hal.h>

// Keep Track of used EEPROM Addresses
#define ADDR_SLP 0 // Sleep Interval, 2 Bytes

#include "config.h"
#include "debug.h"

// define the blink function and  BLINK_LED Macro depending
// on the definition of LED_PIN
#ifdef LED_PIN
void blink(uint8_t num) {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);
  for (uint8_t i = 0; i < num * 2; i++) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  digitalWrite(LED_PIN, 0);
}
#define BLINK_LED(COUNT) blink(COUNT);
#else
#define BLINK_LED(COUNT)
#endif

// WS2812B RGB LEDs on the CO2 Addon Board
// Defines the  Macro Function WS2812B_SETLED so we don't need #ifdefs everywhere
#ifdef WS2812B_PIN
  #include <tinyNeoPixel_Static.h>
  byte pixels[WS2812B_NUM * 3];
  tinyNeoPixel leds = tinyNeoPixel(WS2812B_NUM, WS2812B_PIN, NEO_GRB, pixels);
  #define WS2812B_SETLED(led,r,g,b) leds.setPixelColor(led,r,g,b); leds.show()
  #define WS2812B_BLINK(led,r,g,b,ms) leds.setPixelColor(led,r,g,b); leds.show(); delay(ms); leds.setPixelColor(led,0,0,0); leds.show()
#else
  #define WS2812B_SETLED(led,r,g,b)
  #define WS2812B_BLINK(led,r,g,b,ms)
#endif

// Create the Sensor Objects
#if defined HAS_NO_SENSOR
  struct lora_data {
    uint8_t bat;
  } __attribute ((packed));
#elif defined HAS_MHZ19C
  #include <MHZ19C.h>
  MHZ19C sensor;
#elif defined HAS_SG112A
  #include <SG112A.h>
  SG112A sensor;
#elif defined HAS_BME280
  #include <BME280.h>
  BME280 sensor;
#elif defined HAS_SHT21
  #include <SHT21.h>
  SHT21 sensor;
#elif defined HAS_SG112A
  #include <SG112A.h>
  SG112A sensor;
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
void do_send(osjob_t* j);

// Pin-Mapping for ATTNode v3
const lmic_pinmap lmic_pins = {
  .nss = PIN_PA5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN,
  .dio = {PIN_PA4, PIN_PA6, LMIC_UNUSED_PIN},
};

// List of unused Pins - will be disabled for Power Saving
#if defined DEBUG || defined HAS_SG112A || defined HAS_MHZ19C
const int disabledPins[] = {PIN_PB5, PIN_PB4, PIN_PB1, PIN_PB0, PIN_PC3, PIN_PC2, PIN_PC1, PIN_PC0};
#else
const int disabledPins[] = {PIN_PB5, PIN_PB4, PIN_PB3, PIN_PB2, PIN_PB1, PIN_PB0, PIN_PC3, PIN_PC2, PIN_PC1, PIN_PC0};
#endif

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
      WS2812B_BLINK(1,0,127,0,1000);
      DEBUG_PRINTLN("OTAA Join Succeeded");
      break;
    case EV_TXCOMPLETE:
      // Check for Downlink
      DEBUG_PRINTLN("LoRa Packet Sent");
      WS2812B_BLINK(1,0,127,0,1000);
      if ((int)LMIC.dataLen == 2) {
        // We got a Packet with the right size - lets assemble it into a uint16_t
        DEBUG_PRINTLN("Received Downlink")
        uint16_t tmpslp = (LMIC.frame[LMIC.dataBeg] << 8) | LMIC.frame[LMIC.dataBeg+1];
        DEBUG_PRINT("Setting Sleep Time to: ");
        DEBUG_PRINTLN(tmpslp);
        sleep_time = tmpslp;
        EEPROM.put(ADDR_SLP, tmpslp);
        WS2812B_BLINK(1,0,0,127,250);
      }

      // Got to sleep for specified Time
      DEBUG_PRINTLN("Going to Sleep");
      for (uint16_t i = 0; i < sleep_time*2; i++)
        sleep_32s();

      // Schedule Next Transmit
      do_send(&sendjob);
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

// Read Sensors and Send Data
void do_send(osjob_t* j) {
  // Prepare LoRa Data Packet
  // The struct is defined in the sensor class (or above for use without a sensor)
  lora_data data; 
  
  if (LMIC.opmode & OP_TXRXPEND) {
    delay(1);
  } else {
    // Add Battery Voltage (0.2V Accuracy stored in 1 byte)
    uint32_t batv = readSupplyVoltage();
    data.bat = (uint8_t)(batv / 20);
    if (batv % 20 > 9)
      data.bat += 1;

    // Get Sensor Readings Into Data Paket
    #ifndef HAS_NO_SENSOR
    sensor.getSensorData(data);
    
    // Queue Packet for Sending
    DEBUG_PRINTLN("LoRa-Packet Queued");
    LMIC_setTxData2(1, (unsigned char *)&data, sizeof(data), 0);

    #if defined WS2812B_PIN && (defined HAS_SG112A || defined HAS_MHZ19C)

    // CO2 PPM Levels and LED Colors
    // < 1000 ppm green
    // < 1800 ppm yellow
    // > 1000 ppm red

    if (data.ppm > 0 && data.ppm <= 1000) {
      WS2812B_SETLED(0,0,127,0);
    } else if (data.ppm > 1000 && data.ppm <= 1800) {
      WS2812B_SETLED(0,127,127,0);
    } else if (data.ppm > 1800) {
      WS2812B_SETLED(0,127,0,0);
    } else {
      WS2812B_SETLED(0,0,0,0);
    }
    #endif // WS2812B
    #endif // #infdef HAS_NO_SENSOR


  }
}

void setup()
{
  #ifdef DEBUG
    Serial.begin(115200);
  #endif
  // Initialize SPI and I2C
  Wire.begin();
  SPI.begin();

   // Disable unused Pins (for power saving)
  for (int i = 0; i < (sizeof(disabledPins) / sizeof(disabledPins[0])) - 1; i++)
    pinMode(disabledPins[i], INPUT_PULLUP);

  #ifdef WS2812B_PIN
    pinMode(WS2812B_PIN, OUTPUT);
    leds.setBrightness(WS2812B_BRIGHT);
  #endif

  // Set RTC
  while (RTC.STATUS > 0) {}
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;
  while (RTC.PITSTATUS > 0) {}

  // Initialize Sensor(s)
  #ifdef HAS_BME280
  sensor.getCalData();
  #endif
  #ifdef HAS_MHZ19C
  sensor.initialize();
  #endif

  // Setup LMIC
  DEBUG_PRINT("Initializing LMIC...")
  os_init();
  LMIC_reset();                                  // Reset LMIC state and cancel all queued transmissions
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100); // Compensate for Clock Skew
  LMIC.dn2Dr = DR_SF9;                           // Downlink Band
  LMIC_setDrTxpow(DR_SF7, 14);                   // Default to SF7
  DEBUG_PRINTLN("Done");

  // Check if Sending Interval is set in EEPROM
  // if we get 65535 (0xFFFF) EEPROM was not written
  uint16_t tmpsleep = 0;
  EEPROM.get(ADDR_SLP, tmpsleep);
  if (tmpsleep < 65535) {
    DEBUG_PRINT("Setting Sleep Time from EEPROM to ");
    DEBUG_PRINTLN(tmpsleep);
    sleep_time = tmpsleep;
  }
    
  DEBUG_PRINTLN("Setup Finished");
  
  // Schedule First Send (Triggers OTAA Join as well)
  WS2812B_SETLED(1,127,127,0);
  do_send(&sendjob);
}

void loop()
{
  // Only Run the LMIC loop here. Actual Sending Code is in do_send()
  os_runloop_once(); 
}