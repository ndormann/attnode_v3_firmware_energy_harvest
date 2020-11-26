
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h> 
#include "LoRaWAN.h"

#define F_CPU 1000000UL

#include "config.h"


// For Storing  Frame Counter

// Objects and Variables for LoRa
#define DIO0 PIN_PA4
#define NSS  PIN_PA5
RFM95 rfm(DIO0, NSS);
LoRaWAN lora = LoRaWAN(rfm);
uint16_t Frame_Counter_Tx = 0x0000;

// Global Variables for DeepSleep
volatile uint16_t counter = SLEEP_TIME;

// List of unused Pins - will be disabled for Power Saving
const int disabledPins[] = {PIN_PB5, PIN_PB4, PIN_PB3, PIN_PB2, PIN_PB1, PIN_PB0, PIN_PC3, PIN_PC2, PIN_PC1, PIN_PC0};

//ISR Routine for Sleep
ISR(RTC_PIT_vect)
{
  /* Clear interrupt flag by writing '1' (required) */
  RTC.PITINTFLAGS = RTC_PI_bm;
  if (counter >= SLEEP_TIME) {
    counter = 0;
  }
  else {
    counter++;
  }
}

// Sleep Routine
void sleep_32s() {
  Wire.end();
  SPI.end();
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

// Get CPU Temp 
uint16_t readTemp() {
  //based on the datasheet, in section 30.3.2.5 Temperature Measurement
  int8_t sigrow_offset = SIGROW.TEMPSENSE1; // Read signed value from signature row
  uint8_t sigrow_gain = SIGROW.TEMPSENSE0; // Read unsigned value from signature row
  analogReference(INTERNAL1V1);
  ADC0.SAMPCTRL = 0x1F; //Appears very necessary!
  ADC0.CTRLD |= ADC_INITDLY_DLY32_gc; //Doesn't seem so necessary?
  uint16_t adc_reading = analogRead(ADC_TEMPERATURE); // ADC conversion result with 1.1 V internal reference
  analogReference(VDD);
  ADC0.SAMPCTRL = 0x0;
  ADC0.CTRLD &= ~(ADC_INITDLY_gm);
  uint32_t temp = adc_reading - sigrow_offset;
  temp *= sigrow_gain;
  temp += 0x80; // Add 1/2 to get correct rounding on division below 
  temp >>= 8; // Divide result to get Kelvin
  uint16_t temperature_in_K = temp;
  uint16_t temp_in_C = temperature_in_K - 273;
  return temp_in_C /10; // Return Celsius temperature
}

// Crude  Wear Leveling Algorithm to Spread the EEPROM Cell Wear Over
// the first 64 Byte. Using this Method the Theoretical EEPROM Livetime
// should be around 60 Years at a 10 Minute Sending Interval
// (100000 Erase Cycles per Cell * 32 Locations / 144 Measurements a day * 365)
//
// Returns the Next EEPROM Address for Saving the Frame Counter
uint8_t calcEepromAddr(uint16_t framecounter) {
  uint8_t eeprom_addr = ((framecounter % 32) * sizeof(framecounter));
  if (eeprom_addr == 0) {
    eeprom_addr = 62;
  } else {
    eeprom_addr = eeprom_addr - sizeof(framecounter);
  }
  return eeprom_addr;
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

void setup()
{
  // Disable unused Pins (power saving)
  for (int i=0; i<(sizeof(disabledPins)/sizeof(disabledPins[0]))-1; i++)
    pinMode(disabledPins[i], INPUT_PULLUP);

  Wire.begin();
  delay(250);

  // Set RTC
  while (RTC.STATUS > 0) {}
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;
  while (RTC.PITSTATUS > 0) {}

  // Setup LoraWAN
  rfm.init();
  lora.setKeys(NwkSkey, AppSkey, DevAddr);

  // Get Framecounter from EEPROM
  // Check if EEPROM is initialized
  if (EEPROM.read(120) != 0x42) {
    // Set first 64 byte to 0x00 for the wear leveling hack to work
    for (int i = 0; i < 64; i++)
      EEPROM.write(i, 0x00);
    // Write the magic value so we know it's initialized
    EEPROM.write(120, 0x42);
  } else {
    // Get the Last Saved (=Highest) Frame Counter
    uint16_t Frame_Counter_Sv = 0x00000000;
    uint8_t eeprom_addr = 0x0000;
    EEPROM.get(eeprom_addr, Frame_Counter_Sv);
    while (eeprom_addr < 32 * sizeof(Frame_Counter_Tx)) {
      if (Frame_Counter_Sv > Frame_Counter_Tx) {
        Frame_Counter_Tx = Frame_Counter_Sv;
      } else {
        break;
      }
      eeprom_addr += sizeof(Frame_Counter_Tx);
      EEPROM.get(eeprom_addr, Frame_Counter_Sv);
    }
  }

#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
  blink(1);
#endif

}

void loop()
{
  if (counter >= SLEEP_TIME ) {
    Wire.begin();
    SPI.begin();

    // Create Datastructure for Sending
    struct lora_data {
      uint8_t bat;
      int32_t temp;
    } __attribute__ ((packed)) data;
    
    //Add Battery Voltage
    uint32_t batv = readSupplyVoltage();
    data.bat = (uint8_t)(batv/20);
    if (batv % 20 > 9)
      data.bat += 1;
    
    // Read CPU Temperature
    data.temp = readTemp();

    // Send LoRa Packet, Increment Frame Counter
    lora.Send_Data((unsigned char *)&data, sizeof(data), Frame_Counter_Tx, SF7BW125, 0x01);

    // Save the next FrameCounter to EEPROM
    Frame_Counter_Tx++;
    EEPROM.put(calcEepromAddr(Frame_Counter_Tx), Frame_Counter_Tx);

  }
  // Sleep until next Measurement
  sleep_32s();
}
