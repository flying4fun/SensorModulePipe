#include <OneWire.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "nRF24L01.h"
#include "RF24.h"

#define  CE_PIN  4
#define  CSN_PIN 5
#define  DS18B20_PIN 7

const short sleep_cycles_per_transmission = 8;
volatile short sleep_cycles_remaining = sleep_cycles_per_transmission;
const uint64_t pipes[5] = { 0xF0F0F0F0B1LL, 0xF0F0F0F0B2LL, 0xF0F0F0F0B3LL, 0xF0F0F0F0B4LL, 0xF0F0F0F0B5LL };

typedef struct {
  float  temp;
  float  voltage;
  int    setTemp;
  bool   activated;
} S_message;

RF24 radio(CE_PIN, CSN_PIN);
OneWire ds(DS18B20_PIN);
S_message sensorPayload;

void setup_watchdog(void);    // hard code to 1024k cycles (~8.0 sec)
void system_sleep(void);
float readTemperature(void);
long readVcc(void);

void setup() {
  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  setup_watchdog(); //wdt_8s???
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  //CLKPR = (1<<CLKPCE);                 // Divide CPU by 8.  (clock to 1MHz)
  //CLKPR = B00000011;                   // Divide by 8
  Serial.begin(115200);
  Serial.print("F_CPU = ");
  Serial.println(F_CPU);
  
  radio.begin();
  radio.openWritingPipe(pipes[0]);
  sensorPayload.temp = 70.0;
  sensorPayload.voltage = 3.3;
  sensorPayload.setTemp = 0.0;
  sensorPayload.activated = FALSE;
}

void loop() {
  sensorPayload.temp = readTemperature();
  sensorPayload.voltage = readVcc()/1000.0;
  radio.write(&sensorPayload, sizeof(sensorPayload));
  //Serial.print("Module: ");
  //Serial.print(pipes[0]);
  Serial.print(" Temp: ");
  Serial.print(sensorPayload.temp);
  Serial.print(" Voltage: ");
  Serial.println(sensorPayload.voltage);
  
  //delay(50);
  radio.powerDown();
  do_sleep();                       // 8 cycles of ~8.0 sec = ~1 minute
}

void setup_watchdog(void)
{
  cli();                            // disable global interrupts
  //set WD_ChangeEnable and WD_resetEnable to alter the register
  WDTCSR |= (1<<WDCE) | (1<<WDE);  // this is a timed sequence to protect WDTCSR
  // set new watchdog timeout value to 1024K cycles (~8.0 sec)
  WDTCSR = (1<<WDP3) | (1<<WDP0);  // enable watchdog interrupt
  WDTCSR |= (1<<WDIE);    
  sei();                           // re-enable global interrupts
}

ISR(WDT_vect)
{
  --sleep_cycles_remaining;
}

void do_sleep(void)
{
  ADCSRA |= (0<<ADEN);                 // disable ADC
  sleep_enable();
  while(sleep_cycles_remaining > 0) {
    // turn off brown-out enable in software
    MCUCR = bit (BODS) | bit (BODSE);   // turn on brown-out enable select
    MCUCR = bit (BODS);                 // this must be done within 4 clock cycles of above
    sleep_cpu();                        // sleep within 3 clock cycles of above
    //sleep_mode();                      // System sleeps here
  }
  sleep_disable();                     // System continues execution here when watchdog timed out
  ADCSRA |= (1<<ADEN);                 // switch ADC back on
  
  sleep_cycles_remaining = sleep_cycles_per_transmission;
}

float readTemperature(void) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  
  if( !ds.search(addr) ) {
    ds.reset_search();
    delay(250);
  }
  
  if(OneWire::crc8(addr, 7) != addr[7]) {
    return 0.0;
  }
  
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);
  
  delay(750);
  
  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);
  
  for( i = 0; i < 9; i++ ) {
    data[i] = ds.read();
  }
  
  int raw = (data[1] << 8) | data[0];
  if(type_s) {
    raw = raw << 3;
    if( data[7] = 0x10) {
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if( cfg == 0x00) raw = raw << 3;
    else if( cfg == 0x20) raw = raw << 2;
    else if( cfg == 0x40) raw = raw << 1;
  }
  
  raw = raw >> 4;  // divide by 16. degrees in C output
  
  return (float)raw * 1.8 + 32.0;
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
