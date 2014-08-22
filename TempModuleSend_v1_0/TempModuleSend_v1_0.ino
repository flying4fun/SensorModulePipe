#include <OneWire.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "nRF24L01.h"
#include "RF24.h"

#define  CE_PIN  4
#define  CSN_PIN 5
#define  DS18B20_PIN 7

void setup_watchdog(void);    // hard code to 1024k cycles (~8.0 sec)
void system_sleep(void);
int32_t readTemperature(void);
uint16_t readVcc(void);
uint64_t getId(void);

const short sleep_cycles_per_transmission = 8;  // 8 cycles at ~8 sec each = ~ 1 minute
volatile short sleep_cycles_remaining = sleep_cycles_per_transmission;
const uint64_t pipe = 0xF0F0F0F0F0LL; // "production" sensor net
//const uint64_t pipe = 0x1010101010LL; // "testing" sensor net

typedef struct {
  uint64_t  id;
  int32_t  temperature;
  uint32_t  humidity;
  uint16_t  voltage;
  uint16_t  reserved[1];
  uint32_t  counter;
} TS_message;

RF24 radio(CE_PIN, CSN_PIN);
OneWire ds(DS18B20_PIN);

TS_message sensorPayload;

void setup() {
  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT);
  setup_watchdog(); //wdt_8s???
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  //CLKPR = (1<<CLKPCE);                 // Divide CPU by 8.  (clock to 1MHz)
  //CLKPR = B00000011;                   // Divide by 8
  
  //Serial.begin(115200);
  //Serial.print("F_CPU = ");
  //Serial.println(F_CPU);
  
  radio.begin();
  radio.setRetries(15,15);
  radio.openWritingPipe(pipe);
  radio.enableDynamicPayloads();
  radio.setCRCLength(RF24_CRC_16);
  radio.setChannel(23);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  
  sensorPayload.temperature = 70000;
  sensorPayload.humidity = 0;
  sensorPayload.voltage = 3210;
  sensorPayload.counter = 0;
  sensorPayload.id = getId();
}

void loop() {
  radio.powerUp();
  delay(20);
  sensorPayload.temperature = readTemperature();
  sensorPayload.voltage = readVcc();
  sensorPayload.counter++;
  while(!radio.write(&sensorPayload, sizeof(sensorPayload)))
    ;
  
  //uint64_t llow = sensorPayload.id;
  //uint64_t hhow = llow / 1000000000ULL;
  //Serial.print("Module ID: ");
  //Serial.print((long)hhow);
  //Serial.print((long)(llow-hhow*1000000000));
  //Serial.print(" Temp: ");
  //Serial.print(sensorPayload.temperature);
  //Serial.print(" Humidity: ");
  //Serial.print(sensorPayload.humidity);
  //Serial.print(" Voltage: ");
  //Serial.print(sensorPayload.voltage);
  //Serial.print(" Counter: ");
  //Serial.println(sensorPayload.counter);
  delay(20);
  
  radio.powerDown();
  do_sleep();                       // 8 cycles of ~8.0 sec = ~1 minute
}

uint64_t getId(void) {
  union address {
    byte baddr[8];
    uint64_t raddr;
  } addr;
  
  ds.search(addr.baddr);
  ds.reset_search();
  return addr.raddr;
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


int32_t readTemperature(void)
{
        int HighByte, LowByte, TReading, SignBit;
        int32_t degC;
        byte i;
        byte present = 0;
        byte data[12];
        byte addr[8];
        char buf[20];
 
        if ( !ds.search(addr)) {
            ds.reset_search();
            delay(250);
        }
 
 
        if ( OneWire::crc8( addr, 7) != addr[7]) {
            return 0.0;
        }
 
        ds.reset();
        ds.select(addr);
        ds.write(0x44,1);         // start conversion, with parasite power on at the end
 
        delay(750);     // maybe 750ms is enough, maybe not should be according to the datasheet
 
        present = ds.reset();
        ds.select(addr);    
        ds.write(0xBE);         // Read Scratchpad
 
        for ( i = 0; i < 9; i++) {           // we need 9 bytes
          data[i] = ds.read();
        }
 
        LowByte = data[0];
        HighByte = data[1];
        TReading = (HighByte << 8) + LowByte;
        SignBit = TReading & 0x8000;  // test most sig bit
        if (SignBit) // negative
        {
          TReading = (TReading ^ 0xffff) + 1; // 2's comp
        }
        float convC = TReading * 0.0625; // divide by 16 which is the same as multiply by 0.0625 (1/16)
        convC = convC + ((convC * 4) / 5) + 32; // convert C to F
        degC = convC * 1000; //changed to fixed point
        
        return degC;
}

uint16_t readVcc() {
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

int otod( int octal) {
  int decimal = 0, i=0;
  while(octal != 0) {
    decimal += (octal % 10) * power(8, i++);
    octal = (int)octal/10;
  }
  return decimal;
}

unsigned long power(unsigned long x, unsigned int y) {
  unsigned long res = 1;
  while(y>0) {
    if( y & 1)
      res *= x;
    y >>= 1;
    x *= x;
  }
  return res;
}
