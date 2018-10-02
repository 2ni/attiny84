/*
 *
 * Based on https://github.com/CmdrZin/chips_I2C_Slave_tutorial/tree/master/Slave_A2B2/Slave_A2B2_CodeDev
 *
 * The Commands from the Master are either:
 * SDA_W 00		- turn the LED OFF
 * SDA_W 01		- turn the LED ON
 * where SDA_W is the I2C address in Write Mode. [ i.e. (SLAVE_ADRS<<1)|0 ]
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "i2c_slave.h"
#include "mf52.h"

#define SLAVE_ADDR	0x21 // 0x40 does not work?!?

#define DDR		DDRA
#define PORT	PORTA
#define LED_A	PA0
#define LED_K	PA1
#define CHANNEL_THERM   3 // PA3
#define CHANNEL_MOIST_L 7 // PA7
#define CHANNEL_MOIST_H 5 // PA5

#define SCL PA4
#define SDA PA6

#define DATA_ADC_NUM 3
#define I2C_RESET     0x15
#define I2C_GET_RAW   0x14
#define I2C_GET_MOIST 0x13
#define I2C_SET_BLINK 0x12
#define I2C_GET_BLINK 0x11
#define I2C_GET_TEMP  0x10

inline static void ledSetup(){
  DDR |= _BV(LED_A) | _BV(LED_K); // enable as output (set 1)
  PORT &= ~_BV(LED_A); // set low
  PORT &= ~_BV(LED_K);
}

inline static void ledOn() {
  PORT |= _BV(LED_A); // set high
}

inline static void ledOff() {
  PORT &= ~_BV(LED_A); // set low
}

inline static void ledToggle() {
  PORT ^= _BV(LED_A); // invert (exclusive or)
}

// **************** ADC ******************

inline static void adcSetup() {
  // ADEN enable adc (if cleared ADC does not consume power)
  // ADIE enable adc interrupt
  // ADPS[2..0] prescaler 64 (should be 50-200kHz)
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1);
  ADCSRA |= _BV(ADIE);
}

/*
 * 10bit
 * value 512 = 25°C (100kΩ - 100kΩ)
 * the higher the value, the colder it is
 */
void adcStart(uint8_t channel) {
  // TODO clear PRADC to power ADC?
  // VCC as reference -> REFS[1..0] = 0
  // channel defined by MUX[5..0]
  ADMUX = channel;

  // start conversion
  ADCSRA |= _BV(ADSC);
}

/*
 * 0: temp
 * 1: moist low
 * 2: moist high
 */
volatile static uint16_t data_adc[DATA_ADC_NUM]; // raw data saved from ADC
volatile static int16_t temperature;
volatile static int16_t moisture;
volatile static uint8_t data_index = 0;
static uint8_t data_channels[3] = {CHANNEL_THERM, CHANNEL_MOIST_L, CHANNEL_MOIST_H};

/*
 * isr for adc completed
 * loop trough all ADC to be completed
 * if raw dat available, calculate human readable values ready to ship
 *
 * they must be done beforehand, or i2c might be too slow and loose connection
 *
 * uint16_t value = ADC; // ADCH, ADCL
 * uint16_t value = ADCL | (ADCH << 8); // ADCH, ADCL
 */
ISR(ADC_vect) {
  data_adc[data_index] = ADC;
  if (data_index == 0) {
    temperature = getMF52Temp(data_adc[data_index]);
  } else if (data_index == 2) {
    moisture = 1023 - (data_adc[2] - data_adc[1]);
  }

  data_index = (data_index+1) % DATA_ADC_NUM; // alternative w/o division: x = (x + 1 == n ? 0: x + 1);
  adcStart(data_channels[data_index]);
  //data_adc[0] = ADC;
  //adcStart(CHANNEL_THERM);
}

// **************** watchdog ******************

/*
 * Turn off watchdog
 * based on ATtiny84 datasheet p.44
 */
void wdtOff() {
  wdt_reset();
  MCUSR = 0x00; // clear WDRF in MCUSR
  WDTCSR |= _BV(WDCE) | _BV(WDE); // write logical one to WDCE, WDE
  WDTCSR = 0x00; // turn off wdt
}

void wdtOn() {
  WDTCSR = _BV(WDE); // timeout at 16ms
}

#define reset() wdtOn(); while(1) {}


// **************** timer for blinking ******************
void timerSetup() {
  TCCR1B |= 1<<CS11 | 1<<CS10; // divide by 64
  OCR1A = 15625; // count 1sec = (1/4000000*64)*62500, 1/4sec -> 15625
  TCCR1B |= 1<<WGM12; // put timer/counter1 in CTC mode (Clear Timer on Compare)
  TIMSK1 |= 1<<OCIE1A; //enable timer compare interrupt
}

void timerStart() {
  TIMSK1 |= 1<<OCIE1A; //enable timer compare interrupt
}

void timerStop() {
  TIMSK1 &= 0<<OCIE1A; //disable timer compare interrupt
}

uint8_t count = 0;
uint8_t lastCount = 0;
ISR(TIM1_COMPA_vect) {
  ledToggle();
  if (count == 0) {
      ledOff();
      timerStop();
  }
  count--;
}

int main(void) {
  wdtOff();
  ledSetup();

  // simple blink on startup
  for (int i=0; i<6; i++) {
    _delay_ms(200);
    ledToggle();
  }

  cli();
  timerSetup();
  adcSetup();
  i2cSlaveInit(SLAVE_ADDR);
  sei();

  adcStart(data_channels[data_index]);
  //TODO wait until all channels updated once
  //_delay_ms(100);

  // temperature and capacitance (moisture) is constantly updated by ISR
  while (1) {
    if (i2cDataInReceiveBuffer()) {
      uint8_t in = i2cReceiveByte();

      if (I2C_RESET == in) {
        reset();
      } else if (I2C_GET_RAW == in) {
        for (uint8_t i=0; i<DATA_ADC_NUM; i++) {
          i2cTransmitByte(data_adc[i] >> 8); // higher byte
          i2cTransmitByte(data_adc[i] & 0x00ff); // lower byte
        }
      } else if (I2C_GET_MOIST == in) {
        // transmit moisture from raw data
        i2cTransmitByte(moisture >> 8); // higher byte
        i2cTransmitByte(moisture & 0x00ff); // lower byte

      } else if (I2C_GET_TEMP == in) {
        // transmit temperature from existing raw data
        i2cTransmitByte(temperature >> 8); // higher byte
        i2cTransmitByte(temperature & 0x00ff); // lower byte

      } else if (I2C_SET_BLINK == in) {
        // read number of blinks
        ledOff();
        timerStop();
        lastCount = i2cReceiveByte();
        count = 2*lastCount;
        timerStart();

      } else if (I2C_GET_BLINK == in) {
        // transmit last number of blinks
        i2cTransmitByte(lastCount);
      }
    }
  }

  /*
  // turn LED on/off (any number turns it on, 0 turns it off)
  uint8_t data;
  while (1) {
    if (i2cDataInReceiveBuffer()) {
      data = i2cReceiveByte();
      if (data == 0) {
        ledOff();
      } else {
        ledOn();
      }
    }
  }
  */
}
