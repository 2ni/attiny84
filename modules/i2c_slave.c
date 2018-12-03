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

#include "uart.h"

#define SLAVE_ADDR	0x21 // above 0x40 does not work?!?

#define DDR		DDRA
#define PORT	PORTA
#define LED_A	PA0
#define LED_K	PA1
#define CHANNEL_THERM   3 // PA3
#define CHANNEL_MOIST_L 7 // PA7
#define CHANNEL_MOIST_H 5 // PA5

#define SCL PA4
#define SDA PA6

#define HUM_SETS 3
#define DATA_ADC_NUM (HUM_SETS*2+1)

#define I2C_GET_MOISTRAW 0x16
#define I2C_RESET        0x15
#define I2C_GET_RAW      0x14
#define I2C_GET_MOIST    0x13
#define I2C_SET_BLINK    0x12
#define I2C_GET_BLINK    0x11
#define I2C_GET_TEMP     0x10

#define MOIST_MIN 325
#define MOIST_MAX 615

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

/*
 * 0: temp
 * 1: moist low 0
 * 2: moist high 0
 * 3: moist low 1
 * 4: moist high 1
 * 5: moist low 2
 * 6: moist high 2
 */
volatile static uint16_t data_adc[DATA_ADC_NUM] = {0}; // raw data saved from ADC
volatile static int16_t temperature = 0;
volatile static int16_t moisture = 0;
volatile static int16_t moisture_percent = 0;
volatile static uint8_t data_index = 0;
volatile static uint8_t next_channel = 0;
static uint8_t data_channels[3] = {CHANNEL_MOIST_L, CHANNEL_MOIST_H, CHANNEL_THERM};

uint8_t blink_count = 0;
uint8_t blink_last_count = 0;

// **************** timer for blinking ******************

/*
 * prescaler 64
 * 1 cycle = 64/4M = 16us (resolution)
 *
 * 1ms   = 62.5
 * 250ms = 15625
 *
 * Timer0: 8-bit,  max: 4.08ms
 * Timer1: 16-bit, max: 1.05s
 * -----------------------------
 * prescaler 1024
 * 1 cycle = 1024/4M = 256us (resolution)
 *
 * 1ms   = 3.90625
 * 250ms = 976.5625
 *
 * Timer0: 8-bit,  max: 65.28ms
 * Timer1: 16-bit, max: 16.77s
 *
 */
void timerSetup() {
  // Timer 1
  TCCR1B |= _BV(CS12) | _BV(CS10); // divide by 1024
  OCR1A = 976; // ~250ms
  TCCR1B |= _BV(WGM12); // counter1 in CTC mode (Clear Timer on Compare)

  // Timer 0
  TCCR0B |= _BV(CS02) | _BV(CS00); // divide by 1024
  OCR0A = 195; // ~50ms
  TCCR0A |= _BV(WGM01); // counter0 in CTC mode
}

void blinkTimerStart() {
  TIMSK1 |= _BV(OCIE1A); //enable timer compare interrupt
}

void blinkTimerStop() {
  TIMSK1 &= ~_BV(OCIE1A); //disable timer compare interrupt
}

void measureTimerStart() {
  TIMSK0 |= _BV(OCIE0A); // enable timer compare interrupt
}

void measureTimerStop() {
  TIMSK0 &= ~_BV(OCIE0A); // disable timer compare interrupt
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
 * isr for adc completed
 * loop trough all ADC to be completed
 * if raw dat available, calculate human readable values ready to ship
 *
 * they must be done beforehand, or i2c might be too slow and loose connection
 *
 * uint16_t value = ADC; // ADCH, ADCL
 * uint16_t value = ADCL | (ADCH << 8); // ADCH, ADCL
 *
 * get average of 3 moisture values
 *
 * 0: moist_sum_l
 * 1: moist_sum_h
 * 2: moist_sum_l
 * 3: moist_sum_h
 * 4: moist_sum_l
 * 5: moist_sum_h
 * 7: temp
 */
ISR(ADC_vect) {
  cli();
  data_adc[data_index] = ADC;
  data_index = ((data_index+1) == DATA_ADC_NUM ? 0 : data_index+1); // avoid divisions with %

  // whole measurement cycle done -> calculate temperature and moisture from raw values
  if (data_index == 0) {
    uint16_t sum_l = 0;
    uint16_t sum_h = 0;
    for (uint8_t i=0; i<(DATA_ADC_NUM-1); i++) {
      if (i%2) {
        // odd sum_h
        sum_h += data_adc[i];
      } else {
        // even sum_l
        sum_l += data_adc[i];
      }
    }
    moisture = 1023 - (sum_h - sum_l)/HUM_SETS;
    if (moisture < MOIST_MIN) {
        moisture_percent = 0;
    } else if (moisture > MOIST_MAX) {
        moisture_percent = 1000;
    } else {
        uint16_t range = MOIST_MAX - MOIST_MIN;
        moisture_percent = (moisture - MOIST_MIN);
        moisture_percent *= 1000;
        moisture_percent /= range;
    }
    temperature = getMF52Temp(data_adc[DATA_ADC_NUM-1]);
  }

  // if last measurement -> temperature
  next_channel = data_index % 2;
  if (data_index == (DATA_ADC_NUM-1)) {
    next_channel = 2;
  }

  // wait some time after each h, l moist measurement, ie start counter
  if (data_index%2) {
    // odd - no break
    adcStart(data_channels[next_channel]);
  } else {
    // even - start measurement in some time
    measureTimerStart();
  }
  sei();
}

/*
 * ISR for blinking led
 */
ISR(TIM1_COMPA_vect) {
  ledToggle();
  if (blink_count == 0) {
    ledOff();
    blinkTimerStop();
  }
  blink_count--;
}

/*
 * ISR for starting measurement
 */
ISR(TIM0_COMPA_vect) {
  measureTimerStop();
  adcStart(data_channels[next_channel]);
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

int main(void) {
  wdtOff();
  ledSetup();

  DINIT();
  DL("hello from ATtiny84.");

  // simple blink on startup
  /*
  for (int i=0; i<6; i++) {
    _delay_ms(200);
    ledToggle();
  }
  */

  cli();
  timerSetup();
  blink_count=6;
  blinkTimerStart();

  adcSetup();
  i2cSlaveInit(SLAVE_ADDR);
  sei();

  adcStart(data_channels[0]);
  //TODO wait until all channels updated once

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
        i2cTransmitByte(moisture_percent >> 8); // higher byte
        i2cTransmitByte(moisture_percent & 0x00ff); // lower byte
      } else if (I2C_GET_MOISTRAW == in) {
        i2cTransmitByte(moisture >> 8); // higher byte
        i2cTransmitByte(moisture & 0x00ff); // lower byte
      } else if (I2C_GET_TEMP == in) {
        // transmit temperature from existing raw data
        i2cTransmitByte(temperature >> 8); // higher byte
        i2cTransmitByte(temperature & 0x00ff); // lower byte

      } else if (I2C_SET_BLINK == in) {
        // read number of blinks
        ledOff();
        blinkTimerStop();
        blink_last_count = i2cReceiveByte();
        blink_count = 2*blink_last_count;
        blinkTimerStart();

      } else if (I2C_GET_BLINK == in) {
        // transmit last number of blinks
        i2cTransmitByte(blink_last_count);
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
