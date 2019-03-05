/*
 * simple capacitive touch pin / moisture sensor
 * using 500k instead of 1M
 * GND around sensor and on bottom layer
 * without GND sensor does almost not detect water
 *
 *
 * PA1 -----|500k|-----|+
 *       |
 *       |
 *     ----
 * GND |  | GND
 *     |  |
 * GND |  | GND
 *     |  |
 * GND ---- GND
 *     GND
 *
 *
 * based on https://github.com/cnlohr/magfestbadges/blob/master/basictouch/test.c
 * (or https://www.youtube.com/watch?v=BO3umH4Ht8o&t=547s)
 *
 */

#include <avr/io.h>
#include <util/delay.h>

#include "uart.h"

#include <avr/interrupt.h>
#include <avr/sleep.h>


#define DDR	DDRB
#define PORT	PORTB
#define LED	PB2

// DDRA, PORTA
// SCL PA4 (also SCK)
// SDA PA6 (also MOSI)
// SCL, SDA do not work, as they have low pullups!
// using INT_RFM PA1
#define TOUCH PA1
#define TOUCHINT PCINT1

inline static void ledSetup(){
  DDR |= _BV(LED);
  PORT &= ~_BV(LED); // set low
}

inline static void ledOn() {
  PORT |= _BV(LED); // set high
}

inline static void ledOff() {
  PORT &= ~_BV(LED); // set low
}

inline static void ledToggle() {
  PORT ^= _BV(LED); // invert (exclusive or)
}

//tr is a temporary variable for storing the value in TCNT1 (timer 1)
volatile uint16_t tr;
volatile uint16_t count;

//This is an interrupt that automatically gets called by the AVR when the touch pin rises.
ISR(PCINT0_vect) {
	tr = TCNT1;
    count++;
	if (TIFR1 & _BV(OCF1A))
		tr = 0xfff0;
}

//This is an interrupt that automatically gets called if the pin takes too long to rise.
ISR(TIM1_COMPA_vect) {
	tr = 0xfff0;
}

/*
 * measure values by blocking CPU
 */
uint16_t measure() {
    DDRA |= _BV(TOUCH);  // set as output -> low to discharge capacity

    // wait a short time to ensure discharge
    _delay_ms(10);

    GIFR = (1<<PCIF0); // clear any pending interrupt flag
    PCMSK0 |= _BV(TOUCHINT); // set pin change interrupt

    DDRA &= ~_BV(TOUCH);  // set as input -> capacity is charged via resistor

    //Reset the timer
    TCNT1 = 0;

    //Put the CPU to sleep until we either get a pin change or overflow compare A.
    sleep_cpu();
    PCMSK0 &= ~_BV(TOUCHINT); // disable pin change interrupt

    return tr;
}


int main(void) {
	DINIT();
	uint16_t value;
	unsigned short calibrate = 0;  //Initial "zero" value for touch sensor

	//Setup timer

	TCCR1B = _BV(CS10);       //Use prescalar 2. CS10 = /1, CS11 = /8, ...
	TIMSK1 |= _BV(OCIE1A);  //Enable overflow compare A (to detect if we're taking too long)
	OCR1A = 0xfff0;            //Set overflow A to be 0xfff0 cycles

	sleep_enable();          //Allow the CPU to sleep.

	GIMSK |= _BV(PCIE0);      //Enable Pin change interrupts.


    TIFR1 |= _BV(OCF1A); // interrupt flag register, compares to OCR1A

	PORTA &= ~_BV(TOUCH); // set port low as default

	tr = 0x0000;

	DL("Hello there");
    count = 0;
    sei();

    // calibrate sensor
    DL("calibrating. Do not touch sensor");
    for (int i=0; i<10; i++) {
        calibrate = measure();
    }

    while (1) {
        value = measure();
        uint16_t valueCalibrated = value - calibrate;
        // if (value > 0xfff0) value = 0x00; // if negative set it 0
        DF("value: %u (uncalibrated: %u)", valueCalibrated, value);

        if (valueCalibrated > 50) {
            ledOn();
        } else {
            ledOff();
        }
        _delay_ms(500);
    }
    return 0;
}
