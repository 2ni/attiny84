/**
 * Copyright (C) PlatformIO <contact@platformio.org>
 * See LICENSE for details.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#define DDR DDRA
#define PORT PORTA
#define LED_K PA1
#define LED_A PA0

#define SCL PA4
#define SDA PA6

// _BV(3) => 1 << 3 => 0x08
// PORTB = PORTB | (1 << 4);
// set: PORTB |= (1<<BIT2);
// clear: PORTB &= ~(1<<BIT2);

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

int main(void) {
    ledSetup();

    cli();
    TCCR1B |= 1<<CS11 | 1<<CS10; // divide by 64
    OCR1A = 15625; // count 1sec = (1/4000000*64)*62500, 1/4sec -> 15625
    TCCR1B |= 1<<WGM12; // put timer/counter1 in CTC mode (Clear Timer on Compare)
    TIMSK1 |= 1<<OCIE1A; //enable timer compare interrupt
    sei();

    while(1) {} // loop forever, interrupts handle it
}

ISR(TIM1_COMPA_vect) {
    ledToggle();
}
