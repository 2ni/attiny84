/**
 * Copyright (C) PlatformIO <contact@platformio.org>
 * See LICENSE for details.
 */

#include <avr/io.h>
#include <util/delay.h>

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

    // (pull-up and high DDR = 1, PORT = 1)
    // make tri-state DDR = 0, PORT = 0
    // to test i2c / twi ports
    DDR &= ~_BV(SCL);
    PORT &= ~_BV(SCL);
    DDR &= ~_BV(SDA);
    PORT &= ~_BV(SDA);
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

    while (1) {
        _delay_ms(500);
        ledToggle();
    }

    return 0;
}
