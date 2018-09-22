/**
 * Based on https://github.com/JDat/AtTiny-I2C-master-slave-USI
 *
 */

#include <avr/io.h>
#include <util/delay.h>

#include "usi_i2c_slave.h"

#define LED_K PA1
#define LED_A PA0
#define LED_DDR DDRA
#define LED_PORT PORTA

extern char* USI_Slave_register_buffer[];

inline static void ledSetup(){
    LED_DDR |= _BV(LED_A) | _BV(LED_K);
    LED_PORT &= ~_BV(LED_A);
    LED_PORT &= ~_BV(LED_K);
}

inline static void ledOn() {
    LED_PORT |= _BV(LED_A);
}

inline static void ledOff() {
    LED_PORT &= ~_BV(LED_A);
}

inline static void ledToggle() {
    LED_PORT ^= _BV(LED_A);
}

int main(void) {
    ledSetup();

    // blink led on startup
    for (int i=0; i<6; i++) {
        _delay_ms(500);
        ledToggle();
    }
    _delay_ms(3000);


    // setting pointer of val to internal var
    // i2c interrupt will set value
    unsigned val = 0;
    USI_Slave_register_buffer[0] = (unsigned char*)&val;

    // init slave device on address 0x12
    USI_I2C_Init(0x12);

    while(1) {
        for (int i=0; i<val; i++) {
            ledToggle();
            _delay_ms(200);
        }
    }


    return 0;
}
