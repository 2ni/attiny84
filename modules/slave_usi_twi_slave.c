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
#include <util/delay.h>

#include "usi_twi_slave.h"

#define SLAVE_ADRS	0x21 // 0x40 does not work?!?

#define DDR		DDRA
#define PORT	PORTA
#define LED_A	PA0
#define LED_K	PA1

#define SCL PA4
#define SDA PA6

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

    usiTwiSlaveInit(SLAVE_ADRS);	// Initialize TWI hardware for Slave operation.
    usiTwiSlaveEnable();			// Enable the TWI interface to receive data.
    sei();		            		// Enable interrupts.
    /*
    usiInit(0x14);
    sei();
    */

    // blink on startup
    for (int i=0; i<6; i++) {
        _delay_ms(500);
        ledToggle();
    }
    _delay_ms(2000);

    // write data
    uint8_t count = 0;
    while (1) {
        if(!usiTwiDataInTransmitBuffer()) {
            usiTwiTransmitByte(count);
            ++count;
        }
    }

    /*
    // read data
    uint8_t data;
    while(1) {
        if(usiTwiDataInReceiveBuffer()) {
            data = usiTwiReceiveByte();

            // If the data is 00, then turn OFF the LED. Turn it ON for any non-zero value.
            if( data == 0) {
                PORT &= ~(1<<LED_A);		// Turn LED OFF.
            }
			else {
                PORT |= (1<<LED_A);			// Turn LED ON.
            }
        }
    }
    */
}
