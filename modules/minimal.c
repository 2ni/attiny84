/*
 * Code to test my boards
 */

#include <avr/io.h>
#include <util/delay.h>


#define DDR	DDRB
#define PORT	PORTB
#define LED	PB2

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


int main(void) {
  ledSetup();

  while (1) {
    _delay_ms(200);
    ledToggle();
  }

  return 0;
}
