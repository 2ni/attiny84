/*
 * Code to test my boards
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "uart.h"

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

uint8_t blink_count = 0;

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
}

void blinkTimerStart() {
  TIMSK1 |= _BV(OCIE1A); //enable timer compare interrupt
}

void blinkTimerStop() {
  TIMSK1 &= ~_BV(OCIE1A); //disable timer compare interrupt
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

int main(void) {
  ledSetup();

  DINIT();
  DL("hello from ATtiny84.");

  cli();
  timerSetup();
  sei();

  blink_count=6; //blink 3x
  blinkTimerStart();

  while(1) {} // loop forever
}
