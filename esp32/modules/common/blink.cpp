#include <Arduino.h>
#include "blink.h"

/*
 * blink LED <amount> of times and on/off <duration> ms
 */
void blink(int amount, int duration) {
  for (byte i = 0; i < amount; ++i) {
    digitalWrite(LED, HIGH); // turn LED on
    delay(duration);
    digitalWrite(LED, LOW); // turn LED off
    if (amount > 1) delay(duration);
  }
}
