#include <avr/interrupt.h>
#include <avr/io.h>
#include "led.h"

#define LED_DDR   DDRC
#define LED_PORT  PORTC
#define LED_PIN   6

ISR(TIMER1_COMPA_vect) {
  led_toggle();
}

inline void led_init() {
  LED_DDR |= (1 << LED_PIN);

  led_on();

  // One second time to blink LED
  OCR1A = (F_CPU / 1024) - 1;
  TCCR1B |= (1 << WGM12) | (1 << CS10) | (1 << CS12);
  TIMSK1 = (1 << OCIE1A);
}

inline void led_on() {
  LED_PORT |= (1 << LED_PIN);
}

inline void led_off() {
  LED_PORT &= ~(1 << LED_PIN);
}

inline void led_toggle() {
  LED_PORT ^= (1 << LED_PIN);
}
