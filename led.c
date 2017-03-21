#include <avr/io.h>
#include "led.h"

#define LED_DDR   DDRC
#define LED_PORT  PORTC
#define LED_PIN   6

inline void led_init() {
  LED_DDR |= (1 << LED_PIN);
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
