#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "config.h"
#include "driver.h"
#include "tty.h"
#include "cc1101.h"
#include "led.h"
#include "transcoder.h"

void main_init(void) {
  // OSCCAL=((uint32_t)OSCCAL * 10368) / 10000;

  wdt_disable();
  led_init();

  // Wire up components
  transcoder_init(&tty_write_str, &driver_send_byte);
  driver_init(&transcoder_accept_inbound_byte);
  tty_init(&transcoder_accept_outbound_byte);
  cc_init(&driver_accept_bit, &driver_request_bit);

  led_off();
  sei();
}

void main_work(void) {
  driver_work();
  tty_work();
  cc_work();
}

#ifdef NEEDS_MAIN
int main(void) {
  main_init();

  while(1) {
    main_work();
  }
}
#endif
