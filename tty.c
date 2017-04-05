#include <avr/interrupt.h>
#include <avr/io.h>

#include "config.h"
#include "ringbuf.h"
#include "tty.h"
#include "led.h"

static outbound_byte_fn outbound_byte;

static rb_t tx_buffer;
static rb_t rx_buffer;

// TX complete. Send next char; disable interrupt if nothing else to do
ISR(TTY_UDRE_VECT) {
  UDR0 = rb_get(&tx_buffer);
  if (tx_buffer.nbytes == 0) {
	  UCSR0B &= ~(1 << UDRIE0);
  }
}

// RX byte ready
ISR(TTY_RX_VECT) {
  uint8_t data  = UDR0;
  uint8_t flags = UCSR0A;

  if ((flags & ((1 << FE0) | (1 << DOR0))) == 0) {
    rb_put(&rx_buffer, data);
  }
}

void tty_init(outbound_byte_fn o) {
  outbound_byte = o;
  rb_reset(&tx_buffer);
  rb_reset(&rx_buffer);

  UCSR0A = (1 << U2X0);
  UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);  // async, 8N1
  UBRR0 = UART_BAUD_RATE;

  // Enable USART receiver and transmitter and receive complete interrupt
  UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
}

void tty_work(void) {
  if (rx_buffer.nbytes) {
    led_toggle();
    uint8_t b = rb_get(&rx_buffer);
    outbound_byte(b);
  }
}

static void tty_write_char(char c) {
  // Silently drop characters if the buffer is full...
  if ((tx_buffer.nbytes < RINGBUF_SIZE - 2) ||
      (tx_buffer.nbytes < RINGBUF_SIZE && (c == '\r' || c == '\n'))) {
    rb_put(&tx_buffer, c);
    UCSR0B |= (1 << UDRIE0);  // Enable transmit interrupt
  }
}

void tty_write_str(char *s) {
  while (*s) tty_write_char(*s++);
}

// void tty_write_hex(uint8_t n) {
//   uint8_t b = n >> 4;
//   if (b < 10) tty_write_char(b + 48);
//   if (b >= 10) tty_write_char(b + 55);
//   b = n & 0x0F;
//   if (b < 10) tty_write_char(b + 48);
//   if (b >= 10) tty_write_char(b + 55);
// }
