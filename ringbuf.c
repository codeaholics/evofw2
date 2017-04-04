#include <avr/interrupt.h>
#include "ringbuf.h"

void rb_reset(rb_t *rb) {
  rb->getoff = rb->putoff = rb->nbytes = 0;
}

void rb_put(rb_t *rb, uint8_t data) {
  uint8_t sreg;
  sreg = SREG;
  cli();
  if (rb->nbytes >= RINGBUF_SIZE) {
    SREG = sreg;
    return;
  }
  rb->nbytes++;
  rb->buf[rb->putoff++] = data;
  if (rb->putoff == RINGBUF_SIZE)
    rb->putoff = 0;
  SREG = sreg;
}

uint8_t rb_get(rb_t *rb) {
  uint8_t sreg;
  uint8_t ret;
  sreg = SREG;
  cli();
  if (rb->nbytes == 0) {
    SREG = sreg;
    return 0;
  }
  rb->nbytes--;
  ret = rb->buf[rb->getoff++];
  if (rb->getoff == RINGBUF_SIZE)
    rb->getoff = 0;
  SREG = sreg;
  return ret;
}
