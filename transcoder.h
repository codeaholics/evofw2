#ifndef __TRANSCODER_H__
#define __TRANSCODER_H__

#include <stdint.h>

typedef void (*write_str_fn)(char*);
typedef void (*send_byte_fn)(uint8_t, uint8_t end);
void transcoder_init(write_str_fn, send_byte_fn);

void transcoder_accept_inbound_byte(uint8_t b, uint8_t status);
void transcoder_accept_outbound_byte(uint8_t b);

#endif
