#ifndef _CC1101_H_
#define _CC1101_H_

#include <stdint.h>

typedef uint8_t (*accept_bit_fn)(uint8_t);
typedef uint8_t (*request_bit_fn)(void);

void cc_init(accept_bit_fn a, request_bit_fn r);
void cc_work(void);

#endif
