/***********************************************************************
 *
 * bitstream.h
 *
 */

#ifndef _BITSTREAM_H_
#define _BITSTREAM_H_

// Return values from bs_accept_octet
enum bs_accept_octet_codes {
  BS_NOT_SYNCHRONISED = 0,
  BS_SYNCHRONISED,
  BS_END_OF_PACKET,
  BS_MANCHESTER_ERROR,
  BS_ABORT,
  BS_MAX_STATUS = 16    // Radio packets must always be bigger than this
  // Anything bigger than this is a packet length indication
};
extern uint16_t bs_accept_octet( uint8_t bits );

extern void bs_init(void);

#endif
