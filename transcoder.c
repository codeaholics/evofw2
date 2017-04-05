#include <stdio.h>
#include <string.h>
#include "errors.h"
#include "transcoder.h"

#define S_HEADER     0
#define S_ADDR0      1
#define S_ADDR1      2
#define S_ADDR2      3
#define S_PARAM0     4
#define S_PARAM1     5
#define S_CMD        6
#define S_LEN        7
#define S_PAYLOAD    8
#define S_CHECKSUM   9
#define S_COMPLETE  10
#define S_ERROR     11

static const uint8_t HEADER_FLAGS[16] = {
  0x0F, 0x0C, 0x0D, 0x0B,
  0x27, 0x24, 0x25, 0x23,
  0x47, 0x44, 0x45, 0x43,
  0x17, 0x14, 0x15, 0x13
};

inline uint8_t is_information(uint8_t flags) { return flags & 0x20; }
inline uint8_t is_request(uint8_t flags)     { return flags & 0x08; }
inline uint8_t is_response(uint8_t flags)    { return flags & 0x10; }
inline uint8_t is_write(uint8_t flags)       { return flags & 0x40; }
inline uint8_t has_addr0(uint8_t flags)      { return flags & 0x01; }
inline uint8_t has_addr1(uint8_t flags)      { return flags & 0x02; }
inline uint8_t has_addr2(uint8_t flags)      { return flags & 0x04; }
inline uint8_t has_param0(uint8_t header)    { return header & 0x02; }
inline uint8_t has_param1(uint8_t header)    { return header & 0x01; }

inline void set_information(uint8_t *flags) { *flags |= 0x20; }
inline void set_request(uint8_t *flags)     { *flags |= 0x08; }
inline void set_response(uint8_t *flags)    { *flags |= 0x10; }
inline void set_write(uint8_t *flags)       { *flags |= 0x40; }

static write_str_fn write_str;
static send_byte_fn send_byte;

void transcoder_init(write_str_fn w, send_byte_fn s) {
  write_str = w;
  send_byte = s;
}

void transcoder_accept_inbound_byte(uint8_t b, uint8_t status) {
  static uint8_t checksum = 0;
  static uint8_t state = S_HEADER;
  static uint8_t multi_bytes = 0;
  static union {
    uint16_t word16;
    uint32_t word32;
  } minibuf;
  static uint8_t header;
  static uint8_t flags;
  static char str[12];

  if (status != 0) {
    if (state != S_COMPLETE || status != ERR_NONE) {
      switch (status) {
        case ERR_BAD_START_BIT:
          write_str("\x09*ERR_BAD_START_BIT*");
          break;
        case ERR_BAD_STOP_BIT:
          write_str("\x09*ERR_BAD_STOP_BIT*");
          break;
        case ERR_UNEXPECTED_START_OF_FRAME:
          write_str("\x09*ERR_UNEXPECTED_START_OF_FRAME*");
          break;
        case ERR_UNEXPECTED_END_OF_FRAME:
          write_str("\x09*ERR_UNEXPECTED_END_OF_FRAME*");
          break;
        case ERR_MANCHESTER_ENCODING:
          write_str("\x09*ERR_MANCHESTER_ENCODING*");
          break;
        default:
          write_str("\x09*ERR_UNKNOWN*");
          break;
      }
    }
    write_str("\r\n");
    state = S_HEADER;
    return;
  }

  if (state == S_COMPLETE) {
    // bytes after end of packet?
    write_str("\x09*E-DATA*");
    state = S_ERROR;
    return;
  }

  if (state == S_ERROR) {
    // ignore bytes while in error state
    return;
  }

  if (state == S_HEADER) {
    checksum = b;
    header = b;
    flags = HEADER_FLAGS[(b >> 2) & 0x0F];
    state = S_ADDR0;
    multi_bytes = 0;
    minibuf.word32 = 0;

    if (is_information(flags)) { write_str("---  I --- "); return; }
    if (is_request(flags))     { write_str("--- RQ --- "); return; }
    if (is_response(flags))    { write_str("--- RP --- "); return; }
    if (is_write(flags))       { write_str("---  W --- "); return; }

    write_str("\x09*HDR*");
    state = S_ERROR;
    return;
  }

  checksum += b;

  if (state == S_ADDR0) {
    if (has_addr0(flags)) {
      minibuf.word32 <<= 8;
      minibuf.word32 |= b;

      multi_bytes++;
      if (multi_bytes == 3) {
        sprintf(str, "%02hu:%06lu ", (uint8_t)(minibuf.word32 >> 18) & 0x3F, minibuf.word32 & 0x3FFFF);
        write_str(str);

        state = S_ADDR1;
        multi_bytes = 0;
        minibuf.word32 = 0;
      }
      return;
    } else {
      write_str("--:------ ");
      state = S_ADDR1;  // and fall through
    }
  }

  if (state == S_ADDR1) {
    if (has_addr1(flags)) {
      minibuf.word32 <<= 8;
      minibuf.word32 |= b;

      multi_bytes++;
      if (multi_bytes == 3) {
        sprintf(str, "%02hu:%06lu ", (uint8_t)(minibuf.word32 >> 18) & 0x3F, minibuf.word32 & 0x3FFFF);
        write_str(str);

        state = S_ADDR2;
        multi_bytes = 0;
        minibuf.word32 = 0;
      }
      return;
    } else {
      write_str("--:------ ");
      state = S_ADDR2;  // and fall through
    }
  }

  if (state == S_ADDR2) {
    if (has_addr2(flags)) {
      minibuf.word32 <<= 8;
      minibuf.word32 |= b;

      multi_bytes++;
      if (multi_bytes == 3) {
        sprintf(str, "%02hu:%06lu ", (uint8_t)(minibuf.word32 >> 18) & 0x3F, minibuf.word32 & 0x3FFFF);
        write_str(str);

        state = S_PARAM0;
        multi_bytes = 0;
        minibuf.word32 = 0;
      }
      return;
    } else {
      write_str("--:------ ");
      state = S_PARAM0;  // and fall through
    }
  }

  if (state == S_PARAM0) {
    if (has_param0(header)) {
      // we don't use params; ditch it and move on
      state = S_PARAM1;
      return;
    } else {
      state = S_PARAM1;  // and fall through
    }
  }

  if (state == S_PARAM1) {
    if (has_param1(header)) {
      // we don't use params; ditch it and move on
      state = S_CMD;
      return;
    } else {
      state = S_CMD;  // and fall through
    }
  }

  if (state == S_CMD) {
    minibuf.word16 <<= 8;
    minibuf.word16 |= b;

    multi_bytes++;
    if (multi_bytes == 2) {
      sprintf(str, "%04X ", minibuf.word16);
      write_str(str);

      state = S_LEN;
      multi_bytes = 0;
      minibuf.word32 = 0;
    }
    return;
  }

  if (state == S_LEN) {
    multi_bytes = b;
    sprintf(str, "%03hu ", b);
    write_str(str);

    state = S_PAYLOAD;
    return;
  }

  if (state == S_PAYLOAD) {
    if (multi_bytes > 0) {
      sprintf(str, "%02hX", b);
      write_str(str);
      multi_bytes--;
    } else {
      state = S_CHECKSUM;  // and fall through
    }
  }

  if (state == S_CHECKSUM) {
    if (checksum != 0) {
      write_str("\x09*CHK*");
      state = S_ERROR;
    } else {
      state = S_COMPLETE;
    }
    return;
  }
}

#define SEND(byte) { checksum += byte; send_byte(byte, 0); }

static uint8_t pack_flags(uint8_t flags) {
  for (uint8_t i = 0; i < sizeof(HEADER_FLAGS); i++) {
    if (HEADER_FLAGS[i] == flags) return i << 2;
  }
  return 0xFF;
}

void transcoder_accept_outbound_byte(uint8_t b) {
  static uint8_t flags = 0;
  static uint32_t addrs[3];
  static uint8_t field = 0;
  static uint8_t p = 0;
  static uint8_t checksum = 0;
  static char buff[12];

  if (b == '\n' || b == '\r') {
    if (field == 0 && p == 0) return;  // Empty string; most likely CR-LF pair

    if (field == 7) {
      send_byte(-checksum, 1);
    } else {
      send_byte(0x11, 1);
    }

    // reset state
    flags = 0;
    field = 0;
    p = 0;
    checksum = 0;
    return;
  }

  if (field == -1) {
    // Something went wrong with the packet. Skip all further bytes until EOL
    return;
  }

  if (field == 7) {
    // Payload comes last; no further separators; buffer character pairs to
    // convert to bytes and write bytes directly as they're ready
    buff[p++] = b;

    if (p == 2) {
      buff[p] = 0; // null terminate
      uint8_t payload_byte;
      sscanf(buff, "%02hhX", &payload_byte);
      SEND(payload_byte);
      p = 0;
    }

    return;
  }

  // Fields before 7, buffer until whitespace
  if (b != ' ') {
    if (p < sizeof(buff) - 1) {
      buff[p++] = b;
    }
    return;
  }

  if (!p) return;  // Double whitespace

  buff[p] = 0;  // null terminate

  if (field == 0) {
    if (!strcmp(buff, "I")) {
      set_information(&flags);
    } else if(!strcmp(buff,"RQ")) {
      set_request(&flags);
    } else if(!strcmp(buff,"RP")) {
      set_response(&flags);
    } else if(!strcmp(buff,"W")) {
      set_write(&flags);
    }
  }

  if (field >= 2 && field <= 4 && buff[0] != '-') {
    uint8_t head;
    uint32_t tail;
    sscanf(buff, "%02hhu:%06lu", &head, &tail);
    if (head == 18 && tail == 730) {  // blank marker from Domoticz
      addrs[field - 2] = 0x48DADA;
    } else {
      addrs[field - 2] = (tail | ((uint32_t)head << 18));
    }
    flags |= (1 << (field - 2));
  }

  if (field == 4) {
    // We have now seen enough to complete the packet header byte, and
    // can start transmitting
    uint8_t header = pack_flags(flags);
    if (header != 0xFF) {
      SEND(header);
    } else {
      field = -1;
      return;
    }

    if (has_addr0(flags)) {
      SEND((addrs[0] >> 16) & 0xFF);
      SEND((addrs[0] >> 8) & 0xFF);
      SEND(addrs[0] & 0xFF);
    }

    if (has_addr1(flags)) {
      SEND((addrs[1] >> 16) & 0xFF);
      SEND((addrs[1] >> 8) & 0xFF);
      SEND(addrs[1] & 0xFF);
    }

    if (has_addr2(flags)) {
      SEND((addrs[2] >> 16) & 0xFF);
      SEND((addrs[2] >> 8) & 0xFF);
      SEND(addrs[2] & 0xFF);
    }
  }

  if (field == 5) {
    uint16_t cmd;
    sscanf(buff, "%04X", &cmd);
    SEND((cmd >> 8) & 0xFF);
    SEND(cmd & 0xFF);
  }

  if (field == 6) {
    uint8_t len;
    sscanf(buff, "%03hhu", &len);
    SEND(len);
  }

  p = 0;
  field++;
}
