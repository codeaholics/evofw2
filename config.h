#ifndef _CONFIG_H_
#define _CONFIG_H_

#if defined TARGET_SCC_V2
#  include "hw/scc_v2.h"
#elif defined SHA_NANO_V3
#  include "hw/sha_nano_v3.h"
#elif defined ARDUINO
#  include "hw/arduino.h"
#else
#  error "No hardware target defined"
#endif

#endif
