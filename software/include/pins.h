#ifndef SWARMBOT_PINS_H
#define SWARMBOT_PINS_H

#if defined(BOARD_NANO_ESP32)
#include "pins_nano_esp32.h"
#elif defined(BOARD_WROOM_E)
#include "pins_wroom_e.h"
#else
#error "Select a board via build_flags"
#endif

#endif  // SWARMBOT_PINS_H
