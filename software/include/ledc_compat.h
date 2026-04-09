#ifndef SWARMBOT_LEDC_COMPAT_H
#define SWARMBOT_LEDC_COMPAT_H

#include <Arduino.h>

// Provide ledcAttachChannel on cores that only have ledcSetup + ledcAttachPin.
static inline void ledcAttachChannel(uint8_t pin, uint32_t freq, uint8_t resolution, uint8_t channel) {
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(pin, channel);
}

#endif  // SWARMBOT_LEDC_COMPAT_H
