#ifndef SWARMBOT_PINS_WROOM_E_H
#define SWARMBOT_PINS_WROOM_E_H

#include <Arduino.h>

namespace config {
// Motor driver pins (ESP32-WROOM-E DevKit)
constexpr int AIN1 = 4;
constexpr int AIN2 = 5;
constexpr int PWMA = 18;
constexpr int BIN1 = 16;
constexpr int BIN2 = 17;
constexpr int PWMB = 19;
constexpr int STBY = 23;

// Encoder pins (C1/C2) - input-only pins are OK for encoders
constexpr int ENCA_C1 = 34;
constexpr int ENCA_C2 = 35;
constexpr int ENCB_C1 = 32;
constexpr int ENCB_C2 = 33;

// RGB LED pins (common cathode)
constexpr int LED_PIN_R = 25;
constexpr int LED_PIN_G = 26;
constexpr int LED_PIN_B = 27;

// I2C pins
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
}  // namespace config

#endif  // SWARMBOT_PINS_WROOM_E_H
