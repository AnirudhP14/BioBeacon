#ifndef SWARMBOT_PINS_NANO_ESP32_H
#define SWARMBOT_PINS_NANO_ESP32_H

#include <Arduino.h>

namespace config {
// Motor driver pins (Arduino Nano ESP32 pin labels -> GPIO)
constexpr int AIN1 = 5;
constexpr int AIN2 = 6;
constexpr int PWMA = 9;
constexpr int BIN1 = 7;
constexpr int BIN2 = 8;
constexpr int PWMB = 10;
constexpr int STBY = 4;

// Encoder pins (C1/C2)
constexpr int ENCA_C1 = 2;
constexpr int ENCA_C2 = 3;
constexpr int ENCB_C1 = 11;
constexpr int ENCB_C2 = 12;

// RGB LED pins (common cathode)
constexpr int LED_PIN_R = A0;  // A6 to A0
constexpr int LED_PIN_G = A6;  // A7 to A1
constexpr int LED_PIN_B = A7;  // TX1 to D13 to A2

// I2C pins (use core defaults for Nano ESP32)
constexpr int SDA_PIN = SDA;
constexpr int SCL_PIN = SCL;
}  // namespace config

#endif  // SWARMBOT_PINS_NANO_ESP32_H
