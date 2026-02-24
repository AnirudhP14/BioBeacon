#ifndef SWARMBOT_CONFIG_H
#define SWARMBOT_CONFIG_H

#include <Arduino.h>

namespace config {
constexpr bool RUN_BOOT_DIAGNOSTICS = true;

constexpr const char* WIFI_SSID_BASE = "firefly";
constexpr const char* WIFI_PASS = "debug123";

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

// ESP-NOW
constexpr uint8_t DEVICE_ID = 1;
constexpr uint8_t SWARM_ID = 1;

constexpr int OFFSET_A = 1;
constexpr int OFFSET_B = 1;
constexpr int ALIGN_A = (DEVICE_ID == 3) ? 1 : -1;
constexpr int ALIGN_B = -1; // (DEVICE_ID == 1) ? 1 : -1;

constexpr float WHEEL_DIAM_MM = 53.0f;
constexpr int QUADRATURE = 4;
constexpr int ENCODER_PPR = 1050 * QUADRATURE;
constexpr float MM_PER_TICK = (PI * WHEEL_DIAM_MM) / ENCODER_PPR;

// Drive timings/speeds
constexpr unsigned long FORWARD_TIME_MS = 1000;
constexpr unsigned long BACKWARD_TIME_MS = 1000;
constexpr unsigned long BRAKE_TIME_MS = 1000;
constexpr int DRIVE_SPEED = 2000;
constexpr int TURN_SPEED = 1600;
constexpr unsigned long RANDOM_STEP_MIN_MS = 300;
constexpr unsigned long RANDOM_STEP_MAX_MS = 900;
constexpr int MAX_PATH_STEPS = 16;

// Grid navigation
constexpr bool ENABLE_GRID_NAV = true;
constexpr int GRID_WIDTH = 5;
constexpr int GRID_HEIGHT = 4;
constexpr float CELL_WIDTH_MM = 500.0f;
constexpr float CELL_HEIGHT_MM = 500.0f;
constexpr float BOUNDARY_MARGIN_MM = 50.0f;
constexpr float POSITION_TOLERANCE_MM = 60.0f;
constexpr float HEADING_TOLERANCE_DEG = 12.0f;
constexpr float HEADING_KP = 0.8f;

constexpr float MAX_X_MM = GRID_WIDTH * CELL_WIDTH_MM;
constexpr float MAX_Y_MM = GRID_HEIGHT * CELL_HEIGHT_MM;
constexpr float SPEED_MM_PER_SEC = 150.0f;  // Calibrate: measure time for 300mm straight, speed = 300 / time_sec
constexpr float TIMED_FORWARD_SEC_PER_CELL = 1.25f;  // Calibrate: time to move one cell forward 1.25
constexpr float TIMED_TURN_90_SEC = 0.285f;           // Calibrate: time for 90 deg spot turn at TURN_SPEED 0.25 (0.31 for dev 1)
constexpr int TIMED_DRIVE_SPEED = 500;             // PWM value
constexpr unsigned long SCAN_BRAKE_MS = 500;

// RGB LED pins (common cathode)
constexpr int LED_PIN_R = A0;  // A6 to A0
constexpr int LED_PIN_G = A6;  // A7 to A1
constexpr int LED_PIN_B = A7; //TX1 to D13 to A2

constexpr uint8_t SENSOR_CO2 = 1;
constexpr uint8_t SENSOR_TVOC = 2;
constexpr uint8_t SENSOR_TOF = 3;
constexpr uint8_t SENSOR_DIST = 4;
constexpr uint8_t SENSOR_HEARTBEAT = 250;
constexpr unsigned long ESP_NOW_PERIOD_MS = 200;
constexpr unsigned long ESP_NOW_HEARTBEAT_MS = 1000;
constexpr float NEARBY_MM = 200.0f;
constexpr float CO2_ALERT = 1000.0f;
constexpr uint16_t SCAN_CO2_STOP_PPM = 600;
constexpr uint16_t SCAN_TOF_STOP_MM = 200;
}  // namespace config

#endif  // SWARMBOT_CONFIG_H
