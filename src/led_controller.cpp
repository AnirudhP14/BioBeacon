#include "led_controller.h"

#include "config.h"
#include "logging.h"

namespace {
constexpr int kPwmFreq = 5000;
constexpr int kPwmRes = 8;
constexpr int kChanR = 0;
constexpr int kChanG = 1;
constexpr int kChanB = 2;

constexpr unsigned long kHeartbeatMs = 200;
constexpr unsigned long kCommandWindowMs = 250;
constexpr unsigned long kMessageBlinkMs = 200;
constexpr unsigned long kRemoteCo2BlinkMs = 120;
constexpr unsigned long kCo2BlinkMs = 300;
constexpr unsigned long kStopBlinkMs = 200;
constexpr unsigned long kPulsePeriodMs = 1200;

constexpr uint8_t kRed[3] = {255, 0, 0};
constexpr uint8_t kGreen[3] = {0, 255, 0};
constexpr uint8_t kBlue[3] = {0, 0, 255};
constexpr uint8_t kWhite[3] = {255, 255, 255};
constexpr uint8_t kPurple[3] = {255, 0, 255};
constexpr uint8_t kCyan[3] = {0, 128, 128};
constexpr uint8_t kYellow[3] = {255, 200, 0};
constexpr uint8_t kAmber[3] = {255, 140, 0};
}  // namespace

LedController::LedController(Logger& log) : log_(log) {}

bool LedController::begin() {
  enabled_ = true;
  error_ = false;
  stopLatched_ = false;
  pinMode(config::LED_PIN_R, OUTPUT);
  pinMode(config::LED_PIN_G, OUTPUT);
  pinMode(config::LED_PIN_B, OUTPUT);
  // digitalWrite(config::LED_PIN_R, 0);
  // digitalWrite(config::LED_PIN_G, 255);
  // digitalWrite(config::LED_PIN_B, 0);

  setBooting();
  return true;
}

void LedController::disable() {
  enabled_ = false;
}

void LedController::setBooting() {
  baseState_ = BaseState::Booting;
  error_ = false;
}

void LedController::setReady() {
  baseState_ = BaseState::Ready;
  error_ = false;
}

void LedController::setDriveMode() {
  baseState_ = BaseState::Drive;
}

void LedController::setPaused() {
  baseState_ = BaseState::Paused;
}

void LedController::setError() {
  error_ = true;
}

void LedController::setCO2High(bool enabled) {
  co2High_ = enabled;
}

void LedController::setRemoteCO2(bool enabled) {
  remoteCo2_ = enabled;
}

void LedController::pulseHeartbeat() {
  heartbeatStartMs_ = millis();
}

void LedController::pulseCommand() {
  commandStartMs_ = millis();
}

void LedController::setStop(bool enabled) {
  stopLatched_ = enabled;
}

float LedController::triangle(float phase) const {
  if (phase < 0.5f) return phase * 2.0f;
  return (1.0f - phase) * 2.0f;
}

LedController::Color LedController::applyOverlay(const Color& base, const Color& overlay, float alpha) {
  auto blend = [&](uint8_t b, uint8_t o) -> uint8_t {
    float v = b + (o - b) * alpha;
    if (v < 0.0f) v = 0.0f;
    if (v > 255.0f) v = 255.0f;
    return (uint8_t)v;
  };
  return {blend(base.r, overlay.r), blend(base.g, overlay.g), blend(base.b, overlay.b)};
}

void LedController::writeColor(const Color& c) {
  analogWrite(config::LED_PIN_R, c.r);
  analogWrite(config::LED_PIN_G, c.g);
  analogWrite(config::LED_PIN_B, c.b);
}

void LedController::update(unsigned long now) {
  if (!enabled_) return;

  if (lastUpdateMs_ == 0) {
    lastUpdateMs_ = now;
  }

  Color base{0, 0, 0};
  switch (baseState_) {
    case BaseState::Booting: {
      float phase = fmodf((now % kPulsePeriodMs) / (float)kPulsePeriodMs, 1.0f);
      uint8_t v = (uint8_t)(triangle(phase) * 255.0f);
      base = {v, v, v};
      break;
    }
    case BaseState::Ready:
      base = {kWhite[0], kWhite[1], kWhite[2]};
      break;
    case BaseState::Drive:
      base = {200, 200, 200};
      break;
    case BaseState::Paused:
      base = {kCyan[0], kCyan[1], kCyan[2]};
      break;
  }

  // STOP is an external safety override; ERROR indicates an internal fault.
  if (stopLatched_) {
    bool on = ((now / kStopBlinkMs) % 2) == 0;
    Color pulse = on ? Color{kRed[0], kRed[1], kRed[2]} : Color{0, 0, 0};
    writeColor(pulse);
    return;
  }

  if (error_) {
    writeColor({kRed[0], kRed[1], kRed[2]});
    return;
  }

  if (remoteCo2_) {
    bool on = ((now / kRemoteCo2BlinkMs) % 2) == 0;
    Color pulse = on ? Color{kAmber[0], kAmber[1], kAmber[2]} : Color{0, 0, 0};
    writeColor(pulse);
    return;
  }

  Color current = base;

  if (commandStartMs_ && (now - commandStartMs_) < kCommandWindowMs) {
    bool on = ((now / kMessageBlinkMs) % 2) == 0;
    current = on ? Color{0, 255, 0} : Color{0, 0, 0};
  } else if (commandStartMs_ && (now - commandStartMs_) >= kCommandWindowMs) {
    commandStartMs_ = 0;
  } else if (co2High_) {
    bool on = ((now / kCo2BlinkMs) % 2) == 0;
    current = on ? Color{kBlue[0], kBlue[1], kBlue[2]} : Color{0, 0, 0};
    if (millis() - lastUpdateMs_ > 1000) {
      log_.logMsg("LED CO2 overlay pulse");
      lastUpdateMs_ = millis();
    }
  } else if (heartbeatStartMs_ && (now - heartbeatStartMs_) < kHeartbeatMs) {
    bool on = ((now / kMessageBlinkMs) % 2) == 0;
    if (baseState_ == BaseState::Drive) {
      current = on ? Color{kYellow[0], kYellow[1], kYellow[2]} : Color{0, 0, 0};
    }
  } else if (heartbeatStartMs_ && (now - heartbeatStartMs_) >= kHeartbeatMs) {
    heartbeatStartMs_ = 0;
  }

  writeColor(current);
}
