#ifndef SWARMBOT_LED_CONTROLLER_H
#define SWARMBOT_LED_CONTROLLER_H

#include <Arduino.h>

class Logger;

class LedController {
 public:
  explicit LedController(Logger& log);

  bool begin();
  void disable();
  void update(unsigned long now);

  void setBooting();
  void setReady();
  void setDriveMode();
  void setPaused();
  void setError();

  void setCO2High(bool enabled);
  void setRemoteCO2(bool enabled);

  void pulseHeartbeat();
  void pulseCommand();
  void setStop(bool enabled);

 private:
  enum class BaseState {
    Booting,
    Ready,
    Drive,
    Paused
  };

  struct Color {
    uint8_t r;
    uint8_t g;
    uint8_t b;
  };

  void writeColor(const Color& c);
  Color applyOverlay(const Color& base, const Color& overlay, float alpha);
  float triangle(float phase) const;

  Logger& log_;
  bool enabled_ = false;
  BaseState baseState_ = BaseState::Booting;
  bool error_ = false;
  bool co2High_ = false;
  bool remoteCo2_ = false;
  bool stopLatched_ = false;

  unsigned long heartbeatStartMs_ = 0;
  unsigned long commandStartMs_ = 0;

  unsigned long lastUpdateMs_ = 0;
};

#endif  // SWARMBOT_LED_CONTROLLER_H
