#ifndef SWARMBOT_DIAGNOSTICS_H
#define SWARMBOT_DIAGNOSTICS_H

#include <Arduino.h>

class Logger;
class MotorDriver;
class SensorManager;

class Diagnostics {
 public:
  Diagnostics(Logger& log, MotorDriver& motors, SensorManager& sensors);
  void runBootDiagnostics();
  void logBootPins();
  void motorSelfTest();

 private:
  Logger& log_;
  MotorDriver& motors_;
  SensorManager& sensors_;
};

#endif  // SWARMBOT_DIAGNOSTICS_H
