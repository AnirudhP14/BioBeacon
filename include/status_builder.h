#ifndef SWARMBOT_STATUS_BUILDER_H
#define SWARMBOT_STATUS_BUILDER_H

#include <Arduino.h>

class SensorManager;
class MotorDriver;
class DriveController;
class EspNowManager;
class GridNavigator;

class StatusBuilder {
 public:
  StatusBuilder(SensorManager& sensors, MotorDriver& motors, DriveController& drive, EspNowManager& comms, GridNavigator& grid);
  String build(unsigned long now);

 private:
  SensorManager& sensors_;
  MotorDriver& motors_;
  DriveController& drive_;
  EspNowManager& comms_;
  GridNavigator& grid_;
};

#endif  // SWARMBOT_STATUS_BUILDER_H
