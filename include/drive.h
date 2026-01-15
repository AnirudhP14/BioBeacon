#ifndef SWARMBOT_DRIVE_H
#define SWARMBOT_DRIVE_H

#include <Arduino.h>
#include "config.h"
#include "motors.h"

class Logger;
class MotorDriver;

enum class DriveMode {
  Auto,
  Path
};

const char* driveModeLabel(DriveMode mode);

struct DriveStatus {
  MotorState motorState = MotorState::Idle;
  unsigned long motorStateStart = 0;
  DriveMode driveMode = DriveMode::Auto;
  int pathStepIndex = 0;
  int pathStepCount = 0;
};

class DriveController {
 public:
  DriveController(MotorDriver& motors, Logger& log);
  bool setPathFromString(const String& path, String& error);
  void setMode(DriveMode mode);
  void setAvoid(bool avoid);
  void update(unsigned long now);
  DriveStatus status() const;

 private:
  struct DriveStep {
    MotorState state;
    unsigned long durationMs;
  };

  void startDriveStep(const DriveStep& step, unsigned long now);
  void handlePathDrive(unsigned long now);
  void handleAutoDrive(unsigned long now);

  MotorDriver& motors_;
  Logger& log_;
  DriveMode driveMode_ = DriveMode::Auto;
  MotorState motorState_ = MotorState::Idle;
  unsigned long motorStateStart_ = 0;
  unsigned long pathStepStart_ = 0;
  unsigned long randomStepDuration_ = 0;
  bool avoid_ = false;

  DriveStep pathSteps_[config::MAX_PATH_STEPS];
  int pathStepCount_ = 0;
  int pathStepIndex_ = 0;
};

#endif  // SWARMBOT_DRIVE_H
