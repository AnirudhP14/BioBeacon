#include "drive.h"

#include <ctype.h>
#include "config.h"
#include "logging.h"
#include "motors.h"

const char* driveModeLabel(DriveMode mode) {
  switch (mode) {
    case DriveMode::Auto: return "auto";
    case DriveMode::Path: return "path";
    default: return "unknown";
  }
}

DriveController::DriveController(MotorDriver& motors, Logger& log)
    : motors_(motors), log_(log) {}

bool DriveController::setPathFromString(const String& path, String& error) {
  String trimmed = path;
  trimmed.trim();
  if (trimmed.length() == 0) {
    error = "Empty path";
    return false;
  }

  pathStepCount_ = 0;
  int start = 0;
  while (start < trimmed.length() && pathStepCount_ < config::MAX_PATH_STEPS) {
    int end = trimmed.indexOf(',', start);
    if (end < 0) end = trimmed.length();
    String token = trimmed.substring(start, end);
    token.trim();

    if (token.equalsIgnoreCase("br")) {
      pathSteps_[pathStepCount_++] = {MotorState::Brake, config::BRAKE_TIME_MS};
    } else if (token.length() >= 1) {
      char dir = tolower(token.charAt(0));
      int sec = token.substring(1).toInt();
      if (sec <= 0) sec = 1;
      unsigned long duration = (unsigned long)sec * 1000;
      if (dir == 'f') {
        pathSteps_[pathStepCount_++] = {MotorState::Forward, duration};
      } else if (dir == 'b') {
        pathSteps_[pathStepCount_++] = {MotorState::Backward, duration};
      } else if (dir == 'l') {
        pathSteps_[pathStepCount_++] = {MotorState::TurnLeft, duration};
      } else if (dir == 'r') {
        pathSteps_[pathStepCount_++] = {MotorState::TurnRight, duration};
      } else {
        error = "Invalid token";
        return false;
      }
    }

    start = end + 1;
  }

  if (pathStepCount_ == 0) {
    error = "No valid steps";
    return false;
  }

  driveMode_ = DriveMode::Path;
  pathStepIndex_ = 0;
  pathStepStart_ = 0;
  motorState_ = MotorState::Idle;
  log_.logFmt("Drive path set: %s", trimmed.c_str());
  return true;
}

void DriveController::setMode(DriveMode mode) {
  driveMode_ = mode;
}

void DriveController::setAvoid(bool avoid) {
  avoid_ = avoid;
}

void DriveController::update(unsigned long now) {
  if (avoid_) {
    motors_.brake();
    return;
  }

  if (driveMode_ == DriveMode::Path) {
    handlePathDrive(now);
  } else {
    //handleAutoDrive(now);
  }
}

DriveStatus DriveController::status() const {
  DriveStatus status;
  status.motorState = motorState_;
  status.motorStateStart = motorStateStart_;
  status.driveMode = driveMode_;
  status.pathStepIndex = pathStepIndex_;
  status.pathStepCount = pathStepCount_;
  return status;
}

void DriveController::startDriveStep(const DriveStep& step, unsigned long now) {
  motorState_ = step.state;
  motorStateStart_ = now;
  pathStepStart_ = now;

  switch (step.state) {
    case MotorState::Forward:
      log_.logMsg("Motors drive FORWARD");
      motors_.forward(config::DRIVE_SPEED);
      break;
    case MotorState::Backward:
      log_.logMsg("Motors drive BACKWARD");
      motors_.backward(config::DRIVE_SPEED);
      break;
    case MotorState::Brake:
      log_.logMsg("Motors brake/pause");
      motors_.brake();
      break;
    case MotorState::TurnLeft:
      log_.logMsg("Motors turn LEFT");
      motors_.turnLeft(config::TURN_SPEED);
      break;
    case MotorState::TurnRight:
      log_.logMsg("Motors turn RIGHT");
      motors_.turnRight(config::TURN_SPEED);
      break;
    case MotorState::Idle:
    default:
      break;
  }
}

void DriveController::handlePathDrive(unsigned long now) {
  if (pathStepIndex_ >= pathStepCount_) {
    motors_.brake();
    motorState_ = MotorState::Brake;
    motorStateStart_ = now;
    driveMode_ = DriveMode::Auto;
    log_.logMsg("Drive path complete");
    return;
  }

  const DriveStep& step = pathSteps_[pathStepIndex_];
  if (pathStepStart_ == 0) {
    startDriveStep(step, now);
    return;
  }

  if ((now - pathStepStart_) >= step.durationMs) {
    pathStepIndex_++;
    pathStepStart_ = 0;
    if (pathStepIndex_ >= pathStepCount_) {
      motors_.brake();
      motorState_ = MotorState::Brake;
      motorStateStart_ = now;
      driveMode_ = DriveMode::Auto;
      log_.logMsg("Drive path complete");
      return;
    }
    startDriveStep(pathSteps_[pathStepIndex_], now);
  }
}

void DriveController::handleAutoDrive(unsigned long now) {
  if (motorState_ == MotorState::Idle || randomStepDuration_ == 0 ||
      (now - motorStateStart_) >= randomStepDuration_) {
    int r = random(0, 100);
    MotorState nextState = MotorState::Forward;
    if (r < 55) {
      nextState = MotorState::Forward;
    } else if (r < 75) {
      nextState = MotorState::TurnLeft;
    } else if (r < 95) {
      nextState = MotorState::TurnRight;
    } else {
      nextState = MotorState::Brake;
    }

    if (nextState == MotorState::Brake) {
      randomStepDuration_ = random(150, 350);
    } else {
      randomStepDuration_ = random(config::RANDOM_STEP_MIN_MS, config::RANDOM_STEP_MAX_MS);
    }

    motorState_ = nextState;
    motorStateStart_ = now;

    switch (nextState) {
      case MotorState::Forward:
        log_.logMsg("Motors drive FORWARD");
        motors_.forward(config::DRIVE_SPEED);
        break;
      case MotorState::TurnLeft:
        log_.logMsg("Motors turn LEFT");
        motors_.turnLeft(config::TURN_SPEED);
        break;
      case MotorState::TurnRight:
        log_.logMsg("Motors turn RIGHT");
        motors_.turnRight(config::TURN_SPEED);
        break;
      case MotorState::Brake:
        log_.logMsg("Motors brake/pause");
        motors_.brake();
        break;
      case MotorState::Backward:
      case MotorState::Idle:
      default:
        break;
    }
  }
}
