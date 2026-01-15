#include "grid_nav.h"

#include <Arduino.h>
#include <math.h>
#include <cstring>
#include "config.h"
#include "logging.h"
#include "motors.h"
#include "sensors.h"

namespace {
float wrapHeading(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

float nearestCardinal(float deg) {
  float h = wrapHeading(deg);
  if (h < -135.0f) return -180.0f;
  if (h < -45.0f) return -90.0f;
  if (h < 45.0f) return 0.0f;
  if (h < 135.0f) return 90.0f;
  return 180.0f;
}

class PidController {
 public:
  PidController(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd) {}

  void reset() {
    integral_ = 0.0f;
    lastErr_ = 0.0f;
    lastMs_ = 0;
  }

  float compute(float err, unsigned long nowMs) {
    if (lastMs_ == 0) {
      lastMs_ = nowMs;
      lastErr_ = err;
      return 0.0f;
    }
    float dt = (nowMs - lastMs_) / 1000.0f;
    if (dt <= 0.0f) return 0.0f;
    lastMs_ = nowMs;

    integral_ += err * dt;
    float deriv = (err - lastErr_) / dt;
    lastErr_ = err;
    return (kp_ * err) + (ki_ * integral_) + (kd_ * deriv);
  }

 private:
  float kp_;
  float ki_;
  float kd_;
  float integral_ = 0.0f;
  float lastErr_ = 0.0f;
  unsigned long lastMs_ = 0;
};
}  // namespace

GridNavigator::GridNavigator(MotorDriver& motors, SensorManager& sensors, Logger& log)
    : motors_(motors), sensors_(sensors), log_(log) {}

void GridNavigator::executeTimedCommand(const char* cmd, float durationSec) {
  if (strcmp(cmd, "forward") == 0) {
    motors_.forward(timedDriveSpeed_);
  } else if (strcmp(cmd, "left") == 0) {
    motors_.turnLeft(config::TURN_SPEED);
  } else if (strcmp(cmd, "right") == 0) {
    motors_.turnRight(config::TURN_SPEED);
  }
  unsigned long startMs = millis();
  unsigned long durationMs = (unsigned long)(durationSec * 1000.0f);
  while (millis() - startMs < durationMs) {
    delay(10);
  }
  motors_.brake();
}

void GridNavigator::begin() {
  enabled_ = false;
  status_.rowDir = RowDirection::LeftToRight;
  status_.pattern = TraversalPattern::LawnMower;
  status_.state = DriveState::Stopped;
  status_.pose.valid = false;
  haveTarget_ = false;
  homing_ = false;
  homingFirstLeg_ = false;
  resetTraversal();
  GridCell start;
  start.cx = 2;
  start.cy = 0;
  setCell(start);
  lastEncATicks_ = 0;
  lastEncBTicks_ = 0;
  useTimed_ = false;
  useTimedGrid_ = false;
  scanMode_ = false;
  scanStopLatched_ = false;
  endPoint_ = EndPoint::EndE;
  gridStartMs_ = 0;
  scanStopMs_ = 0;
  scanStopPpm_ = 0;
  scanStopTofMm_ = 0;
  timedForwardSec_ = config::TIMED_FORWARD_SEC_PER_CELL;
  timedTurn90Sec_ = config::TIMED_TURN_90_SEC;
  timedDriveSpeed_ = config::TIMED_DRIVE_SPEED;
  lastPoseMs_ = 0;
}

void GridNavigator::setEnabled(bool enabled) {
  enabled_ = enabled;
  if (enabled_) {
    scanStopLatched_ = false;
    gridStartMs_ = millis();
    scanStopMs_ = 0;
    scanStopPpm_ = 0;
    scanStopTofMm_ = 0;
  }
}

bool GridNavigator::isEnabled() const {
  return enabled_;
}

void GridNavigator::setCell(const GridCell& cell) {
  status_.cell = cell;
  startCell_ = cell;
  resetTraversal();
}

void GridNavigator::setRowDirection(RowDirection dir) {
  status_.rowDir = dir;
}

void GridNavigator::setTraversal(TraversalPattern pattern) {
  status_.pattern = pattern;
  resetTraversal();
}

void GridNavigator::setUseTimed(bool use) {
  useTimed_ = use;
  lastPoseMs_ = 0;
  if (useTimed_) {
    log_.logMsg("TimedGrid enabled: expect drift without encoders");
  }
}

void GridNavigator::setStartCellOverride(const GridCell& cell) {
  startCell_ = cell;
  status_.cell = cell;
  resetTraversal();
  haveTarget_ = false;
}

GridStatus GridNavigator::status() const {
  return status_;
}

void GridNavigator::setTimedForwardSec(float seconds) {
  if (seconds > 0.0f) {
    timedForwardSec_ = seconds;
  }
}

void GridNavigator::setTimedTurn90Sec(float seconds) {
  if (seconds > 0.0f) {
    timedTurn90Sec_ = seconds;
  }
}

void GridNavigator::setTimedDriveSpeed(int speed) {
  if (speed > 0) {
    timedDriveSpeed_ = speed;
  }
}

float GridNavigator::scanStopElapsedSec() const {
  if (!scanStopLatched_ || gridStartMs_ == 0 || scanStopMs_ == 0) {
    return 0.0f;
  }
  return (scanStopMs_ - gridStartMs_) / 1000.0f;
}

void GridNavigator::update(unsigned long now) {
  if (!enabled_) return;

  if (useTimedGrid_) {
    traverseSection(config::DEVICE_ID);
    return;
  }

  if (!updatePose(now)) {
    enterState(DriveState::Error, "Pose invalid");
    motors_.brake();
    return;
  }

  if (!checkWithinGrid(status_.pose)) {
    enterState(DriveState::Error, "Boundary violation");
    motors_.brake();
    return;
  }

  if (!status_.pose.valid) {
    return;
  }

  if (!haveTarget_) {
    GridCell cell;
    cell.cx = (int)round(status_.pose.xMm / config::CELL_WIDTH_MM - 0.5f);
    cell.cy = (int)round(status_.pose.yMm / config::CELL_HEIGHT_MM - 0.5f);
    if (cell.cx < 0) cell.cx = 0;
    if (cell.cy < 0) cell.cy = 0;
    if (cell.cx >= config::GRID_WIDTH) cell.cx = config::GRID_WIDTH - 1;
    if (cell.cy >= config::GRID_HEIGHT) cell.cy = config::GRID_HEIGHT - 1;
    status_.cell = cell;

    GridTarget center = cellCenter(cell);
    float dx = center.xMm - status_.pose.xMm;
    float dy = center.yMm - status_.pose.yMm;
    if (fabsf(dx) > config::POSITION_TOLERANCE_MM || fabsf(dy) > config::POSITION_TOLERANCE_MM) {
      homing_ = true;
      homingFinal_ = center;
      homingFirstLeg_ = true;
      float absDx = fabsf(dx);
      float absDy = fabsf(dy);
      if (absDx >= absDy) {
        GridTarget leg;
        leg.xMm = center.xMm;
        leg.yMm = status_.pose.yMm;
        setTarget(leg, cell, false);
      } else {
        GridTarget leg;
        leg.xMm = status_.pose.xMm;
        leg.yMm = center.yMm;
        setTarget(leg, cell, false);
      }
    } else {
      if (status_.pattern == TraversalPattern::Concentric && !spiralInitialized_) {
        startCell_ = status_.cell;
        resetTraversal();
      }
      GridCell next = (status_.pattern == TraversalPattern::Concentric)
                          ? nextCellConcentric()
                          : nextCell(status_.cell, status_.rowDir);
      setTarget(cellCenter(next), next, true);
    }
  }

  if (!haveTarget_) return;

  if (targetReached(target_)) {
    if (!targetIsFinal_) {
      setTarget(homingFinal_, status_.cell, true);
      homingFirstLeg_ = false;
      return;
    }

    if (homing_) {
      homing_ = false;
      if (!spiralInitialized_ && status_.pattern == TraversalPattern::Concentric) {
        startCell_ = status_.cell;
        resetTraversal();
      }
      GridCell next = (status_.pattern == TraversalPattern::Concentric)
                          ? nextCellConcentric()
                          : nextCell(status_.cell, status_.rowDir);
      setTarget(cellCenter(next), next, true);
      return;
    }

    status_.cell = targetCell_;
    int idx = status_.cell.cy * config::GRID_WIDTH + status_.cell.cx;
    if (idx >= 0 && idx < (int)(config::GRID_WIDTH * config::GRID_HEIGHT)) {
      if (!visited_[idx]) {
        visited_[idx] = true;
        visitedCount_++;
      }
    }

    GridCell next = (status_.pattern == TraversalPattern::Concentric)
                        ? nextCellConcentric()
                        : nextCell(status_.cell, status_.rowDir);
    if (next.cy >= config::GRID_HEIGHT) {
      enterState(DriveState::Stopped, "Grid complete");
      motors_.brake();
      haveTarget_ = false;
      return;
    }

    setTarget(cellCenter(next), next, true);
  }

  driveToTarget(target_);
}

void GridNavigator::traverseSection(int id) {
  // TimedGrid is open-loop; ignore encoders/IMU pose.
  auto scanPause = [this]() -> bool {
    if (!scanMode_) return false;
    motors_.brake();
    delay(config::SCAN_BRAKE_MS);
    sensors_.poll();
    log_.logFmt("Scan: ToF=%u mm CO2=%u ppm", sensors_.tofMm(), sensors_.sgpCO2());
    if (sensors_.hasTof() && sensors_.tofMm() < config::SCAN_TOF_STOP_MM) {
      scanStopLatched_ = true;
      scanStopMs_ = millis();
      scanStopTofMm_ = sensors_.tofMm();
      enabled_ = false;
      motors_.brake();
      enterState(DriveState::Stopped, "ToF scan stop");
      log_.logFmt("Scan stop: ToF=%u mm", sensors_.tofMm());
      return true;
    }
    if (sensors_.hasAir() && sensors_.sgpCO2() > config::SCAN_CO2_STOP_PPM) {
      scanStopLatched_ = true;
      scanStopMs_ = millis();
      scanStopPpm_ = sensors_.sgpCO2();
      enabled_ = false;
      motors_.brake();
      enterState(DriveState::Stopped, "CO2 scan stop");
      log_.logFmt("Scan stop: CO2=%u ppm", sensors_.sgpCO2());
      return true;
    }
    return false;
  };

  auto forwardSteps = [this, &scanPause](int steps) -> bool {
    for (int i = 0; i < steps; i++) {
      executeTimedCommand("forward", timedForwardSec_);
      if (scanPause()) return true;
    }
    return false;
  };

  enterState(DriveState::Driving, "TimedGrid section");

 if (id == 1) {
  executeTimedCommand("forward", timedForwardSec_);  // C1 -> D1
  if (scanPause()) return;
  executeTimedCommand("forward", timedForwardSec_);  // D1 -> E1
  if (scanPause()) return;
  executeTimedCommand("left", timedTurn90Sec_);      // face north
  if (forwardSteps(3)) return;                       // E1 -> E2 -> E3 -> E4
  executeTimedCommand("left", timedTurn90Sec_);      // face west
  executeTimedCommand("forward", timedForwardSec_);  // E4 -> D4
  if (scanPause()) return;
  enterState(DriveState::Stopped, "TimedGrid complete 1");
} else if (id == 2) {
  // Already facing left → forward to B1, then A1
  executeTimedCommand("forward", timedForwardSec_);  // C1 → B1
  if (scanPause()) return;
  executeTimedCommand("forward", timedForwardSec_);  // B1 → A1
  if (scanPause()) return;
  executeTimedCommand("right", timedTurn90Sec_);     // face up
  if (forwardSteps(3)) return;                       // A1 → A2 → A3 → A4
  executeTimedCommand("right", timedTurn90Sec_);     // face right
  if (forwardSteps(1)) return;                       // A4 → B4
  executeTimedCommand("right", timedTurn90Sec_);     // face down
  executeTimedCommand("forward", timedForwardSec_);  // B4 → C4 (half cell if needed)
  if (scanPause()) return;
  enterState(DriveState::Stopped, "TimedGrid complete 2");
 } else if (id == 3) {
    // ID3: C1 -> C2 -> D2 -> D3 -> C3 -> B3 -> B2
    executeTimedCommand("forward", timedForwardSec_);
    if (scanPause()) return;
    executeTimedCommand("right", timedTurn90Sec_);
    executeTimedCommand("forward", timedForwardSec_);
    if (scanPause()) return;
    executeTimedCommand("left", timedTurn90Sec_);
    executeTimedCommand("forward", timedForwardSec_);
    if (scanPause()) return;
    executeTimedCommand("left", timedTurn90Sec_);
    executeTimedCommand("forward", timedForwardSec_);
    if (scanPause()) return;
    executeTimedCommand("forward", timedForwardSec_);
    if (scanPause()) return;
    executeTimedCommand("left", timedTurn90Sec_);
    enterState(DriveState::Stopped, "TimedGrid complete 3");
  } else {
    log_.logMsg("TimedGrid section: unknown DEVICE_ID");
    enterState(DriveState::Stopped, "TimedGrid invalid id");
  }

  motors_.brake();
  enabled_ = false;
}

bool GridNavigator::updatePose(unsigned long now) {
  SensorStatus s = sensors_.status();
  if (!s.mpuReady || !s.mpuYawValid) {
    status_.pose.valid = false;
    return false;
  }

  float dL = 0.0f;
  float dR = 0.0f;
  float dCenter = 0.0f;
  if (!useTimed_) {
    EncoderStatus enc = motors_.getEncoderStatus();
    long dA = enc.aTicks - lastEncATicks_;
    long dB = enc.bTicks - lastEncBTicks_;
    lastEncATicks_ = enc.aTicks;
    lastEncBTicks_ = enc.bTicks;

    dL = dA * config::MM_PER_TICK;
    dR = dB * config::MM_PER_TICK;
    dCenter = (dL + dR) * 0.5f;
  } else {
    if (status_.state == DriveState::Driving) {
      float dt = (now - lastPoseMs_) / 1000.0f;
      if (dt > 0.0f) {
        dCenter = config::SPEED_MM_PER_SEC * dt;
      }
      lastPoseMs_ = now;
    } else {
      dCenter = 0.0f;
    }
    dL = dCenter;
    dR = dCenter;
  }

  float heading = wrapHeading(s.mpuYawDeg);
  float headingRad = heading * (PI / 180.0f);
  status_.pose.xMm += dCenter * cosf(headingRad);
  status_.pose.yMm += dCenter * sinf(headingRad);
  if (fabsf(status_.pose.xMm) < 2.0f) status_.pose.xMm = 0.0f;
  if (fabsf(status_.pose.yMm) < 2.0f) status_.pose.yMm = 0.0f;
  status_.pose.headingDeg = heading;
  status_.pose.valid = true;
  (void)now;
  return true;
}

bool GridNavigator::checkWithinGrid(const Pose& pose) const {
  const float eps = 2.0f;
  if (pose.xMm < -eps) return false;
  if (pose.yMm < -eps) return false;
  if (pose.xMm > config::MAX_X_MM) return false;
  if (pose.yMm > config::MAX_Y_MM) return false;
  return true;
}

bool GridNavigator::predictBounds(float headingDeg, float distanceMm) const {
  float hRad = headingDeg * (PI / 180.0f);
  Pose predicted = status_.pose;
  predicted.xMm += distanceMm * cosf(hRad);
  predicted.yMm += distanceMm * sinf(hRad);
  if (millis() - lastLogMs_ > 500) {
    log_.logFmt(
        "Grid predict x=%.1f y=%.1f margin=%.1f",
        predicted.xMm,
        predicted.yMm,
        config::BOUNDARY_MARGIN_MM);
    lastLogMs_ = millis();
  }
  if (predicted.xMm < config::BOUNDARY_MARGIN_MM) return false;
  if (predicted.yMm < config::BOUNDARY_MARGIN_MM) return false;
  if (predicted.xMm > (config::MAX_X_MM - config::BOUNDARY_MARGIN_MM)) return false;
  if (predicted.yMm > (config::MAX_Y_MM - config::BOUNDARY_MARGIN_MM)) return false;
  return true;
}

GridTarget GridNavigator::cellCenter(const GridCell& cell) const {
  GridTarget target;
  target.xMm = (cell.cx + 0.5f) * config::CELL_WIDTH_MM;
  target.yMm = (cell.cy + 0.5f) * config::CELL_HEIGHT_MM;
  return target;
}

GridCell GridNavigator::nextCell(const GridCell& current, RowDirection& dir) const {
  GridCell next = current;
  if (dir == RowDirection::LeftToRight) {
    if (next.cx < config::GRID_WIDTH - 1) {
      next.cx++;
    } else {
      next.cy++;
      dir = RowDirection::RightToLeft;
    }
  } else {
    if (next.cx > 0) {
      next.cx--;
    } else {
      next.cy++;
      dir = RowDirection::LeftToRight;
    }
  }
  return next;
}

GridCell GridNavigator::nextCellConcentric() {
  const int maxCells = config::GRID_WIDTH * config::GRID_HEIGHT;
  if (visitedCount_ >= maxCells) {
    GridCell done = status_.cell;
    done.cy = config::GRID_HEIGHT;
    return done;
  }

  if (!spiralInitialized_) {
    startCell_ = status_.cell;
    spiralInitialized_ = true;
    spiralDir_ = 0;
    spiralLegLen_ = 1;
    spiralLegProgress_ = 0;
    spiralLegsDone_ = 0;
    int idx = startCell_.cy * config::GRID_WIDTH + startCell_.cx;
    if (idx >= 0 && idx < maxCells) {
      visited_[idx] = true;
      visitedCount_ = 1;
    }
  }

  GridCell candidate = targetCell_;
  if (visitedCount_ <= 1) {
    candidate = startCell_;
  }
  int attempts = 0;
  while (attempts < maxCells * 4) {
    switch (spiralDir_) {
      case 0: candidate.cx += 1; break;
      case 1: candidate.cy += 1; break;
      case 2: candidate.cx -= 1; break;
      case 3: candidate.cy -= 1; break;
    }
    spiralLegProgress_++;
    if (spiralLegProgress_ >= spiralLegLen_) {
      spiralLegProgress_ = 0;
      spiralDir_ = (spiralDir_ + 1) % 4;
      spiralLegsDone_++;
      if (spiralLegsDone_ % 2 == 0) {
        spiralLegLen_++;
      }
    }

    if (isCellInBounds(candidate)) {
      int idx = candidate.cy * config::GRID_WIDTH + candidate.cx;
      if (!visited_[idx]) {
        return candidate;
      }
    }
    attempts++;
  }

  GridCell done = status_.cell;
  done.cy = config::GRID_HEIGHT;
  return done;
}

bool GridNavigator::isCellInBounds(const GridCell& cell) const {
  return cell.cx >= 0 && cell.cy >= 0 && cell.cx < config::GRID_WIDTH && cell.cy < config::GRID_HEIGHT;
}

void GridNavigator::resetTraversal() {
  int total = config::GRID_WIDTH * config::GRID_HEIGHT;
  for (int i = 0; i < total; i++) {
    visited_[i] = false;
  }
  visitedCount_ = 0;
  spiralInitialized_ = false;
  spiralDir_ = 0;
  spiralLegLen_ = 1;
  spiralLegProgress_ = 0;
  spiralLegsDone_ = 0;
}

void GridNavigator::enterState(DriveState state, const char* reason) {
  if (status_.state != state) {
    log_.logFmt(
        "Grid state -> %d (%s) t=%lu x=%.1f y=%.1f h=%.1f",
        (int)state,
        reason,
        millis(),
        status_.pose.xMm,
        status_.pose.yMm,
        status_.pose.headingDeg);
  }
  status_.state = state;
}

void GridNavigator::setTarget(const GridTarget& target, const GridCell& cell, bool finalTarget) {
  target_ = target;
  targetCell_ = cell;
  haveTarget_ = true;
  targetIsFinal_ = finalTarget;
}

bool GridNavigator::targetReached(const GridTarget& target) const {
  float dx = target.xMm - status_.pose.xMm;
  float dy = target.yMm - status_.pose.yMm;
  float dist = sqrtf(dx * dx + dy * dy);
  return dist <= config::POSITION_TOLERANCE_MM;
}

float GridNavigator::headingErrorDeg(float targetHeading) const {
  float err = wrapHeading(targetHeading - status_.pose.headingDeg);
  return err;
}

void GridNavigator::rotateToHeading(float targetHeadingDeg) {
  float err = headingErrorDeg(targetHeadingDeg);
  SensorStatus s = sensors_.status();
  float angularVel = s.mpuYawRateDps;
  if (millis() - lastLogMs_ > 200) {
    log_.logFmt("Grid rotate target=%.1f err=%.1f rate=%.1f", targetHeadingDeg, err, angularVel);
    lastLogMs_ = millis();
  }
  if (fabsf(err) <= config::HEADING_TOLERANCE_DEG && fabsf(angularVel) < 5.0f) {
    motors_.brake();
    enterState(DriveState::Stopped, "Aligned");
    return;
  }

  static PidController pid(1.0f, 0.01f, 0.5f);
  float output = pid.compute(err, millis());
  const float maxTurn = 1992.0f;
  const float minTurn = 1392.0f;
  if (output > maxTurn) output = maxTurn;
  if (output < -maxTurn) output = -maxTurn;
  if (fabsf(output) < minTurn && fabsf(err) > config::HEADING_TOLERANCE_DEG) {
    output = (output >= 0.0f) ? minTurn : -minTurn;
  }
  if (useTimed_) {
    output *= 0.1f;
  } 

  enterState(DriveState::Rotating, "Rotating");
  MotorState turnState = (output >= 0.0f) ? MotorState::TurnLeft : MotorState::TurnRight;
  motors_.driveRaw((int)(-output * config::ALIGN_A), (int)(output * config::ALIGN_B), turnState);
  if (millis() - lastLogMs_ > 200) {
    log_.logFmt("Grid turn out=%.1f state=%s", output, motorStateLabel(turnState));
    lastLogMs_ = millis();
  }
}

void GridNavigator::driveToTarget(const GridTarget& target) {
  float dx = target.xMm - status_.pose.xMm;
  float dy = target.yMm - status_.pose.yMm;
  float targetHeading = atan2f(dy, dx) * (180.0f / PI);
  float err = headingErrorDeg(targetHeading);
  float dist = sqrtf(dx * dx + dy * dy);
  if (millis() - lastLogMs_ > 500) {
    log_.logFmt("Grid drive dist=%.1f err=%.1f", dist, err);
    lastLogMs_ = millis();
  }

  if (fabsf(err) > config::HEADING_TOLERANCE_DEG) {
    rotateToHeading(targetHeading);
    return;
  }

  if (!predictBounds(targetHeading, dist)) {
    enterState(DriveState::Error, "Boundary pre-check");
    motors_.brake();
    return;
  }

  enterState(DriveState::Driving, "Driving");
  float correction = config::HEADING_KP * err;
  int leftSpeed = (int)(config::DRIVE_SPEED - correction);
  int rightSpeed = (int)(config::DRIVE_SPEED + correction);
  motors_.driveRaw(leftSpeed * config::ALIGN_A, rightSpeed * config::ALIGN_B, MotorState::Forward);
}
