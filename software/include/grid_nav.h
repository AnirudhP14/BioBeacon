#ifndef SWARMBOT_GRID_NAV_H
#define SWARMBOT_GRID_NAV_H

#include <Arduino.h>
#include "config.h"

struct Pose {
  float xMm = 0.0f;
  float yMm = 0.0f;
  float headingDeg = 0.0f;
  bool valid = false;
};

enum class DriveState {
  Stopped,
  Rotating,
  Driving,
  Error
};

enum class RowDirection {
  LeftToRight,
  RightToLeft
};

enum class TraversalPattern {
  LawnMower,
  Concentric
};

enum class EndPoint {
  EndE,
  EndC,
  EndA
};

struct GridCell {
  int cx = 0;
  int cy = 0;
};

struct GridTarget {
  float xMm = 0.0f;
  float yMm = 0.0f;
};

struct GridStatus {
  Pose pose;
  GridCell cell;
  RowDirection rowDir = RowDirection::LeftToRight;
  TraversalPattern pattern = TraversalPattern::LawnMower;
  DriveState state = DriveState::Stopped;
};

class MotorDriver;
class SensorManager;
class Logger;

class GridNavigator {
 public:
  GridNavigator(MotorDriver& motors, SensorManager& sensors, Logger& log);

  void begin();
  void update(unsigned long now);
  GridStatus status() const;

  void setEnabled(bool enabled);
  bool isEnabled() const;
  void setCell(const GridCell& cell);
  void setRowDirection(RowDirection dir);
  void setTraversal(TraversalPattern pattern);
  void setUseTimed(bool use);
  void setStartCellOverride(const GridCell& cell);
  void setTimedGrid(bool enable) { useTimedGrid_ = enable; }
  bool isTimedGrid() const { return useTimedGrid_; }
  void setScanMode(bool enable) { scanMode_ = enable; }
  void setEndPoint(EndPoint end) { endPoint_ = end; }
  bool isScanStopLatched() const { return scanStopLatched_; }
  float scanStopElapsedSec() const;
  uint16_t scanStopPpm() const { return scanStopPpm_; }
  uint16_t scanStopTofMm() const { return scanStopTofMm_; }
  void setTimedForwardSec(float seconds);
  void setTimedTurn90Sec(float seconds);
  void setTimedDriveSpeed(int speed);

 private:
  bool updatePose(unsigned long now);
  bool checkWithinGrid(const Pose& pose) const;
  bool predictBounds(float headingDeg, float distanceMm) const;

  GridTarget cellCenter(const GridCell& cell) const;
  GridCell nextCell(const GridCell& current, RowDirection& dir) const;
  GridCell nextCellConcentric();
  bool isCellInBounds(const GridCell& cell) const;
  void resetTraversal();
  void executeTimedCommand(const char* cmd, float durationSec);
  void traverseSection(int id);

  void enterState(DriveState state, const char* reason);
  void rotateToHeading(float targetHeadingDeg);
  void driveToTarget(const GridTarget& target);
  void setTarget(const GridTarget& target, const GridCell& cell, bool finalTarget);
  bool targetReached(const GridTarget& target) const;
  float headingErrorDeg(float targetHeading) const;

  MotorDriver& motors_;
  SensorManager& sensors_;
  Logger& log_;
  bool enabled_ = false;
  GridStatus status_;
  GridTarget target_;
  GridCell targetCell_;
  bool haveTarget_ = false;
  bool targetIsFinal_ = true;
  bool homing_ = false;
  bool homingFirstLeg_ = false;
  GridTarget homingFinal_;
  GridCell startCell_;
  mutable unsigned long lastLogMs_ = 0;
  bool useTimed_ = false;
  bool useTimedGrid_ = false;
  bool scanMode_ = false;
  bool scanStopLatched_ = false;
  EndPoint endPoint_ = EndPoint::EndE;
  unsigned long gridStartMs_ = 0;
  unsigned long scanStopMs_ = 0;
  uint16_t scanStopPpm_ = 0;
  uint16_t scanStopTofMm_ = 0;
  float timedForwardSec_ = config::TIMED_FORWARD_SEC_PER_CELL;
  float timedTurn90Sec_ = config::TIMED_TURN_90_SEC;
  int timedDriveSpeed_ = config::TIMED_DRIVE_SPEED;
  unsigned long lastPoseMs_ = 0;
  bool spiralInitialized_ = false;
  uint8_t spiralDir_ = 0;
  int spiralLegLen_ = 1;
  int spiralLegProgress_ = 0;
  int spiralLegsDone_ = 0;
  int visitedCount_ = 0;
  bool visited_[config::GRID_WIDTH * config::GRID_HEIGHT] = {false};
  long lastEncATicks_ = 0;
  long lastEncBTicks_ = 0;
};

#endif  // SWARMBOT_GRID_NAV_H
