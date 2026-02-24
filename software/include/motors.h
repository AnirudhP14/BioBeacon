#ifndef SWARMBOT_MOTORS_H
#define SWARMBOT_MOTORS_H

#include <Arduino.h>

class Logger;

enum class MotorState {
  Idle,
  Forward,
  Backward,
  Brake,
  TurnLeft,
  TurnRight
};

const char* motorStateLabel(MotorState state);

struct EncoderStatus {
  long aTicks = 0;
  long bTicks = 0;
  float aDistMm = 0.0f;
  float bDistMm = 0.0f;
  float avgDistMm = 0.0f;
};

class MotorDriver {
 public:
  void begin(Logger& log);
  void forward(int speed);
  void backward(int speed);
  void brake();
  void turnLeft(int speed);
  void turnRight(int speed);
  void driveRaw(int leftSpeed, int rightSpeed, MotorState state);

  EncoderStatus getEncoderStatus() const;
  unsigned long commandCount() const;
  unsigned long lastCommandMs() const;
  MotorState lastCommandState() const;

 private:
  void driveMotors(int m1, int m2, MotorState state, const char* label);
  Logger* log_ = nullptr;
  unsigned long commandCount_ = 0;
  unsigned long lastCmdMs_ = 0;
  MotorState lastCmdState_ = MotorState::Idle;
};

#endif  // SWARMBOT_MOTORS_H
