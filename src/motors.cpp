#include "motors.h"

#include <TB6612_ESP32.h>
#include "config.h"
#include "logging.h"

namespace {
volatile long encoderATicks = 0;
volatile long encoderBTicks = 0;

Motor motor1(config::AIN1, config::AIN2, config::PWMA, config::OFFSET_A, config::STBY, 5000, 8, 0);
Motor motor2(config::BIN1, config::BIN2, config::PWMB, config::OFFSET_B, config::STBY, 5000, 8, 1);

void IRAM_ATTR encoderAIsr() {
  int a = digitalRead(config::ENCA_C1);
  int b = digitalRead(config::ENCA_C2);
  encoderATicks += (a == b) ? 1 : -1;
}

void IRAM_ATTR encoderBIsr() {
  int a = digitalRead(config::ENCB_C1);
  int b = digitalRead(config::ENCB_C2);
  encoderBTicks += (a == b) ? -1 : 1;
}
}  // namespace

const char* motorStateLabel(MotorState state) {
  switch (state) {
    case MotorState::Idle: return "idle";
    case MotorState::Forward: return "forward";
    case MotorState::Backward: return "backward";
    case MotorState::Brake: return "brake";
    case MotorState::TurnLeft: return "turn_left";
    case MotorState::TurnRight: return "turn_right";
    default: return "unknown";
  }
}

void MotorDriver::begin(Logger& log) {
  log_ = &log;
  pinMode(config::ENCA_C1, INPUT_PULLUP);
  pinMode(config::ENCA_C2, INPUT_PULLUP);
  pinMode(config::ENCB_C1, INPUT_PULLUP);
  pinMode(config::ENCB_C2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(config::ENCA_C1), encoderAIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(config::ENCB_C1), encoderBIsr, CHANGE);
  log.logMsg("Encoders initialized");
}

void MotorDriver::driveMotors(int m1, int m2, MotorState state, const char* label) {
  if (log_) {
    log_->logFmt(label, m1, m2);
  }
  motor1.drive(m1);
  motor2.drive(m2);
  commandCount_++;
  lastCmdMs_ = millis();
  lastCmdState_ = state;
}

void MotorDriver::forward(int speed) {
  int s = speed;
  if (s < 0) s = -s;
  if (s > 255) s = 255;
  int m1 = s * config::ALIGN_A;
  int m2 = s * config::ALIGN_B;
  driveMotors(m1, m2, MotorState::Forward, "Motor cmd forward m1=%d m2=%d");
}

void MotorDriver::backward(int speed) {
  int s = speed;
  if (s < 0) s = -s;
  if (s > 255) s = 255;
  int m1 = -s * config::ALIGN_A;
  int m2 = -s * config::ALIGN_B;
  driveMotors(m1, m2, MotorState::Backward, "Motor cmd backward m1=%d m2=%d");
}

void MotorDriver::brake() {
  motor1.brake();
  motor2.brake();
  commandCount_++;
  lastCmdMs_ = millis();
  lastCmdState_ = MotorState::Brake;
}

void MotorDriver::turnLeft(int speed) {
  int s = speed;
  if (s < 0) s = -s;
  if (s > 255) s = 255;
  int m1 = -s * config::ALIGN_A;
  int m2 = s * config::ALIGN_B;
  driveMotors(m1, m2, MotorState::TurnLeft, "Motor cmd left turn m1=%d m2=%d");
}

void MotorDriver::turnRight(int speed) {
  int s = speed;
  if (s < 0) s = -s;
  if (s > 255) s = 255;
  int m1 = s * config::ALIGN_A;
  int m2 = -s * config::ALIGN_B;
  driveMotors(m1, m2, MotorState::TurnRight, "Motor cmd right turn m1=%d m2=%d");
}

void MotorDriver::driveRaw(int leftSpeed, int rightSpeed, MotorState state) {
  int m1 = leftSpeed;
  int m2 = rightSpeed;
  if (m1 > 255) m1 = 255;
  if (m1 < -255) m1 = -255;
  if (m2 > 255) m2 = 255;
  if (m2 < -255) m2 = -255;
  driveMotors(m1, m2, state, "Motor cmd raw m1=%d m2=%d");
}

EncoderStatus MotorDriver::getEncoderStatus() const {
  EncoderStatus status;
  noInterrupts();
  status.aTicks = encoderATicks;
  status.bTicks = encoderBTicks;
  interrupts();
  status.aDistMm = status.aTicks * config::MM_PER_TICK;
  status.bDistMm = status.bTicks * config::MM_PER_TICK;
  status.avgDistMm = (status.aDistMm + status.bDistMm) * 0.5f;
  return status;
}

unsigned long MotorDriver::commandCount() const {
  return commandCount_;
}

unsigned long MotorDriver::lastCommandMs() const {
  return lastCmdMs_;
}

MotorState MotorDriver::lastCommandState() const {
  return lastCmdState_;
}
