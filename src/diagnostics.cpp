#include "diagnostics.h"

#include "config.h"
#include "logging.h"
#include "motors.h"
#include "sensors.h"

Diagnostics::Diagnostics(Logger& log, MotorDriver& motors, SensorManager& sensors)
    : log_(log), motors_(motors), sensors_(sensors) {}

void Diagnostics::runBootDiagnostics() {
  logBootPins();
  motorSelfTest();
}

void Diagnostics::logBootPins() {
  log_.logFmt(
      "Pins AIN1=%d AIN2=%d PWMA=%d BIN1=%d BIN2=%d PWMB=%d STBY=%d",
      config::AIN1, config::AIN2, config::PWMA, config::BIN1, config::BIN2, config::PWMB, config::STBY);
  log_.logFmt("Motor align A=%d B=%d", config::ALIGN_A, config::ALIGN_B);
  log_.logFmt(
      "Encoder pins A=%d/%d B=%d/%d",
      config::ENCA_C1, config::ENCA_C2, config::ENCB_C1, config::ENCB_C2);
}

void Diagnostics::motorSelfTest() {
  log_.logMsg("Motor self-test begin");
  motors_.forward(120);
  delay(200);
  motors_.backward(120);
  delay(200);
  motors_.brake();
  log_.logMsg("Motor self-test end");
}
