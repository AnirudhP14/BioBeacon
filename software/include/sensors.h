#ifndef SWARMBOT_SENSORS_H
#define SWARMBOT_SENSORS_H

#include <Arduino.h>
#include <Adafruit_SGP30.h>

class Logger;

enum class SgpType {
  None,
  Sgp30,
  Sgp40
};

struct SensorStatus {
  bool tofReady = false;
  bool tofValid = false;
  uint16_t tofMm = 0;
  unsigned long tofAgeMs = 0;
  unsigned long tofTimeoutCount = 0;
  bool tofInitOk = false;

  bool sgpReady = false;
  SgpType sgpType = SgpType::None;
  unsigned long sgpAgeMs = 0;
  bool sgpHasRaw = false;
  bool sgpHasAir = false;
  uint16_t sgpRaw = 0;
  uint16_t sgpCO2 = 0;
  uint16_t sgpTVOC = 0;

  bool mpuReady = false;
  uint8_t mpuWhoAmI = 0;
  uint8_t mpuAddr = 0;
  unsigned long mpuAgeMs = 0;
  float mpuAxG = 0.0f;
  float mpuAyG = 0.0f;
  float mpuAzG = 0.0f;
  float mpuGxDps = 0.0f;
  float mpuGyDps = 0.0f;
  float mpuGzDps = 0.0f;
  float mpuYawRateDps = 0.0f;
  float mpuTempC = 0.0f;
  float mpuYawDeg = 0.0f;
  bool mpuYawValid = false;

  unsigned long i2cScanCount = 0;
};

class SensorManager {
 public:
  SensorManager(Logger& log);
  void begin(bool runDiagnostics);
  void poll();
  void pollToF();
  void pollSGP();
  void scanI2C();
  SensorStatus status() const;

  bool hasTof() const;
  uint16_t tofMm() const;
  bool hasAir() const;
  uint16_t sgpCO2() const;
  uint16_t sgpTVOC() const;

 private:
  void setupToF();
  void setupSGP();
  void setupMPU6050();
  void pollMPU6050();
  bool mpuReadSample();
  void updateYaw(unsigned long now);

  Logger& log_;
  SensorStatus status_;
  unsigned long lastTofMs_ = 0;
  unsigned long lastSgpMs_ = 0;
  unsigned long lastMpuMs_ = 0;
  unsigned long lastYawMs_ = 0;
  uint16_t lastTofMm_ = 0;
  Adafruit_SGP30 sgp_;
  unsigned long sgpNextInitMs_ = 0;
  unsigned long sgpWarmupStartMs_ = 0;
  unsigned int sgpFailCount_ = 0;
};

#endif  // SWARMBOT_SENSORS_H
