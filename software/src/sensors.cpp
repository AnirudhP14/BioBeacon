#include "sensors.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_SGP30.h>
#include <VL53L0X.h>
#include <Wire.h>
#include "config.h"
#include "logging.h"

namespace {
VL53L0X tof;
Adafruit_MPU6050 mpu;
}  // namespace

SensorManager::SensorManager(Logger& log) : log_(log) {}

void SensorManager::begin(bool runDiagnostics) {
  Wire.begin(SDA, SCL);
  Wire.setClock(100000);
  log_.logMsg("I2C started");
  if (runDiagnostics) {
    scanI2C();
  }
  setupToF();
  setupSGP();
  setupMPU6050();
}

void SensorManager::poll() {
  pollToF();
  pollSGP();
  pollMPU6050();
  updateYaw(millis());
}

void SensorManager::scanI2C() {
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      log_.logFmt("I2C device at 0x%02X", addr);
      found++;
    }
  }
  status_.i2cScanCount++;
  log_.logFmt("I2C scan complete, found=%u", found);
}

SensorStatus SensorManager::status() const {
  SensorStatus out = status_;
  if (status_.tofValid) {
    out.tofAgeMs = millis() - lastTofMs_;
  }
  if (status_.sgpReady) {
    out.sgpAgeMs = millis() - lastSgpMs_;
  }
  if (status_.mpuReady) {
    out.mpuAgeMs = millis() - lastMpuMs_;
  }
  return out;
}

bool SensorManager::hasTof() const {
  return status_.tofValid;
}

uint16_t SensorManager::tofMm() const {
  return lastTofMm_;
}

bool SensorManager::hasAir() const {
  return status_.sgpHasAir;
}

uint16_t SensorManager::sgpCO2() const {
  return status_.sgpCO2;
}

uint16_t SensorManager::sgpTVOC() const {
  return status_.sgpTVOC;
}

void SensorManager::setupToF() {
  if (!tof.init()) {
    log_.logMsg("VL53L0X NOT detected");
    status_.tofInitOk = false;
    status_.tofReady = false;
    return;
  }

  tof.setTimeout(500);
  tof.setMeasurementTimingBudget(20000);
  tof.startContinuous();

  status_.tofReady = true;
  status_.tofInitOk = true;
  log_.logMsg("VL53L0X ready");
}

void SensorManager::pollToF() {
  static unsigned long lastRead = 0;
  if (!status_.tofReady) return;
  if (millis() - lastRead <= 200) return;
  lastRead = millis();

  uint16_t mm = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred()) {
    status_.tofValid = false;
    status_.tofTimeoutCount++;
    log_.logFmt("VL53L0X timeout (raw=%u)", mm);
  } else {
    char buf[64];
    sprintf(buf, "ToF: %u mm", mm);
    log_.logMsg(buf);
    lastTofMm_ = mm;
    lastTofMs_ = millis();
    status_.tofValid = true;
    status_.tofMm = mm;
  }
}

void SensorManager::setupSGP() {
  status_.sgpReady = false;
  status_.sgpType = SgpType::None;
  status_.sgpHasAir = false;
  status_.sgpHasRaw = false;

  if (!sgp_.begin(&Wire)) {
    log_.logMsg("SGP30 init failed");
    sgpNextInitMs_ = millis() + 2000;
    return;
  }

  // SGP30 requires local decoupling capacitors (0.1 uF + 10 uF) at VCC/GND for reliable IAQ on robots.
  if (!sgp_.IAQinit()) {
    log_.logMsg("SGP30 IAQ init failed");
    sgpNextInitMs_ = millis() + 2000;
    return;
  }

  status_.sgpReady = true;
  status_.sgpType = SgpType::Sgp30;
  sgpFailCount_ = 0;
  sgpWarmupStartMs_ = millis();
  log_.logMsg("SGP30 initialized");
}

void SensorManager::pollSGP() {
  static unsigned long lastRead = 0;
  unsigned long now = millis();
  if (!status_.sgpReady) {
    if (now >= sgpNextInitMs_) {
      setupSGP();
    }
    return;
  }
  if (now - lastRead < 1000) return;
  lastRead = now;

  bool warmup = (now - sgpWarmupStartMs_) < 20000;
  if (warmup) {
    unsigned long remaining = (20000 - (now - sgpWarmupStartMs_)) / 1000;
    log_.logFmt("SGP30 warming up (%lus remaining)", remaining);
  }

  if (sgp_.IAQmeasure()) {
    status_.sgpCO2 = sgp_.eCO2;
    status_.sgpTVOC = sgp_.TVOC;
    status_.sgpHasAir = true;
    lastSgpMs_ = now;
    sgpFailCount_ = 0;
    log_.logFmt("SGP30 OK eCO2=%u TVOC=%u", status_.sgpCO2, status_.sgpTVOC);
  } else {
    log_.logMsg("SGP30 IAQmeasure() failed");
    if (!warmup) {
      sgpFailCount_++;
      if (sgpFailCount_ >= 5) {
        sgpFailCount_ = 0;
        if (sgp_.IAQinit()) {
          sgpWarmupStartMs_ = now;
          log_.logMsg("SGP30 re-init");
        } else {
          log_.logMsg("SGP30 re-init failed");
        }
      }
    }
  }
}

bool mpuReadWhoAmI(uint8_t addr, uint8_t& who) {
  Wire.beginTransmission(addr);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  who = Wire.read();
  return true;
}

void SensorManager::setupMPU6050() {
  if (mpu.begin(0x68, &Wire)) {
    status_.mpuAddr = 0x68;
  } else if (mpu.begin(0x69, &Wire)) {
    status_.mpuAddr = 0x69;
  } else {
    status_.mpuReady = false;
    log_.logMsg("MPU6050 not found");
    return;
  }

  if (mpuReadWhoAmI(status_.mpuAddr, status_.mpuWhoAmI)) {
    log_.logFmt("MPU WHO_AM_I = 0x%02X", status_.mpuWhoAmI);
  } else {
    status_.mpuWhoAmI = 0;
    log_.logMsg("MPU WHO_AM_I read failed");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  status_.mpuReady = true;
  if (!mpuReadSample()) {
    status_.mpuReady = false;
    log_.logMsg("MPU6050 read failed");
    return;
  }

  bool accelLooksValid = !(status_.mpuAxG == 0.0f && status_.mpuAyG == 0.0f && status_.mpuAzG == 0.0f);
  bool gyroLooksValid = !(status_.mpuGxDps == 0.0f && status_.mpuGyDps == 0.0f && status_.mpuGzDps == 0.0f);
  if (!accelLooksValid || !gyroLooksValid) {
    log_.logFmt("MPU6050 validation suspect at 0x%02X", status_.mpuAddr);
  }

  log_.logFmt(
      "MPU6050 OK @0x%02X ax=%.2fg ay=%.2fg az=%.2fg temp=%.2fC",
      status_.mpuAddr, status_.mpuAxG, status_.mpuAyG, status_.mpuAzG, status_.mpuTempC);
}

void SensorManager::pollMPU6050() {
  static unsigned long lastRead = 0;
  if (!status_.mpuReady) return;
  if (millis() - lastRead < 200) return;
  lastRead = millis();
  if (!mpuReadSample()) {
    log_.logMsg("MPU6050 sample read failed");
  }
}

bool SensorManager::mpuReadSample() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu.getEvent(&accel, &gyro, &temp);
  status_.mpuAxG = accel.acceleration.x / 9.80665f;
  status_.mpuAyG = accel.acceleration.y / 9.80665f;
  status_.mpuAzG = accel.acceleration.z / 9.80665f;
  status_.mpuGxDps = gyro.gyro.x * 57.2958f;
  status_.mpuGyDps = gyro.gyro.y * 57.2958f;
  status_.mpuGzDps = gyro.gyro.z * 57.2958f;
  status_.mpuTempC = temp.temperature;
  lastMpuMs_ = millis();
  return true;
}

void SensorManager::updateYaw(unsigned long now) {
  if (!status_.mpuReady) {
    status_.mpuYawValid = false;
    return;
  }

  if (lastYawMs_ == 0) {
    lastYawMs_ = now;
    status_.mpuYawValid = true;
    return;
  }

  float dt = (now - lastYawMs_) / 1000.0f;
  lastYawMs_ = now;
  // Sensor inverted on front: use roll axis (Gx) for turn rate, invert sign.
  status_.mpuYawRateDps = -status_.mpuGxDps;
  status_.mpuYawDeg += status_.mpuYawRateDps * dt;
  if (status_.mpuYawDeg > 180.0f) status_.mpuYawDeg -= 360.0f;
  if (status_.mpuYawDeg < -180.0f) status_.mpuYawDeg += 360.0f;
  status_.mpuYawValid = true;

  static unsigned long lastLogMs = 0;
  if (now - lastLogMs > 1000) {
    log_.logFmt(
        "IMU dbg gx=%.1f gy=%.1f gz=%.1f yawRate=%.1f yaw=%.1f",
        status_.mpuGxDps,
        status_.mpuGyDps,
        status_.mpuGzDps,
        status_.mpuYawRateDps,
        status_.mpuYawDeg);
    lastLogMs = now;
  }
}
