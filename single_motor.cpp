#include <Arduino.h>
#include <WiFi.h>

#include "comms.h"
#include "config.h"
#include "diagnostics.h"
#include "drive.h"
#include "grid_nav.h"
#include "led_controller.h"
#include "logging.h"
#include "motors.h"
#include "sensors.h"
#include "status_builder.h"
#include "web.h"

Logger logger;
LedController led(logger);
MotorDriver motors;
SensorManager sensors(logger);
DriveController drive(motors, logger);
GridNavigator gridNav(motors, sensors, logger);
EspNowManager comms(logger, motors, sensors, &led, &gridNav);
StatusBuilder statusBuilder(sensors, motors, drive, comms, gridNav);
WebServerManager web(logger, drive, statusBuilder, gridNav, comms);
Diagnostics diagnostics(logger, motors, sensors);

char wifiSsid[32];

void setupWiFi() {
  snprintf(wifiSsid, sizeof(wifiSsid), "%s-%u", config::WIFI_SSID_BASE, config::DEVICE_ID);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(wifiSsid, config::WIFI_PASS);

  IPAddress ip = WiFi.softAPIP();
  logger.logMsg("WiFi AP started");
  logger.logFmt("SSID: %s", wifiSsid);
  logger.logFmt("IP: %s", ip.toString().c_str());
}

void setup() {
  Serial.println("Booting...");
  delay(500);

  logger.logMsg("==== BOOT ====");
  logger.logFmt("Reset reason: %d", esp_reset_reason());
  randomSeed((uint32_t)ESP.getEfuseMac());

  setupWiFi();
  comms.begin();
  web.begin();

  if (!led.begin()) {
    logger.logMsg("LED init failed");
    led.disable();
    logger.logMsg("LED controller disabled");
  }
  led.setBooting();
  // digitalWrite(config::LED_PIN_R, 255);
  // digitalWrite(config::LED_PIN_G, 0);
  // digitalWrite(config::LED_PIN_B, 0);

  motors.begin(logger);
  if (config::RUN_BOOT_DIAGNOSTICS) {
    diagnostics.logBootPins();
  }
  sensors.begin(config::RUN_BOOT_DIAGNOSTICS);
  gridNav.begin();

  logger.logMsg("Initializing motors");
  motors.forward(10);
  motors.brake();
  logger.logMsg("Motors initialized");

  if (config::RUN_BOOT_DIAGNOSTICS) {
    diagnostics.motorSelfTest();
  }

  led.setReady();
  logger.logMsg("Setup complete");
}

void loop() {
  web.handleClient();
  sensors.poll();

  unsigned long now = millis();
  comms.update(now);
  if (config::ENABLE_GRID_NAV && gridNav.isEnabled()) {
    gridNav.update(now);
  } else {
    drive.setAvoid(comms.isAvoiding(now));
    drive.update(now);
  }

  SensorStatus s = sensors.status();
  static bool lastDriveActive = false;
  bool driveActive =
      (config::ENABLE_GRID_NAV && gridNav.isEnabled()) ||
      (drive.status().driveMode == DriveMode::Path);
  if (driveActive != lastDriveActive) {
    if (driveActive) {
      led.setDriveMode();
      led.setStop(false);
    } else {
      led.setReady();
    }
    lastDriveActive = driveActive;
  }
  static bool co2Latched = false;
  if (s.sgpHasAir) {
    if (!co2Latched && s.sgpCO2 > 1000) co2Latched = true; //1000 for artifcial co2 source
    if (co2Latched && s.sgpCO2 < 800) co2Latched = false; //800 for turning off
  }
  led.setCO2High(web.co2TestEnabled() ? true : co2Latched);
  CommsStatus commsStatus = comms.status();
  led.setRemoteCO2(commsStatus.remoteCo2Alert);
  if (gridNav.isScanStopLatched()) {
    led.setStop(true);
  }
  static bool scanStopSent = false;
  if (!gridNav.isScanStopLatched()) {
    scanStopSent = false;
  } else if (!scanStopSent) {
    if (gridNav.scanStopTofMm() > 0 && gridNav.scanStopTofMm() < config::SCAN_TOF_STOP_MM) {
      comms.sendStop();
      logger.logFmt(
          "Grid scan ToF stop after %.1fs (ToF=%u mm)",
          gridNav.scanStopElapsedSec(),
          gridNav.scanStopTofMm());
    } else if (gridNav.scanStopPpm() > config::SCAN_CO2_STOP_PPM) {
      comms.sendStop();
      logger.logFmt(
          "Grid scan CO2 stop after %.1fs (CO2=%u ppm)",
          gridNav.scanStopElapsedSec(),
          gridNav.scanStopPpm());
    }
    scanStopSent = true;
  }
  led.update(now);
}
