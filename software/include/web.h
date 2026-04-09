#ifndef SWARMBOT_WEB_H
#define SWARMBOT_WEB_H

#include <WebServer.h>

class Logger;
class DriveController;
class StatusBuilder;
class GridNavigator;
class EspNowManager;
class DashboardState;

class WebServerManager {
 public:
  WebServerManager(
      Logger& log,
      DriveController& drive,
      StatusBuilder& statusBuilder,
      GridNavigator& gridNav,
      EspNowManager& comms,
      DashboardState& dashboard);
  void begin();
  void handleClient();
  bool co2TestEnabled() const;

 private:
  void handleRoot();
  void handleDashboardPage();
  void handleOpsDashboard();
  void handleChartAsset();
  void handleStatus();
  void handleClear();
  void handleDrive();
  void handleSwarmTimedGridScan();

  Logger& log_;
  DriveController& drive_;
  StatusBuilder& statusBuilder_;
  GridNavigator& gridNav_;
  EspNowManager& comms_;
  DashboardState& dashboard_;
  WebServer server_;
  bool co2TestMode_ = false;
};

#endif  // SWARMBOT_WEB_H
