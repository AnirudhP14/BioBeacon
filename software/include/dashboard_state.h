#ifndef SWARMBOT_DASHBOARD_STATE_H
#define SWARMBOT_DASHBOARD_STATE_H

#include <Arduino.h>

#include "comms.h"
#include "drive.h"
#include "grid_nav.h"
#include "motors.h"
#include "sensors.h"

class Logger;
class StatusBuilder;

class DashboardState {
 public:
  DashboardState(
      Logger& log,
      SensorManager& sensors,
      MotorDriver& motors,
      DriveController& drive,
      EspNowManager& comms,
      GridNavigator& gridNav,
      StatusBuilder& statusBuilder);

  void begin(unsigned long now);
  void update(unsigned long now);

  void onPeerPacket(const EspNowPacket& packet, unsigned long now);
  void onLocalCommand(const char* command, unsigned long now);
  void onLocalEvent(const char* type, const char* title, const char* detail, const char* severity, unsigned long now);
  void onScanStop(const char* reason, unsigned long now);

  String buildDashboardJson(unsigned long now) const;

 private:
  struct PeerSnapshot {
    bool active = false;
    uint8_t id = 0;
    uint8_t swarmId = 0;
    unsigned long lastSeenMs = 0;
    unsigned long lastHeartbeatMs = 0;
    float x = 0.0f;
    bool hasCO2 = false;
    float co2 = 0.0f;
    bool hasTVOC = false;
    float tvoc = 0.0f;
    bool hasTof = false;
    float tof = 0.0f;
    bool stopSignal = false;
    char lastCommand[24] = {0};
    EspNowPacket lastPacket = {};
  };

  struct TimelineEvent {
    bool used = false;
    unsigned long tsMs = 0;
    uint8_t robotId = 0;
    char type[20] = {0};
    char severity[12] = {0};
    char title[40] = {0};
    char detail[96] = {0};
  };

  struct MetricSeries {
    unsigned long ts[config::MAX_PATH_STEPS * 8] = {0};
    float value[config::MAX_PATH_STEPS * 8] = {0.0f};
    uint8_t count = 0;
    uint8_t next = 0;

    void push(unsigned long t, float v);
  };

  struct RobotCharts {
    bool active = false;
    uint8_t id = 0;
    MetricSeries eco2;
    MetricSeries tof;
    MetricSeries distance;
  };

  static constexpr uint8_t kMaxPeers = 8;
  static constexpr uint8_t kMaxEvents = 48;
  static constexpr uint8_t kMaxChartRobots = kMaxPeers + 1;
  static constexpr unsigned long kChartSampleMs = 1000;
  static constexpr unsigned long kOnlineFreshMs = 2500;
  static constexpr unsigned long kStaleMs = 8000;

  PeerSnapshot* findPeer(uint8_t id);
  const PeerSnapshot* findPeer(uint8_t id) const;
  RobotCharts* chartSeries(uint8_t id);
  const RobotCharts* chartSeries(uint8_t id) const;

  void appendEvent(
      unsigned long now,
      uint8_t robotId,
      const char* type,
      const char* severity,
      const char* title,
      const char* detail);
  void sampleCharts(unsigned long now);
  void updateDerivedEvents(unsigned long now);
  const char* peerFreshness(const PeerSnapshot& peer, unsigned long now) const;
  const char* localMissionState(unsigned long now) const;
  void appendJsonString(String& out, const char* value) const;
  void appendMetricSeries(String& out, const MetricSeries& series) const;
  void appendRobotJson(String& out, unsigned long now) const;
  void appendPeerRobotsJson(String& out, unsigned long now) const;
  void appendMapJson(String& out, unsigned long now) const;
  void appendChartsJson(String& out) const;
  void appendTimelineJson(String& out) const;
  void appendCommsJson(String& out, unsigned long now) const;
  void appendRawJson(String& out) const;
  int onlineRobotCount(unsigned long now) const;
  int alertCount(unsigned long now) const;
  const char* lastCommand() const;
  const char* triggerRobotLabel(unsigned long now) const;
  const char* sectionLabel(uint8_t robotId) const;
  float approximatePeerY(uint8_t robotId) const;

  Logger& log_;
  SensorManager& sensors_;
  MotorDriver& motors_;
  DriveController& drive_;
  EspNowManager& comms_;
  GridNavigator& gridNav_;
  StatusBuilder& statusBuilder_;

  unsigned long missionStartMs_ = 0;
  unsigned long nextChartSampleMs_ = 0;
  unsigned long lastHeartbeatEventMs_ = 0;
  bool readyEventLogged_ = false;
  bool lastRemoteAlert_ = false;
  bool lastInteresting_ = false;
  bool lastScanStop_ = false;
  bool lastPathActive_ = false;
  uint8_t triggerRobotId_ = 0;
  char lastCommand_[24] = "boot";
  char lastStopReason_[48] = {0};

  PeerSnapshot peers_[kMaxPeers];
  TimelineEvent events_[kMaxEvents];
  uint8_t eventNext_ = 0;
  uint8_t eventCount_ = 0;
  RobotCharts charts_[kMaxChartRobots];
};

const char* dashboardHtml();

#endif  // SWARMBOT_DASHBOARD_STATE_H
