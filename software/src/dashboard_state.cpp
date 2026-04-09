#include "dashboard_state.h"

#include <stdio.h>
#include <string.h>

#include "config.h"
#include "logging.h"
#include "status_builder.h"

namespace {
template <size_t N>
void copyString(char (&dst)[N], const char* src) {
  if (!src) {
    dst[0] = '\0';
    return;
  }
  strncpy(dst, src, N - 1);
  dst[N - 1] = '\0';
}

const char* sensorTypeLabel(uint8_t type) {
  switch (type) {
    case config::SENSOR_CO2: return "co2";
    case config::SENSOR_TVOC: return "tvoc";
    case config::SENSOR_TOF: return "tof";
    case config::SENSOR_DIST: return "distance";
    case config::SENSOR_HEARTBEAT: return "heartbeat";
    default: return "unknown";
  }
}

const char* driveStateLabel(DriveState state) {
  switch (state) {
    case DriveState::Stopped: return "stopped";
    case DriveState::Rotating: return "rotating";
    case DriveState::Driving: return "driving";
    case DriveState::Error: return "error";
    default: return "unknown";
  }
}
}  // namespace

void DashboardState::MetricSeries::push(unsigned long t, float v) {
  ts[next] = t;
  value[next] = v;
  next = (next + 1) % (uint8_t)(sizeof(ts) / sizeof(ts[0]));
  if (count < (uint8_t)(sizeof(ts) / sizeof(ts[0]))) {
    count++;
  }
}

DashboardState::DashboardState(
    Logger& log,
    SensorManager& sensors,
    MotorDriver& motors,
    DriveController& drive,
    EspNowManager& comms,
    GridNavigator& gridNav,
    StatusBuilder& statusBuilder)
    : log_(log),
      sensors_(sensors),
      motors_(motors),
      drive_(drive),
      comms_(comms),
      gridNav_(gridNav),
      statusBuilder_(statusBuilder) {}

void DashboardState::begin(unsigned long now) {
  missionStartMs_ = now;
  nextChartSampleMs_ = now;
  appendEvent(now, config::DEVICE_ID, "boot", "info", "Boot", "Dashboard state initialized");
}

void DashboardState::update(unsigned long now) {
  updateDerivedEvents(now);
  sampleCharts(now);
}

void DashboardState::onPeerPacket(const EspNowPacket& packet, unsigned long now) {
  if (packet.swarm_id != config::SWARM_ID || packet.id == config::DEVICE_ID) return;
  PeerSnapshot* peer = findPeer(packet.id);
  if (!peer) return;

  bool firstSeen = !peer->active;
  peer->active = true;
  peer->id = packet.id;
  peer->swarmId = packet.swarm_id;
  peer->lastSeenMs = now;
  peer->x = packet.position[0];
  peer->stopSignal = false;
  peer->lastPacket = packet;
  copyString(peer->lastCommand, packet.command);

  for (uint8_t i = 0; i < packet.num_sensors && i < 10; i++) {
    switch (packet.sensors[i].type) {
      case config::SENSOR_HEARTBEAT:
        peer->lastHeartbeatMs = now;
        if (packet.sensors[i].value == 0.0f) {
          peer->stopSignal = true;
        }
        break;
      case config::SENSOR_CO2:
        peer->hasCO2 = true;
        peer->co2 = packet.sensors[i].value;
        break;
      case config::SENSOR_TVOC:
        peer->hasTVOC = true;
        peer->tvoc = packet.sensors[i].value;
        break;
      case config::SENSOR_TOF:
        peer->hasTof = true;
        peer->tof = packet.sensors[i].value;
        break;
      default:
        break;
    }
  }

  if (firstSeen) {
    char detail[96];
    snprintf(detail, sizeof(detail), "Peer %u joined swarm %u", packet.id, packet.swarm_id);
    appendEvent(now, packet.id, "peer_online", "info", "Peer online", detail);
  }

  if (packet.command[0] != '\0') {
    char detail[96];
    snprintf(detail, sizeof(detail), "Peer %u command %s", packet.id, packet.command);
    appendEvent(now, packet.id, "command", "info", "Peer command", detail);
    copyString(lastCommand_, packet.command);
  }

  if (peer->stopSignal) {
    char detail[96];
    snprintf(detail, sizeof(detail), "Peer %u propagated stop", packet.id);
    appendEvent(now, packet.id, "stop", "danger", "Stop propagation", detail);
    triggerRobotId_ = packet.id;
    copyString(lastStopReason_, "remote_stop");
  }

  for (uint8_t i = 0; i < packet.num_sensors && i < 10; i++) {
    if (packet.sensors[i].type == config::SENSOR_CO2 && packet.sensors[i].value >= config::CO2_ALERT) {
      char detail[96];
      snprintf(detail, sizeof(detail), "Peer %u high eCO2 %.0f ppm", packet.id, packet.sensors[i].value);
      appendEvent(now, packet.id, "remote_alert", "warn", "Remote alert", detail);
      triggerRobotId_ = packet.id;
      break;
    }
  }
}

void DashboardState::onLocalCommand(const char* command, unsigned long now) {
  copyString(lastCommand_, command);
  appendEvent(now, config::DEVICE_ID, "command", "info", "Manual command", command);
}

void DashboardState::onLocalEvent(const char* type, const char* title, const char* detail, const char* severity, unsigned long now) {
  appendEvent(now, config::DEVICE_ID, type, severity, title, detail);
}

void DashboardState::onScanStop(const char* reason, unsigned long now) {
  copyString(lastStopReason_, reason);
  triggerRobotId_ = config::DEVICE_ID;
  appendEvent(now, config::DEVICE_ID, "scan_stop", "danger", "Scan stop", reason);
}

DashboardState::PeerSnapshot* DashboardState::findPeer(uint8_t id) {
  for (uint8_t i = 0; i < kMaxPeers; i++) {
    if (peers_[i].active && peers_[i].id == id) return &peers_[i];
  }
  for (uint8_t i = 0; i < kMaxPeers; i++) {
    if (!peers_[i].active) return &peers_[i];
  }
  return &peers_[id % kMaxPeers];
}

const DashboardState::PeerSnapshot* DashboardState::findPeer(uint8_t id) const {
  for (uint8_t i = 0; i < kMaxPeers; i++) {
    if (peers_[i].active && peers_[i].id == id) return &peers_[i];
  }
  return nullptr;
}

DashboardState::RobotCharts* DashboardState::chartSeries(uint8_t id) {
  for (uint8_t i = 0; i < kMaxChartRobots; i++) {
    if (charts_[i].active && charts_[i].id == id) return &charts_[i];
  }
  for (uint8_t i = 0; i < kMaxChartRobots; i++) {
    if (!charts_[i].active) {
      charts_[i].active = true;
      charts_[i].id = id;
      return &charts_[i];
    }
  }
  return &charts_[0];
}

const DashboardState::RobotCharts* DashboardState::chartSeries(uint8_t id) const {
  for (uint8_t i = 0; i < kMaxChartRobots; i++) {
    if (charts_[i].active && charts_[i].id == id) return &charts_[i];
  }
  return nullptr;
}

void DashboardState::appendEvent(
    unsigned long now,
    uint8_t robotId,
    const char* type,
    const char* severity,
    const char* title,
    const char* detail) {
  TimelineEvent& event = events_[eventNext_];
  event.used = true;
  event.tsMs = now;
  event.robotId = robotId;
  copyString(event.type, type);
  copyString(event.severity, severity);
  copyString(event.title, title);
  copyString(event.detail, detail);
  eventNext_ = (eventNext_ + 1) % kMaxEvents;
  if (eventCount_ < kMaxEvents) eventCount_++;
}

void DashboardState::sampleCharts(unsigned long now) {
  if (now < nextChartSampleMs_) return;
  nextChartSampleMs_ = now + kChartSampleMs;

  SensorStatus sensors = sensors_.status();
  EncoderStatus enc = motors_.getEncoderStatus();
  RobotCharts* local = chartSeries(config::DEVICE_ID);
  if (sensors.sgpHasAir) local->eco2.push(now, sensors.sgpCO2);
  if (sensors.tofValid) local->tof.push(now, sensors.tofMm);
  local->distance.push(now, enc.avgDistMm);

  for (uint8_t i = 0; i < kMaxPeers; i++) {
    if (!peers_[i].active) continue;
    RobotCharts* peerCharts = chartSeries(peers_[i].id);
    if (peers_[i].hasCO2) peerCharts->eco2.push(now, peers_[i].co2);
    if (peers_[i].hasTof) peerCharts->tof.push(now, peers_[i].tof);
    peerCharts->distance.push(now, peers_[i].x);
  }
}

void DashboardState::updateDerivedEvents(unsigned long now) {
  if (!readyEventLogged_ && now > 1500) {
    readyEventLogged_ = true;
    appendEvent(now, config::DEVICE_ID, "ready", "ok", "Robot ready", "Setup complete");
  }

  CommsStatus comms = comms_.status();
  if (comms.lastHeartbeatMs != 0 && comms.lastHeartbeatMs != lastHeartbeatEventMs_) {
    lastHeartbeatEventMs_ = comms.lastHeartbeatMs;
    char detail[96];
    snprintf(detail, sizeof(detail), "Heartbeat from robot %u", comms.lastHeartbeatId);
    appendEvent(now, comms.lastHeartbeatId, "heartbeat", "info", "Heartbeat received", detail);
  }

  if (comms.remoteCo2Alert && !lastRemoteAlert_) {
    appendEvent(now, comms.lastHeartbeatId, "remote_alert", "warn", "Remote CO2 alert", "Peer broadcasted high eCO2");
  }
  lastRemoteAlert_ = comms.remoteCo2Alert;

  if (comms.rxInteresting && !lastInteresting_) {
    appendEvent(now, comms.lastHeartbeatId, "interesting", "warn", "Interesting event", "Local robot paused due to nearby alert");
  }
  lastInteresting_ = comms.rxInteresting;

  bool scanStop = gridNav_.isScanStopLatched();
  if (scanStop && !lastScanStop_) {
    if (gridNav_.scanStopTofMm() > 0) {
      char detail[96];
      snprintf(detail, sizeof(detail), "ToF stop at %u mm", gridNav_.scanStopTofMm());
      onScanStop(detail, now);
    } else if (gridNav_.scanStopPpm() > 0) {
      char detail[96];
      snprintf(detail, sizeof(detail), "CO2 stop at %u ppm", gridNav_.scanStopPpm());
      onScanStop(detail, now);
    }
  }
  lastScanStop_ = scanStop;

  DriveStatus drive = drive_.status();
  bool pathActive = drive.driveMode == DriveMode::Path;
  if (lastPathActive_ && !pathActive) {
    appendEvent(now, config::DEVICE_ID, "drive_complete", "ok", "Drive complete", "Path execution finished");
  }
  lastPathActive_ = pathActive;
}

const char* DashboardState::peerFreshness(const PeerSnapshot& peer, unsigned long now) const {
  unsigned long age = now - peer.lastSeenMs;
  if (age <= kOnlineFreshMs) return "online";
  if (age <= kStaleMs) return "stale";
  return "offline";
}

const char* DashboardState::localMissionState(unsigned long now) const {
  (void)now;
  if (gridNav_.isScanStopLatched()) return "stopped";
  if (gridNav_.isEnabled() && gridNav_.isTimedGrid()) return "timed_scan";
  if (gridNav_.isEnabled()) return driveStateLabel(gridNav_.status().state);
  if (drive_.status().driveMode == DriveMode::Path) return "path";
  if (comms_.status().rxInteresting) return "avoiding";
  return "ready";
}

void DashboardState::appendJsonString(String& out, const char* value) const {
  out += '"';
  if (value) {
    for (const char* p = value; *p; ++p) {
      switch (*p) {
        case '\\': out += "\\\\"; break;
        case '"': out += "\\\""; break;
        case '\n': out += "\\n"; break;
        case '\r': break;
        case '\t': out += "\\t"; break;
        default: out += *p; break;
      }
    }
  }
  out += '"';
}

int DashboardState::onlineRobotCount(unsigned long now) const {
  int count = 1;
  for (uint8_t i = 0; i < kMaxPeers; i++) {
    if (peers_[i].active && strcmp(peerFreshness(peers_[i], now), "offline") != 0) count++;
  }
  return count;
}

int DashboardState::alertCount(unsigned long now) const {
  int count = 0;
  SensorStatus s = sensors_.status();
  if (s.sgpHasAir && s.sgpCO2 >= config::CO2_ALERT) count++;
  if (gridNav_.isScanStopLatched()) count++;
  if (comms_.status().remoteCo2Alert) count++;
  for (uint8_t i = 0; i < kMaxPeers; i++) {
    if (!peers_[i].active || strcmp(peerFreshness(peers_[i], now), "offline") == 0) continue;
    if (peers_[i].stopSignal) count++;
    if (peers_[i].hasCO2 && peers_[i].co2 >= config::CO2_ALERT) count++;
  }
  return count;
}

const char* DashboardState::lastCommand() const {
  return lastCommand_;
}

const char* DashboardState::triggerRobotLabel(unsigned long now) const {
  (void)now;
  static char label[16];
  if (triggerRobotId_ == 0) return "none";
  snprintf(label, sizeof(label), "R%u", triggerRobotId_);
  return label;
}

const char* DashboardState::sectionLabel(uint8_t robotId) const {
  switch (robotId) {
    case 1: return "east_flank";
    case 2: return "west_flank";
    case 3: return "center";
    case 4: return "northwest";
    case 5: return "northeast";
    default: return "peer";
  }
}

float DashboardState::approximatePeerY(uint8_t robotId) const {
  uint8_t lanes = config::EXPECTED_ROBOTS > 1 ? config::EXPECTED_ROBOTS : 2;
  uint8_t laneIndex = robotId > 0 ? (robotId - 1) % lanes : 0;
  float spacing = config::MAX_Y_MM / (float)lanes;
  return spacing * laneIndex + spacing * 0.5f;
}

void DashboardState::appendRobotJson(String& out, unsigned long now) const {
  SensorStatus sensors = sensors_.status();
  EncoderStatus enc = motors_.getEncoderStatus();
  DriveStatus drive = drive_.status();
  GridStatus grid = gridNav_.status();
  CommsStatus comms = comms_.status();

  out += "{";
  out += "\"id\":" + String(config::DEVICE_ID);
  out += ",\"local\":true";
  out += ",\"state\":\"online\"";
  out += ",\"uptime_ms\":" + String(now);
  out += ",\"heap_free\":" + String(ESP.getFreeHeap());
  out += ",\"drive_mode\":\"" + String(driveModeLabel(drive.driveMode)) + "\"";
  out += ",\"mission_state\":\"" + String(localMissionState(now)) + "\"";
  out += ",\"heartbeat_age_ms\":0";
  out += ",\"section\":\"local\"";
  out += ",\"assigned_section\":";
  appendJsonString(out, sectionLabel(config::DEVICE_ID));
  out += ",\"readiness\":{";
  out += "\"tof\":" + String(sensors.tofReady ? "true" : "false");
  out += ",\"air\":" + String(sensors.sgpReady ? "true" : "false");
  out += ",\"imu\":" + String(sensors.mpuReady ? "true" : "false");
  out += ",\"encoder\":true";
  out += ",\"pose\":" + String(grid.pose.valid ? "true" : "false");
  out += "}";
  out += ",\"latest\":{";
  out += "\"eco2\":" + String(sensors.sgpHasAir ? String(sensors.sgpCO2) : "null");
  out += ",\"tof_mm\":" + String(sensors.tofValid ? String(sensors.tofMm) : "null");
  out += ",\"distance_mm\":" + String(enc.avgDistMm, 1);
  out += "}";
  out += ",\"alert\":\"";
  if (gridNav_.isScanStopLatched()) {
    out += "stop";
  } else if (comms.remoteCo2Alert || (sensors.sgpHasAir && sensors.sgpCO2 >= config::CO2_ALERT)) {
    out += "warn";
  } else {
    out += "normal";
  }
  out += "\"";
  out += ",\"pose\":{";
  out += "\"x\":" + String(grid.pose.xMm, 1);
  out += ",\"y\":" + String(grid.pose.yMm, 1);
  out += ",\"heading_deg\":" + String(grid.pose.headingDeg, 1);
  out += ",\"approx\":false";
  out += ",\"valid\":" + String(grid.pose.valid ? "true" : "false");
  out += "}";
  out += ",\"comms\":{";
  out += "\"remote_alert\":" + String(comms.remoteCo2Alert ? "true" : "false");
  out += ",\"interesting\":" + String(comms.rxInteresting ? "true" : "false");
  out += "}";
  out += "}";
}

void DashboardState::appendPeerRobotsJson(String& out, unsigned long now) const {
  for (uint8_t i = 0; i < kMaxPeers; i++) {
    if (!peers_[i].active) continue;
    out += ",{";
    out += "\"id\":" + String(peers_[i].id);
    out += ",\"local\":false";
    out += ",\"state\":\"" + String(peerFreshness(peers_[i], now)) + "\"";
    out += ",\"uptime_ms\":null";
    out += ",\"heap_free\":null";
    out += ",\"drive_mode\":\"peer\"";
    out += ",\"mission_state\":\"";
    if (peers_[i].stopSignal) {
      out += "stopped";
    } else if (peers_[i].hasCO2 && peers_[i].co2 >= config::CO2_ALERT) {
      out += "alert";
    } else {
      out += "tracking";
    }
    out += "\"";
    out += ",\"heartbeat_age_ms\":";
    out += peers_[i].lastHeartbeatMs > 0 ? String(now - peers_[i].lastHeartbeatMs) : "null";
    out += ",\"assigned_section\":";
    appendJsonString(out, sectionLabel(peers_[i].id));
    out += ",\"readiness\":{";
    out += "\"tof\":" + String(peers_[i].hasTof ? "true" : "false");
    out += ",\"air\":" + String(peers_[i].hasCO2 ? "true" : "false");
    out += ",\"imu\":false";
    out += ",\"encoder\":true";
    out += ",\"pose\":true";
    out += "}";
    out += ",\"latest\":{";
    out += "\"eco2\":" + String(peers_[i].hasCO2 ? String(peers_[i].co2, 0) : "null");
    out += ",\"tof_mm\":" + String(peers_[i].hasTof ? String(peers_[i].tof, 0) : "null");
    out += ",\"distance_mm\":" + String(peers_[i].x, 1);
    out += "}";
    out += ",\"alert\":\"";
    if (peers_[i].stopSignal) out += "stop";
    else if (peers_[i].hasCO2 && peers_[i].co2 >= config::CO2_ALERT) out += "warn";
    else out += "normal";
    out += "\"";
    out += ",\"pose\":{";
    out += "\"x\":" + String(peers_[i].x, 1);
    out += ",\"y\":" + String(approximatePeerY(peers_[i].id), 1);
    out += ",\"heading_deg\":0";
    out += ",\"approx\":true";
    out += ",\"valid\":true";
    out += "}";
    out += ",\"comms\":{";
    out += "\"remote_alert\":" + String(peers_[i].hasCO2 && peers_[i].co2 >= config::CO2_ALERT ? "true" : "false");
    out += ",\"interesting\":false";
    out += "}";
    out += "}";
  }
}

void DashboardState::appendMapJson(String& out, unsigned long now) const {
  (void)now;
  GridStatus grid = gridNav_.status();
  out += "\"map\":{";
  out += "\"grid\":{\"width\":" + String(config::GRID_WIDTH) + ",\"height\":" + String(config::GRID_HEIGHT);
  out += ",\"cell_w_mm\":" + String(config::CELL_WIDTH_MM, 1);
  out += ",\"cell_h_mm\":" + String(config::CELL_HEIGHT_MM, 1) + "},";
  out += "\"local_pose\":{\"x\":" + String(grid.pose.xMm, 1);
  out += ",\"y\":" + String(grid.pose.yMm, 1);
  out += ",\"heading_deg\":" + String(grid.pose.headingDeg, 1);
  out += ",\"valid\":" + String(grid.pose.valid ? "true" : "false") + "},";
  out += "\"trigger_robot\":";
  appendJsonString(out, triggerRobotLabel(now));
  out += "},";
}

void DashboardState::appendMetricSeries(String& out, const MetricSeries& series) const {
  out += "[";
  uint8_t capacity = sizeof(series.ts) / sizeof(series.ts[0]);
  for (uint8_t i = 0; i < series.count; i++) {
    uint8_t idx = (series.next + capacity - series.count + i) % capacity;
    if (i) out += ",";
    out += "{\"t\":" + String(series.ts[idx]) + ",\"v\":" + String(series.value[idx], 1) + "}";
  }
  out += "]";
}

void DashboardState::appendChartsJson(String& out) const {
  out += "\"charts\":{";
  out += "\"eco2\":[";
  bool firstSeries = true;
  for (uint8_t i = 0; i < kMaxChartRobots; i++) {
    if (!charts_[i].active || charts_[i].eco2.count == 0) continue;
    if (!firstSeries) out += ",";
    firstSeries = false;
    out += "{\"id\":" + String(charts_[i].id) + ",\"points\":";
    appendMetricSeries(out, charts_[i].eco2);
    out += "}";
  }
  out += "],\"tof\":[";
  firstSeries = true;
  for (uint8_t i = 0; i < kMaxChartRobots; i++) {
    if (!charts_[i].active || charts_[i].tof.count == 0) continue;
    if (!firstSeries) out += ",";
    firstSeries = false;
    out += "{\"id\":" + String(charts_[i].id) + ",\"points\":";
    appendMetricSeries(out, charts_[i].tof);
    out += "}";
  }
  out += "],\"distance\":[";
  firstSeries = true;
  for (uint8_t i = 0; i < kMaxChartRobots; i++) {
    if (!charts_[i].active || charts_[i].distance.count == 0) continue;
    if (!firstSeries) out += ",";
    firstSeries = false;
    out += "{\"id\":" + String(charts_[i].id) + ",\"points\":";
    appendMetricSeries(out, charts_[i].distance);
    out += "}";
  }
  out += "]},";
}

void DashboardState::appendTimelineJson(String& out) const {
  out += "\"timeline\":[";
  for (uint8_t i = 0; i < eventCount_; i++) {
    uint8_t idx = (eventNext_ + kMaxEvents - eventCount_ + i) % kMaxEvents;
    const TimelineEvent& event = events_[idx];
    if (!event.used) continue;
    if (i) out += ",";
    out += "{";
    out += "\"ts_ms\":" + String(event.tsMs);
    out += ",\"robot_id\":" + String(event.robotId);
    out += ",\"type\":";
    appendJsonString(out, event.type);
    out += ",\"severity\":";
    appendJsonString(out, event.severity);
    out += ",\"title\":";
    appendJsonString(out, event.title);
    out += ",\"detail\":";
    appendJsonString(out, event.detail);
    out += "}";
  }
  out += "],";
}

void DashboardState::appendCommsJson(String& out, unsigned long now) const {
  CommsStatus comms = comms_.status();
  out += "\"comms\":{";
  out += "\"tx_count\":" + String(comms.txCount);
  out += ",\"last_tx_ms\":" + String(comms.lastTxMs);
  out += ",\"last_rx_ms\":" + String(comms.lastRxMs);
  out += ",\"peers\":[";
  bool first = true;
  for (uint8_t i = 0; i < kMaxPeers; i++) {
    if (!peers_[i].active) continue;
    if (!first) out += ",";
    first = false;
    out += "{";
    out += "\"id\":" + String(peers_[i].id);
    out += ",\"last_seen_ms\":" + String(peers_[i].lastSeenMs);
    out += ",\"age_ms\":" + String(now - peers_[i].lastSeenMs);
    out += ",\"freshness\":\"" + String(peerFreshness(peers_[i], now)) + "\"";
    out += ",\"heartbeat_age_ms\":";
    out += peers_[i].lastHeartbeatMs > 0 ? String(now - peers_[i].lastHeartbeatMs) : "null";
    out += ",\"x_surrogate_mm\":" + String(peers_[i].x, 1);
    out += ",\"eco2\":" + String(peers_[i].hasCO2 ? String(peers_[i].co2, 0) : "null");
    out += ",\"tof_mm\":" + String(peers_[i].hasTof ? String(peers_[i].tof, 0) : "null");
    out += ",\"tvoc\":" + String(peers_[i].hasTVOC ? String(peers_[i].tvoc, 0) : "null");
    out += ",\"state\":\"";
    if (peers_[i].stopSignal) out += "stopped";
    else out += peerFreshness(peers_[i], now);
    out += "\"";
    out += "}";
  }
  out += "]},";
}

void DashboardState::appendRawJson(String& out) const {
  out += "\"raw\":{";
  out += "\"local_status\":";
  appendJsonString(out, statusBuilder_.build(millis()).c_str());
  out += ",\"last_rx_packet\":";
  CommsStatus comms = comms_.status();
  if (!comms.lastRxValid) {
    out += "null";
  } else {
    out += "{";
    out += "\"id\":" + String(comms.lastRxData.id);
    out += ",\"swarm_id\":" + String(comms.lastRxData.swarm_id);
    out += ",\"x\":" + String(comms.lastRxData.position[0], 1);
    out += ",\"num_sensors\":" + String(comms.lastRxData.num_sensors);
    out += ",\"command\":";
    appendJsonString(out, comms.lastRxData.command);
    out += "}";
  }
  out += ",\"peer_cache\":[";
  bool first = true;
  for (uint8_t i = 0; i < kMaxPeers; i++) {
    if (!peers_[i].active) continue;
    if (!first) out += ",";
    first = false;
    out += "{";
    out += "\"id\":" + String(peers_[i].id);
    out += ",\"sensors\":[";
    for (uint8_t s = 0; s < peers_[i].lastPacket.num_sensors && s < 10; s++) {
      if (s) out += ",";
      out += "{";
      out += "\"type\":";
      appendJsonString(out, sensorTypeLabel(peers_[i].lastPacket.sensors[s].type));
      out += ",\"value\":" + String(peers_[i].lastPacket.sensors[s].value, 1);
      out += "}";
    }
    out += "],\"command\":";
    appendJsonString(out, peers_[i].lastCommand);
    out += "}";
  }
  out += "]}";
}

String DashboardState::buildDashboardJson(unsigned long now) const {
  String out;
  out.reserve(20000);
  out += "{";
  out += "\"mission\":{";
  out += "\"swarm_id\":" + String(config::SWARM_ID);
  out += ",\"mission_mode\":";
  appendJsonString(out, localMissionState(now));
  out += ",\"robots_online\":" + String(onlineRobotCount(now));
  out += ",\"robots_expected\":" + String(config::EXPECTED_ROBOTS);
  out += ",\"active_alerts\":" + String(alertCount(now));
  out += ",\"state\":\"" + String(gridNav_.isScanStopLatched() ? "stopped" : "running") + "\"";
  out += ",\"elapsed_ms\":" + String(now - missionStartMs_);
  out += ",\"trigger_robot\":";
  appendJsonString(out, triggerRobotLabel(now));
  out += ",\"last_command\":";
  appendJsonString(out, lastCommand());
  out += ",\"stop_reason\":";
  appendJsonString(out, lastStopReason_);
  out += "},";

  out += "\"robots\":[";
  appendRobotJson(out, now);
  appendPeerRobotsJson(out, now);
  out += "],";

  appendMapJson(out, now);
  appendChartsJson(out);
  appendTimelineJson(out);
  appendCommsJson(out, now);
  appendRawJson(out);
  out += "}";
  return out;
}

const char* dashboardHtml() {
  return R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>BioBeacon - Hive Mind</title>
  <script src="/static/chart.min.js"></script>
  <style>
    :root{color-scheme:light;--bg:#a0c2db;--panel:#F6D649;--panel-2:#f2cc2c;--line:rgba(0,0,0,.22);--text:#141414;--muted:#2b2b2b;--ok:#1b7f3a;--warn:#a15d00;--danger:#a72222;--info:#1D49A7}
    *{box-sizing:border-box}body{margin:0;font-family:ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;background:var(--bg);color:var(--text)}
    .shell{max-width:1500px;margin:0 auto;padding:20px}
    .hero{display:flex;justify-content:space-between;align-items:end;gap:20px;margin-bottom:18px}
    .hero h1{margin:0;font-size:32px;letter-spacing:.08em;text-transform:uppercase}
    .hero p{margin:6px 0 0;color:var(--muted)}
    .actions{display:flex;gap:10px;flex-wrap:wrap}
    button{border:1px solid rgba(0,0,0,.18);background:#1D49A7;color:#fff;padding:10px 14px;border-radius:12px;cursor:pointer}
    .grid{display:grid;gap:16px}
    .summary{grid-template-columns:repeat(6,minmax(0,1fr));margin-bottom:16px}
    .cards{grid-template-columns:repeat(auto-fit,minmax(240px,1fr));margin-bottom:16px}
    .main{grid-template-columns:1.4fr 1fr;margin-bottom:16px}
    .bottom{grid-template-columns:1fr 1fr 1fr}
    .panel{background:linear-gradient(180deg,rgba(246,214,73,.98),rgba(240,206,53,.98));border:1px solid var(--line);border-radius:18px;padding:16px;box-shadow:0 18px 48px rgba(0,0,0,.12)}
    .panel h2,.panel h3{margin:0 0 12px;font-size:14px;letter-spacing:.12em;text-transform:uppercase;color:var(--text)}
    .metric{display:flex;flex-direction:column;gap:8px;min-height:102px}
    .metric .value{font-size:28px;font-weight:700}
    .metric .label{font-size:12px;color:var(--muted);letter-spacing:.12em;text-transform:uppercase}
    .subtle{color:var(--muted);font-size:13px}
    .robot-card{position:relative;overflow:hidden}
    .robot-card::after{content:"";position:absolute;inset:auto -40px -40px auto;width:120px;height:120px;border-radius:50%;background:radial-gradient(circle,rgba(29,73,167,.16),transparent 72%)}
    .robot-card.online{border-color:rgba(43,214,123,.35)}
    .robot-card.warn{border-color:rgba(255,176,32,.4)}
    .robot-card.stop{border-color:rgba(255,93,93,.45)}
    .robot-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px}
    .badge{display:inline-flex;align-items:center;padding:4px 9px;border-radius:999px;font-size:11px;letter-spacing:.08em;text-transform:uppercase}
    .badge.ok{background:rgba(43,214,123,.16);color:var(--ok)}
    .badge.warn{background:rgba(255,176,32,.16);color:var(--warn)}
    .badge.danger{background:rgba(255,93,93,.16);color:var(--danger)}
    .badge.info{background:rgba(70,166,255,.16);color:var(--info)}
    .robot-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px}
    .mini{padding:10px;border-radius:12px;background:rgba(255,255,255,.2);border:1px solid rgba(0,0,0,.12)}
    .mini .k{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.08em}
    .mini .v{font-size:18px;margin-top:4px}
    .readiness{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}
    .map-wrap{min-height:420px}
    svg{width:100%;height:100%}
    .charts{display:grid;grid-template-columns:1fr;gap:14px;max-height:560px;overflow-y:auto;padding-right:4px}
    .chart-card{position:relative;height:180px;padding:10px;border-radius:14px;background:rgba(255,255,255,.16);border:1px solid rgba(0,0,0,.12)}
    .chart-card canvas{display:block;width:100% !important;height:100% !important}
    .chart-empty{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;padding:16px;text-align:center;color:var(--muted);font-size:13px}
    .timeline,.table-wrap,.raw{max-height:420px;overflow:auto}
    .event{display:flex;gap:12px;padding:12px 0;border-bottom:1px solid rgba(0,0,0,.12)}
    .event:last-child{border-bottom:0}
    .event .dot{width:10px;height:10px;border-radius:50%;margin-top:6px}
    .event.ok .dot{background:var(--ok)} .event.warn .dot{background:var(--warn)} .event.danger .dot{background:var(--danger)} .event.info .dot{background:var(--info)}
    table{width:100%;border-collapse:collapse;font-size:14px}
    th,td{text-align:left;padding:10px 8px;border-bottom:1px solid rgba(0,0,0,.12)}
    th{font-size:11px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted)}
    pre{white-space:pre-wrap;word-break:break-word;background:rgba(255,255,255,.28);border:1px solid rgba(0,0,0,.14);padding:12px;border-radius:12px;color:var(--text)}
    .raw-tabs{display:flex;gap:8px;margin-bottom:10px;flex-wrap:wrap}
    .raw-tabs button.active{border-color:rgba(0,0,0,.22);background:#1D49A7}
    @media (max-width:1100px){.summary{grid-template-columns:repeat(3,minmax(0,1fr))}.main,.bottom{grid-template-columns:1fr}}
    @media (max-width:700px){.summary{grid-template-columns:repeat(2,minmax(0,1fr))}.hero{flex-direction:column;align-items:start}}
  </style>
</head>
<body>
  <div class="shell">
    <div class="hero">
      <div>
        <h1>BioBeacon - Hive Mind</h1>
        <p id="subtitle">BioBeacon Fireflies' Mission Control</p>
      </div>
      <div class="actions">
        <button data-action="scan">Start Swarm Scan</button>
        <button data-action="stop">Broadcast Stop</button>
        <button data-action="drive">Drive Console</button>
      </div>
    </div>

    <section id="summary" class="grid summary"></section>
    <section id="cards" class="grid cards"></section>
    <section class="grid main">
      <div class="panel map-wrap">
        <h2>Live Map</h2>
        <div id="map"></div>
      </div>
      <div class="panel">
        <h2>Mission Charts</h2>
        <div class="charts">
          <div class="chart-card"><canvas id="chart-eco2"></canvas><div id="empty-eco2" class="chart-empty" hidden>No eCO2 series available yet.</div></div>
          <div class="chart-card"><canvas id="chart-tof"></canvas><div id="empty-tof" class="chart-empty" hidden>No ToF series available yet.</div></div>
          <div class="chart-card"><canvas id="chart-distance"></canvas><div id="empty-distance" class="chart-empty" hidden>No distance series available yet.</div></div>
        </div>
      </div>
    </section>
    <section class="grid bottom">
      <div class="panel"><h2>Timeline</h2><div id="timeline" class="timeline"></div></div>
      <div class="panel"><h2>Comms Peers</h2><div class="table-wrap"><table><thead><tr><th>Robot</th><th>Freshness</th><th>Last Seen</th><th>X</th><th>eCO2</th><th>ToF</th><th>TVOC</th><th>State</th></tr></thead><tbody id="comms-body"></tbody></table></div></div>
      <div class="panel"><h2>Raw Drill-Down</h2><div class="raw-tabs"><button class="active" data-raw="local">Local /status</button><button data-raw="peer">Peer Cache</button><button data-raw="packet">Last Packet</button></div><pre id="raw" class="raw"></pre></div>
    </section>
  </div>
  <script>
    const state={rawTab:'local',charts:{},data:null,chartWarned:false,fetchWarned:false};
    const colors={1:'#46a6ff',2:'#2bd67b',3:'#ffb020',4:'#ff5d5d',5:'#d067ff',6:'#8ce3ff'};
    const fmtMs=(ms)=>ms==null?'--':(ms<1000?ms+' ms':(ms<60000?(ms/1000).toFixed(1)+' s':(ms/60000).toFixed(1)+' min'));
    const fmtVal=(v,s='')=>v==null?'--':`${v}${s}`;
    const el=(id)=>document.getElementById(id);

    async function sendAction(kind){
      if(kind==='scan') await fetch('/swarm_timed_grid_scan');
      if(kind==='stop') await fetch('/drive?mode=stop');
      if(kind==='drive') location.href='/drive';
    }

    function summaryMetric(label,value,sub,badge){
      return `<div class="panel metric"><div class="label">${label}</div><div class="value">${value}</div><div class="subtle">${sub||''}</div>${badge?`<span class="badge ${badge.cls}">${badge.text}</span>`:''}</div>`;
    }

    function renderSummary(data){
      const m=data.mission;
      el('subtitle').textContent=`Swarm ${m.swarm_id} • mode ${m.mission_mode} • last command ${m.last_command || 'none'}`;
      el('summary').innerHTML=[
        summaryMetric('Swarm',`#${m.swarm_id}`,`${m.robots_online}/${m.robots_expected} robots online`,{cls:'info',text:m.state}),
        summaryMetric('Mission Mode',m.mission_mode,`Elapsed ${fmtMs(m.elapsed_ms)}`),
        summaryMetric('Alerts',m.active_alerts,m.stop_reason||'No active stop reason',m.active_alerts?{cls:'warn',text:'attention'}:{cls:'ok',text:'stable'}),
        summaryMetric('Trigger Robot',m.trigger_robot,'First notable detector / stop origin'),
        summaryMetric('Last Command',m.last_command || 'none','Latest operator or peer command'),
        summaryMetric('Robots Online',m.robots_online,`${m.robots_expected-m.robots_online} unavailable`,m.robots_online>=m.robots_expected?{cls:'ok',text:'full swarm'}:{cls:'warn',text:'partial'})
      ].join('');
    }

    function readinessBadge(name,ready){
      return `<span class="badge ${ready?'ok':'warn'}">${name}</span>`;
    }

    function renderCards(data){
      el('cards').innerHTML=data.robots.map(robot=>{
        const statusClass=robot.alert==='stop'?'stop':(robot.alert==='warn'?'warn':'online');
        return `<div class="panel robot-card ${statusClass}">
          <div class="robot-head">
            <div><h3>Robot ${robot.id}</h3><div class="subtle">${robot.local?'Local node':'Peer node'} • ${robot.state}</div></div>
            <span class="badge ${robot.alert==='stop'?'danger':robot.alert==='warn'?'warn':'ok'}">${robot.mission_state}</span>
          </div>
          <div class="robot-grid">
            <div class="mini"><div class="k">eCO2</div><div class="v">${fmtVal(robot.latest.eco2,' ppm')}</div></div>
            <div class="mini"><div class="k">ToF</div><div class="v">${fmtVal(robot.latest.tof_mm,' mm')}</div></div>
            <div class="mini"><div class="k">Distance</div><div class="v">${fmtVal(robot.latest.distance_mm?.toFixed?.(0) ?? robot.latest.distance_mm,' mm')}</div></div>
            <div class="mini"><div class="k">Heartbeat</div><div class="v">${fmtMs(robot.heartbeat_age_ms)}</div></div>
            <div class="mini"><div class="k">Drive Mode</div><div class="v">${robot.drive_mode}</div></div>
            <div class="mini"><div class="k">Section</div><div class="v">${robot.assigned_section}</div></div>
          </div>
          <div class="readiness">
            ${readinessBadge('ToF',robot.readiness.tof)}
            ${readinessBadge('Air',robot.readiness.air)}
            ${readinessBadge('IMU',robot.readiness.imu)}
            ${readinessBadge('Enc',robot.readiness.encoder)}
            ${readinessBadge('Pose',robot.readiness.pose)}
          </div>
          <div class="subtle" style="margin-top:12px">Uptime ${fmtMs(robot.uptime_ms)} • Heap ${fmtVal(robot.heap_free)} • Pose ${robot.pose.valid?`${robot.pose.x.toFixed(0)}, ${robot.pose.y.toFixed(0)}`:'n/a'}</div>
        </div>`;
      }).join('');
    }

    function renderMap(data){
      const g=data.map.grid;
      const width=620,height=380,pad=28;
      const scaleX=(width-pad*2)/(g.width*g.cell_w_mm||1);
      const scaleY=(height-pad*2)/(g.height*g.cell_h_mm||1);
      const robots=data.robots.map(r=>{
        const x=pad+(r.pose.x||0)*scaleX;
        const y=height-pad-(r.pose.y||0)*scaleY;
        const heading=((r.pose.heading_deg||0)-90)*Math.PI/180;
        const hx=x+Math.cos(heading)*18;
        const hy=y+Math.sin(heading)*18;
        const color=colors[r.id]||'#46a6ff';
        return `<g opacity="${r.state==='offline'?0.35:1}">
          <circle cx="${x}" cy="${y}" r="${r.pose.approx?8:10}" fill="${color}" stroke="${r.pose.approx?'#f1f5f9':'#fff'}" stroke-dasharray="${r.pose.approx?'3 3':'0'}" stroke-width="2"></circle>
          <line x1="${x}" y1="${y}" x2="${hx}" y2="${hy}" stroke="${color}" stroke-width="3" stroke-linecap="round"></line>
          <text x="${x+12}" y="${y-12}" fill="#141414" font-size="12">R${r.id}${data.mission.trigger_robot===`R${r.id}`?' !':''}</text>
        </g>`;
      }).join('');
      let grid='';
      for(let x=0;x<=g.width;x++){const px=pad+x*(width-pad*2)/g.width;grid+=`<line x1="${px}" y1="${pad}" x2="${px}" y2="${height-pad}" stroke="rgba(0,0,0,.16)"/>`;}
      for(let y=0;y<=g.height;y++){const py=pad+y*(height-pad*2)/g.height;grid+=`<line x1="${pad}" y1="${py}" x2="${width-pad}" y2="${py}" stroke="rgba(0,0,0,.16)"/>`;}
      el('map').innerHTML=`<svg viewBox="0 0 ${width} ${height}">
        <defs><linearGradient id="bg" x1="0" x2="1"><stop offset="0%" stop-color="#f8de69"/><stop offset="100%" stop-color="#f2cd3b"/></linearGradient></defs>
        <rect x="0" y="0" width="${width}" height="${height}" rx="22" fill="url(#bg)"></rect>
        <rect x="${pad}" y="${pad}" width="${width-pad*2}" height="${height-pad*2}" rx="16" fill="rgba(255,255,255,.18)" stroke="rgba(0,0,0,.16)"></rect>
        ${grid}
        ${robots}
      </svg>`;
    }

    function seriesToDataset(series,label){
      return series.map(s=>({
        label:`R${s.id}`,
        data:s.points.map(p=>({x:p.t,y:p.v})),
        borderColor:colors[s.id]||'#46a6ff',
        backgroundColor:'transparent',
        tension:.25,
        pointRadius:0,
        borderWidth:2
      }));
    }

    function setChartEmpty(name, empty){
      const note=el(`empty-${name}`);
      const canvas=el(`chart-${name}`);
      if(note) note.hidden=!empty;
      if(canvas) canvas.style.visibility=empty?'hidden':'visible';
    }

    function buildChart(canvasId,title,yLabel,yType='linear'){
      if(typeof Chart==='undefined') return null;
      return new Chart(el(canvasId),{
        type:'line',
        data:{datasets:[]},
        options:{
          responsive:true,
          maintainAspectRatio:false,
          plugins:{legend:{labels:{color:'#141414'}},title:{display:true,text:title,color:'#141414'}},
          scales:{
            x:{type:'linear',ticks:{color:'#141414',callback:v=>`${Math.round(v/1000)}s`},grid:{color:'rgba(0,0,0,.12)'}},
            y:{type:yType,ticks:{color:'#141414'},title:{display:true,text:yLabel,color:'#141414'},grid:{color:'rgba(0,0,0,.12)'}}
          }
        }
      });
    }

    function renderCharts(data){
      if(typeof Chart==='undefined'){
        if(!state.chartWarned){
          console.warn('Chart.js is not available; charts are disabled.');
          state.chartWarned=true;
        }
        ['eco2','tof','distance'].forEach(name=>{
          const canvas=el(`chart-${name}`);
          const panel=canvas?.parentElement;
          if(panel && !panel.querySelector('.chart-fallback')){
            const note=document.createElement('div');
            note.className='subtle chart-fallback';
            note.textContent='Charts unavailable: Chart.js failed to load.';
            panel.appendChild(note);
          }
        });
        return;
      }
      if(!state.charts.eco2){
        state.charts.eco2=buildChart('chart-eco2','eCO2 Over Time','ppm','logarithmic');
        state.charts.tof=buildChart('chart-tof','ToF Over Time','mm','logarithmic');
        state.charts.distance=buildChart('chart-distance','Distance Over Time','mm','linear');
      }
      if(!state.charts.eco2 || !state.charts.tof || !state.charts.distance) return;
      [['eco2','eco2'],['tof','tof'],['distance','distance']].forEach(([key,name])=>{
        const datasets=seriesToDataset(data.charts[name]).filter(ds=>Array.isArray(ds.data) && ds.data.length > 0);
        setChartEmpty(name, datasets.length===0);
        state.charts[key].data.datasets=datasets;
        state.charts[key].update('none');
      });
    }

    function renderTimeline(data){
      el('timeline').innerHTML=[...data.timeline].reverse().map(ev=>`
        <div class="event ${ev.severity}">
          <div class="dot"></div>
          <div><div><strong>${ev.title}</strong> <span class="subtle">R${ev.robot_id} • ${fmtMs(data.mission.elapsed_ms-ev.ts_ms+data.timeline[0]?.ts_ms||0)}</span></div><div class="subtle">${ev.detail}</div></div>
        </div>`).join('');
    }

    function renderComms(data){
      el('comms-body').innerHTML=data.comms.peers.map(peer=>`
        <tr>
          <td>R${peer.id}</td>
          <td><span class="badge ${peer.freshness==='online'?'ok':peer.freshness==='stale'?'warn':'danger'}">${peer.freshness}</span></td>
          <td>${fmtMs(peer.age_ms)}</td>
          <td>${fmtVal(peer.x_surrogate_mm?.toFixed?.(0) ?? peer.x_surrogate_mm,' mm')}</td>
          <td>${fmtVal(peer.eco2,' ppm')}</td>
          <td>${fmtVal(peer.tof_mm,' mm')}</td>
          <td>${fmtVal(peer.tvoc)}</td>
          <td>${peer.state}</td>
        </tr>`).join('');
    }

    function renderRaw(data){
      let content='';
      if(state.rawTab==='local') content=data.raw.local_status;
      if(state.rawTab==='peer') content=JSON.stringify(data.raw.peer_cache,null,2);
      if(state.rawTab==='packet') content=JSON.stringify(data.raw.last_rx_packet,null,2);
      el('raw').textContent=content || 'No data';
      document.querySelectorAll('[data-raw]').forEach(btn=>btn.classList.toggle('active',btn.dataset.raw===state.rawTab));
    }

    async function refresh(){
      try{
        const res=await fetch('/ops/dashboard',{cache:'no-store'});
        if(!res.ok) throw new Error(`HTTP ${res.status}`);
        const data=await res.json();
        state.fetchWarned=false;
        state.data=data;
        renderSummary(data);
        renderCards(data);
        renderMap(data);
        renderCharts(data);
        renderTimeline(data);
        renderComms(data);
        renderRaw(data);
      }catch(err){
        if(!state.fetchWarned){
          console.warn('Dashboard refresh failed:', err);
          state.fetchWarned=true;
        }
      }
    }

    document.querySelectorAll('[data-action]').forEach(btn=>btn.addEventListener('click',()=>sendAction(btn.dataset.action)));
    document.querySelectorAll('[data-raw]').forEach(btn=>btn.addEventListener('click',()=>{state.rawTab=btn.dataset.raw; if(state.data) renderRaw(state.data);}));
    refresh();
    setInterval(refresh,1000);
  </script>
</body>
</html>
)HTML";
}
