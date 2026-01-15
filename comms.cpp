#include "comms.h"

#include <WiFi.h>
#include "config.h"
#include "logging.h"
#include "led_controller.h"
#include "motors.h"
#include "sensors.h"
#include "grid_nav.h"

namespace {
EspNowManager* g_instance = nullptr;
constexpr unsigned long kCommandCooldownMs = 500;

void onEspNowRecvThunk(const uint8_t* mac, const uint8_t* incomingData, int len) {
  if (g_instance) {
    g_instance->onReceive(mac, incomingData, len);
  }
}

uint32_t fnv1a32(const uint8_t* data, size_t len) {
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < len; i++) {
    hash ^= data[i];
    hash *= 16777619u;
  }
  return hash;
}
}  // namespace

EspNowManager::EspNowManager(
    Logger& log,
    MotorDriver& motors,
    SensorManager& sensors,
    LedController* led,
    GridNavigator* gridNav)
    : log_(log), motors_(motors), sensors_(sensors), led_(led), gridNav_(gridNav) {}

void EspNowManager::begin() {
  if (esp_now_init() != ESP_OK) {
    log_.logMsg("ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  memcpy(peerInfo.peer_addr, broadcastAddr, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    log_.logMsg("ESP-NOW add peer failed");
    return;
  }

  g_instance = this;
  esp_now_register_recv_cb(onEspNowRecvThunk);
  log_.logFmt("ESP-NOW ready, payload bytes=%u", (unsigned)sizeof(EspNowPacket));
}

void EspNowManager::update(unsigned long now) {
  if (status_.rxInteresting && (now - status_.rxInterestingMs) >= 1000) {
    status_.rxInteresting = false;
  }
  if (status_.remoteCo2Alert && (now - status_.remoteCo2Ms) >= 2000) {
    status_.remoteCo2Alert = false;
  }
  sendNow(now);
  sendHeartbeat(now);
}

bool EspNowManager::isAvoiding(unsigned long now) const {
  if (!status_.rxInteresting) return false;
  return (now - status_.rxInterestingMs) < 1000;
}

CommsStatus EspNowManager::status() const {
  return status_;
}

void EspNowManager::onReceive(const uint8_t* mac, const uint8_t* incomingData, int len) {
  (void)mac;
  if (len < (int)sizeof(EspNowPacket)) return;
  EspNowPacket msg;
  memcpy(&msg, incomingData, sizeof(EspNowPacket));
  uint32_t hash = fnv1a32(reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
  unsigned long now = millis();
  bool isRepeat = (hash == lastMsgHash_) && (now - lastMsgMs_ < kCommandCooldownMs);
  if (!isRepeat && led_) {
    led_->pulseCommand();
  }
  lastMsgHash_ = hash;
  lastMsgMs_ = now;
  status_.lastRxData = msg;
  status_.lastRxMs = millis();
  status_.lastRxValid = true;
  if (msg.command[0] != '\0') {
    handleCommand(msg.command);
  }

  if (msg.swarm_id != config::SWARM_ID || msg.id == config::DEVICE_ID) return;

  float dx = msg.position[0] - status_.localPosX;
  if (dx < 0) dx = -dx;
  bool nearby = dx <= config::NEARBY_MM;
  bool alert = false;

  for (uint8_t i = 0; i < msg.num_sensors && i < 10; i++) {
    if (msg.sensors[i].type == config::SENSOR_HEARTBEAT) {
      status_.lastHeartbeatId = msg.id;
      status_.lastHeartbeatMs = millis();
      if (led_) {
        led_->pulseHeartbeat();
      }
    }
    if (msg.sensors[i].type == config::SENSOR_CO2 && msg.sensors[i].value >= config::CO2_ALERT) {
      alert = true;
      status_.remoteCo2Alert = true;
      status_.remoteCo2Ms = millis();
      break;
    }
    if (msg.sensors[i].type == config::SENSOR_HEARTBEAT && msg.sensors[i].value == 0.0f) {
      if (led_) {
        led_->setStop(true);
      }
    }
  }

  if (nearby && alert) {
    status_.rxInteresting = true;
    status_.rxInterestingMs = millis();
  }
}

void EspNowManager::sendNow(unsigned long now) {
  if (now < nextSendMs_) return;
  nextSendMs_ = now + config::ESP_NOW_PERIOD_MS;

  EncoderStatus enc = motors_.getEncoderStatus();
  status_.localPosX = enc.avgDistMm;

  memset(&sendData_, 0, sizeof(sendData_));
  sendData_.id = config::DEVICE_ID;
  sendData_.swarm_id = config::SWARM_ID;
  sendData_.position[0] = enc.avgDistMm;
  sendData_.position[1] = 0.0f;
  sendData_.position[2] = 0.0f;

  uint8_t count = 0;
  sendData_.sensors[count++] = {config::SENSOR_DIST, enc.avgDistMm};
  if (sensors_.hasAir()) {
    sendData_.sensors[count++] = {config::SENSOR_CO2, (float)sensors_.sgpCO2()};
    sendData_.sensors[count++] = {config::SENSOR_TVOC, (float)sensors_.sgpTVOC()};
  }
  if (sensors_.hasTof()) {
    sendData_.sensors[count++] = {config::SENSOR_TOF, (float)sensors_.tofMm()};
  }
  sendData_.num_sensors = count;
  sendData_.command[0] = '\0';

  if (sendPacket(sendData_)) {
    status_.txCount++;
    status_.lastTxMs = now;
  }
}

void EspNowManager::sendHeartbeat(unsigned long now) {
  if (now < nextHeartbeatMs_) return;
  nextHeartbeatMs_ = now + config::ESP_NOW_HEARTBEAT_MS;

  memset(&sendData_, 0, sizeof(sendData_));
  sendData_.id = config::DEVICE_ID;
  sendData_.swarm_id = config::SWARM_ID;
  sendData_.position[0] = status_.localPosX;
  sendData_.position[1] = 0.0f;
  sendData_.position[2] = 0.0f;
  sendData_.num_sensors = 1;
  sendData_.sensors[0] = {config::SENSOR_HEARTBEAT, (float)now / 1000.0f};
  sendData_.command[0] = '\0';

  if (sendPacket(sendData_)) {
    status_.txCount++;
    status_.lastTxMs = now;
  }
}

bool EspNowManager::sendPacket(const EspNowPacket& packet) {
  uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  return esp_now_send(broadcastAddr, (uint8_t*)&packet, sizeof(packet)) == ESP_OK;
}

void EspNowManager::sendStop() {
  EspNowPacket packet = {};
  packet.id = config::DEVICE_ID;
  packet.swarm_id = config::SWARM_ID;
  packet.position[0] = status_.localPosX;
  packet.position[1] = 0.0f;
  packet.position[2] = 0.0f;
  packet.num_sensors = 1;
  packet.sensors[0] = {config::SENSOR_HEARTBEAT, 0.0f};
  packet.command[0] = '\0';
  sendPacket(packet);
  if (led_) {
    led_->setStop(true);
  }
}

void EspNowManager::sendBroadcastCommand(const char* command) {
  EspNowPacket packet = {};
  packet.id = config::DEVICE_ID;
  packet.swarm_id = config::SWARM_ID;
  packet.position[0] = status_.localPosX;
  packet.position[1] = 0.0f;
  packet.position[2] = 0.0f;
  packet.num_sensors = 0;
  if (command) {
    strncpy(packet.command, command, sizeof(packet.command) - 1);
    packet.command[sizeof(packet.command) - 1] = '\0';
  }
  sendPacket(packet);
}

void EspNowManager::handleCommand(const char* command) {
  if (!command || command[0] == '\0' || !gridNav_) return;
  if (strcmp(command, "run_swarm_timed_scan") != 0) return;
  switch (config::DEVICE_ID) {
    case 1:
      gridNav_->setEndPoint(EndPoint::EndA);
      break;
    case 2:
      gridNav_->setEndPoint(EndPoint::EndC);
      break;
    case 3:
      gridNav_->setEndPoint(EndPoint::EndE);
      break;
    default:
      log_.logMsg("Swarm timed grid scan: invalid DEVICE_ID");
      break;
  }
  gridNav_->setTimedGrid(true);
  gridNav_->setScanMode(true);
  gridNav_->setEnabled(true);
  log_.logMsg("Swarm timed grid scan command handled");
}
