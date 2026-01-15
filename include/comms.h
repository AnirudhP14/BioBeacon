#ifndef SWARMBOT_COMMS_H
#define SWARMBOT_COMMS_H

#include <Arduino.h>
#include <esp_now.h>

class Logger;
class SensorManager;
class LedController;
class MotorDriver;
class GridNavigator;

typedef struct {
  uint8_t id;
  uint8_t swarm_id;
  float position[3];
  uint8_t num_sensors;
  struct {
    uint8_t type;
    float value;
  } sensors[10];
  char command[24];
} __attribute__((packed)) EspNowPacket;

struct CommsStatus {
  bool rxInteresting = false;
  unsigned long rxInterestingMs = 0;
  bool remoteCo2Alert = false;
  unsigned long remoteCo2Ms = 0;
  bool lastRxValid = false;
  unsigned long lastRxMs = 0;
  unsigned long txCount = 0;
  unsigned long lastTxMs = 0;
  uint8_t lastHeartbeatId = 0;
  unsigned long lastHeartbeatMs = 0;
  EspNowPacket lastRxData = {};
  float localPosX = 0.0f;
};

class EspNowManager {
 public:
  EspNowManager(Logger& log, MotorDriver& motors, SensorManager& sensors, LedController* led, GridNavigator* gridNav);
  void begin();
  void update(unsigned long now);
  bool isAvoiding(unsigned long now) const;
  CommsStatus status() const;
  void onReceive(const uint8_t* mac, const uint8_t* incomingData, int len);
  void sendStop();
  void sendBroadcastCommand(const char* command);
  void handleCommand(const char* command);

 private:
  void sendNow(unsigned long now);
  void sendHeartbeat(unsigned long now);
  bool sendPacket(const EspNowPacket& packet);

  Logger& log_;
  MotorDriver& motors_;
  SensorManager& sensors_;
  LedController* led_ = nullptr;
  GridNavigator* gridNav_ = nullptr;
  CommsStatus status_;
  uint32_t lastMsgHash_ = 0;
  unsigned long lastMsgMs_ = 0;
  unsigned long nextSendMs_ = 0;
  unsigned long nextHeartbeatMs_ = 0;
  EspNowPacket sendData_ = {};
};

#endif  // SWARMBOT_COMMS_H
