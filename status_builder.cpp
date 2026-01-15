#include "status_builder.h"

#include "config.h"
#include "comms.h"
#include "drive.h"
#include "grid_nav.h"
#include "motors.h"
#include "sensors.h"

StatusBuilder::StatusBuilder(
    SensorManager& sensors,
    MotorDriver& motors,
    DriveController& drive,
    EspNowManager& comms,
    GridNavigator& grid)
    : sensors_(sensors), motors_(motors), drive_(drive), comms_(comms), grid_(grid) {}

String StatusBuilder::build(unsigned long now) {
  SensorStatus sensors = sensors_.status();
  EncoderStatus enc = motors_.getEncoderStatus();
  DriveStatus drive = drive_.status();
  CommsStatus comms = comms_.status();
  GridStatus grid = grid_.status();

  String json = "{";
  json += "\"uptime_ms\":" + String(now) + ",";
  json += "\"heap_free\":" + String(ESP.getFreeHeap());

  json += ",\"tof_mm\":";
  json += (sensors.tofValid ? String(sensors.tofMm) : "null");
  json += ",\"tof_age_ms\":" + String(sensors.tofValid ? sensors.tofAgeMs : 0);
  json += ",\"tof_ready\":" + String(sensors.tofReady ? "true" : "false");

  json += ",\"motor_state\":\"" + String(motorStateLabel(drive.motorState)) + "\"";
  json += ",\"motor_state_ms\":" + String(now - drive.motorStateStart);
  json += ",\"motor_speed\":" + String(config::DRIVE_SPEED);
  json += ",\"drive_mode\":\"" + String(driveModeLabel(drive.driveMode)) + "\"";
  json += ",\"path_active\":" + String(drive.driveMode == DriveMode::Path ? "true" : "false");
  json += ",\"path_step\":" + String(drive.pathStepIndex);
  json += ",\"path_steps\":" + String(drive.pathStepCount);
  json += ",\"grid_enabled\":" + String(config::ENABLE_GRID_NAV ? "true" : "false");
  json += ",\"grid_state\":" + String((int)grid.state);
  json += ",\"grid_cx\":" + String(grid.cell.cx);
  json += ",\"grid_cy\":" + String(grid.cell.cy);
  json += ",\"pose_x_mm\":" + String(grid.pose.xMm, 1);
  json += ",\"pose_y_mm\":" + String(grid.pose.yMm, 1);
  json += ",\"pose_heading_deg\":" + String(grid.pose.headingDeg, 1);
  json += ",\"pose_valid\":" + String(grid.pose.valid ? "true" : "false");

  json += ",\"motor_cmd_count\":" + String(motors_.commandCount());
  json += ",\"motor_last_cmd\":\"" + String(motorStateLabel(motors_.lastCommandState())) + "\"";
  json += ",\"motor_last_cmd_ms\":" + String(motors_.lastCommandMs());

  json += ",\"tof_timeout_count\":" + String(sensors.tofTimeoutCount);
  json += ",\"i2c_scan_count\":" + String(sensors.i2cScanCount);
  json += ",\"tof_init_ok\":" + String(sensors.tofInitOk ? "true" : "false");

  json += ",\"sgp_ready\":" + String(sensors.sgpReady ? "true" : "false");
  json += ",\"sgp_type\":\"";
  json += (sensors.sgpType == SgpType::Sgp30 ? "sgp30" : (sensors.sgpType == SgpType::Sgp40 ? "sgp40" : "unknown"));
  json += "\"";
  json += ",\"sgp_age_ms\":" + String(sensors.sgpReady ? sensors.sgpAgeMs : 0);
  json += ",\"sgp_raw\":";
  json += (sensors.sgpHasRaw ? String(sensors.sgpRaw) : "null");
  json += ",\"sgp_co2eq\":";
  json += (sensors.sgpHasAir ? String(sensors.sgpCO2) : "null");
  json += ",\"sgp_tvoc\":";
  json += (sensors.sgpHasAir ? String(sensors.sgpTVOC) : "null");

  json += ",\"mpu_ready\":" + String(sensors.mpuReady ? "true" : "false");
  json += ",\"mpu_whoami\":" + String(sensors.mpuWhoAmI);
  json += ",\"mpu_age_ms\":" + String(sensors.mpuReady ? sensors.mpuAgeMs : 0);
  json += ",\"mpu_ax_g\":" + String(sensors.mpuAxG, 3);
  json += ",\"mpu_ay_g\":" + String(sensors.mpuAyG, 3);
  json += ",\"mpu_az_g\":" + String(sensors.mpuAzG, 3);
  json += ",\"mpu_gx_dps\":" + String(sensors.mpuGxDps, 2);
  json += ",\"mpu_gy_dps\":" + String(sensors.mpuGyDps, 2);
  json += ",\"mpu_gz_dps\":" + String(sensors.mpuGzDps, 2);
  json += ",\"mpu_temp_c\":" + String(sensors.mpuTempC, 2);

  json += ",\"enc_a_ticks\":" + String(enc.aTicks);
  json += ",\"enc_b_ticks\":" + String(enc.bTicks);
  json += ",\"dist_a_mm\":" + String(enc.aDistMm, 1);
  json += ",\"dist_b_mm\":" + String(enc.bDistMm, 1);
  json += ",\"dist_avg_mm\":" + String(enc.avgDistMm, 1);

  json += ",\"rx_interesting\":" + String(comms.rxInteresting ? "true" : "false");
  json += ",\"rx_interesting_ms\":" + String(comms.rxInterestingMs);
  json += ",\"espnow_tx_count\":" + String(comms.txCount);
  json += ",\"espnow_last_tx_ms\":" + String(comms.lastTxMs);
  json += ",\"espnow_last_rx_ms\":" + String(comms.lastRxMs);
  json += ",\"espnow_last_rx_valid\":" + String(comms.lastRxValid ? "true" : "false");
  json += ",\"heartbeat_last_id\":" + String(comms.lastHeartbeatId);
  json += ",\"heartbeat_last_ms\":" + String(comms.lastHeartbeatMs);

  if (comms.lastRxValid) {
    json += ",\"espnow_last_rx_id\":" + String(comms.lastRxData.id);
    json += ",\"espnow_last_rx_swarm\":" + String(comms.lastRxData.swarm_id);
    json += ",\"espnow_last_rx_x\":" + String(comms.lastRxData.position[0], 1);
    json += ",\"espnow_last_rx_num_sensors\":" + String(comms.lastRxData.num_sensors);
    if (comms.lastRxData.num_sensors > 0) {
      json += ",\"espnow_last_rx_s0_type\":" + String(comms.lastRxData.sensors[0].type);
      json += ",\"espnow_last_rx_s0_value\":" + String(comms.lastRxData.sensors[0].value, 1);
    }
  }

  json += "}";
  return json;
}
