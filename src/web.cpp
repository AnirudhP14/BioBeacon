#include "web.h"

#include "comms.h"
#include "config.h"
#include "drive.h"
#include "grid_nav.h"
#include "logging.h"
#include "status_builder.h"
#include "config.h"

WebServerManager::WebServerManager(
    Logger& log,
    DriveController& drive,
    StatusBuilder& statusBuilder,
    GridNavigator& gridNav,
    EspNowManager& comms)
    : log_(log), drive_(drive), statusBuilder_(statusBuilder), gridNav_(gridNav), comms_(comms), server_(80) {}

void WebServerManager::begin() {
  server_.on("/", [this]() { handleRoot(); });
  server_.on("/status", [this]() { handleStatus(); });
  server_.on("/clear", [this]() { handleClear(); });
  server_.on("/drive", [this]() { handleDrive(); });
  server_.on("/swarm_timed_grid_scan", HTTP_GET, [this]() { handleSwarmTimedGridScan(); });
  server_.begin();

  log_.logMsg("HTTP server started");
  log_.logMsg("Endpoints:");
  log_.logMsg("  /        -> logs");
  log_.logMsg("  /status  -> health");
  log_.logMsg("  /clear   -> clear logs");
  log_.logMsg("  /drive   -> path");
}

void WebServerManager::handleClient() {
  server_.handleClient();
}

bool WebServerManager::co2TestEnabled() const {
  return co2TestMode_;
}

void WebServerManager::handleRoot() {
  server_.send(200, "text/plain", log_.buffer());
}

void WebServerManager::handleStatus() {
  String json = statusBuilder_.build(millis());
  server_.send(200, "application/json", json);
}

void WebServerManager::handleClear() {
  log_.clear();
  server_.send(200, "text/plain", "Log cleared");
}

void WebServerManager::handleDrive() {
  if (!server_.hasArg("path") && !server_.hasArg("mode")) {
    String html;
    html += "<!doctype html><html><head><meta charset=\"utf-8\">";
    html += "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">";
    html += "<title>Drive Path</title></head><body>";
    html += "<h3>Drive Path</h3>";
    html += "<form method=\"GET\" action=\"/drive\">";
    html += "<label>Mode: <input name=\"mode\" value=\"path\" size=\"8\"></label><br>";
    html += "<label>Pattern: <input name=\"pattern\" value=\"lawnmower\" size=\"10\"></label><br>";
    html += "<label>Nav: <select name=\"nav\"><option value=\"encoder\">Encoder+IMU</option><option value=\"timed\">TimedGrid</option></select></label><br>";
    html += "<label>End Point: <select name=\"end\"><option value=\"e\">E4</option><option value=\"c\">C4</option><option value=\"a\">A4</option></select></label><br>";
    html += "<label>Scan: <input type=\"checkbox\" name=\"scan\" value=\"1\"></label><br>";
    html += "<label>Timed Forward Sec: <input name=\"t_fwd\" value=\"";
    html += String(config::TIMED_FORWARD_SEC_PER_CELL, 2);
    html += "\" size=\"6\"></label><br>";
    html += "<label>Timed Turn 90 Sec: <input name=\"t_turn\" value=\"";
    html += String(config::TIMED_TURN_90_SEC, 2);
    html += "\" size=\"6\"></label><br>";
    html += "<label>Timed Drive Speed: <input name=\"t_speed\" value=\"";
    html += String(config::TIMED_DRIVE_SPEED);
    html += "\" size=\"6\"></label><br>";
    html += "<label>Path: <input name=\"path\" value=\"f1,br,b1\" size=\"16\"></label> ";
    html += "<button type=\"submit\">Submit</button></form>";
    html += "<p>Nav mode: ";
    html += gridNav_.isTimedGrid() ? "TimedGrid" : "Encoder+IMU";
    html += "</p>";
    html += "<form method=\"GET\" action=\"/drive\">";
    html += "<input type=\"hidden\" name=\"mode\" value=\"stop\">";
    html += "<button type=\"submit\">Stop Swarm</button></form>";
    html += "<form method=\"GET\" action=\"/drive\">";
    html += "<input type=\"hidden\" name=\"mode\" value=\"co2test\">";
    html += "<button type=\"submit\">Toggle CO2 Test</button></form>";
    html += "<button onclick=\"fetch('/swarm_timed_grid_scan')\">Swarm Timed Grid Scan</button>";
    html += "<p>Tokens: fN=forward N sec, bN=backward N sec, lN=left N sec, rN=right N sec, br=brake 1 sec.</p>";
    html += "<p>Grid: mode=grid, pattern=lawnmower|concentric, optional cx/cy, row=lr|rl.</p>";
    html += "</body></html>";
    server_.send(200, "text/html", html);
    return;
  }

  String mode = server_.arg("mode");
  mode.trim();
  mode.toLowerCase();

  if (mode == "stop") {
    comms_.sendStop();
    server_.send(200, "text/plain", "Stop sent");
    return;
  }

  if (mode == "resume") {
    drive_.setMode(DriveMode::Auto);
    gridNav_.setEnabled(false);
    server_.send(200, "text/plain", "Resumed");
    return;
  }

  if (mode == "co2test") {
    co2TestMode_ = !co2TestMode_;
    server_.send(200, "text/plain", co2TestMode_ ? "CO2 test ON" : "CO2 test OFF");
    return;
  }

  if (mode == "grid") {
    String pattern = server_.arg("pattern");
    pattern.trim();
    pattern.toLowerCase();
    String nav = server_.arg("nav");
    nav.trim();
    nav.toLowerCase();
    gridNav_.setTimedGrid(nav == "timed");
    gridNav_.setScanMode(server_.hasArg("scan"));
    if (nav != "timed") {
      gridNav_.setUseTimed(false);
    }
    if (server_.hasArg("end")) {
      String end = server_.arg("end");
      end.trim();
      end.toLowerCase();
      if (end == "a") {
        gridNav_.setEndPoint(EndPoint::EndA);
      } else if (end == "c") {
        gridNav_.setEndPoint(EndPoint::EndC);
      } else {
        gridNav_.setEndPoint(EndPoint::EndE);
      }
    }
    if (server_.hasArg("t_fwd")) {
      float tFwd = server_.arg("t_fwd").toFloat();
      gridNav_.setTimedForwardSec(tFwd);
    }
    if (server_.hasArg("t_turn")) {
      float tTurn = server_.arg("t_turn").toFloat();
      gridNav_.setTimedTurn90Sec(tTurn);
    }
    if (server_.hasArg("t_speed")) {
      int tSpeed = server_.arg("t_speed").toInt();
      gridNav_.setTimedDriveSpeed(tSpeed);
    }
    if (pattern == "concentric") {
      gridNav_.setTraversal(TraversalPattern::Concentric);
    } else {
      gridNav_.setTraversal(TraversalPattern::LawnMower);
    }

    if (server_.hasArg("cx") && server_.hasArg("cy")) {
      GridCell cell;
      cell.cx = server_.arg("cx").toInt();
      cell.cy = server_.arg("cy").toInt();
      gridNav_.setStartCellOverride(cell);
    }

    if (server_.hasArg("row")) {
      String row = server_.arg("row");
      row.trim();
      row.toLowerCase();
      if (row == "rl") {
        gridNav_.setRowDirection(RowDirection::RightToLeft);
      } else {
        gridNav_.setRowDirection(RowDirection::LeftToRight);
      }
    }

    gridNav_.setEnabled(true);
    drive_.setMode(DriveMode::Auto);
    server_.send(200, "text/plain", "Grid traversal enabled");
    return;
  }

  gridNav_.setEnabled(false);
  gridNav_.setTimedGrid(false);
  gridNav_.setScanMode(false);
  gridNav_.setUseTimed(false);
  if (mode == "path") {
    co2TestMode_ = false;
  }
  String error;
  if (!drive_.setPathFromString(server_.arg("path"), error)) {
    server_.send(400, "text/plain", error);
    return;
  }

  drive_.setMode(DriveMode::Path);
  // LED STOP latch is cleared when drive becomes active in loop.
  server_.send(200, "text/plain", "Path accepted");
}

void WebServerManager::handleSwarmTimedGridScan() {
  comms_.sendBroadcastCommand("run_swarm_timed_scan");
  comms_.handleCommand("run_swarm_timed_scan");
  server_.send(200, "text/plain", "Swarm Timed Grid Scan command broadcasted");
}
