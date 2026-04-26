#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>
#include <sys/time.h>

#include "ControlParams.h"
#include "logging.h"

#include <SPI.h>
#include <SD.h>

#define SD_CS_PIN 14
#define SD_SCK_PIN 12
#define SD_MISO_PIN 13
#define SD_MOSI_PIN 11

extern ControlParams cp;   // global instance from main.ino
extern float drift_value;
extern float normSteering;

// -----------------------
// Externs from main.ino
// -----------------------
// In your main.ino you should have:
//   volatile uint16_t s_pw = 1500;
//   portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
extern volatile uint16_t s_pw;
extern float yawRate_dps;
extern portMUX_TYPE s_mux;
extern Servo steerServo;
extern bool settings_changed;

Preferences prefs;
WebServer server(80);
WebSocketsServer ws(81);   // WebSocket on port 81

// ---------- AP credentials ----------
const char* AP_SSID = "JASAGYRO-A1";
const char* AP_PASS = "12345678";

// ---------- Example EPA variables ----------
int epa_low_us   = 1100;
int epa_center_us = 1500;
int epa_high_us  = 1900;

// Parameters are stored in global ControlParams cp

// Replace with your real measurement source (we’ll set it from s_pw snapshot)
int current_steering_us = 1500;

// ---------- Settings variables (8 parameters) ----------

uint32_t lastPush = 0;

int exitSettings = 0;

int ws_speedup = 0;
unsigned long ws_speedup_starttime;


uint64_t syncedUnixMs = 0;
uint32_t syncMillisAtReceive = 0;
bool timeValid = false;
int32_t syncedTzOffsetMin = 0;

// ---------- Helpers ----------
static String htmlHeader(const String& title) {
  String s;
  s += "<!doctype html><html><head>";
  s += "<meta charset='utf-8'>";
  s += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  s += "<title>" + title + "</title>";
  s += "<style>";
  s += "body{font-family:Arial,Helvetica,sans-serif;margin:20px;max-width:720px}";
  s += "a{display:inline-block;margin:6px 0}";
  s += "button{padding:10px 16px;margin:6px 6px 6px 0;font-size:16px}";
  s += "input{padding:8px;font-size:16px;width:180px}";
  s += "label{display:block;margin-top:10px}";
  s += ".card{padding:12px;border:1px solid #ddd;border-radius:10px;margin:12px 0}";
  s += ".row{display:flex;gap:12px;flex-wrap:wrap;align-items:center}";
  s += ".small{color:#666;font-size:14px}";
  s += "</style>";
  s += "</head><body>";
  s += "<h2>" + title + "</h2>";
  return s;
}
static String htmlFooter() { return "</body></html>"; }

static int getArgInt(const char* name, int fallback) {
  return server.hasArg(name) ? server.arg(name).toInt() : fallback;
}
static float getArgFloat(const char* name, float fallback) {
  return server.hasArg(name) ? server.arg(name).toFloat() : fallback;
}

// ---------- Safe snapshot of ISR-updated s_pw ----------
static uint16_t getSteerPwSnapshot() {
  uint16_t v;
  portENTER_CRITICAL(&s_mux);
  v = s_pw;
  //v = steering pwm;
  portEXIT_CRITICAL(&s_mux);
  return v;
}

static float getDriftValueSnapshot() {
  float v;
  v = drift_value;
  return v;
}

void setSystemTimeFromUnixMs(uint64_t unixMs)
{
    struct timeval tv;

    tv.tv_sec  = unixMs / 1000ULL;
    tv.tv_usec = (unixMs % 1000ULL) * 1000ULL;

    settimeofday(&tv, NULL);

    Serial.println("System time updated");

    time_t now;
    time(&now);

    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    char buffer[64];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);

    String timeString = String(buffer);

    Serial.println(timeString);
}

static void setBrowserTime(uint64_t unixMs, int32_t tzOffsetMin = 0) {
  syncedUnixMs = unixMs;
  syncMillisAtReceive = millis();
  syncedTzOffsetMin = tzOffsetMin;
  timeValid = true;

  Serial.printf("Time synced: %llu ms, tz offset min: %ld\n",
                syncedUnixMs, (long)syncedTzOffsetMin);
}

static uint64_t getCurrentUnixMs() {
  if (!timeValid) return 0;
  return syncedUnixMs + (uint32_t)(millis() - syncMillisAtReceive);
}

// ---------- WebSocket events ----------
static void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      IPAddress ip = ws.remoteIP(num);
      Serial.printf("[WS] Client #%u connected from %s\n", num, ip.toString().c_str());
      String msg = "{\"steer\":" + String(getSteerPwSnapshot()) + "}";
      
      ws.sendTXT(num, msg);
    } break;

    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u disconnected\n", num);
      break;

    case WStype_TEXT: {
      Serial.printf("[WS] Received: %.*s\n", (int)length, (const char*)payload);

      String msg = String((const char*)payload);

      // Example expected:
      // {"type":"time_sync","unix_ms":1710000000000,"tz_offset_min":-120}

      if (msg.indexOf("\"type\":\"time_sync\"") >= 0) {
        int p1 = msg.indexOf("\"unix_ms\":");
        if (p1 >= 0) {
          p1 += 10; // skip "unix_ms":
          int p2 = msg.indexOf(",", p1);
          if (p2 < 0) p2 = msg.indexOf("}", p1);

          String unixStr = msg.substring(p1, p2);
          uint64_t unixMs = strtoull(unixStr.c_str(), nullptr, 10);

          int32_t tzOffsetMin = 0;
          int tzPos = msg.indexOf("\"tz_offset_min\":");
          if (tzPos >= 0) {
            tzPos += 16; // skip "tz_offset_min":
            int tzEnd = msg.indexOf(",", tzPos);
            if (tzEnd < 0) tzEnd = msg.indexOf("}", tzPos);
            String tzStr = msg.substring(tzPos, tzEnd);
            tzOffsetMin = tzStr.toInt();
          }

          setBrowserTime(unixMs, tzOffsetMin);
          setSystemTimeFromUnixMs(unixMs);

          ws.sendTXT(num, "{\"type\":\"time_ack\"}");
        }
      }
    } break;

    default:
      break;
  }
}

// ---------- NVS load/save ----------
static void loadSettings() {
  prefs.begin("rc", true);
  epa_low_us   = prefs.getInt("epaL", 1100);
  epa_center_us = prefs.getInt("epaC", 1500);
  epa_high_us  = prefs.getInt("epaH", 1900);
  
  cp.gain = prefs.getFloat("gain_main", 1.0);
  cp.gyro_avg = prefs.getInt("gyro_avg", 6);
  cp.debug_serial = prefs.getInt("debug_serial", 0);
  cp.steering_prio = prefs.getFloat("s_p", 1.0);
  cp.correction_exp = prefs.getFloat("g_exp", 1.0);
  cp.gyro_lp_hz = prefs.getInt("gyro_lp_hz", 20);
  cp.derivative_lp_hz = prefs.getInt("d_lp_hz", 20);
  cp.steer_in_lp_hz = prefs.getInt("s_in_lp", 20);
  cp.steer_out_lp_hz = prefs.getInt("s_out_lp", 20);
  cp.pid_p = prefs.getFloat("pid_p", 5.0);
  cp.pid_d = prefs.getFloat("pid_d", 1.0);
  cp.wobble_det_a = prefs.getFloat("wobble_det_a", 0.2);
  cp.max_d_corr = prefs.getFloat("max_d_corr", 0.05);
  cp.dd_min_steer = prefs.getFloat("dd_min_steer", 0.05);
  cp.dd_min_yaw = prefs.getFloat("dd_min_yaw", 15);
  cp.corr_lp_hz = prefs.getFloat("corr_lp_hz",5);
  cp.acc_lat_multip = prefs.getFloat("acc_lat_multip", 10.0);

  prefs.end();

  Serial.println("Settings loaded from NVS (or defaults)");
}

static bool ensureSdMounted() {
  static bool sdReady = false;

  if (sdReady) return true;

  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD mount failed in settings task");
    sdReady = false;
    return false;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    sdReady = false;
    return false;
  }

  sdReady = true;
  return true;
}

static void handleLogs() {
  String s = htmlHeader("Log Files");
  s += "<div class='card'>";

  if (!ensureSdMounted()) {
    s += "<p>SD card not available</p>";
    s += "<a href='/'>Back</a></div>";
    s += htmlFooter();
    server.send(500, "text/html", s);
    return;
  }

  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    s += "<p>Failed to open SD root</p>";
    s += "<a href='/'>Back</a></div>";
    s += htmlFooter();
    server.send(500, "text/html", s);
    return;
  }

  bool found = false;
  File file = root.openNextFile();

  while (file) {
    if (!file.isDirectory()) {
      String name = file.name();

      // Keep only your log files
      if ((name.startsWith("log_") || name.startsWith("/log_")) && name.endsWith(".bin")) {
        found = true;

        s += "<div>";
        s += "<a href='/download?name=" + name + "'>" + name + "</a>";

        size_t sz = file.size();
        s += " <span class='small'>(" + String((unsigned long)sz) + " bytes)</span>";
        s += "</div>";
      }
    }

    file = root.openNextFile();
  }

  if (!found) {
    s += "<p>No log files found.</p>";
  }

  s += "<br><a href='/'>Back</a>";
  s += "</div>";
  s += htmlFooter();

  server.send(200, "text/html", s);
}

static void handleDownload() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "Missing file name");
    return;
  }

  if (!ensureSdMounted()) {
    server.send(500, "text/plain", "SD not available");
    return;
  }

  String path = server.arg("name");

  // Basic validation
  if (!path.startsWith("/")) path = "/" + path;
  if (path.indexOf("..") >= 0) {
    server.send(400, "text/plain", "Invalid file name");
    return;
  }

  // Optional: block downloads while logger is running
  if (loggingIsRunning()) {
    server.send(409, "text/plain", "Stop logging before downloading files");
    return;
  }

  File file = SD.open(path, FILE_READ);
  if (!file || file.isDirectory()) {
    server.send(404, "text/plain", "File not found");
    return;
  }

  String shortName = path;
  int slash = shortName.lastIndexOf('/');
  if (slash >= 0) shortName = shortName.substring(slash + 1);

  server.sendHeader("Content-Type", "application/octet-stream");
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + shortName + "\"");
  server.sendHeader("Connection", "close");

  server.streamFile(file, "application/octet-stream");
  file.close();
}

static void saveEpa() {
  prefs.begin("rc", false);
  prefs.putInt("epaL", epa_low_us);
  prefs.putInt("epaC", epa_center_us);
  prefs.putInt("epaH", epa_high_us);
  prefs.end();
}

static void saveParameters() {

  prefs.begin("rc", false);
  prefs.putFloat("gain_main", cp.gain);
  prefs.putInt("gyro_avg", cp.gyro_avg);
  prefs.putInt("debug_serial", cp.debug_serial);
  prefs.putFloat("s_p", cp.steering_prio);
  prefs.putFloat("g_exp", cp.correction_exp);
  prefs.putInt("gyro_lp_hz", cp.gyro_lp_hz);
  prefs.putInt("d_lp_hz", cp.derivative_lp_hz);
  prefs.putInt("s_in_lp", cp.steer_in_lp_hz);
  prefs.putInt("s_out_lp", cp.steer_out_lp_hz);
  prefs.putFloat("pid_p", cp.pid_p);
  prefs.putFloat("pid_d", cp.pid_d);
  prefs.putFloat("wobble_det_a", cp.wobble_det_a);
  prefs.putFloat("max_d_corr", cp.max_d_corr);
  prefs.putFloat("dd_min_steer", cp.dd_min_steer);
  prefs.putFloat("dd_min_yaw", cp.dd_min_yaw);
  prefs.putFloat("dd_multiplier", cp.dd_multiplier);
  prefs.putFloat("corr_lp_hz", cp.corr_lp_hz);
  prefs.putFloat("acc_lat_multip", cp.acc_lat_multip);

  prefs.end();

  settings_changed = true;
}

// ---------- Pages ----------
static void handleRoot() {
  if (server.hasArg("logging")) {
    String val = server.arg("logging");

    if (val == "1") {
      if (!loggingIsRunning()) {
        Serial.println("start logging");

        time_t now;
        time(&now);
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        char timeBuf[32];
        strftime(timeBuf, sizeof(timeBuf), "%Y%m%d_%H%M%S", &timeinfo);
        char filename[64];
        snprintf(filename, sizeof(filename), "/log_%s.bin", timeBuf);
        loggingBegin(SD_CS_PIN, SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, filename,0);

      }
    } 
    else if (val == "0") {
      if (loggingIsRunning()) {
        Serial.println("stop logging");
        loggingStop();
      }
    }

    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }
  
  String s = htmlHeader("ESP32 Control");
  s += "<div class='card'>";
  s += "<p>Main page</p>";
  s += "<a href='/epa'>EPA</a><br>";
  s += "<a href='/settings'>Settings</a><br>";
  s += "<a href='/monitor'>Monitor</a><br>";
  if(!loggingIsRunning()) {
    s += "<a href='/?logging=1'>Start Logger</a><br>";
    } else {
    s += "<a href='/?logging=0'>Stop Logger</a><br>";
  }
  s += "<br><br><br>";
  s += "<a href='/logs'>Logs</a><br>";
  s += "<p><a href='/exit'>Exit</a><br>";
  s += "</div>";
  s += "<div class='small'>AP IP: " + WiFi.softAPIP().toString() + "</div>";
    s += R"rawliteral(
<script>
let ws;
function connectWS() {
  ws = new WebSocket(`ws://${location.hostname}:81/`);
  ws.onopen = () => {
    const now = new Date();
    ws.send(JSON.stringify({
      type: "time_sync",
      unix_ms: Date.now(),
      tz_offset_min: now.getTimezoneOffset()
    }));
  };
  
  ws.onmessage = (evt) => {
    try {
      const d = JSON.parse(evt.data);
      } catch(e) {}
  };
  ws.onclose = () => setTimeout(connectWS, 1000);
}
connectWS();
</script>
)rawliteral";
  s += htmlFooter();
  server.send(200, "text/html", s);
}

static void handleEPA() {
  String msg = server.hasArg("msg") ? server.arg("msg") : "";

  String s = htmlHeader("EPA");
  s += "<div class='card'>";
  s += "<div class='row'>";
  s += "<form method='POST' action='/epa/action'><button name='cmd' value='low'>Low</button></form>";
  s += "<form method='POST' action='/epa/action'><button name='cmd' value='center'>Center</button></form>";
  s += "<form method='POST' action='/epa/action'><button name='cmd' value='high'>High</button></form>";
  s += "<form method='POST' action='/epa/action'><button name='cmd' value='save'>Save</button></form>";
  s += "</div>";

  if (msg.length()) s += "<p><b>Status:</b> " + msg + "</p>";

  // Live value from websocket
  s += "<p><b>Current steering:</b> <span id='steer'>--</span> us</p>";

  s += "<p><b>EPA low:</b> " + String(epa_low_us) + " us<br>";
  s += "<b>EPA Center:</b> " + String(epa_center_us) + " us<br>";
  s += "<b>EPA high:</b> " + String(epa_high_us) + " us</p>";

  s += "<a href='/'>Back</a>";
  s += "</div>";

  s += R"rawliteral(
<script>
let ws;
function connectWS() {
  ws = new WebSocket(`ws://${location.hostname}:81/`);
  ws.onmessage = (evt) => {
    try {
      const d = JSON.parse(evt.data);
      if (d.steer !== undefined) document.getElementById('steer').textContent = d.steer;
    } catch(e) {}
  };
  ws.onclose = () => setTimeout(connectWS, 1000);
}
connectWS();
</script>
)rawliteral";

  s += htmlFooter();
  server.send(200, "text/html", s);
}


static void handleMonitor() {
  ws_speedup = 1;
  
  String msg = server.hasArg("msg") ? server.arg("msg") : "";

  String s = htmlHeader("Monitor");
  s += "<div class='card'>";
  s += "<div class='row'>";
  
  s += "</div>";

  if (msg.length()) s += "<p><b>Status:</b> " + msg + "</p>";

  // Live value from websocket
  s += "<p><b>Current steering:</b> <span id='steer'>--</span> us</p>";
  s += "<p><b>Current driftvalue:</b> <span id='dd_value'>--</span></p>";
  s += "<p><b>Current normSteering:</b> <span id='normSteering'>--</span></p>";
  
  s += "<a href='/'>Back</a>";
  s += "</div>";

  s += R"rawliteral(
<script>
let ws;
function connectWS() {
  ws = new WebSocket(`ws://${location.hostname}:81/`);
  ws.onopen = () => {
    const now = new Date();
    ws.send(JSON.stringify({
      type: "time_sync",
      unix_ms: Date.now(),
      tz_offset_min: now.getTimezoneOffset()
    }));
  };
  
  ws.onmessage = (evt) => {
    try {
      const d = JSON.parse(evt.data);
      if (d.steer !== undefined) document.getElementById('steer').textContent = d.steer;
      if (d.dd_value !== undefined) document.getElementById('dd_value').textContent = d.dd_value;
      if (d.normSteering !== undefined) document.getElementById('normSteering').textContent = d.normSteering;
    } catch(e) {}
  };
  ws.onclose = () => setTimeout(connectWS, 1000);
}
connectWS();
</script>
)rawliteral";

  s += htmlFooter();
  server.send(200, "text/html", s);
}

static void handleEPAAction() {
  String cmd = server.hasArg("cmd") ? server.arg("cmd") : "";

  int measured = (int)getSteerPwSnapshot();
  current_steering_us = measured;

  String msg = "Unknown command";
  if (cmd == "low")   { epa_low_us = s_pw;   msg = "Recorded LOW = "   + String(epa_low_us) + " us"; }
  if (cmd == "center") { epa_center_us = s_pw; msg = "Recorded CENTER = " + String(epa_center_us) + " us"; }
  if (cmd == "high")  { epa_high_us = s_pw;  msg = "Recorded HIGH = "  + String(epa_high_us) + " us"; }
  if (cmd == "save")   { saveEpa(); msg = "EPA values saved to persistent memory"; }

  server.sendHeader("Location", "/epa?msg=" + msg);
  server.send(303);
}

static void handleSettings() {
  String msg = server.hasArg("msg") ? server.arg("msg") : "";

  String s = htmlHeader("Settings");
  s += "<div class='card'>";

  if (msg.length()) s += "<p><b>Status:</b> " + msg + "</p>";

  s += "<style>"
       ".formgrid{display:grid;grid-template-columns:140px auto;gap:10px 12px;align-items:center;}"
       ".formgrid label{margin:0;font-weight:600;}"
       ".val8{width:8ch;}"
       "input[type=number]{padding:8px;font-size:16px;}"
       "</style>";

  s += "<form method='POST' action='/settings/set'>";
  s += "<div class='formgrid'>";

  s += "<label for='gain'>gain</label><input class='val8' id='gain' name='gain' type='number' step='0.05' value='" + String(cp.gain) + "'>";
  s += "<label for='gyro_avg'>gyro_avg</label><input class='val8' id='gyro_avg' name='gyro_avg' type='number' step='1' value='" + String(cp.gyro_avg) + "'>";
  s += "<label for='debug_serial'>debug_serial</label><input class='val8' id='debug_serial' name='debug_serial' type='number' step='1' value='" + String(cp.debug_serial) + "'>";
  s += "<label for='steering_prio'>steering_prio</label><input class='val8' id='steering_prio' name='steering_prio' type='number' step='0.05' value='" + String(cp.steering_prio) + "'>";
  s += "<label for='pid_p'>pid_p</label><input class='val8' id='pid_p' name='pid_p' type='number' step='0.05' value='" + String(cp.pid_p) + "'>";
  s += "<label for='pid_d'>pid_d</label><input class='val8' id='pid_d' name='pid_d' type='number' step='0.05' value='" + String(cp.pid_d) + "'>";
  s += "<label for='correction_exp'>correction_exp</label><input class='val8' id='correction_exp' name='correction_exp' type='number' step='0.01' value='" + String(cp.correction_exp) + "'>";
  s += "<label for='gyro_lp_hz'>gyro_lp_hz</label><input class='val8' id='gyro_lp_hz' name='gyro_lp_hz' type='number' step='1' value='" + String(cp.gyro_lp_hz) + "'>";
  s += "<label for='derivative_lp_hz'>derivative_lp_hz</label><input class='val8' id='derivative_lp_hz' name='derivative_lp_hz' type='number' step='1' value='" + String(cp.derivative_lp_hz) + "'>";
  s += "<label for='steer_in_lp_hz'>steer_in_lp_hz</label><input class='val8' id='steer_in_lp_hz' name='steer_in_lp_hz' type='number' step='1' value='" + String(cp.steer_in_lp_hz) + "'>";
  s += "<label for='steer_out_lp_hz'>steer_out_lp_hz</label><input class='val8' id='steer_out_lp_hz' name='steer_out_lp_hz' type='number' step='1' value='" + String(cp.steer_out_lp_hz) + "'>";
  s += "<label for='wobble_det_a'>wobble_det_amplitude</label><input class='val8' id='wobble_det_a' name='wobble_det_a' type='number' step='0.01' value='" + String(cp.wobble_det_a) + "'>";
  s += "<label for='max_d_corr'>max_d_corr</label><input class='val8' id='max_d_corr' name='max_d_corr' type='number' step='0.001' value='" + String(cp.max_d_corr) + "'>";
  s += "<label for='dd_min_steer'>drift_detect_min_steer</label><input class='val8' id='dd_min_steer' name='dd_min_steer' type='number' step='0.005' value='" + String(cp.dd_min_steer) + "'>";
  s += "<label for='dd_min_yaw'>drift_detect_min_yaw</label><input class='val8' id='dd_min_yaw' name='dd_min_yaw' type='number' step='0.5' value='" + String(cp.dd_min_yaw) + "'>";
  s += "<label for='dd_multiplier'>drift_detect_multipl.</label><input class='val8' id='dd_multiplier' name='dd_multiplier' type='number' step='0.1' value='" + String(cp.dd_multiplier) + "'>";
  s += "<label for='corr_lp_hz'>corr_lp_hz</label><input class='val8' id='corr_lp_hz' name='corr_lp_hz' type='number' step='0.1' value='" + String(cp.corr_lp_hz) + "'>";
  s += "<label for='acc_lat_multip'>acc_lat_multip</label><input class='val8' id='acc_lat_multip' name='acc_lat_multip' type='number' step='0.1' value='" + String(cp.acc_lat_multip) + "'>";  


  s += "</div>";

  s += "<div style='margin-top:14px'><button type='submit'>Set</button></div>";
  s += "</form>";

  s += "<a href='/'>Back</a>";
  s += "</div>";
  s += htmlFooter();

  server.send(200, "text/html", s);
}

static void handleSettingsSet() {
  cp.gain = getArgFloat("gain", cp.gain);
  cp.gyro_avg = getArgInt("gyro_avg", cp.gyro_avg);
  cp.debug_serial = getArgInt("debug_serial", cp.debug_serial);
  cp.steering_prio = getArgFloat("steering_prio", cp.steering_prio);
  cp.pid_p = getArgFloat("pid_p", cp.pid_p);
  cp.pid_d = getArgFloat("pid_d", cp.pid_d);
  cp.correction_exp = getArgFloat("correction_exp", cp.correction_exp);
  cp.gyro_lp_hz = getArgInt("gyro_lp_hz", cp.gyro_lp_hz);
  cp.derivative_lp_hz = getArgInt("derivative_lp_hz", cp.derivative_lp_hz);
  cp.steer_in_lp_hz = getArgInt("steer_in_lp_hz", cp.steer_in_lp_hz);
  cp.steer_out_lp_hz = getArgInt("steer_out_lp_hz", cp.steer_out_lp_hz);
  cp.wobble_det_a = getArgFloat("wobble_det_a", cp.wobble_det_a);
  cp.max_d_corr = getArgFloat("max_d_corr", cp.max_d_corr);
  cp.dd_min_steer = getArgFloat("dd_min_steer", cp.dd_min_steer);
  cp.dd_min_yaw = getArgFloat("dd_min_yaw", cp.dd_min_yaw);
  cp.dd_multiplier = getArgFloat("dd_multiplier", cp.dd_multiplier);
  cp.corr_lp_hz = getArgFloat("corr_lp_hz", cp.corr_lp_hz);
  cp.acc_lat_multip = getArgFloat("acc_lat_multip", cp.acc_lat_multip);
  
  //Serial.print("cp.gyro_avg="); Serial.print(cp.gyro_avg);

  saveParameters();

  server.sendHeader("Location", "/settings?msg=Updated");
  server.send(303);
}

static void handleExit() {
  server.send(200, "text/html", "EXIT");
  exitSettings = 1;
}

static void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ---------- Public API ----------
void setupSettings() {
  loadSettings();

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);

  Serial.println();
  Serial.print("AP start: ");
  Serial.println(ok ? "OK" : "FAIL");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/epa", HTTP_GET, handleEPA);
  server.on("/epa/action", HTTP_POST, handleEPAAction);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/settings/set", HTTP_POST, handleSettingsSet);
  server.on("/logs", HTTP_GET, handleLogs);
  server.on("/download", HTTP_GET, handleDownload);
  server.on("/monitor", HTTP_GET, handleMonitor);
  server.on("/exit", HTTP_GET, handleExit);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started.");

  ws.begin();
  ws.onEvent(onWsEvent);
  Serial.println("WebSocket server started on port 81.");
}

void stopSettings() {
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
}

bool makeSettings() {
  //setupSettings();

  uint32_t start = millis();
  uint32_t interval;
  
  if(exitSettings==1) {
    return false;
  }

  //while (millis() - start < 600000 && exitSettings == 0) {
    server.handleClient();
    ws.loop();

    // Push ~5 Hz
    if(ws_speedup==1) {
      ws_speedup_starttime = millis();
      ws_speedup = 0;
    }
    
    if(ws_speedup_starttime + 60000 > millis()) {
      interval = 200;
    } else {
      interval = 1000;
    }

    if (millis() - lastPush >= interval) {
      lastPush = millis();
      String msg = "{\"steer\":" + String(getSteerPwSnapshot()) + ",\"dd_value\":" + String(getDriftValueSnapshot()) + ",\"normSteering\":" +  String(normSteering) + "}";
      ws.broadcastTXT(msg);
    }
  return true;
  
}