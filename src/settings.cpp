#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>

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

float gain_main = 1;
int gyro_avg = 6;
int deriv_yaw_window = 15;
int deriv_steer_window = 5;
float steer_prio = 1;
float gyro_dp = 0.5;
int return_damping = 5;
float gain_exp = 1;
int gyro_lp_hz=20;
int derivative_lp_hz=20;
int servo_in_lp_hz=20;
int servo_out_lp_hz=20;

// Replace with your real measurement source (we’ll set it from s_pw snapshot)
int current_steering_us = 1500;

// ---------- Settings variables (8 parameters) ----------



int exitSettings = 0;

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
  //v = yawRate_dps;
  portEXIT_CRITICAL(&s_mux);
  return v;
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

    case WStype_TEXT:
      Serial.printf("[WS] Received: %.*s\n", (int)length, (const char*)payload);
      break;

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
  
  gain_main = prefs.getFloat("gain_main", 1);
  gyro_avg = prefs.getInt("gyro_avg", 6);
  deriv_yaw_window = prefs.getInt("d_y_a", 15);
  deriv_steer_window = prefs.getInt("d_y_s", 15);
  steer_prio = prefs.getFloat("s_p", 1);
  gyro_dp = prefs.getFloat("g_dp", 0.5);
  return_damping = prefs.getInt("return_damp", 5);
  gain_exp = prefs.getFloat("g_exp", 1);
  gyro_lp_hz = prefs.getInt("gyro_lp_hz", 20);
  derivative_lp_hz = prefs.getInt("d_lp_hz", 20);
  servo_in_lp_hz = prefs.getInt("s_in_lp", 20);
  servo_out_lp_hz = prefs.getInt("s_out_lp", 20);

  
  prefs.end();

  Serial.println("Settings loaded from NVS (or defaults)");
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
  prefs.putFloat("gain_main", gain_main);
  prefs.putInt("gyro_avg", gyro_avg);
  prefs.putInt("d_y_a", deriv_yaw_window);
  prefs.putInt("d_y_s", deriv_steer_window);
  prefs.putFloat("s_p", steer_prio);
  prefs.putFloat("g_dp", gyro_dp);
  prefs.putInt("return_damp", return_damping);
  prefs.putFloat("g_exp", gain_exp);
  prefs.putInt("gyro_lp_hz", gyro_lp_hz);
  prefs.putInt("d_lp_hz", derivative_lp_hz);
  prefs.putInt("s_in_lp", servo_in_lp_hz);
  prefs.putInt("s_out_lp", servo_out_lp_hz);

  prefs.end();
}

// ---------- Pages ----------
static void handleRoot() {
  String s = htmlHeader("ESP32 Control");
  s += "<div class='card'>";
  s += "<p>Main page</p>";
  s += "<a href='/epa'>EPA</a><br>";
  s += "<a href='/settings'>Settings</a><br>";
  s += "<a href='/exit'>Exit</a><br>";
  s += "</div>";
  s += "<div class='small'>AP IP: " + WiFi.softAPIP().toString() + "</div>";
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

  s += "<label for='gain_main'>gain_main</label><input class='val8' id='gain_main' name='gain_main' type='number' step='0.05' value='" + String(gain_main) + "'>";
  s += "<label for='gyro_avg'>gyro_avg</label><input class='val8' id='gyro_avg' name='gyro_avg' type='number' value='" + String(gyro_avg) + "'>";
  s += "<label for='deriv_yaw_window'>deriv_yaw_window</label><input class='val8' id='deriv_yaw_window' name='deriv_yaw_window' type='number' value='" + String(deriv_yaw_window) + "'>";
  s += "<label for='deriv_steer_window'>deriv_steer_window</label><input class='val8' id='deriv_steer_window' name='deriv_steer_window' type='number' value='" + String(deriv_steer_window) + "'>";
  s += "<label for='steer_prio'>steer_prio</label><input class='val8' id='steer_prio' name='steer_prio' type='number' step='0.05' value='" + String(steer_prio) + "'>";
  s += "<label for='gyro_dp'>gyro_dp</label><input class='val8' id='gyro_dp' name='gyro_dp' type='number' step='0.05' value='" + String(gyro_dp) + "'>";
  s += "<label for='return_damping'>return_damping</label><input class='val8' id='return_damping' name='return_damping' type='number' step='1.0' value='" + String(return_damping) + "'>";
  s += "<label for='gain_exp'>gain_exp</label><input class='val8' id='gain_exp' name='gain_exp' type='number' step='0.01' value='" + String(gain_exp) + "'>";
  s += "<label for='gyro_lp_hz'>gyro_lp_hz</label><input class='val8' id='gyro_lp_hz' name='gyro_lp_hz' type='number' step='1.0' value='" + String(gyro_lp_hz) + "'>";
  s += "<label for='derivative_lp_hz'>derivative_lp_hz</label><input class='val8' id='derivative_lp_hz' name='derivative_lp_hz' type='number' step='1.0' value='" + String(derivative_lp_hz) + "'>";
  s += "<label for='servo_in_lp_hz'>servo_in_lp_hz</label><input class='val8' id='servo_in_lp_hz' name='servo_in_lp_hz' type='number' step='1.0' value='" + String(servo_in_lp_hz) + "'>";
  s += "<label for='servo_out_lp_hz'>servo_out_lp_hz</label><input class='val8' id='servo_out_lp_hz' name='servo_out_lp_hz' type='number' step='1.0' value='" + String(servo_out_lp_hz) + "'>";

  s += "</div>";

  s += "<div style='margin-top:14px'><button type='submit'>Set</button></div>";
  s += "</form>";

  s += "<a href='/'>Back</a>";
  s += "</div>";
  s += htmlFooter();

  server.send(200, "text/html", s);
}

static void handleSettingsSet() {
  gain_main = getArgFloat("gain_main", gain_main);
  gyro_avg = getArgInt("gyro_avg", gyro_avg);
  deriv_yaw_window = getArgInt("deriv_yaw_window", deriv_yaw_window);
  deriv_steer_window = getArgInt("deriv_steer_window", deriv_steer_window);
  steer_prio = getArgFloat("steer_prio", steer_prio);
  gyro_dp = getArgFloat("gyro_dp", gyro_dp);
  return_damping = getArgInt("return_damping", return_damping);
  gain_exp = getArgFloat("gain_exp", gain_exp);
  gyro_lp_hz = getArgInt("gyro_lp_hz", gyro_lp_hz);
  derivative_lp_hz = getArgInt("derivative_lp_hz", derivative_lp_hz);
  servo_in_lp_hz = getArgInt("servo_in_lp_hz", servo_in_lp_hz);
  servo_out_lp_hz = getArgInt("servo_out_lp_hz", servo_out_lp_hz);
  //Serial.print("gyro_avg="); Serial.print(gyro_avg);
  //Serial.print("deriv_yaw_window="); Serial.print(deriv_yaw_window);

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
  server.on("/exit", HTTP_GET, handleExit);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started.");

  ws.begin();
  ws.onEvent(onWsEvent);
  Serial.println("WebSocket server started on port 81.");
}

void makeSettings() {
  setupSettings();

  uint32_t start = millis();
  uint32_t lastPush = 0;
  exitSettings = 0;
  while (millis() - start < 600000 && exitSettings == 0) {
    server.handleClient();
    ws.loop();

    // Push ~5 Hz
    if (millis() - lastPush >= 50) {
      lastPush = millis();
      String msg = "{\"steer\":" + String(getSteerPwSnapshot()) + "}";
      ws.broadcastTXT(msg);
      steerServo.writeMicroseconds(getSteerPwSnapshot());
    }

    delay(2);
  }

  server.stop();                 // stop HTTP
  WiFi.softAPdisconnect(true);   // stop AP
  WiFi.disconnect(true);         // stop STA (if used)
  WiFi.mode(WIFI_OFF);           // disable Wi-Fi
  esp_wifi_stop();               // stop driver
}