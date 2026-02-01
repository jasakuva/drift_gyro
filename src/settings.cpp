#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

Preferences prefs;

WebServer server(80);

// ---------- AP credentials ----------
const char* AP_SSID = "ESP32-EPA";
const char* AP_PASS = "12345678";   // min 8 chars (or set "" for open)

// ---------- Example EPA variables ----------
int epa_left_us   = 1100;
int epa_center_us = 1500;
int epa_right_us  = 1900;

// Replace with your real measurement source
int current_steering_us = 1500;

// ---------- Settings variables (8 parameters) ----------
int   p1;
int   p2;
int   p3;
int   p4;
float p5;
float p6;
float p7;
int   p8;

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

void loadSettings() {
  prefs.begin("rc", true);  // read-only
  epa_left_us   = prefs.getInt("epaL", 1100);
  epa_center_us = prefs.getInt("epaC", 1500);
  epa_right_us  = prefs.getInt("epaR", 1900);

  p1 = prefs.getInt("p1", 10);
  p2 = prefs.getInt("p2", 20);
  p3 = prefs.getInt("p3", 30);
  p4 = prefs.getInt("p4", 40);
  p5 = prefs.getFloat("p5", 0.5f);
  p6 = prefs.getFloat("p6", 1.0f);
  p7 = prefs.getFloat("p7", 2.5f);
  p8 = prefs.getInt("p8", 1);
  prefs.end();

  Serial.println("Settings loaded from NVS (or defaults)");
}



void saveEpa() {
  prefs.begin("rc", false);  // not read-only
  prefs.putInt("epaL", epa_left_us);
  prefs.putInt("epaC", epa_center_us);
  prefs.putInt("epaR", epa_right_us);
  prefs.end();

}

void saveParameters() {
  prefs.begin("rc", false);  // not read-only
  prefs.putInt("p1", p1);
  prefs.putInt("p2", p2);
  prefs.putInt("p3", p3);
  prefs.putInt("p4", p4);
  prefs.putFloat("p5", p5);
  prefs.putFloat("p6", p6);
  prefs.putFloat("p7", p7);
  prefs.putInt("p8", p8);
  prefs.end();

}

// ---------- Pages ----------
void handleRoot() {
  String s = htmlHeader("ESP32 Control");
  s += "<div class='card'>";
  s += "<p>Main page</p>";
  s += "<a href='/epa'>EPA</a><br>";
  s += "<a href='/settings'>Settings</a>";
  s += "</div>";
  s += "<div class='small'>AP IP: " + WiFi.softAPIP().toString() + "</div>";
  s += htmlFooter();
  server.send(200, "text/html", s);
}

void handleEPA() {
  String msg = server.hasArg("msg") ? server.arg("msg") : "";

  String s = htmlHeader("EPA");
  s += "<div class='card'>";
  s += "<div class='row'>";
  s += "<form method='POST' action='/epa/action'><button name='cmd' value='left'>Left</button></form>";
  s += "<form method='POST' action='/epa/action'><button name='cmd' value='center'>Center</button></form>";
  s += "<form method='POST' action='/epa/action'><button name='cmd' value='right'>Right</button></form>";
  s += "<form method='POST' action='/epa/action'><button name='cmd' value='save'>Save</button></form>";
  s += "</div>";

  if (msg.length()) s += "<p><b>Status:</b> " + msg + "</p>";

  s += "<p><b>Current steering (example):</b> " + String(current_steering_us) + " us</p>";
  s += "<p><b>EPA Left:</b> " + String(epa_left_us) + " us<br>";
  s += "<b>EPA Center:</b> " + String(epa_center_us) + " us<br>";
  s += "<b>EPA Right:</b> " + String(epa_right_us) + " us</p>";

  s += "<a href='/'>Back</a>";
  s += "</div>";
  s += htmlFooter();

  server.send(200, "text/html", s);
}

void handleEPAAction() {
  String cmd = server.hasArg("cmd") ? server.arg("cmd") : "";
  int measured = current_steering_us; // TODO replace with real reading

  String msg = "Unknown command";
  if (cmd == "left")   { epa_left_us = measured;   msg = "Recorded LEFT = "   + String(epa_left_us) + " us"; }
  if (cmd == "center") { epa_center_us = measured; msg = "Recorded CENTER = " + String(epa_center_us) + " us"; }
  if (cmd == "right")  { epa_right_us = measured;  msg = "Recorded RIGHT = "  + String(epa_right_us) + " us"; }
  if (cmd == "save")   { 
    saveEpa(); 
    msg = "EPA values saved to persistend memory"; }

  server.sendHeader("Location", "/epa?msg=" + msg);
  server.send(303);
}

void handleSettings() {
  String msg = server.hasArg("msg") ? server.arg("msg") : "";

  String s = htmlHeader("Settings");
  s += "<div class='card'>";

  if (msg.length()) s += "<p><b>Status:</b> " + msg + "</p>";

  s += "<style>"
       ".formgrid{display:grid;grid-template-columns:140px auto;gap:10px 12px;align-items:center;}"
       ".formgrid label{margin:0;font-weight:600;}"
       ".val8{width:8ch;}"                 // ~8 digits wide
       "input[type=number]{padding:8px;font-size:16px;}"
       "</style>";

  s += "<form method='POST' action='/settings/set'>";
  s += "<div class='formgrid'>";

  s += "<label for='p1'>Correction Power (0-100)</label><input class='val8' id='p1' name='p1' type='number' value='" + String(p1) + "'>";
  s += "<label for='p2'>Correction Derivative (0-100)</label><input class='val8' id='p2' name='p2' type='number' value='" + String(p2) + "'>";
  s += "<label for='p3'>Driver prio, base (0-10)</label><input class='val8' id='p3' name='p3' type='number' value='" + String(p3) + "'>";
  s += "<label for='p4'>Driver prio, derivative (0-10)</label><input class='val8' id='p4' name='p4' type='number' value='" + String(p4) + "'>";

  // floats: keep step any + show with 6 decimals like before
  s += "<label for='p5'>Gyro filter (0-1)</label><input class='val8' id='p5' name='p5' type='number' step='any' value='" + String(p5, 6) + "'>";
  s += "<label for='p6'>Steering filter (0-1)</label><input class='val8' id='p6' name='p6' type='number' step='any' value='" + String(p6, 6) + "'>";
  s += "<label for='p7'>Drift detection (0-1)</label><input class='val8' id='p7' name='p7' type='number' step='any' value='" + String(p7, 6) + "'>";

  s += "<label for='p8'>Steering marginal (0-1)</label><input class='val8' id='p8' name='p8' type='number' value='" + String(p8) + "'>";

  s += "</div>"; // formgrid

  s += "<div style='margin-top:14px'>"
       "<button type='submit'>Set</button>"
       "</div>";

  s += "</form>";

  s += "<a href='/'>Back</a>";
  s += "</div>";
  s += htmlFooter();

  server.send(200, "text/html", s);
}

void handleSettingsSet() {
  p1 = getArgInt("p1", p1);
  p2 = getArgInt("p2", p2);
  p3 = getArgInt("p3", p3);
  p4 = getArgInt("p4", p4);
  p5 = getArgFloat("p5", p5);
  p6 = getArgFloat("p6", p6);
  p7 = getArgFloat("p7", p7);
  p8 = getArgInt("p8", p8);

  saveParameters();

  server.sendHeader("Location", "/settings?msg=Updated");
  server.send(303);
  
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void setupSettings() {
  loadSettings();             // read saved values (or defaults)
  
  // --- Start AP ---
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

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started.");
}

void makeSettings() {
  setupSettings();

   unsigned long start = millis();
  while (millis() - start < 360000) {
    server.handleClient();
    delay(1);  // yield to WiFi
  }
  

  // Update current_steering_us from your real input here, e.g.
  // current_steering_us = readReceiverPulseUs();
}