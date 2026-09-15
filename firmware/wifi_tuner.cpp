#include "wifi_tuner.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

static AttitudeController* attPtr = nullptr;
static AsyncWebServer server(80);

// Helper: build one gain-row of HTML for a given PID label
String pidRow(const char* label, const char* key, PID &p) {
  String row = "<div class='row'><h3>" + String(label) + "</h3>";
  row += "<label>Kp</label><input type='number' step='0.01' name='" + String(key) + "kp' value='" + String(p.Kp, 3) + "'>";
  row += "<label>Ki</label><input type='number' step='0.001' name='" + String(key) + "ki' value='" + String(p.Ki, 3) + "'>";
  row += "<label>Kd</label><input type='number' step='0.001' name='" + String(key) + "kd' value='" + String(p.Kd, 3) + "'></div>";
  return row;
}

void handleRoot(AsyncWebServerRequest *request) {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial;background:#121212;color:#fff;text-align:center;}"
          ".row{background:#1e1e1e;margin:10px auto;padding:10px;border-radius:8px;width:90%;max-width:400px;}"
          "input{width:70px;margin:0 8px;padding:6px;border-radius:4px;border:none;}"
          "button{background:#008CBA;color:#fff;border:none;padding:12px 30px;font-size:1rem;border-radius:5px;margin-top:15px;}"
          "</style></head><body><h2>Attitude PID Tuner</h2><form action='/update' method='GET'>";

  html += pidRow("Roll Angle (outer)", "ra", attPtr->rollAngle);
  html += pidRow("Pitch Angle (outer)", "pa", attPtr->pitchAngle);
  html += pidRow("Roll Rate (inner)", "rr", attPtr->rollRate);
  html += pidRow("Pitch Rate (inner)", "pr", attPtr->pitchRate);
  html += pidRow("Yaw Rate", "yr", attPtr->yawRate);

  html += "<button type='submit'>Update Gains</button></form></body></html>";
  request->send(200, "text/html", html);
}

// Helper: apply one param set if present
void applyIfPresent(AsyncWebServerRequest *request, const char* prefix, PID &p) {
  String kp = String(prefix) + "kp";
  String ki = String(prefix) + "ki";
  String kd = String(prefix) + "kd";
  if (request->hasParam(kp)) p.Kp = request->getParam(kp)->value().toFloat();
  if (request->hasParam(ki)) p.Ki = request->getParam(ki)->value().toFloat();
  if (request->hasParam(kd)) p.Kd = request->getParam(kd)->value().toFloat();
}

void handleUpdate(AsyncWebServerRequest *request) {
  applyIfPresent(request, "ra", attPtr->rollAngle);
  applyIfPresent(request, "pa", attPtr->pitchAngle);
  applyIfPresent(request, "rr", attPtr->rollRate);
  applyIfPresent(request, "pr", attPtr->pitchRate);
  applyIfPresent(request, "yr", attPtr->yawRate);

  Serial.println("=== PID gains updated via WiFi ===");
  Serial.printf("RollAngle  Kp=%.3f Ki=%.3f Kd=%.3f\n", attPtr->rollAngle.Kp, attPtr->rollAngle.Ki, attPtr->rollAngle.Kd);
  Serial.printf("PitchAngle Kp=%.3f Ki=%.3f Kd=%.3f\n", attPtr->pitchAngle.Kp, attPtr->pitchAngle.Ki, attPtr->pitchAngle.Kd);
  Serial.printf("RollRate   Kp=%.3f Ki=%.3f Kd=%.3f\n", attPtr->rollRate.Kp, attPtr->rollRate.Ki, attPtr->rollRate.Kd);
  Serial.printf("PitchRate  Kp=%.3f Ki=%.3f Kd=%.3f\n", attPtr->pitchRate.Kp, attPtr->pitchRate.Ki, attPtr->pitchRate.Kd);
  Serial.printf("YawRate    Kp=%.3f Ki=%.3f Kd=%.3f\n", attPtr->yawRate.Kp, attPtr->yawRate.Ki, attPtr->yawRate.Kd);

  request->redirect("/");
}

void initWifiTuner(AttitudeController &attCtrlRef, const char* ssid, const char* password) {
  attPtr = &attCtrlRef;

  WiFi.softAP(ssid, password);
  Serial.print("WiFi Tuner AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/update", HTTP_GET, handleUpdate);
  server.begin();
}

void handleWifiTuner() {
  // AsyncWebServer is non-blocking, nothing needed here.
  // Function kept for API symmetry / future sync-server swap.
}