#include "sd_logger.h"

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!initSD()) {
    Serial.println("Halting: SD init failed.");
    while (true);   // stop here, nothing else useful to do without SD
  }
}

void loop() {
  static unsigned long lastLog = 0;
  if (millis() - lastLog >= 1000) {
    lastLog = millis();

    // Dummy test values until this is merged with real EKF/GPS data
    LogEntry entry;
    entry.timestamp_ms = millis();
    entry.roll = 1.23;
    entry.pitch = -0.45;
    entry.yaw = 88.0;
    entry.altitude_m = 12.5;
    entry.latitude = 0.0;
    entry.longitude = 0.0;
    entry.gpsFixValid = false;
    entry.modeFlag = 0;

    logEntry(entry);
    Serial.println("Logged one entry.");
  }
}