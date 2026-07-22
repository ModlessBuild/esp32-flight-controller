#include "lora_link.h"

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!initLoRa()) {
    Serial.println("Halting: LoRa init failed.");
    while (true);
  }
}

void loop() {
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 1000) {
    lastSend = millis();

    // Dummy test values until merged with real EKF/GPS data
    TelemetryPacket pkt;
    pkt.timestamp_ms = millis();
    pkt.roll = 1.2;
    pkt.pitch = -0.8;
    pkt.yaw = 45.0;
    pkt.altitude_m = 10.5;
    pkt.latitude = 25.20480f;
    pkt.longitude = 55.27080f;
    pkt.gpsFixValid = 1;
    pkt.modeFlag = 0;

    sendTelemetry(pkt);
    Serial.println("Sent one packet.");
  }
}