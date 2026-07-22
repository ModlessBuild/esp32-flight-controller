#include "gps.h"

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("GPS parsing started");
}

void loop() {
  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    GPSData current = readGPS();

    if (current.fixValid) {
      Serial.print("Lat: "); Serial.print(current.latitude, 6);
      Serial.print("  Lon: "); Serial.println(current.longitude, 6);
      Serial.print("Alt: "); Serial.print(current.altitude_m); Serial.println(" m");
      Serial.print("Sats: "); Serial.print(current.satellites);
      Serial.print("  HDOP: "); Serial.println(current.hdop);
    } else {
      Serial.println("No fix yet.");
    }
    Serial.println("---");
  }
}