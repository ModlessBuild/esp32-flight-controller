#include "sd_logger.h"

bool initSD() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed!");
    return false;
  }

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card detected.");
    return false;
  }

  Serial.println("SD card initialized OK.");

  // Write CSV header if the file doesn't exist yet
  if (!SD.exists("/log.csv")) {
    File f = SD.open("/log.csv", FILE_WRITE);
    if (f) {
      f.println("timestamp_ms,roll,pitch,yaw,altitude_m,latitude,longitude,gpsFixValid,modeFlag");
      f.close();
    }
  }

  return true;
}

void logEntry(LogEntry entry) {
  File f = SD.open("/log.csv", FILE_APPEND);
  if (!f) {
    Serial.println("Failed to open log.csv for append.");
    return;
  }

  f.print(entry.timestamp_ms);   f.print(",");
  f.print(entry.roll, 3);        f.print(",");
  f.print(entry.pitch, 3);       f.print(",");
  f.print(entry.yaw, 3);         f.print(",");
  f.print(entry.altitude_m, 3);  f.print(",");
  f.print(entry.latitude, 6);    f.print(",");
  f.print(entry.longitude, 6);   f.print(",");
  f.print(entry.gpsFixValid);    f.print(",");
  f.println(entry.modeFlag);

  f.close();
}