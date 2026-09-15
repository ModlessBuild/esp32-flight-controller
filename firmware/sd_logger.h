#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <SPI.h>
#include <SD.h>

#define SD_CS_PIN 5

struct LogEntry {
  uint32_t timestamp_ms;
  float roll, pitch, yaw;
  float altitude_m;
  double latitude, longitude;
  bool gpsFixValid;
  uint8_t modeFlag;   // 0 = drone, 1 = rocket
};

bool initSD();
void logEntry(LogEntry entry);

#endif