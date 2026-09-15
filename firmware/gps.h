#ifndef GPS_H
#define GPS_H

#include <TinyGPSPlus.h>

extern TinyGPSPlus gps;

struct GPSData {
  bool fixValid;
  double latitude;
  double longitude;
  float altitude_m;
  int satellites;
  float hdop;
  uint32_t timestamp_ms;
};

GPSData readGPS();

#endif