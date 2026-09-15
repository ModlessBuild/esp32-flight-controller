#include "gps.h"

TinyGPSPlus gps;

GPSData readGPS() {
  GPSData data;

  data.fixValid = gps.location.isValid();
  data.timestamp_ms = millis();

  if (data.fixValid) {
    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();
  } else {
    data.latitude = 0.0;
    data.longitude = 0.0;
  }

  data.altitude_m = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  data.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
  data.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 99.9;

  return data;
}