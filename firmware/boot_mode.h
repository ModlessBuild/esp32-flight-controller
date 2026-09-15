#ifndef BOOT_MODE_H
#define BOOT_MODE_H

#include <Arduino.h>
#include <Preferences.h>

enum FlightMode {
  MODE_DRONE = 0,
  MODE_ROCKET = 1
};

void initBootMode();
FlightMode getFlightMode();
void setFlightMode(FlightMode mode);

#endif