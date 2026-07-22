#ifndef STATUS_H
#define STATUS_H

#include <Arduino.h>

#define LED_PIN    2
#define BUZZER_PIN 4

enum StatusState {
  STATUS_NO_GPS,
  STATUS_GPS_FIX,
  STATUS_ARMED,
  STATUS_LOW_BATTERY,
  STATUS_ERROR
};

void initStatus();
void setStatus(StatusState state);
void updateStatus();

#endif