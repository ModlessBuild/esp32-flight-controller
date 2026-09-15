#ifndef WIFI_TUNER_H
#define WIFI_TUNER_H

#include "attitude_control.h"

void initWifiTuner(AttitudeController &attCtrlRef, const char* ssid, const char* password);
void handleWifiTuner(); // call once per loop() if using sync server; not needed for Async

#endif