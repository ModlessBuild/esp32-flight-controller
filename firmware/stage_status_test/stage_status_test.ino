#include "status.h"

void setup() {
  Serial.begin(115200);
  initStatus();
  Serial.println("Status system started");
}

void loop() {
  // Simulate changing states for testing
  // Cycle through each state every 5 seconds
  uint32_t phase = (millis() / 5000) % 5;

  switch (phase) {
    case 0: setStatus(STATUS_NO_GPS);      break;
    case 1: setStatus(STATUS_GPS_FIX);     break;
    case 2: setStatus(STATUS_ARMED);       break;
    case 3: setStatus(STATUS_LOW_BATTERY); break;
    case 4: setStatus(STATUS_ERROR);       break;
  }

  updateStatus();   // must be called every loop iteration
}