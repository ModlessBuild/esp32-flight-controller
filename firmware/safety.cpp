#include "safety.h"

static bool armed = false;

void initSafety() {
  armed = false;
}

void updateArmState(uint16_t throttle, uint16_t armChannel) {
  if (!armed) {
    // Arm only when switch is high AND throttle is low (safe start)
    if (armChannel > ARM_CHANNEL_THRESHOLD && throttle < THROTTLE_SAFE_THRESHOLD) {
      armed = true;
      Serial.println("*** ARMED ***");
    }
  } else {
    // Disarm when switch goes back down
    if (armChannel < DISARM_THRESHOLD) {
      armed = false;
      Serial.println("*** DISARMED ***");
    }
  }
}

bool isArmed() {
  return armed;
}