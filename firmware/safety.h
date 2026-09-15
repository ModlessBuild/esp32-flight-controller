#ifndef SAFETY_H
#define SAFETY_H

#include <Arduino.h>

#define ARM_CHANNEL_THRESHOLD    1800
#define THROTTLE_SAFE_THRESHOLD  1050
#define DISARM_THRESHOLD         1200

void initSafety();
void updateArmState(uint16_t throttle, uint16_t armChannel);
bool isArmed();

#endif