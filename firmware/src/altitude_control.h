#ifndef ALTITUDE_CONTROL_H
#define ALTITUDE_CONTROL_H

#include "pid.h"

struct AltitudeController {
  PID altPID;
  float baseThrottle;  // hover throttle baseline (0-1000 scale)
};

void altitudeInit(AltitudeController &alt, float baseThrottle);

float altitudeCompute(AltitudeController &alt,
                      float altSetpoint, float altMeasurement, float dt);

void altitudeReset(AltitudeController &alt);

#endif