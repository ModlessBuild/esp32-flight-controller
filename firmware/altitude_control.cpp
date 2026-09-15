#include "altitude_control.h"

void altitudeInit(AltitudeController &alt, float baseThrottle) {
  alt.baseThrottle = baseThrottle;
  // Output limits: +-300 throttle adjustment around base
  pidInit(alt.altPID, 120.0, 30.0, 80.0, -300.0, 300.0, 150.0);
}

float altitudeCompute(AltitudeController &alt,
                      float altSetpoint, float altMeasurement, float dt) {
  float adjustment = pidCompute(alt.altPID, altSetpoint, altMeasurement, dt);
  float throttle = alt.baseThrottle + adjustment;

  // Clamp to valid throttle range
  if (throttle > 1000.0f) throttle = 1000.0f;
  if (throttle < 0.0f)    throttle = 0.0f;

  return throttle;
}

void altitudeReset(AltitudeController &alt) {
  pidReset(alt.altPID);
}