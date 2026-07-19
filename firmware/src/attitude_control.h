#ifndef ATTITUDE_CONTROL_H
#define ATTITUDE_CONTROL_H

#include "pid.h"

struct AttitudeController {
  // Outer loops: angle error → desired rate (deg/s)
  PID rollAngle;
  PID pitchAngle;

  // Inner loops: rate error → torque command
  PID rollRate;
  PID pitchRate;
  PID yawRate;   // yaw is rate-only, no angle loop
};

void attitudeInit(AttitudeController &att);

void attitudeCompute(AttitudeController &att,
                     float rollAngleSP, float pitchAngleSP, float yawRateSP,
                     float rollMeas, float pitchMeas,
                     float gyroX, float gyroY, float gyroZ,
                     float dt,
                     float &rollCmd, float &pitchCmd, float &yawCmd);

void attitudeReset(AttitudeController &att);

#endif