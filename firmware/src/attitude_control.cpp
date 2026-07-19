#include "attitude_control.h"

void attitudeInit(AttitudeController &att) {
  // --- Outer loops: angle → desired rate (deg/s) ---
  // Output limits: max desired rate of 250 deg/s
  pidInit(att.rollAngle,  4.0, 0.0, 0.0, -250.0, 250.0, 50.0);
  pidInit(att.pitchAngle, 4.0, 0.0, 0.0, -250.0, 250.0, 50.0);

  // --- Inner loops: rate → torque command ---
  // Output limits: arbitrary +-500 units, mixer will normalize
  pidInit(att.rollRate,  0.7, 0.1, 0.01, -500.0, 500.0, 200.0);
  pidInit(att.pitchRate, 0.7, 0.1, 0.01, -500.0, 500.0, 200.0);
  pidInit(att.yawRate,   1.0, 0.1, 0.0,  -500.0, 500.0, 200.0);
}

void attitudeCompute(AttitudeController &att,
                     float rollAngleSP, float pitchAngleSP, float yawRateSP,
                     float rollMeas, float pitchMeas,
                     float gyroX, float gyroY, float gyroZ,
                     float dt,
                     float &rollCmd, float &pitchCmd, float &yawCmd) {

  // --- Roll cascade ---
  // Outer: angle error → desired roll rate
  float rollRateSP = pidCompute(att.rollAngle, rollAngleSP, rollMeas, dt);
  // Inner: rate error → roll torque command
  rollCmd = pidCompute(att.rollRate, rollRateSP, gyroX, dt);

  // --- Pitch cascade ---
  float pitchRateSP = pidCompute(att.pitchAngle, pitchAngleSP, pitchMeas, dt);
  pitchCmd = pidCompute(att.pitchRate, pitchRateSP, gyroY, dt);

  // --- Yaw (rate-only, no outer angle loop) ---
  yawCmd = pidCompute(att.yawRate, yawRateSP, gyroZ, dt);
}

void attitudeReset(AttitudeController &att) {
  pidReset(att.rollAngle);
  pidReset(att.pitchAngle);
  pidReset(att.rollRate);
  pidReset(att.pitchRate);
  pidReset(att.yawRate);
}