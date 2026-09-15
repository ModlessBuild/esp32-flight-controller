#include "mixer.h"

MotorOutputs mixMotors(float throttle, float rollCmd, float pitchCmd, float yawCmd) {
  MotorOutputs out;

  // Standard X-quad mixing:
  // Roll+  = tilt right = left motors speed up, right motors slow down
  // Pitch+ = tilt forward = back motors speed up, front motors slow down
  // Yaw+   = rotate CW = CW motors speed up, CCW motors slow down
  //
  // M1: front-right, CW   -> -roll, -pitch, +yaw
  // M2: front-left,  CCW  -> +roll, -pitch, -yaw
  // M3: back-left,   CW   -> +roll, +pitch, +yaw
  // M4: back-right,  CCW  -> -roll, +pitch, -yaw

  out.m1 = throttle + rollCmd - pitchCmd + yawCmd;
  out.m2 = throttle + rollCmd + pitchCmd - yawCmd;
  out.m3 = throttle - rollCmd + pitchCmd + yawCmd;
  out.m4 = throttle - rollCmd - pitchCmd - yawCmd;

  // Clamp to valid ESC range (0-1000 internal scale)
  if (out.m1 < 0.0f) out.m1 = 0.0f;  if (out.m1 > 1000.0f) out.m1 = 1000.0f;
  if (out.m2 < 0.0f) out.m2 = 0.0f;  if (out.m2 > 1000.0f) out.m2 = 1000.0f;
  if (out.m3 < 0.0f) out.m3 = 0.0f;  if (out.m3 > 1000.0f) out.m3 = 1000.0f;
  if (out.m4 < 0.0f) out.m4 = 0.0f;  if (out.m4 > 1000.0f) out.m4 = 1000.0f;

  return out;
}