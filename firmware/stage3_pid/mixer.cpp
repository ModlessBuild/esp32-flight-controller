#include "mixer.h"

MotorOutputs mixMotors(float throttle, float rollCmd, float pitchCmd, float yawCmd) {
  MotorOutputs out;

//                  throttle   roll       pitch      yaw
  out.m1 = throttle  + rollCmd  - pitchCmd  - yawCmd;  // front-left,  CCW
  out.m2 = throttle  - rollCmd  - pitchCmd  + yawCmd;  // front-right, CW
  out.m3 = throttle  + rollCmd  + pitchCmd  + yawCmd;  // back-left,   CW
  out.m4 = throttle  - rollCmd  + pitchCmd  - yawCmd;  // back-right,  CCW

  // Clamp all to valid range
  if (out.m1 < 0.0f) out.m1 = 0.0f;  if (out.m1 > 1000.0f) out.m1 = 1000.0f;
  if (out.m2 < 0.0f) out.m2 = 0.0f;  if (out.m2 > 1000.0f) out.m2 = 1000.0f;
  if (out.m3 < 0.0f) out.m3 = 0.0f;  if (out.m3 > 1000.0f) out.m3 = 1000.0f;
  if (out.m4 < 0.0f) out.m4 = 0.0f;  if (out.m4 > 1000.0f) out.m4 = 1000.0f;

  return out;
}