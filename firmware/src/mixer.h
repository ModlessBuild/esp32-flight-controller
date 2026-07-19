#ifndef MIXER_H
#define MIXER_H

struct MotorOutputs {
  float m1;  // front-left,  CW,  GPIO 25
  float m2;  // front-right, CCW, GPIO 26
  float m3;  // back-left,   CCW, GPIO 27
  float m4;  // back-right,  CW,  GPIO 32
};

MotorOutputs mixMotors(float throttle, float rollCmd, float pitchCmd, float yawCmd);

#endif