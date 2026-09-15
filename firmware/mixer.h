#ifndef MIXER_H
#define MIXER_H

// Physical layout (confirmed from motor testing):
//
//         FRONT
//   M1(CW)    M4(CCW)
//       \      /
//        \    /
//          X
//        /    \
//       /      \
//   M2(CCW)    M3(CW)
//         BACK
//
// GPIO: M1=25, M2=26, M3=27, M4=32

struct MotorOutputs {
  float m1;  // front-left, CW,  GPIO 25
  float m2;  // back-left,  CCW, GPIO 26
  float m3;  // back-right,   CW,  GPIO 27
  float m4;  // front-right,  CCW, GPIO 32
};

MotorOutputs mixMotors(float throttle, float rollCmd, float pitchCmd, float yawCmd);

#endif