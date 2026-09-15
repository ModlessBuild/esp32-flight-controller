#ifndef PID_H
#define PID_H

struct PID {
  // Gains
  float Kp;
  float Ki;
  float Kd;

  // Output limits
  float outMin;
  float outMax;

  // Integral windup limit
  float integMax;

  // Internal state
  float integral;
  float prevMeasurement;
  bool firstRun;
};

void pidInit(PID &pid, float Kp, float Ki, float Kd,
             float outMin, float outMax, float integMax);

float pidCompute(PID &pid, float setpoint, float measurement, float dt);

void pidReset(PID &pid);

#endif