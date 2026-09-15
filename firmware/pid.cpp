#include "pid.h"

void pidInit(PID &pid, float Kp, float Ki, float Kd,
             float outMin, float outMax, float integMax) {
  pid.Kp = Kp;
  pid.Ki = Ki;
  pid.Kd = Kd;
  pid.outMin = outMin;
  pid.outMax = outMax;
  pid.integMax = integMax;
  pid.integral = 0.0f;
  pid.prevMeasurement = 0.0f;
  pid.firstRun = true;
}

float pidCompute(PID &pid, float setpoint, float measurement, float dt) {
  // 1. Error
  float error = setpoint - measurement;

  // 2. Proportional term
  float P = pid.Kp * error;

  // 3. Integral term with windup clamp
  pid.integral += error * dt;
  if (pid.integral > pid.integMax)  pid.integral = pid.integMax;
  if (pid.integral < -pid.integMax) pid.integral = -pid.integMax;
  float I = pid.Ki * pid.integral;

  // 4. Derivative term — on measurement, not error
  float D = 0.0f;
  if (!pid.firstRun) {
    float dMeasurement = (measurement - pid.prevMeasurement) / dt;
    D = -pid.Kd * dMeasurement;
  }
  pid.prevMeasurement = measurement;
  pid.firstRun = false;

  // 5. Total output, clamped
  float output = P + I + D;
  if (output > pid.outMax) output = pid.outMax;
  if (output < pid.outMin) output = pid.outMin;

  return output;
}

void pidReset(PID &pid) {
  pid.integral = 0.0f;
  pid.prevMeasurement = 0.0f;
  pid.firstRun = true;
}