#include "ekf.h"
#include "pid.h"
#include "attitude_control.h"
#include "altitude_control.h"
#include "mixer.h"

// ---- PID Controllers ----
AttitudeController attCtrl;
AltitudeController altCtrl;

float rollSP    = 0.0f;  // degrees
float pitchSP   = 0.0f;  // degrees
float yawRateSP = 0.0f;  // deg/s
float altSP     = 0.0f;  // meters

MotorOutputs motors;

// ---- Timing ----
unsigned long prevLoopMicros = 0;
unsigned long lastSerialPrintMillis = 0;
const unsigned long IMU_LOOP_PERIOD_MICROS = 5000;    // 200 Hz
const unsigned long SERIAL_PRINT_PERIOD_MILLIS = 100;  // 10 Hz

void setup() {
  Serial.begin(115200);

  if (!initEKF()) {
    Serial.println("FATAL ERROR: EKF init failed. Halting.");
    while (true);
  }

  attitudeInit(attCtrl);
  altitudeInit(altCtrl, 500.0f);
  altSP = 0.0f;

  prevLoopMicros = micros();
}

void loop() {
  // ---- 200 Hz gate ----
  unsigned long now = micros();
  if (now - prevLoopMicros < IMU_LOOP_PERIOD_MICROS) {
    return;
  }
  prevLoopMicros = now;

  // ---- EKF: read sensors, predict, correct ----
  updateEKF();
  EKFState s = getEKFState();

  // ---- PID Control + Motor Mixing ----
  float rollCmd, pitchCmd, yawCmd;

  attitudeCompute(attCtrl,
                  rollSP, pitchSP, yawRateSP,
                  s.roll  * RAD_TO_DEG,
                  s.pitch * RAD_TO_DEG,
                  s.gx * RAD_TO_DEG,
                  s.gy * RAD_TO_DEG,
                  s.gz * RAD_TO_DEG,
                  s.dt,
                  rollCmd, pitchCmd, yawCmd);

  float throttle = altitudeCompute(altCtrl, altSP, s.alt, s.dt);

  motors = mixMotors(throttle, rollCmd, pitchCmd, yawCmd);

  // ---- Diagnostics Telemetry (10 Hz) ----
  unsigned long currentPrintMillis = millis();
  if (currentPrintMillis - lastSerialPrintMillis >= SERIAL_PRINT_PERIOD_MILLIS) {
    lastSerialPrintMillis = currentPrintMillis;

    Serial.print("CMD r:"); Serial.print(rollCmd, 1);
    Serial.print(" p:");    Serial.print(pitchCmd, 1);
    Serial.print(" y:");    Serial.print(yawCmd, 1);
    Serial.print(" t:");    Serial.print(motors.m1 + motors.m2 + motors.m3 + motors.m4, 0);
    Serial.print(" thr:");  Serial.print(throttle, 0);
    Serial.println();

    Serial.print("R:");  Serial.print(s.roll * RAD_TO_DEG, 1);
    Serial.print(" P:"); Serial.print(s.pitch * RAD_TO_DEG, 1);
    Serial.print(" Y:"); Serial.print(s.yaw * RAD_TO_DEG, 1);
    Serial.print(" A:"); Serial.print(s.alt, 2);
    Serial.print(" | M1:"); Serial.print(motors.m1, 0);
    Serial.print(" M2:");   Serial.print(motors.m2, 0);
    Serial.print(" M3:");   Serial.print(motors.m3, 0);
    Serial.print(" M4:");   Serial.println(motors.m4, 0);
  }
}