#include "pid.h"
#include "attitude_control.h"
#include "altitude_control.h"
#include "mixer.h"

#include <Wire.h>
#include <MPU6500_WE.h>
#include <Adafruit_BMP280.h>

#define MPU6500_ADDR 0x68

MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);
Adafruit_BMP280 bmp;

// ---- IMU Calibration ----
float gyroBias[3]  = {0.0f, 0.0f, 0.0f};
float accelBias[3] = {0.0f, 0.0f, 0.0f};
const int CAL_SAMPLES = 2000;

// ---- Attitude 1D KF States ----
float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;
float P_roll  = 1.0f;
float P_pitch = 1.0f;
const float Q_angle = 0.001f;
const float R_angle = 0.03f;

// ---- Altitude True 2-State Kalman Filter Matrix Elements ----
float alt = 0.0f;
float vz  = 0.0f;

float P_00 = 1.0f;
float P_01 = 0.0f;
float P_11 = 1.0f;

const float Q_alt  = 0.01f;
const float Q_vz   = 0.1f;
const float R_baro = 0.25f;

// ---- Baro Ground Reference ----
float groundPressure = 0.0f;

// ---- Multi-Rate Timing Control ----
unsigned long prevTimeMicros = 0;
unsigned long lastBaroUpdateMillis = 0;
unsigned long lastSerialPrintMillis = 0;

const unsigned long IMU_LOOP_PERIOD_MICROS = 5000;
const unsigned long BARO_LOOP_PERIOD_MILLIS = 50;
const unsigned long SERIAL_PRINT_PERIOD_MILLIS = 100;

// ---- PID Controllers ----
AttitudeController attCtrl;
AltitudeController altCtrl;

float rollSP    = 0.0f;  // degrees
float pitchSP   = 0.0f;  // degrees
float yawRateSP = 0.0f;  // deg/s
float altSP     = 0.0f;  // meters

MotorOutputs motors;

void calibrateIMU() {
  float gxSum = 0.0f, gySum = 0.0f, gzSum = 0.0f;
  float axSum = 0.0f, aySum = 0.0f, azSum = 0.0f;

  Serial.println("Calibrating IMU... Keep board perfectly still.");

  for (int i = 0; i < CAL_SAMPLES; i++) {
    xyzFloat gyr = myMPU6500.getGyrValues();
    xyzFloat acc = myMPU6500.getGValues();

    gxSum += gyr.x;  gySum += gyr.y;  gzSum += gyr.z;
    axSum += acc.x;  aySum += acc.y;  azSum += acc.z;

    delay(1);
  }

  gyroBias[0] = gxSum / (float)CAL_SAMPLES;
  gyroBias[1] = gySum / (float)CAL_SAMPLES;
  gyroBias[2] = gzSum / (float)CAL_SAMPLES;

  accelBias[0] = axSum / (float)CAL_SAMPLES;
  accelBias[1] = aySum / (float)CAL_SAMPLES;
  accelBias[2] = (azSum / (float)CAL_SAMPLES) - 1.0f;

  Serial.println("IMU Calibration complete.");
  Serial.print("Gyro Bias (deg/s): ");
  Serial.print(gyroBias[0], 4); Serial.print(", ");
  Serial.print(gyroBias[1], 4); Serial.print(", ");
  Serial.println(gyroBias[2], 4);

  Serial.print("Accel Bias (g): ");
  Serial.print(accelBias[0], 4); Serial.print(", ");
  Serial.print(accelBias[1], 4); Serial.print(", ");
  Serial.println(accelBias[2], 4);
}

void calibrateBaro() {
  float pSum = 0.0f;
  Serial.println("Calibrating Barometer base altitude...");

  for (int i = 0; i < CAL_SAMPLES; i++) {
    pSum += bmp.readPressure();
    delay(1);
  }

  groundPressure = pSum / (float)CAL_SAMPLES;
  Serial.print("Ground Reference Pressure (Pa): ");
  Serial.println(groundPressure, 2);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (!myMPU6500.init()) {
    Serial.println("FATAL ERROR: MPU6500 not detected over I2C.");
    while (1);
  }

  myMPU6500.setAccRange(MPU6500_ACC_RANGE_4G);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_500);

  if (!bmp.begin(0x77)) {
    Serial.println("FATAL ERROR: BMP280 not detected at I2C address 0x77.");
    while (1);
  }

  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X4,
    Adafruit_BMP280::STANDBY_MS_1
  );

  delay(1000);
  calibrateIMU();
  calibrateBaro();

  attitudeInit(attCtrl);
  altitudeInit(altCtrl, 500.0f);
  altSP = 0.0f;

  prevTimeMicros = micros();
}

void loop() {
  unsigned long currentTimeMicros = micros();
  unsigned long elapsedMicros = currentTimeMicros - prevTimeMicros;

  if (elapsedMicros < IMU_LOOP_PERIOD_MICROS) {
    return;
  }

  float dt = (float)elapsedMicros / 1000000.0f;
  prevTimeMicros = currentTimeMicros;

  // ---- Step 1: Read and Correct Raw Sensor Values ----
  xyzFloat gyr = myMPU6500.getGyrValues();
  xyzFloat acc = myMPU6500.getGValues();

  float gx = (gyr.x - gyroBias[0]) * DEG_TO_RAD;
  float gy = (gyr.y - gyroBias[1]) * DEG_TO_RAD;
  float gz = (gyr.z - gyroBias[2]) * DEG_TO_RAD;

  float ax = acc.x - accelBias[0];
  float ay = acc.y - accelBias[1];
  float az = acc.z - accelBias[2];

  // ---- Step 2: Attitude EKF Updates ----
  roll  += gx * dt;
  pitch += gy * dt;
  yaw   += gz * dt;

  P_roll  += Q_angle;
  P_pitch += Q_angle;

  float accelRoll  = atan2(ay, az);
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az));

  float K_roll  = P_roll  / (P_roll  + R_angle);
  float K_pitch = P_pitch / (P_pitch + R_angle);

  roll  += K_roll  * (accelRoll  - roll);
  pitch += K_pitch * (accelPitch - pitch);

  P_roll  = (1.0f - K_roll)  * P_roll;
  P_pitch = (1.0f - K_pitch) * P_pitch;

  // ---- Step 3: Altitude Prediction ----
  float az_world = -ax * sin(pitch)
                   + ay * sin(roll) * cos(pitch)
                   + az * cos(roll) * cos(pitch);

  float vert_accel = (az_world - 1.0f) * 9.81f;

  alt += (vz * dt) + (0.5f * vert_accel * dt * dt);
  vz  += vert_accel * dt;

  float P_00_temp = P_00;
  float P_01_temp = P_01;

  P_00 += (2.0f * P_01_temp * dt) + (P_11 * dt * dt) + (Q_alt * dt);
  P_01 += P_11 * dt;
  P_11 += Q_vz * dt;

  // ---- Step 4: Baro Update (20 Hz) ----
  unsigned long currentTimeMillis = millis();
  if (currentTimeMillis - lastBaroUpdateMillis >= BARO_LOOP_PERIOD_MILLIS) {
    lastBaroUpdateMillis = currentTimeMillis;

    float p = bmp.readPressure();
    float baro_alt = 44330.0f * (1.0f - pow(p / groundPressure, 0.1903f));

    float S = P_00 + R_baro;

    float K_alt = P_00 / S;
    float K_vz  = P_01 / S;

    float alt_error = baro_alt - alt;
    alt += K_alt * alt_error;
    vz  += K_vz * alt_error;

    float P_00_old = P_00;
    float P_01_old = P_01;

    P_00 = (1.0f - K_alt) * P_00_old;
    P_01 = (1.0f - K_alt) * P_01_old;
    P_11 = -(K_vz * P_01_old) + P_11;
  }

  // ---- Step 4.5: PID Control + Motor Mixing ----
  float rollCmd, pitchCmd, yawCmd;

  attitudeCompute(attCtrl,
                  rollSP, pitchSP, yawRateSP,
                  roll  * RAD_TO_DEG,
                  pitch * RAD_TO_DEG,
                  gx * RAD_TO_DEG,
                  gy * RAD_TO_DEG,
                  gz * RAD_TO_DEG,
                  dt,
                  rollCmd, pitchCmd, yawCmd);

  float throttle = altitudeCompute(altCtrl, altSP, alt, dt);

  motors = mixMotors(throttle, rollCmd, pitchCmd, yawCmd);

  // ---- Step 5: Diagnostics Telemetry (10 Hz) ----
  unsigned long currentPrintMillis = millis();
  if (currentPrintMillis - lastSerialPrintMillis >= SERIAL_PRINT_PERIOD_MILLIS) {
    lastSerialPrintMillis = currentPrintMillis;
    Serial.print("CMD r:"); Serial.print(rollCmd, 1);
    Serial.print(" p:");    Serial.print(pitchCmd, 1);
    Serial.print(" y:");    Serial.print(yawCmd, 1);
    Serial.print(" t:");    Serial.print(motors.m1 + motors.m2 + motors.m3 + motors.m4, 0);
    Serial.print(" thr:");  Serial.print(throttle, 0);
    Serial.println();

    Serial.print("R:");  Serial.print(roll * RAD_TO_DEG, 1);
    Serial.print(" P:"); Serial.print(pitch * RAD_TO_DEG, 1);
    Serial.print(" Y:"); Serial.print(yaw * RAD_TO_DEG, 1);
    Serial.print(" A:"); Serial.print(alt, 2);
    Serial.print(" | M1:"); Serial.print(motors.m1, 0);
    Serial.print(" M2:");   Serial.print(motors.m2, 0);
    Serial.print(" M3:");   Serial.print(motors.m3, 0);
    Serial.print(" M4:");   Serial.println(motors.m4, 0);
  }
}