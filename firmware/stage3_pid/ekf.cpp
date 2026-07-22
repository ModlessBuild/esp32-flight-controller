#include "ekf.h"

// ---- Hardware Objects (file-scoped) ----
static MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);
static Adafruit_BMP280 bmp;

// ---- Calibration Biases ----
static float gyroBias[3]  = {0.0f, 0.0f, 0.0f};
static float accelBias[3] = {0.0f, 0.0f, 0.0f};
static const int CAL_SAMPLES = 2000;

// ---- Attitude KF States ----
static float roll  = 0.0f;
static float pitch = 0.0f;
static float yaw   = 0.0f;
static float P_roll  = 1.0f;
static float P_pitch = 1.0f;
static const float Q_angle = 0.001f;
static const float R_angle = 0.03f;

// ---- Altitude 2-State KF ----
static float alt = 0.0f;
static float vz  = 0.0f;
static float P_00 = 1.0f;
static float P_01 = 0.0f;
static float P_11 = 1.0f;
static const float Q_alt  = 0.01f;
static const float Q_vz   = 0.1f;
static const float R_baro = 0.25f;

// ---- Baro Ground Reference ----
static float groundPressure = 0.0f;

// ---- Timing ----
static unsigned long prevTimeMicros = 0;
static unsigned long lastBaroUpdateMillis = 0;

// ---- Latest corrected gyro (shared via getEKFState) ----
static float lastGx = 0.0f;
static float lastGy = 0.0f;
static float lastGz = 0.0f;
static float lastDt = 0.0f;

// ---- Internal calibration functions ----

static void calibrateIMU() {
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

static void calibrateBaro() {
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

// ---- Public functions ----

bool initEKF() {
  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (!myMPU6500.init()) {
    Serial.println("FATAL ERROR: MPU6500 not detected over I2C.");
    return false;
  }

  myMPU6500.setAccRange(MPU6500_ACC_RANGE_4G);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_500);

  if (!bmp.begin(BMP280_ADDR)) {
    Serial.println("FATAL ERROR: BMP280 not detected at I2C address 0x77.");
    return false;
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

  prevTimeMicros = micros();

  return true;
}

void updateEKF() {
  // ---- Compute dt ----
  unsigned long now = micros();
  unsigned long elapsed = now - prevTimeMicros;
  prevTimeMicros = now;

  float dt = (float)elapsed / 1000000.0f;
  lastDt = dt;

  // ---- Step 1: Read and correct raw sensor values ----
  xyzFloat gyr = myMPU6500.getGyrValues();
  xyzFloat acc = myMPU6500.getGValues();

  float gx = (gyr.x - gyroBias[0]) * DEG_TO_RAD;
  float gy = (gyr.y - gyroBias[1]) * DEG_TO_RAD;
  float gz = (gyr.z - gyroBias[2]) * DEG_TO_RAD;

  float ax = acc.x - accelBias[0];
  float ay = acc.y - accelBias[1];
  float az = acc.z - accelBias[2];

  // Store for getEKFState()
  lastGx = gx;
  lastGy = gy;
  lastGz = gz;

  // ---- Step 2: Attitude KF predict + correct ----
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

  // ---- Step 3: Altitude prediction ----
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

  // ---- Step 4: Baro correction (20 Hz internal gate) ----
  unsigned long currentMillis = millis();
  if (currentMillis - lastBaroUpdateMillis >= BARO_UPDATE_PERIOD_MS) {
    lastBaroUpdateMillis = currentMillis;

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
}

EKFState getEKFState() {
  EKFState state;
  state.roll  = roll;
  state.pitch = pitch;
  state.yaw   = yaw;
  state.alt   = alt;
  state.vz    = vz;
  state.gx    = lastGx;
  state.gy    = lastGy;
  state.gz    = lastGz;
  state.dt    = lastDt;
  return state;
}