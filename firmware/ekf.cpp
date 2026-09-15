#include "ekf.h"

// ---- Hardware Objects ----
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
static float P_yaw   = 1.0f;
static const float Q_angle = 0.001f;
static const float R_angle = 0.03f;
static const float R_yaw   = 0.1f;

// ---- Magnetometer ----
static const uint8_t MAG_REG_DATA    = 0x01;
static const uint8_t MAG_REG_STATUS  = 0x09;
static const uint8_t MAG_REG_CTRL1   = 0x0A;
static const uint8_t MAG_REG_CTRL2   = 0x0B;
static const uint8_t MAG_REG_SPECIAL = 0x29;

static const int16_t MAG_X_OFFSET = 491;
static const int16_t MAG_Y_OFFSET = 372;
static const int16_t MAG_Z_OFFSET = -1788;

static float lastMagHeading = 0.0f;
static unsigned long lastMagUpdateMillis = 0;

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

// ---- Latest corrected gyro ----
static float lastGx = 0.0f;
static float lastGy = 0.0f;
static float lastGz = 0.0f;
static float lastDt = 0.0f;

// ---- Mag helpers ----
static void magWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static bool magReadReg(uint8_t reg, uint8_t &outVal) {
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { outVal = 0; return false; }
  if (Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)1) != 1) { outVal = 0; return false; }
  outVal = Wire.read();
  return true;
}

// ---- Calibration functions ----
static void calibrateIMU() {
  float gxSum = 0, gySum = 0, gzSum = 0;
  float axSum = 0, aySum = 0, azSum = 0;
  Serial.println("Calibrating IMU... Keep board perfectly still.");
  for (int i = 0; i < CAL_SAMPLES; i++) {
    xyzFloat gyr = myMPU6500.getGyrValues();
    xyzFloat acc = myMPU6500.getGValues();
    gxSum += gyr.x; gySum += gyr.y; gzSum += gyr.z;
    axSum += acc.x; aySum += acc.y; azSum += acc.z;
    delay(1);
  }
  gyroBias[0] = gxSum / CAL_SAMPLES;
  gyroBias[1] = gySum / CAL_SAMPLES;
  gyroBias[2] = gzSum / CAL_SAMPLES;
  accelBias[0] = axSum / CAL_SAMPLES;
  accelBias[1] = aySum / CAL_SAMPLES;
  accelBias[2] = (azSum / CAL_SAMPLES) - 1.0f;
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
  float pSum = 0;
  Serial.println("Calibrating Barometer base altitude...");
  for (int i = 0; i < CAL_SAMPLES; i++) {
    pSum += bmp.readPressure();
    delay(1);
  }
  groundPressure = pSum / CAL_SAMPLES;
  Serial.print("Ground Reference Pressure (Pa): ");
  Serial.println(groundPressure, 2);
}

static bool initMag() {
  Wire.beginTransmission(QMC_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("FATAL ERROR: QMC5883P not detected at 0x2C.");
    return false;
  }
  magWriteReg(MAG_REG_SPECIAL, 0x06);
  magWriteReg(MAG_REG_CTRL2, 0x08);
  magWriteReg(MAG_REG_CTRL1, 0xCD);
  delay(50);
  uint8_t chipID;
  if (!magReadReg(0x00, chipID) || chipID != 0x80) {
    Serial.print("WARNING: QMC5883P chip ID unexpected: 0x");
    Serial.println(chipID, HEX);
  }
  Serial.println("QMC5883P magnetometer initialized.");
  return true;
}

// ---- Public functions ----
bool initEKF() {
  Wire.begin(21, 22);
  Wire.setClock(400000);
  if (!myMPU6500.init()) {
    Serial.println("FATAL ERROR: MPU6500 not detected.");
    return false;
  }
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_4G);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_500);
  if (!bmp.begin(BMP280_ADDR)) {
    Serial.println("FATAL ERROR: BMP280 not detected.");
    return false;
  }
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X4,
    Adafruit_BMP280::STANDBY_MS_1
  );
  if (!initMag()) {
    Serial.println("FATAL ERROR: Magnetometer init failed.");
    return false;
  }
  delay(1000);
  calibrateIMU();
  calibrateBaro();
  prevTimeMicros = micros();
  return true;
}

void updateEKF() {
  unsigned long now = micros();
  float dt = (float)(now - prevTimeMicros) / 1000000.0f;
  prevTimeMicros = now;
  lastDt = dt;

  xyzFloat gyr = myMPU6500.getGyrValues();
  xyzFloat acc = myMPU6500.getGValues();

  float gx = (gyr.x - gyroBias[0]) * DEG_TO_RAD;
  float gy = (gyr.y - gyroBias[1]) * DEG_TO_RAD;
  float gz = (gyr.z - gyroBias[2]) * DEG_TO_RAD;
  float ax = acc.x - accelBias[0];
  float ay = acc.y - accelBias[1];
  float az = acc.z - accelBias[2];

  lastGx = gx; lastGy = gy; lastGz = gz;

  // Attitude predict
  roll  += gx * dt;
  pitch += gy * dt;
  yaw   += gz * dt;
  P_roll  += Q_angle;
  P_pitch += Q_angle;
  P_yaw   += Q_angle;

  // Roll/pitch correct from accel
  float accelRoll  = atan2(ay, az);
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az));
  float K_roll  = P_roll  / (P_roll  + R_angle);
  float K_pitch = P_pitch / (P_pitch + R_angle);
  roll  += K_roll  * (accelRoll  - roll);
  pitch += K_pitch * (accelPitch - pitch);
  P_roll  = (1.0f - K_roll)  * P_roll;
  P_pitch = (1.0f - K_pitch) * P_pitch;

  // Yaw correct from magnetometer (20 Hz)
  unsigned long currentMillis = millis();
  if (currentMillis - lastMagUpdateMillis >= MAG_UPDATE_PERIOD_MS) {
    lastMagUpdateMillis = currentMillis;
    uint8_t magStatus;
    if (magReadReg(MAG_REG_STATUS, magStatus) && (magStatus & 0x01)) {
      Wire.beginTransmission(QMC_ADDR);
      Wire.write(MAG_REG_DATA);
      Wire.endTransmission(false);
      Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)6);
      uint8_t xl = Wire.read(), xh = Wire.read();
      uint8_t yl = Wire.read(), yh = Wire.read();
      uint8_t zl = Wire.read(), zh = Wire.read();

      float sensorX = (float)((int16_t)((xh << 8) | xl) - MAG_X_OFFSET);
      float sensorY = (float)((int16_t)((yh << 8) | yl) - MAG_Y_OFFSET);
      float sensorZ = (float)((int16_t)((zh << 8) | zl) - MAG_Z_OFFSET);

      // Axis remap: sensor physically rotated 90 deg CW about Z to fit
      // breadboard. Body frame (Y=forward/nose, X=right, Z=up) is what
      // the tilt-comp formula expects. Offsets above are subtracted in
      // raw sensor axes BEFORE this remap — do not swap them.
      float mx =  sensorY;   // body-right now comes from sensor's Y
      float my = -sensorX;   // body-forward now comes from sensor's -X
      float mz =  sensorZ;   // Z is the rotation axis, unchanged

      float cosP = cos(pitch), sinP = sin(pitch);
      float cosR = cos(roll),  sinR = sin(roll);
      float Xh = mx * cosR + my * sinP * sinR - mz * cosP * sinR;
      float Yh = my * cosP + mz * sinP;
      float magHeading = atan2(Yh, Xh);
      lastMagHeading = magHeading;

      float yaw_error = magHeading - yaw;
      while (yaw_error >  PI) yaw_error -= 2.0f * PI;
      while (yaw_error < -PI) yaw_error += 2.0f * PI;
      float K_yaw = P_yaw / (P_yaw + R_yaw);
      yaw += K_yaw * yaw_error;
      P_yaw = (1.0f - K_yaw) * P_yaw;
      while (yaw >  PI) yaw -= 2.0f * PI;
      while (yaw < -PI) yaw += 2.0f * PI;
    }
  }

  // Altitude predict
  float az_world = -ax * sin(pitch) + ay * sin(roll) * cos(pitch) + az * cos(roll) * cos(pitch);
  float vert_accel = (az_world - 1.0f) * 9.81f;
  alt += (vz * dt) + (0.5f * vert_accel * dt * dt);
  vz  += vert_accel * dt;
  float P_00_temp = P_00;
  P_00 += (2.0f * P_01 * dt) + (P_11 * dt * dt) + (Q_alt * dt);
  P_01 += P_11 * dt;
  P_11 += Q_vz * dt;

  // Baro correct (20 Hz)
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
  state.magHeading = lastMagHeading;
  return state;
}