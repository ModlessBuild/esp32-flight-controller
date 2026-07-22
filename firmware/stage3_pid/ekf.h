#ifndef EKF_H
#define EKF_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6500_WE.h>
#include <Adafruit_BMP280.h>

#define MPU6500_ADDR 0x68
#define BMP280_ADDR  0x77

#define BARO_UPDATE_PERIOD_MS 50   // 20 Hz baro correction rate

struct EKFState {
  float roll, pitch, yaw;   // filtered attitude, radians
  float alt, vz;            // altitude (m AGL), vertical velocity (m/s)
  float gx, gy, gz;         // bias-corrected gyro rates, rad/s
  float dt;                 // time step used for this update
};

bool initEKF();             // init sensors, run calibration, returns false on hardware failure
void updateEKF();           // run one EKF cycle: IMU read, attitude predict+correct, altitude predict, baro correct at 20Hz
EKFState getEKFState();     // get latest state snapshot

#endif