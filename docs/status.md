# Build Status Log

Running log of progress, kept up to date at the end of each work session.

## Current status

**Last completed:** Stage 3 — PID control loop math verified against
manual hand-tilt testing. Motor mixer implemented.

**Currently working on:** Stage 2c — attitude EKF validation (tilt and
shake tests). In parallel, working through the software backlog while
Batch 2 hardware (motors, ESC, frame, battery) is in transit:

- [ ] GPS (NEO-6M) NMEA parsing into logging pipeline
- [ ] SD card logging (timestamped IMU raw, EKF output, altitude, GPS, mode state)
- [ ] LoRa (Ra-02) telemetry TX/RX pairing and packet structure
- [ ] Buzzer + LED status logic (armed/disarmed, GPS lock, low battery, mode indicator)
- [ ] Dual-mode boot flag switching drone vs rocket control law
- [ ] Rocket mode apogee detection algorithm (unit-tested against simulated altitude data)

**Blockers:** none. Batch 2 parts (motors, 4x ESC, PDB, frame, battery,
charger) ordered, ~10 day shipping.

## Key decisions log

- EKF chosen over complementary filter (deliberate difficulty step-up).
- 4-in-1 ESC path abandoned partway through sourcing in favor of 4x
  individual LittleBee 30A ESCs + a 30.5mm PDB with built-in 5V/12V BEC —
  cheaper as a bundle, PDB replaces the standalone buck-converter
  workaround for ESP32 power.
- GPS altitude is logged but **not** fused into the EKF altitude
  estimate — barometer noise (0.07m std dev, measured Stage 2b) is
  orders of magnitude better than uncorrected NEO-6M vertical accuracy,
  and flight durations are short enough that baro drift isn't a concern.
- Magnetometer (QMC5883L) deferred — not required until autonomous GPS
  waypoint navigation (stretch goal), yaw drift is acceptable for
  tethered hover and rocket mode.

## Stage log

### Stage 1 — Sensor bring-up
ESP32 + MPU6500 + BMP280 reading cleanly over I2C. MPU6500 clone
identified via WHO_AM_I (0x70), MPU6500_WE library used instead of
Adafruit MPU6050 library. BMP280 confirmed at I2C address 0x77.

### Stage 2 — EKF sensor fusion
- 2a: Gyro/accel bias calibration via 2000-sample stationary average.
- 2b: BMP280 ground pressure reference established (~99023 Pa),
  altitude noise std dev measured at 0.07m.
- 2c: Attitude EKF implemented (gyro predict, accel-derived roll/pitch
  update, gyro-only yaw). At-rest roll ~-0.75°, pitch ~-1.2°.
  Tilt/shake validation: pending.

### Stage 3 — PID validation
Verified against manual hand-tilt testing. Motor mixer implemented.
