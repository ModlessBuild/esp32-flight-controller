# ESP32 Dual-Use Flight Controller

A custom ESP32-based avionics board that functions as either a drone flight
controller or a rocket flight computer, selected by a boot-time firmware
flag. Shared sensor fusion and logging stack; separate control law per mode.

Built as a first-year Electrical Engineering student project at University
College Dublin, targeting UCD UAV, FormulaUCD, and EuRocketry Ireland.

## Why this project

Most beginner flight controller builds use a complementary filter and a
single-purpose airframe. This one doesn't:

- **Extended Kalman Filter** for attitude and altitude estimation, not a
  complementary filter.
- **Dual-use architecture**: one board, one codebase, two control laws
  (multirotor PID vs. rocket recovery logic) selected at boot.
- **Redundant recovery logic** for rocket mode: multiple independent
  triggers (barometric apogee detection, freefall cross-check, timer
  backup), not a single sensor / single if-statement.

## Hardware

| Component | Part |
|---|---|
| MCU | ESP32-WROOM DevKit V1 |
| IMU | MPU6500 (GY-521 breakout, I2C 0x68) |
| Barometer | BMP280 (I2C 0x77) |
| GPS | NEO-6M |
| Radio | SX1278 Ra-02 (433MHz LoRa) |
| Storage | microSD |
| Motor driver | 4-in-1 ESC, BLHeli_S |

Full pin map and power architecture in [`docs/`](docs/).

## Build stages

| Stage | Status |
|---|---|
| 1 — Sensor bring-up (I2C, clean readings) | ✅ Done |
| 2 — EKF sensor fusion (attitude + altitude) | 🔄 In progress |
| 3 — PID control loop validation | ✅ Done |
| 4 — Single motor / ESC bench test | ⏳ Pending hardware |
| 5 — RC + motor mixing, full bench test | ⏳ Pending hardware |
| 6 — Perfboard build, GPS + SD logging | 🔄 In progress |
| 7 — Full quad build, tethered hover | ⏳ Pending hardware |
| 8 — Rocket mode: apogee detection, deployment, telemetry | 🔄 In progress |

## Status

See [`docs/status.md`](docs/status.md) for the current build log.

## License

MIT — see [`LICENSE`](LICENSE).
