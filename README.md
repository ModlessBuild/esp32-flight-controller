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

## Status

**Drone mode: flying.** The quadcopter has flown untethered on a soldered
perfboard prototype, including live in-flight PID tuning over WiFi. That
prototype has since been disassembled ahead of a PCB v2 rebuild in KiCad.

**Rocket mode: in progress.** Apogee detection, dual-deployment recovery
logic, and telemetry are the remaining pieces of the original build scope.

## Hardware

| Component | Part |
|---|---|
| MCU | ESP32-WROOM DevKit V1 |
| IMU | MPU6500 (I2C, addr 0x68) |
| Barometer | BMP280 (I2C, addr 0x77) |
| Magnetometer | QMC5883P, labeled QMC5883L (I2C, addr 0x2C) — calibrated, not yet wired in (waypoint nav stretch goal) |
| GPS | NEO-6M (UART2) |
| Radio | SX1278 Ra-02 (433MHz LoRa) |
| Storage | microSD (SPI) |
| Motors | RS2205 2300KV brushless, CW/CCW matched pairs |
| ESCs | 4x LittleBee 30A BLHeli_S |
| Power distribution | 30.5mm PDB with built-in 5V/12V BEC |
| RC | FlySky FS-i6X transmitter / FS-iA6B receiver (iBus) |

Full pin map and power architecture in [`docs/`](docs/).

## Build stages

| Stage | Status |
|---|---|
| 1 — Sensor bring-up (I2C, clean readings) | ✅ Done |
| 2 — EKF sensor fusion (attitude + altitude) | ✅ Done |
| 3 — PID control loop validation | ✅ Done |
| 4 — Single motor / ESC bench test | ✅ Done |
| 5 — RC + motor mixing, full bench test | ✅ Done |
| 6 — Perfboard build, GPS + SD logging | ✅ Done |
| 7 — Full quad build, tethered → untethered flight | ✅ Done |
| 8 — Rocket mode: apogee detection, deployment, telemetry | 🔄 In progress |

## What's next

- Rocket mode: apogee detection state machine, dual-deployment servo
  trigger, LoRa telemetry integration into the unified firmware.
- PCB v2: learning KiCad, moving off breakout-board perfboard to a
  single integrated board.

See [`docs/status.md`](docs/status.md) for the detailed build log.

## License

MIT — see [`LICENSE`](LICENSE).
