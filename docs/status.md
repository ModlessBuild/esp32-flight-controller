# Build Status Log

Running log of progress, kept up to date at the end of each work session.

## Current status

**Last completed:** Stage 3 — PID control loop math verified against manual hand-tilt testing. Motor mixer implemented. Full software backlog (6 items) completed and validated on bench while waiting on Batch 2 hardware. EKF extracted out of stage3\_pid.ino into its own ekf.h/ekf.cpp module (was previously inline global-variable code, identical to the old standalone stage2 EKF-only sketch, now superseded).

**Currently working on:** Waiting on Batch 2 hardware (motors, ESCs, PDB, frame, battery). All standalone software modules pushed to firmware/ on GitHub, not yet merged into one combined firmware. Merge is deliberately deferred until Stage 4/5 (motor + RC code) exist, so the merge session covers real scheduling/integration work rather than premature combination.

* \[x] GPS (NEO-6M) NMEA parsing into logging pipeline
* \[x] SD card logging (timestamped IMU raw, EKF output, altitude, GPS, mode state)
* \[x] LoRa (Ra-02) telemetry TX (RX pairing pending second module)
* \[x] Buzzer + LED status logic (armed/disarmed, GPS lock, low battery, mode indicator)
* \[x] Dual-mode boot flag switching drone vs rocket control law (ESP32 Preferences, flash-stored)
* \[x] Rocket mode apogee detection algorithm (unit-tested against simulated altitude data, one bug found and fixed)

**Blockers:** none. Batch 2 parts (motors, 4x ESC, PDB, frame, battery, charger) in transit.

## Key decisions log

* EKF chosen over complementary filter (deliberate difficulty step-up).
* 4-in-1 ESC path abandoned partway through sourcing in favor of 4x individual LittleBee 30A ESCs + a 30.5mm PDB with built-in 5V/12V BEC. Cheaper as a bundle, PDB replaces the standalone buck-converter workaround for ESP32 power.
* GPS altitude is logged but **not** fused into the EKF altitude estimate. Barometer noise (0.07m std dev, measured Stage 2b) is orders of magnitude better than uncorrected NEO-6M vertical accuracy, and flight durations are short enough that baro drift isn't a concern.
* Magnetometer (QMC5883L) deferred. Not required until autonomous GPS waypoint navigation (stretch goal), yaw drift is acceptable for tethered hover and rocket mode.
* LoRa RST and DIO0 pins were not in the original locked pin map. Added GPIO14 (RST) and GPIO13 (DIO0), both free and non-strapping. LoRa TX power capped at 10dBm (10mW) per UAE 433MHz SRD regulation.
* SD card breakout module requires 5V on VCC, not 3.3V, despite ESP32 logic being 3.3V. Confirmed via multimeter testing. Powered from ESP32 VIN/5V pin instead of the 3.3V pin. SPI data lines (SCK/MISO/MOSI/CS) remain on standard 3.3V GPIOs, module handles its own level shifting internally.
* RGB LED considered for status indication, rejected. Only GPIO0 and GPIO12 were free (both strapping pins with boot risk), not enough clean pins for 3-color RGB. Settled on single status LED (GPIO2) + buzzer (GPIO4) using blink/beep pattern encoding instead.
* Status system priority order (highest wins): Error > Low battery > Armed > GPS fix > No GPS.

## Stage log

### Stage 1 — Sensor bring-up

ESP32 + MPU6500 + BMP280 reading cleanly over I2C. MPU6500 clone identified via WHO\_AM\_I (0x70), MPU6500\_WE library used instead of Adafruit MPU6050 library. BMP280 confirmed at I2C address 0x77.

### Stage 2 — EKF sensor fusion

* 2a: Gyro/accel bias calibration via 2000-sample stationary average.
* 2b: BMP280 ground pressure reference established (\~99023 Pa), altitude noise std dev measured at 0.07m.
* 2c: Attitude EKF implemented (gyro predict, accel-derived roll/pitch update, gyro-only yaw). At-rest roll \~-0.75°, pitch \~-1.2°. Tilt/shake validation: pending.

### Stage 3 — PID validation

Verified against manual hand-tilt testing. Motor mixer implemented. EKF later extracted out of the flat stage3\_pid.ino into a standalone ekf.h/ekf.cpp module (static-scoped internal state, EKFState struct returned via getEKFState()) to avoid global variable collisions ahead of the eventual firmware merge. Verified identical telemetry output before and after the extraction, no behavior change.

### Software backlog (parallel to hardware wait)

Completed while waiting on Batch 2 shipping, each as its own standalone Arduino sketch with modular .h/.cpp pairs:

* **GPS parsing** (stage\_gps\_test): TinyGPSPlus library, UART2 (RX2=GPIO16, TX2=GPIO17), 9600 baud. GPSData struct with fixValid flag. Confirmed real outdoor fix.
* **SD logging** (stage\_sd\_test): SD.h, SPI shared with LoRa (CS=GPIO5). LogEntry struct written to CSV via FILE\_APPEND, file closed after every write to avoid data loss on power interruption.
* **LoRa TX** (stage\_lora\_tx): Ra-02 module, pins NSS=15/RST=14/DIO0=13. Trimmed TelemetryPacket struct (floats, not doubles) sent as raw bytes to minimize airtime. TX power capped at 10dBm. RX pairing pending second module.
* **Status system** (stage\_status\_test): single LED (GPIO2) + buzzer (GPIO4), blink/beep pattern state machine, non-blocking millis() timing.
* **Boot flag** (stage\_bootflag\_test): ESP32 Preferences (flash storage), serial commands DRONE/ROCKET to set mode, takes effect on next reboot, defaults to DRONE if unset.
* **Apogee detection** (stage\_apogee\_test): phase state machine (PAD -> ASCENDING -> APOGEE -> DESCENDING), 5-consecutive-falling-reading confirmation, 20m minimum altitude gate, 10m launch threshold. Validated against a simulated altitude curve. One bug found and fixed: early return was blocking the APOGEE -> DESCENDING transition.

All six modules pushed to firmware/ on GitHub. Not yet merged into a single flight firmware. Merge plan: millis()-gated scheduler in loop(), IMU/EKF/PID ungated (target 200Hz), GPS/SD/status rate-limited (\~10-20Hz), LoRa rate-limited (\~2Hz) for airtime/duty cycle compliance. Deferred until Stage 4/5 motor and RC code exist.

