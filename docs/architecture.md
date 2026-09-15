# Architecture

Design decisions and the reasoning behind them. Treated as locked — don't
re-relitigate without a real reason.

![System architecture diagram](architecture-diagram.svg)

## Dual-use, single codebase

One ESP32, one firmware image, boot-time mode flag (stored in flash via
`Preferences`, set with serial commands `DRONE`/`ROCKET`, defaults to
`DRONE`) selects between two control laws:

- **Drone mode** — cascaded PID (outer angle loop feeding inner rate loop
  for roll/pitch, rate-only yaw), X-quad motor mixer, altitude hold PID.
- **Rocket mode** — apogee detection state machine, dual-deployment
  recovery logic.

Sensor fusion (EKF) and the logging/telemetry stack are shared between
both modes — only the top-level control law differs. This is the core
architectural decision the whole project is built around.

## Sensor fusion: EKF, not a complementary filter

Deliberate step up in difficulty over the far more common complementary
filter approach, for two concrete reasons:

- Online gyro bias estimation — the filter's state vector includes gyro
  bias terms, so drift correction happens automatically rather than being
  hand-tuned.
- Proper statistical weighting of each sensor via Q/R tuning, rather than
  a fixed blend ratio.

**State vector:** roll, pitch, yaw, altitude, plus three gyro bias terms
(7 elements total). Attitude EKF runs on gyro predict / accel-derived
roll-pitch update at 200Hz (IMU rate); altitude EKF is a 2-state filter
at 20Hz (barometer rate), 10Hz serial output.

**Why altitude uses barometer only, not GPS:** barometer noise was
bench-measured at 0.07m std dev — orders of magnitude better than an
uncorrected NEO-6M's vertical accuracy (typically 10-20m+ with no RTK/
differential correction). GPS is also weaker vertically than horizontally
by design (satellite geometry), and update rate (1-5Hz) is far slower
than the EKF's working rate. Flight durations are short enough that
barometric drift isn't a real concern, so there's no need to fuse in a
slow, noisy correction signal. GPS lat/long/altitude is still logged for
telemetry and position tracking — it's just not part of the state
estimate.

## Magnetometer: deferred, not fused

QMC5883P is calibrated (hard-iron offsets known, axis remap worked out)
but not wired into the live sensor bus or EKF. Gyro-only yaw drifts
slowly (<1.5°/min once bias-corrected) — acceptable for tethered/manual
flight and for rocket mode, where heading doesn't matter for recovery.
Magnetometer only becomes necessary for the autonomous GPS waypoint
navigation stretch goal, where absolute heading is required without a
human correcting drift by eye.

## Rocket recovery: redundant, not a single if-statement

The requirement isn't "no if-statements" — it's no single point of
failure. A bare `if (altitude < peak - threshold) fireServo();` means one
bad barometer reading and the parachute never deploys. Instead:

- Primary trigger: barometric apogee detection via a phase state machine
  (PAD → ASCENDING → APOGEE → DESCENDING), with a 5-consecutive-falling-
  reading confirmation window to reject noise, a 20m minimum-altitude gate,
  and a 10m launch threshold.
- Cross-check and backup triggers (accelerometer-derived freefall
  detection, hard timer backup) — planned, not yet implemented.
- Dual-deployment: separate drogue-at-apogee and main-at-set-altitude
  events, each independently triggerable.

## Power distribution: PDB over 4-in-1 ESC

Originally speced as a single 4-in-1 BLHeli_S ESC board. Switched to 4x
individual LittleBee 30A BLHeli_S ESCs feeding into a Matek 30.5mm PDB
with built-in 5V/12V BEC, after the individual-ESC + PDB bundle worked
out cheaper than the 4-in-1 alone, and the PDB's BEC eliminated the need
for a separate UBEC/buck-converter workaround for ESP32 power. The
tradeoff — more individual wiring than a 4-in-1 — was accepted for the
cost saving.

## Build progression: breadboard → perfboard → PCB

Breadboard first to validate sensor fusion, control math, and firmware
logic without the cost of a mistake being permanent. Perfboard next,
soldered, for the actual flying build — this is where decoupling caps,
pull-ups, and protection diodes actually start to matter, since the
board now carries real motor current and vibration instead of just
proving out algorithms on a bench.

**Current state:** the perfboard prototype flew successfully (tethered,
then untethered, with live PID tuning over WiFi), and has since been
disassembled. The project is now moving to a custom PCB (KiCad) as v2 —
a single integrated board instead of a stack of breakout modules.

## Radio: SX1278 Ra-02 (433MHz LoRa)

433MHz confirmed license-exempt for hobby/student use in both UAE and
Ireland. TX power capped at 10dBm per UAE 433MHz SRD regulation.
Telemetry packets use floats instead of doubles to minimize airtime.
