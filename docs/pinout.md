# Pin Map

ESP32-WROOM DevKit V1. All assignments locked — not to be reopened without
good reason.

## Motors (PWM/DShot, via 4x LittleBee 30A BLHeli_S ESCs)

| Motor | Position | Direction | GPIO |
|---|---|---|---|
| M1 | Front-left | CW | 25 |
| M2 | Back-left | CCW | 26 |
| M3 | Back-right | CW | 27 |
| M4 | Front-right | CCW | 32 |

CW/CCW diagonal pairs: M1/M3 (CW), M2/M4 (CCW).

## I2C bus — SDA=21, SCL=22, 400kHz

Daisy-chained: IMU → barometer → magnetometer.

| Sensor | Chip | Address | Notes |
|---|---|---|---|
| IMU | MPU6500 | 0x68 | WHO_AM_I=0x70. Breakout often labeled GY-521/MPU6050 — silicon is MPU6500. Use `MPU6500_WE` library (Wolfgang Ewald). |
| Barometer | BMP280 | 0x77 | SDO pulled high. Altitude noise std dev = 0.07m (measured); R_baro = 0.25 in EKF. |
| Magnetometer | QMC5883P | 0x2C | Breakout labeled QMC5883L — silicon is QMC5883P, address 0x2C not the datasheet 0x0D. Hard-iron offsets X=491, Y=372, Z=-1788. Axis remap: mx=sensorY, my=-sensorX. Calibrated but **not yet wired in** — deferred to autonomous waypoint nav stretch goal. |

## SPI shared bus — MOSI=23, MISO=19, SCK=18

| Peripheral | CS/other pins |
|---|---|
| LoRa (SX1278 Ra-02) | CS=15, RST=14, DIO0=13 |
| microSD | CS=5 |

## UART

| Peripheral | Pins | Baud | Notes |
|---|---|---|---|
| RC receiver (FS-iA6B) | GPIO34 (UART1, RX only) | 115200 | iBus protocol. CH1=Roll, CH2=Pitch, CH3=Throttle, CH4=Yaw, CH5=Arm/SwA (UP=1000/DOWN=2000, armed=DOWN), CH6=Flight mode/SwC (3-position). |
| GPS (NEO-6M) | RX=16, TX=17 (UART2) | 9600 | TinyGPSPlus. Logged, not fused into EKF (see architecture doc). |

## Other GPIO

| Function | GPIO |
|---|---|
| Battery voltage monitor (ADC, via 100kΩ/33kΩ divider) | 35 |
| Deployment servo (rocket mode) | 33 |
| Buzzer | 4 |
| Status LED | 2 |

## Reserved / do not use

- **GPIO6-11** — wired to internal SPI flash. Touching them bricks the board.
- **GPIO34, 35** — input-only. Correctly used for RC signal in and battery ADC.
- **GPIO2, 15** — strapping pins, affect boot behavior. Status LED on 2 and LoRa CS on 15 are fine as long as nothing hard-pulls them low at boot.

## Power

- **PDB:** Matek PDB-XT60, 30.5x30.5mm, 3-4S rated, built-in 5V/12V BEC. 5V BEC output feeds ESP32 VIN/GND directly — no separate UBEC needed.
- **Battery:** 3S 1300mAh 75C LiPo.
- **MCU:** ESP32 DevKit V1, one unit in service (no spare currently).
