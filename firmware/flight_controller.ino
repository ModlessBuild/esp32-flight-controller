// ============================================================================
//  ESP32 Dual-Use Flight Controller -- Unified Firmware
//  Stage 5: RC-driven mixer with ESC output and arm/disarm gating
// ============================================================================

#include "ekf.h"
#include "pid.h"
#include "attitude_control.h"
#include "altitude_control.h"
#include "mixer.h"
#include "gps.h"
#include "sd_logger.h"
#include "status.h"
#include "lora_link.h"
#include "boot_mode.h"
#include "apogee.h"
#include "ibus_reader.h"
#include "safety.h"
#include <ESP32Servo.h>
#include "wifi_tuner.h"

// ---- ESC output ----
Servo escM1, escM2, escM3, escM4;
#define ESC_PIN_M1  25
#define ESC_PIN_M2  26
#define ESC_PIN_M3  27
#define ESC_PIN_M4  32
#define ESC_MIN_US  1000
#define ESC_MAX_US  2000

// ---- Controllers / estimator consumers ----
AttitudeController attCtrl;
AltitudeController altCtrl;
ApogeeDetector     apogeeDet;

// ---- Setpoints ----
float rollSP    = 0.0f;
float pitchSP   = 0.0f;
float yawRateSP = 0.0f;
float throttleRC = 0.0f;  // raw RC throttle mapped to 0-1000

MotorOutputs motors;
FlightMode   mode;

// ---- Deployment (rocket mode) ----
#define DEPLOY_SERVO_PIN    33
#define SERVO_LOCKED_POS     0
#define SERVO_RELEASE_POS   90
Servo deployServo;
bool deploymentFired = false;

// ---- RC input ----
uint16_t rcChannels[IBUS_CHANNELS] = {1500, 1500, 1000, 1500, 1000, 1000};

#define ROLL_MAX_DEG      30.0f
#define PITCH_MAX_DEG     30.0f
#define YAW_RATE_MAX_DPS  90.0f

float mapChannelSigned(uint16_t val, float maxMag) {
  val = constrain(val, 1000, 2000);
  return ((float)val - 1500.0f) / 500.0f * maxMag;
}

// Maps throttle channel (1000-2000) to 0-1000 mixer scale
float mapThrottle(uint16_t val) {
  val = constrain(val, 1000, 2000);
  return (float)(val - 1000);
}

// ---- Battery monitor ----
#define VBAT_ADC_PIN 35
const float VBAT_SCALE = (133.0f / 33.0f) * (3.3f / 4095.0f);
const float VBAT_LOW_THRESHOLD = 10.5f;
float batteryVoltage = 12.0f;

// ---- Health flags ----
bool sdHealthy   = false;
bool loraHealthy = false;

// ============================================================================
//  Scheduler timing
// ============================================================================
unsigned long prevControlMicros = 0;
const unsigned long CONTROL_PERIOD_US = 5000;    // 200 Hz

unsigned long lastSdMillis      = 0;
const unsigned long SD_PERIOD_MS      = 100;     // 10 Hz

unsigned long lastLoraMillis    = 0;
const unsigned long LORA_PERIOD_MS    = 500;     // 2 Hz

unsigned long lastBattMillis    = 0;
const unsigned long BATT_PERIOD_MS    = 200;     // 5 Hz

unsigned long lastGpsCacheMillis = 0;
const unsigned long GPS_CACHE_PERIOD_MS = 200;   // 5 Hz

EKFState curState;
GPSData  curGps;

// ============================================================================
//  Serial command handling
// ============================================================================
void handleSerialCommands() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input == "DRONE") {
    setFlightMode(MODE_DRONE);
    Serial.println("Mode set to DRONE. Reboot to apply.");
  } else if (input == "ROCKET") {
    setFlightMode(MODE_ROCKET);
    Serial.println("Mode set to ROCKET. Reboot to apply.");
  } else if (input.length() > 0) {
    Serial.print("Unknown command: ");
    Serial.println(input);
  }
}

// ============================================================================
//  ESC output: convert 0-1000 internal scale to 1000-2000us PWM
// ============================================================================
void writeMotors(const MotorOutputs &m) {
  escM1.writeMicroseconds(ESC_MIN_US + (int)m.m1);
  escM2.writeMicroseconds(ESC_MIN_US + (int)m.m2);
  escM3.writeMicroseconds(ESC_MIN_US + (int)m.m3);
  escM4.writeMicroseconds(ESC_MIN_US + (int)m.m4);
}

void writeMotorsIdle() {
  escM1.writeMicroseconds(ESC_MIN_US);
  escM2.writeMicroseconds(ESC_MIN_US);
  escM3.writeMicroseconds(ESC_MIN_US);
  escM4.writeMicroseconds(ESC_MIN_US);
}

// ============================================================================
//  Control laws
// ============================================================================
void runDroneControl(const EKFState &s) {
  float rollCmd, pitchCmd, yawCmd;

  attitudeCompute(attCtrl,
                  rollSP, pitchSP, yawRateSP,
                  s.roll  * RAD_TO_DEG,
                  s.pitch * RAD_TO_DEG,
                  s.gx * RAD_TO_DEG,
                  s.gy * RAD_TO_DEG,
                  s.gz * RAD_TO_DEG,
                  s.dt,
                  rollCmd, pitchCmd, yawCmd);

  // Stage 5: throttle comes directly from RC stick, bypassing altitude PID
  // Altitude hold (altitudeCompute) comes later in Stage 7 tuning
  motors = mixMotors(throttleRC, rollCmd, pitchCmd, yawCmd);

if (isArmed() && throttleRC > 30.0f) {
    writeMotors(motors);
  } else {
    writeMotorsIdle();
    attitudeReset(attCtrl);  // clear PID integrators while disarmed
  }
}

void runRocketControl(const EKFState &s) {
  updateApogee(apogeeDet, s.alt);

  if (apogeeDet.apogeeDetected && !deploymentFired) {
    deploymentFired = true;
    deployServo.write(SERVO_RELEASE_POS);
    Serial.println("*** APOGEE -- DEPLOYMENT TRIGGERED ***");
  }
}

// ============================================================================
//  setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // --- Boot mode ---
  initBootMode();
  mode = getFlightMode();
  Serial.print("Boot mode: ");
  Serial.println(mode == MODE_DRONE ? "DRONE" : "ROCKET");

  // --- Status I/O ---
  initStatus();
  setStatus(STATUS_NO_GPS);

  // --- RC input ---
  initIBus();

  // --- Safety ---
  initSafety();

  // --- ESC outputs: attach all 4, send min throttle immediately ---
  escM1.attach(ESC_PIN_M1, ESC_MIN_US, ESC_MAX_US);
  escM2.attach(ESC_PIN_M2, ESC_MIN_US, ESC_MAX_US);
  escM3.attach(ESC_PIN_M3, ESC_MIN_US, ESC_MAX_US);
  escM4.attach(ESC_PIN_M4, ESC_MIN_US, ESC_MAX_US);
  writeMotorsIdle();  // all ESCs get 1000us = min throttle = arm signal
  delay(3000);        // give ESCs time to arm
  Serial.println("ESCs armed (min throttle sent).");

  // --- Deployment servo ---
  deployServo.setPeriodHertz(50);
  deployServo.attach(DEPLOY_SERVO_PIN, 1000, 2000);
  deployServo.write(SERVO_LOCKED_POS);
  analogReadResolution(12);

  // --- EKF ---
  if (!initEKF()) {
    Serial.println("FATAL: EKF init failed. Halting.");
    setStatus(STATUS_ERROR);
    while (true) { updateStatus(); }
  }

  attitudeInit(attCtrl);
  initWifiTuner(attCtrl, "ESP32-Drone-Tuner", "flightcontrol");
  altitudeInit(altCtrl, 500.0f);

  // --- Rocket apogee detector ---
  initApogee(apogeeDet, 0.0f);

  // --- GPS ---
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  // --- SPI peripherals ---
  sdHealthy = initSD();
  if (!sdHealthy) Serial.println("WARN: SD unavailable.");

  loraHealthy = initLoRa();
  if (!loraHealthy) Serial.println("WARN: LoRa unavailable.");

  prevControlMicros = micros();
  Serial.println("Init complete. DISARMED. Flip SwA to arm (throttle must be low).");
}

// ============================================================================
//  loop
// ============================================================================
void loop() {
  unsigned long nowUs = micros();
  unsigned long nowMs = millis();

  // ---------- EVERY PASS: GPS UART drain -----------------------------------
  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }

  // ---------- EVERY PASS: serial commands ----------------------------------
  handleSerialCommands();

  // ---------- EVERY PASS: RC input + arm state -----------------------------
  if (getIBusChannels(rcChannels)) {
    rollSP    = mapChannelSigned(rcChannels[0], ROLL_MAX_DEG);
    pitchSP   = mapChannelSigned(rcChannels[1], PITCH_MAX_DEG);
    throttleRC = mapThrottle(rcChannels[2]);
    yawRateSP = mapChannelSigned(rcChannels[3], YAW_RATE_MAX_DPS);

    // Update arm state every RC frame
    updateArmState(rcChannels[2], rcChannels[4]);
  }

  // ---------- EVERY PASS: if disarmed, force motors idle -------------------
  if (!isArmed()) {
    writeMotorsIdle();
  }

  // ---------- EVERY PASS: status LED/buzzer --------------------------------
  updateStatus();

  // ---------- 200 Hz: EKF + control law ------------------------------------
  if (nowUs - prevControlMicros >= CONTROL_PERIOD_US) {
    prevControlMicros = nowUs;

    updateEKF();
    curState = getEKFState();

    if (mode == MODE_DRONE) runDroneControl(curState);
    else                    runRocketControl(curState);
  }

  // ---------- 5 Hz: GPS snapshot -------------------------------------------
  if (nowMs - lastGpsCacheMillis >= GPS_CACHE_PERIOD_MS) {
    lastGpsCacheMillis = nowMs;
    curGps = readGPS();

    if (batteryVoltage < VBAT_LOW_THRESHOLD)      setStatus(STATUS_LOW_BATTERY);
    else if (curGps.fixValid)                     setStatus(STATUS_GPS_FIX);
    else                                          setStatus(STATUS_NO_GPS);
  }

  // ---------- 5 Hz: battery ADC --------------------------------------------
  if (nowMs - lastBattMillis >= BATT_PERIOD_MS) {
    lastBattMillis = nowMs;
    int raw = analogRead(VBAT_ADC_PIN);
    batteryVoltage = raw * VBAT_SCALE;
  }

  // ---------- 5 Hz: debug print --------------------------------------------
  static unsigned long lastRcPrintMillis = 0;
  if (nowMs - lastRcPrintMillis >= 200) {
    lastRcPrintMillis = nowMs;
    Serial.printf("RC: R=%4d P=%4d T=%4d Y=%4d SwA=%4d SwC=%4d | thr=%.0f roll=%.1f pitch=%.1f yaw=%.1f | M: %.0f %.0f %.0f %.0f | %s\n",
                  rcChannels[0], rcChannels[1], rcChannels[2],
                  rcChannels[3], rcChannels[4], rcChannels[5],
                  throttleRC, rollSP, pitchSP, yawRateSP,
                  motors.m1, motors.m2, motors.m3, motors.m4,
                  isArmed() ? "ARMED" : "DISARMED");
  }

  // ---------- 10 Hz: SD logging --------------------------------------------
  if (sdHealthy && (nowMs - lastSdMillis >= SD_PERIOD_MS)) {
    lastSdMillis = nowMs;

    LogEntry e;
    e.timestamp_ms = nowMs;
    e.roll  = curState.roll  * RAD_TO_DEG;
    e.pitch = curState.pitch * RAD_TO_DEG;
    e.yaw   = curState.yaw   * RAD_TO_DEG;
    e.altitude_m = curState.alt;
    e.latitude   = curGps.latitude;
    e.longitude  = curGps.longitude;
    e.gpsFixValid = curGps.fixValid;
    e.modeFlag    = (uint8_t)mode;
    logEntry(e);
  }

  // ---------- 2 Hz: LoRa telemetry -----------------------------------------
  if (loraHealthy && (nowMs - lastLoraMillis >= LORA_PERIOD_MS)) {
    lastLoraMillis = nowMs;

    TelemetryPacket pkt;
    pkt.timestamp_ms = nowMs;
    pkt.roll  = curState.roll  * RAD_TO_DEG;
    pkt.pitch = curState.pitch * RAD_TO_DEG;
    pkt.yaw   = curState.yaw   * RAD_TO_DEG;
    pkt.altitude_m = curState.alt;
    pkt.latitude   = (float)curGps.latitude;
    pkt.longitude  = (float)curGps.longitude;
    pkt.gpsFixValid = curGps.fixValid ? 1 : 0;
    pkt.modeFlag    = (uint8_t)mode;
    sendTelemetry(pkt);
  }
}