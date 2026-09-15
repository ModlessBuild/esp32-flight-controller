#include "status.h"

static StatusState currentState = STATUS_NO_GPS;
static unsigned long lastToggle = 0;
static bool ledOn = false;
static unsigned long lastBeep = 0;
static uint8_t beepCount = 0;
static bool buzzerOn = false;

void initStatus() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
}

void setStatus(StatusState state) {
  if (state != currentState) {
    currentState = state;
    beepCount = 0;           // reset beep sequence on state change
    buzzerOn = false;
    digitalWrite(BUZZER_PIN, LOW);
  }
}

static void blinkLED(unsigned long interval_ms) {
  if (millis() - lastToggle >= interval_ms) {
    lastToggle = millis();
    ledOn = !ledOn;
    digitalWrite(LED_PIN, ledOn);
  }
}

static void doubleBlink() {
  // Pattern: on-off-on-off----pause---- (repeats)
  // Total cycle: 1000ms
  unsigned long t = millis() % 1000;
  if (t < 80)        digitalWrite(LED_PIN, HIGH);    // first blink on
  else if (t < 160)  digitalWrite(LED_PIN, LOW);     // gap
  else if (t < 240)  digitalWrite(LED_PIN, HIGH);    // second blink on
  else                digitalWrite(LED_PIN, LOW);     // long pause
}

static void tripleBeep() {
  // 3 short beeps, then a long pause. Cycle: 2000ms
  unsigned long t = millis() % 2000;
  if (t < 100)       digitalWrite(BUZZER_PIN, HIGH);  // beep 1
  else if (t < 200)  digitalWrite(BUZZER_PIN, LOW);
  else if (t < 300)  digitalWrite(BUZZER_PIN, HIGH);  // beep 2
  else if (t < 400)  digitalWrite(BUZZER_PIN, LOW);
  else if (t < 500)  digitalWrite(BUZZER_PIN, HIGH);  // beep 3
  else               digitalWrite(BUZZER_PIN, LOW);   // long silence
}

void updateStatus() {
  switch (currentState) {

    case STATUS_NO_GPS:
      blinkLED(500);                          // 1Hz blink (500ms on, 500ms off)
      digitalWrite(BUZZER_PIN, LOW);          // silent
      break;

    case STATUS_GPS_FIX:
      digitalWrite(LED_PIN, HIGH);            // solid on
      digitalWrite(BUZZER_PIN, LOW);          // silent
      break;

    case STATUS_ARMED:
      doubleBlink();                          // distinctive double-flash pattern
      digitalWrite(BUZZER_PIN, LOW);          // silent after initial arm beep
      break;

    case STATUS_LOW_BATTERY:
      blinkLED(100);                          // 5Hz fast blink
      tripleBeep();                           // repeating 3-beep warning
      break;

    case STATUS_ERROR:
      blinkLED(50);                           // 10Hz rapid flash
      digitalWrite(BUZZER_PIN, HIGH);         // continuous tone
      break;
  }
}