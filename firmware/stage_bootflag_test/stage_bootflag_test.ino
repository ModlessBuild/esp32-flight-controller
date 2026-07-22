#include "boot_mode.h"

void setup() {
  Serial.begin(115200);
  delay(500);

  initBootMode();

  FlightMode mode = getFlightMode();
  Serial.print("Current mode: ");
  Serial.println(mode == MODE_DRONE ? "DRONE" : "ROCKET");

  Serial.println("Type 'DRONE' or 'ROCKET' in serial monitor to change mode.");
  Serial.println("Change takes effect on next reboot.");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "DRONE") {
      setFlightMode(MODE_DRONE);
      Serial.println("Mode set to DRONE. Reboot to apply.");
    } else if (input == "ROCKET") {
      setFlightMode(MODE_ROCKET);
      Serial.println("Mode set to ROCKET. Reboot to apply.");
    } else {
      Serial.print("Unknown command: ");
      Serial.println(input);
      Serial.println("Type 'DRONE' or 'ROCKET'.");
    }
  }
}