#include "boot_mode.h"

static Preferences prefs;
static FlightMode currentMode;

void initBootMode() {
  prefs.begin("flight", true);                  // namespace "flight", read-only
  uint8_t stored = prefs.getUChar("mode", 0);   // default to 0 (DRONE) if nothing saved
  prefs.end();

  currentMode = (stored == 1) ? MODE_ROCKET : MODE_DRONE;
}

FlightMode getFlightMode() {
  return currentMode;
}

void setFlightMode(FlightMode mode) {
  prefs.begin("flight", false);           // reopen read-write
  prefs.putUChar("mode", (uint8_t)mode);  // write the enum value as a single byte
  prefs.end();

  currentMode = mode;
}