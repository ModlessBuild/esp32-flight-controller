#ifndef APOGEE_H
#define APOGEE_H

#include <Arduino.h>

#define LAUNCH_THRESHOLD_M    10.0f   // must be this far above ground to count as launched
#define APOGEE_CONFIRM_COUNT  5       // consecutive falling readings to confirm apogee
#define MIN_APOGEE_ALT_M      20.0f   // ignore apogee below this AGL

enum RocketPhase {
  PHASE_PAD,        // sitting on the pad, waiting for launch
  PHASE_ASCENDING,  // launched, altitude increasing
  PHASE_APOGEE,     // apogee confirmed
  PHASE_DESCENDING  // falling (post-apogee)
};

struct ApogeeDetector {
  RocketPhase phase;
  float groundAlt;       // altitude at power-on, used as reference
  float peakAlt;         // highest altitude seen since launch
  uint8_t fallCount;     // consecutive readings below peak
  bool apogeeDetected;
};

void initApogee(ApogeeDetector &det, float groundAltitude);
void updateApogee(ApogeeDetector &det, float currentAlt);

#endif