#include "apogee.h"

void initApogee(ApogeeDetector &det, float groundAltitude) {
  det.phase = PHASE_PAD;
  det.groundAlt = groundAltitude;
  det.peakAlt = groundAltitude;
  det.fallCount = 0;
  det.apogeeDetected = false;
}

void updateApogee(ApogeeDetector &det, float currentAlt) {
  float agl = currentAlt - det.groundAlt;

  switch (det.phase) {

    case PHASE_PAD:
      if (agl >= LAUNCH_THRESHOLD_M) {
        det.phase = PHASE_ASCENDING;
        det.peakAlt = currentAlt;
        det.fallCount = 0;
      }
      break;

    case PHASE_ASCENDING:
      if (currentAlt > det.peakAlt) {
        det.peakAlt = currentAlt;
        det.fallCount = 0;
      } else {
        det.fallCount++;

        if (det.fallCount >= APOGEE_CONFIRM_COUNT && agl >= MIN_APOGEE_ALT_M) {
          det.phase = PHASE_APOGEE;
          det.apogeeDetected = true;
        }
      }
      break;

    case PHASE_APOGEE:
      det.phase = PHASE_DESCENDING;
      break;

    case PHASE_DESCENDING:
      break;
  }
}