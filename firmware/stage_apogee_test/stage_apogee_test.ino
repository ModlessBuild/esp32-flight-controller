#include "apogee.h"

ApogeeDetector detector;

// Simulated altitude profile: pad → launch → climb → apogee → descent
// Each value represents one reading at 20Hz (50ms apart)
float simAltitudes[] = {
  // Ground level (100m ASL)
  100.0, 100.1, 99.9, 100.0, 100.1,

  // Launch and climb
  101.0, 103.0, 107.0, 112.0, 118.0,
  125.0, 134.0, 145.0, 158.0, 172.0,
  188.0, 205.0, 220.0, 237.0, 252.0,
  265.0, 276.0, 284.0, 290.0, 294.0,

  // Near apogee — slowing down
  296.0, 297.5, 298.2, 298.8, 299.0,

  // Peak and start falling
  299.1, 299.0, 298.8, 298.5, 298.0,
  297.2, 296.0, 294.5, 292.0, 289.0,

  // Clear descent
  285.0, 280.0, 274.0, 267.0, 259.0,
  250.0, 240.0, 229.0, 217.0, 204.0
};

int numReadings = sizeof(simAltitudes) / sizeof(simAltitudes[0]);

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Apogee detection simulation");
  Serial.println("Ground altitude: 100.0m ASL");
  Serial.println("---");

  initApogee(detector, 100.0);   // ground level = 100m ASL

  for (int i = 0; i < numReadings; i++) {
    updateApogee(detector, simAltitudes[i]);

    Serial.print("Reading ");
    Serial.print(i);
    Serial.print(": alt=");
    Serial.print(simAltitudes[i], 1);
    Serial.print("  AGL=");
    Serial.print(simAltitudes[i] - 100.0, 1);
    Serial.print("  phase=");

    switch (detector.phase) {
      case PHASE_PAD:        Serial.print("PAD");        break;
      case PHASE_ASCENDING:  Serial.print("ASCENDING");  break;
      case PHASE_APOGEE:     Serial.print("APOGEE");     break;
      case PHASE_DESCENDING: Serial.print("DESCENDING"); break;
    }

    Serial.print("  peak=");
    Serial.print(detector.peakAlt, 1);
    Serial.print("  falls=");
    Serial.print(detector.fallCount);

    if (detector.apogeeDetected) {
      Serial.print("  *** APOGEE DETECTED ***");
    }

    Serial.println();
  }

  Serial.println("---");
  Serial.println("Simulation complete.");
}

void loop() {
  // nothing — simulation runs once in setup
}