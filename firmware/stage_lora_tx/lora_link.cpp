#include "lora_link.h"

bool initLoRa() {
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed!");
    return false;
  }

  LoRa.setTxPower(LORA_TX_POWER);
  Serial.println("LoRa initialized OK.");
  return true;
}

void sendTelemetry(TelemetryPacket pkt) {
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pkt, sizeof(pkt));
  LoRa.endPacket();
}