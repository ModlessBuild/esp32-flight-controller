#ifndef LORA_LINK_H
#define LORA_LINK_H

#include <SPI.h>
#include <LoRa.h>

#define LORA_CS_PIN   15
#define LORA_RST_PIN  14
#define LORA_DIO0_PIN 13
#define LORA_FREQ     433E6
#define LORA_TX_POWER 10   // dBm, 10mW cap per UAE 433MHz SRD limit

struct TelemetryPacket {
  uint32_t timestamp_ms;
  float roll, pitch, yaw;
  float altitude_m;
  float latitude, longitude;
  uint8_t gpsFixValid;
  uint8_t modeFlag;
};

bool initLoRa();
void sendTelemetry(TelemetryPacket pkt);

#endif