#include "ibus_reader.h"

static HardwareSerial IBusSerial(1);  // UART1 (UART0 = USB Serial, UART2 = GPS)
static uint8_t buf[IBUS_FRAME_LEN];
static uint16_t channelValues[IBUS_CHANNELS];
static bool frameReady = false;

void initIBus() {
  IBusSerial.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);  // RX only, no TX needed
  Serial.println("iBus reader initialized on GPIO34 (UART1).");
}

bool getIBusChannels(uint16_t* channels) {
  // read bytes one at a time, looking for a valid frame
  while (IBusSerial.available()) {
    // shift buffer left by 1 byte, append new byte at end
    memmove(buf, buf + 1, IBUS_FRAME_LEN - 1);
    buf[IBUS_FRAME_LEN - 1] = IBusSerial.read();

    // check header bytes
    if (buf[0] != IBUS_HEADER1 || buf[1] != IBUS_HEADER2) continue;

    // verify checksum: 0xFFFF minus sum of first 30 bytes
    uint16_t checksum = 0xFFFF;
    for (int i = 0; i < IBUS_FRAME_LEN - 2; i++) {
      checksum -= buf[i];
    }

    uint16_t received = buf[30] | (buf[31] << 8);  // little-endian checksum at end

    if (checksum != received) continue;  // bad frame, keep scanning

    // valid frame — extract channels (each is 2 bytes little-endian, starting at byte 2)
    for (int i = 0; i < IBUS_CHANNELS; i++) {
      channels[i] = buf[2 + (i * 2)] | (buf[3 + (i * 2)] << 8);
    }

    return true;
  }

  return false;  // no complete valid frame yet
}