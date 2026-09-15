#ifndef IBUS_READER_H
#define IBUS_READER_H

#include <Arduino.h>

#define IBUS_RX_PIN     34
#define IBUS_CHANNELS   6
#define IBUS_FRAME_LEN  32
#define IBUS_HEADER1    0x20
#define IBUS_HEADER2    0x40

void initIBus();
bool getIBusChannels(uint16_t* channels);

#endif