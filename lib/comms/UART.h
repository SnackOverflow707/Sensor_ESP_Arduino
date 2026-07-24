// UART.h

#pragma once

#include <Arduino.h>

namespace UART
{
    enum FrameType : uint8_t
    {
        FRAME_TYPE_IR    = 0x01,   // payload: mag1(u16 LE), mag2(u16 LE), mask(u8)  -> 5 bytes
        FRAME_TYPE_METAL = 0x02,   // payload: sensorId(u8), freqHz(float32 LE)      -> 5 bytes
    };

    void begin();

    void sendIRData(
        uint16_t mag1,
        uint16_t mag2,
        uint8_t mask
    );

    void sendMetalData(
        uint8_t sensorId,
        float freqHz
    );
}