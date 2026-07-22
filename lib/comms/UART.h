// UART.h

#pragma once

#include <Arduino.h>

namespace UART
{
    void begin();

    void sendIRData(
        uint16_t mag1,
        uint16_t mag2,
        uint8_t mask
    );
}