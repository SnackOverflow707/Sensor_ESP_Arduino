// MetalDetector.h

#pragma once

#include <Arduino.h>

namespace Metal
{
    static constexpr uint8_t NUM_DETECTORS = 2;

    // Spins up the PCNT read task for detector `id` on its own FreeRTOS
    // task (core 0), decoupled from loop(). Call once per detector from setup().
    // oscGpioPin = that detector's Schmitt-trigger output pin.
    void begin(uint8_t id, int oscGpioPin);

    // Non-blocking. Returns whatever detector `id`'s background task last measured (Hz).
    float getLatestFreq(uint8_t id);
}