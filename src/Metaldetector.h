// MetalDetector.h

#pragma once

#include <Arduino.h>

namespace Metal
{
    enum Class : uint8_t
    {
        METAL_NONE        = 0,
        METAL_NON_FERROUS = 1,   // freq shifted UP   (e.g. aluminum, copper -- eddy currents dominate)
        METAL_FERROUS     = 2,   // freq shifted DOWN (e.g. steel, iron     -- permeability dominates)
    };

    struct Reading
    {
        float freqHz;
        float deltaHz;
        Class metalClass;
    };

    // Spins up the PCNT read + baseline calibration on its own FreeRTOS
    // task (core 0), decoupled from loop(). Call once from setup().
    void begin();

    // Non-blocking. Returns whatever the background task last published.
    Reading getLatest();
}