// FlowSensor.h
//
// Reads a single PMW3901 optical flow sensor into a running robot pose
// estimate: x, y (meters, body frame) -- no heading source, so this only
// tracks correctly while the robot doesn't turn. theta/omega are held at
// 0 for now; once an IMU is wired in, feed its heading in here and rotate
// (dx, dy) by it before accumulating, the same way the old two-sensor
// version derived heading from sensor disagreement.
//
// Like MetalDetector, this runs its own background FreeRTOS task so the
// SPI reads + integration math never block main.cpp's loop().

#pragma once

#include <Arduino.h>

namespace Flow
{
    // csPin: chip-select GPIO for the sensor.
    // SCK/MOSI/MISO are shared automatically via the default SPI bus.
    //
    // pixelsPerMeter: conversion from raw sensor pixel-shift counts to
    // real-world distance at your fixed mounting height. Determine this by
    // pushing the robot a known distance and dividing total pixel counts by
    // that distance.
    void begin(int csPin, float pixelsPerMeter);

    // Non-blocking snapshot of the current pose estimate. Safe to call from
    // loop() every cycle.
    struct Pose
    {
        float x;      // meters, body frame (see note above re: no heading source)
        float y;      // meters, body frame
        float theta;  // radians -- always 0 until an IMU is wired in
        float vx;     // m/s
        float vy;     // m/s
        float omega;  // rad/s -- always 0, same reason as theta
    };

    Pose getPose();

    // Resets the running (x, y) integration back to zero. Call this
    // when you want to define "here" as the new origin -- e.g. at the start
    // of an autonomous run.
    void resetPose();
}
