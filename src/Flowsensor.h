// FlowSensor.h
//
// Fuses two PMW3901 optical flow sensors (mounted a fixed distance apart on
// the chassis) into a running robot pose estimate: x, y (meters, world
// frame) and theta (radians). No encoders or IMU required -- heading comes
// from the *disagreement* between the two sensors, same principle as
// getting heading from two differential-drive wheel encoders.
//
// Like MetalDetector, this runs its own background FreeRTOS task so the
// SPI reads + integration math never block main.cpp's loop().

#pragma once

#include <Arduino.h>

namespace Flow
{
    // csLeftPin / csRightPin: dedicated chip-select GPIOs, one per sensor.
    // SCK/MOSI/MISO are shared automatically via the default SPI bus.
    //
    // baselineMeters: physical distance between the two sensors, measured
    // along the axis they're aligned on (see .cpp for orientation
    // assumptions). This is the single most important calibration number --
    // get it from CAD/calipers, not a tape measure guess, since heading
    // error scales directly with error here.
    //
    // pixelsPerMeter: conversion from raw sensor pixel-shift counts to
    // real-world distance at your fixed mounting height. Determine this by
    // pushing the robot a known distance and dividing total pixel counts by
    // that distance. Assumed identical for both sensors (same height,
    // same part) -- recalibrate per-unit if you see systematic left/right
    // disagreement that isn't real rotation.
    void begin(int csLeftPin, int csRightPin, float baselineMeters, float pixelsPerMeter);

    // Non-blocking snapshot of the current pose estimate. Safe to call from
    // loop() every cycle.
    struct Pose
    {
        float x;      // meters, world frame
        float y;      // meters, world frame
        float theta;  // radians, world frame, wrapped to [-pi, pi]
        float vx;     // m/s, world frame (useful for the position controller's derivative term)
        float vy;     // m/s, world frame
        float omega;  // rad/s
    };

    Pose getPose();

    // Resets the running (x, y, theta) integration back to zero. Call this
    // when you want to define "here" as the new origin -- e.g. at the start
    // of an autonomous run.
    void resetPose();
}