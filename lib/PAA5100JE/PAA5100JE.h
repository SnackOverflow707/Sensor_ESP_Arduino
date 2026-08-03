// PAA5100JE.h
//
// Minimal Arduino driver for the PixArt PAA5100JE optical flow sensor
// (short-range sibling of the PMW3901 -- same SPI bus/register protocol,
// different power-up "secret sauce" init sequence). Register map and init
// sequence are ported from the PAA5100 class in Pimoroni's reference
// driver: https://github.com/pimoroni/pmw3901-python
//
// Shares the SPI bus with the existing PMW3901 (SCK/MOSI/MISO already
// configured by Flowsensor.cpp before this is constructed) -- only needs
// its own CS pin.
//
// Working range is 15-35mm from the surface -- outside that window (or
// over low-texture surfaces) readMotionBurst() will report invalid.

#pragma once

#include <Arduino.h>

class PAA5100JE
{
public:
    explicit PAA5100JE(uint8_t csPin);

    // Runs power-up reset + the PAA5100JE-specific init sequence, then
    // verifies the product ID register. Returns false if the chip doesn't
    // respond correctly -- check wiring/CS pin/power before assuming the
    // chip itself is bad.
    bool begin();

    // Burst-reads one motion sample. deltaX/deltaY/squal are always
    // written, but the return value tells you whether this sample should
    // actually be trusted: false means out of the 15-35mm range,
    // insufficient surface texture, or no new data ready -- the caller
    // should not integrate deltaX/deltaY into a position estimate when
    // this returns false.
    bool readMotionBurst(int16_t *deltaX, int16_t *deltaY, uint8_t *squal);
    uint8_t getProductId() const;
    uint8_t getRevision() const;
private:
uint8_t _productId = 0;
uint8_t _revision = 0;
    uint8_t _cs;

    void    registerWrite(uint8_t reg, uint8_t value);
    uint8_t registerRead(uint8_t reg);
    void    bulkWriteSeq(const int16_t *data, size_t len);
    void    secretSauce();
};