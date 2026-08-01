#include "PAA5100JE.h"

#include <SPI.h>

namespace
{
    constexpr uint8_t REG_ID             = 0x00; // +1 = revision
    constexpr uint8_t REG_DATA_READY     = 0x02;
    constexpr uint8_t REG_MOTION_BURST   = 0x16;
    constexpr uint8_t REG_POWER_UP_RESET = 0x3A;

    constexpr uint32_t SPI_CLOCK_HZ = 4000000; // matches Bitcraze's PMW3901 driver

    // Sentinel meaning "delay(value) ms here" instead of "write this
    // register" inside the init sequence tables in secretSauce(). Valid
    // register addresses are 0-0x7F so this can't collide.
    constexpr int16_t WAIT = -1;
}

PAA5100JE::PAA5100JE(uint8_t csPin) : _cs(csPin) {}

void PAA5100JE::registerWrite(uint8_t reg, uint8_t value)
{
    SPI.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE3));
    digitalWrite(_cs, LOW);
    delayMicroseconds(50);
    SPI.transfer(reg | 0x80);
    SPI.transfer(value);
    delayMicroseconds(50);
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
    delayMicroseconds(200);
}

uint8_t PAA5100JE::registerRead(uint8_t reg)
{
    SPI.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE3));
    digitalWrite(_cs, LOW);
    delayMicroseconds(50);
    SPI.transfer(reg & 0x7F);
    delayMicroseconds(50);
    uint8_t value = SPI.transfer(0x00);
    delayMicroseconds(100);
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
    return value;
}

void PAA5100JE::bulkWriteSeq(const int16_t *data, size_t len)
{
    for (size_t i = 0; i < len; i += 2)
    {
        int16_t reg = data[i];
        uint8_t val = (uint8_t)data[i + 1];
        if (reg == WAIT)
        {
            delay(val);
        }
        else
        {
            registerWrite((uint8_t)reg, val);
        }
    }
}

// PAA5100JE-specific power-up calibration sequence, ported byte-for-byte
// from PAA5100._secret_sauce() in pimoroni/pmw3901-python. Do not modify
// these values -- they're PixArt proprietary calibration magic, not
// something to hand-tune.
void PAA5100JE::secretSauce()
{
    static const int16_t step1[] = {
        0x7F, 0x00,
        0x55, 0x01,
        0x50, 0x07,
        0x7F, 0x0E,
        0x43, 0x10,
    };
    bulkWriteSeq(step1, sizeof(step1) / sizeof(step1[0]));

    if (registerRead(0x67) & 0x80)
    {
        registerWrite(0x48, 0x04);
    }
    else
    {
        registerWrite(0x48, 0x02);
    }

    static const int16_t step2[] = {
        0x7F, 0x00,
        0x51, 0x7B,
        0x50, 0x00,
        0x55, 0x00,
        0x7F, 0x0E,
    };
    bulkWriteSeq(step2, sizeof(step2) / sizeof(step2[0]));

    if (registerRead(0x73) == 0x00)
    {
        uint8_t c1 = registerRead(0x70);
        uint8_t c2 = registerRead(0x71);
        if (c1 <= 28) c1 += 14;
        if (c1 > 28) c1 += 11;
        if (c1 > 0x3F) c1 = 0x3F;
        c2 = (uint8_t)(((uint16_t)c2 * 45) / 100);

        static const int16_t calSetup[] = {
            0x7F, 0x00,
            0x61, 0xAD,
            0x51, 0x70,
            0x7F, 0x0E,
        };
        bulkWriteSeq(calSetup, sizeof(calSetup) / sizeof(calSetup[0]));
        registerWrite(0x70, c1);
        registerWrite(0x71, c2);
    }

    static const int16_t step3[] = {
        0x7F, 0x00,
        0x61, 0xAD,

        0x7F, 0x03,
        0x40, 0x00,

        0x7F, 0x05,
        0x41, 0xB3,
        0x43, 0xF1,
        0x45, 0x14,

        0x5F, 0x34,
        0x7B, 0x08,
        0x5E, 0x34,
        0x5B, 0x11,
        0x6D, 0x11,
        0x45, 0x17,
        0x70, 0xE5,
        0x71, 0xE5,

        0x7F, 0x06,
        0x44, 0x1B,
        0x40, 0xBF,
        0x4E, 0x3F,

        0x7F, 0x08,
        0x66, 0x44,
        0x65, 0x20,
        0x6A, 0x3A,
        0x61, 0x05,
        0x62, 0x05,

        0x7F, 0x09,
        0x4F, 0xAF,
        0x5F, 0x40,
        0x48, 0x80,
        0x49, 0x80,
        0x57, 0x77,
        0x60, 0x78,
        0x61, 0x78,
        0x62, 0x08,
        0x63, 0x50,

        0x7F, 0x0A,
        0x45, 0x60,

        0x7F, 0x00,
        0x4D, 0x11,
        0x55, 0x80,
        0x74, 0x21,
        0x75, 0x1F,
        0x4A, 0x78,
        0x4B, 0x78,
        0x44, 0x08,

        0x45, 0x50,
        0x64, 0xFF,
        0x65, 0x1F,

        0x7F, 0x14,
        0x65, 0x67,
        0x66, 0x08,
        0x63, 0x70,
        0x6F, 0x1C,

        0x7F, 0x15,
        0x48, 0x48,

        0x7F, 0x07,
        0x41, 0x0D,
        0x43, 0x14,
        0x4B, 0x0E,
        0x45, 0x0F,
        0x44, 0x42,
        0x4C, 0x80,

        0x7F, 0x10,
        0x5B, 0x02,

        0x7F, 0x07,
        0x40, 0x41,

        WAIT, 0x0A,

        0x7F, 0x00,
        0x32, 0x00,

        0x7F, 0x07,
        0x40, 0x40,

        0x7F, 0x06,
        0x68, 0xF0,
        0x69, 0x00,

        0x7F, 0x0D,
        0x48, 0xC0,
        0x6F, 0xD5,

        0x7F, 0x00,
        0x5B, 0xA0,
        0x4E, 0xA8,
        0x5A, 0x90,
        0x40, 0x80,
        0x73, 0x1F,

        WAIT, 0x0A,

        0x73, 0x00,
    };
    bulkWriteSeq(step3, sizeof(step3) / sizeof(step3[0]));
}

bool PAA5100JE::begin()
{
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);

    // Power-up reset
    registerWrite(REG_POWER_UP_RESET, 0x5A);
    delay(20);

    // Read (and discard) the motion registers once, per the reference driver
    for (uint8_t offset = 0; offset < 5; offset++)
    {
        registerRead(REG_DATA_READY + offset);
    }

    secretSauce();

    uint8_t productId = registerRead(REG_ID);
    uint8_t revision  = registerRead(REG_ID + 1);

    if (productId != 0x49 || !(revision == 0x00 || revision == 0x01))
    {
        Serial.printf(
            "[PAA5100JE] init failed -- product ID 0x%02X revision 0x%02X (expected 0x49)\n",
            productId, revision
        );
        return false;
    }

    return true;
}

bool PAA5100JE::readMotionBurst(int16_t *deltaX, int16_t *deltaY, uint8_t *squal)
{
    uint8_t buf[13];

    SPI.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE3));
    digitalWrite(_cs, LOW);
    delayMicroseconds(50);
    buf[0] = SPI.transfer(REG_MOTION_BURST); // dummy byte clocked out with the address
    for (int i = 1; i < 13; i++)
    {
        buf[i] = SPI.transfer(0x00);
    }
    delayMicroseconds(50);
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();

    uint8_t dr           = buf[1];
    int16_t dx           = (int16_t)(buf[3] | (buf[4] << 8));
    int16_t dy           = (int16_t)(buf[5] | (buf[6] << 8));
    uint8_t sq           = buf[7];
    uint8_t shutterUpper = buf[11];

    *deltaX = dx;
    *deltaY = dy;
    *squal  = sq;

    return (dr & 0x80) && !((sq < 0x19) && (shutterUpper == 0x1F));
}