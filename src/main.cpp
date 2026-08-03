#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <ESP32Servo.h>

#include "driver/ledc.h"
#include "driver/adc.h"
#include "UART.h"
#include "Metaldetector.h"
#include "PAA5100JE.h"

#define SAMPLE_RATE 40000
#define N 256

// Optical flow -- back in, on a new PAA5100JE. The old Flowsensor.cpp
// (PMW3901-based) is gone for good; this talks to PAA5100JE.h/.cpp
// directly and owns the SPI bus itself since there's no longer a
// separate flow-sensor setup file configuring it first.
//
// !!! CS/SCK/MISO/MOSI below are placeholders -- I picked GPIOs that
// don't collide with anything else already in use on this board (2, 4,
// 5, 7, 8, 13, 14, plus whatever backs ADC1_CHANNEL_0, and 19/20 which
// are native USB on the S3). Confirm these against your actual wiring
// before flashing.
#define FLOW_CS_PIN   18
#define FLOW_SCK_PIN  17
#define FLOW_MISO_PIN 15
#define FLOW_MOSI_PIN 16

// Sensor reports motion in raw counts, not mm -- this scale factor is
// NOT calibrated for your mounting height. 11.914 counts/mm is the
// PMW3901 datasheet nominal figure at its reference height; treat it as
// a starting guess and re-derive it by pushing the robot a known
// distance and comparing against posX/posY.
static constexpr float FLOW_COUNTS_PER_MM = 11.914f;

// How often to dump a combined debug line to Serial. Printing every
// loop iteration (~200Hz with the delay(5) below) is enough text at
// 115200 baud to noticeably slow the loop down.
#define DEBUG_PRINT_INTERVAL_MS 200

#define WIFI_SWITCH_PIN 2
#define WIFI_SWITCH_MASK 0x04

// Servo control
#define SERVO_CONTROL_PIN 7
#define SERVO_SIGNAL_PIN 8

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// Delay between one-degree movements.
// Larger number means a slower sweep.
#define SERVO_STEP_TIME_MS 20

Servo sweepServo;

int servoAngle = SERVO_MIN_ANGLE;
int servoDirection = 1;
unsigned long lastServoStepMs = 0;

uint16_t samples[N];

float sin1[N];
float cos1[N];
float sin10[N];
float cos10[N];

float actualSampleRate = SAMPLE_RATE;

PAA5100JE flowSensor(FLOW_CS_PIN);
bool flowReady = false;
unsigned long lastFlowUpdateMs = 0;

unsigned long lastDebugPrintMs = 0;

// Position is just dx/dy integrated straight from the sensor, in mm.
float g_flowPosX = 0.0f;
float g_flowPosY = 0.0f;

void generateReference(float freq, float* s, float* c)
{
    for (int i = 0; i < N; i++)
    {
        float t = static_cast<float>(i) / actualSampleRate;

        s[i] = sinf(2.0f * PI * freq * t);
        c[i] = cosf(2.0f * PI * freq * t);
    }
}

void sampleADC()
{
    uint32_t start = micros();

    for (int i = 0; i < N; i++)
    {
        samples[i] = adc1_get_raw(ADC1_CHANNEL_0);
    }

    uint32_t elapsed = micros() - start;

    if (elapsed > 0)
    {
        actualSampleRate =
            static_cast<float>(N) *
            1000000.0f /
            static_cast<float>(elapsed);
    }
}

float correlate(float* s, float* c)
{
    float I = 0.0f;
    float Q = 0.0f;

    for (int i = 0; i < N; i++)
    {
        float x =
            static_cast<float>(samples[i]) -
            2048.0f;

        I += x * s[i];
        Q += x * c[i];
    }

    return sqrtf(I * I + Q * Q);
}

void updateServo()
{
    const bool shouldSweep =
        digitalRead(SERVO_CONTROL_PIN) == HIGH;

    // LOW means stop changing the commanded angle.
    // The servo will hold its current position.
    if (!shouldSweep)
    {
        return;
    }

    const unsigned long now = millis();

    if (now - lastServoStepMs < SERVO_STEP_TIME_MS)
    {
        return;
    }

    lastServoStepMs = now;

    servoAngle += servoDirection;

    if (servoAngle >= SERVO_MAX_ANGLE)
    {
        servoAngle = SERVO_MAX_ANGLE;
        servoDirection = -1;
    }
    else if (servoAngle <= SERVO_MIN_ANGLE)
    {
        servoAngle = SERVO_MIN_ANGLE;
        servoDirection = 1;
    }

    sweepServo.write(servoAngle);
}

// Reads one motion sample and integrates it straight into position --
// no filtering, no SQUAL/shutter validity gating. Whatever the sensor
// reports gets sent, including noise and out-of-range garbage.
void updateFlowSensor(
    float *outPosX, float *outPosY,
    float *outVx, float *outVy)
{
    const unsigned long now = millis();
    const float dt = (now - lastFlowUpdateMs) / 1000.0f;
    lastFlowUpdateMs = now;

    float vx = 0.0f;
    float vy = 0.0f;

    if (flowReady && dt > 0.0f)
    {
        int16_t dx, dy;
        uint8_t squal;
        flowSensor.readMotionBurst(&dx, &dy, &squal);

        const float dxMm = static_cast<float>(dx) / FLOW_COUNTS_PER_MM;
        const float dyMm = static_cast<float>(dy) / FLOW_COUNTS_PER_MM;

        vx = dxMm / dt;
        vy = dyMm / dt;

        g_flowPosX += dxMm;
        g_flowPosY += dyMm;
    }

    *outPosX = g_flowPosX;
    *outPosY = g_flowPosY;
    *outVx = vx;
    *outVy = vy;
}

void setup()
{
    Serial.begin(115200);

    UART::begin();

    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(
        ADC1_CHANNEL_0,
        ADC_ATTEN_DB_12
    );

    pinMode(WIFI_SWITCH_PIN, INPUT_PULLUP);

    // Pin 7 defaults LOW if no signal is connected.
    pinMode(SERVO_CONTROL_PIN, INPUT_PULLDOWN);

    sweepServo.setPeriodHertz(50);

    sweepServo.attach(
        SERVO_SIGNAL_PIN,
        500,
        2500
    );

    sweepServo.write(servoAngle);

    generateReference(1000.0f, sin1, cos1);
    generateReference(10000.0f, sin10, cos10);

    Metal::begin(0, 14);
    Metal::begin(1, 13);

    SPI.begin(FLOW_SCK_PIN, FLOW_MISO_PIN, FLOW_MOSI_PIN, FLOW_CS_PIN);
    flowReady = flowSensor.begin();

    if (flowReady)
    {
        Serial.println("[PAA5100JE] ready");
    }
    else
    {
        Serial.println("[PAA5100JE] init failed -- check wiring/CS pin, position will read 0");
    }

    lastFlowUpdateMs = millis();

    Serial.println("Frequency detector ready");
}

void loop()
{
    // GPIO 7 HIGH: sweep.
    // GPIO 7 LOW: hold current position.
    updateServo();

    uint8_t mask = 0;

    // Switch closed -> LOW -> Wi-Fi ON.
    if (digitalRead(WIFI_SWITCH_PIN) == LOW)
    {
        mask |= WIFI_SWITCH_MASK;
    }

    sampleADC();

    generateReference(1000.0f, sin1, cos1);
    generateReference(10000.0f, sin10, cos10);

    float mag1 = correlate(sin1, cos1);
    float mag10 = correlate(sin10, cos10);

    const uint16_t mag1ToSend =
        static_cast<uint16_t>(
            constrain(mag1, 0.0f, 65535.0f)
        );

    const uint16_t mag10ToSend =
        static_cast<uint16_t>(
            constrain(mag10, 0.0f, 65535.0f)
        );

    UART::sendIRData(
        mag1ToSend,
        mag10ToSend,
        mask
    );

    float freq0 = Metal::getLatestFreq(0);
    float freq1 = Metal::getLatestFreq(1);

    UART::sendMetalData(0, freq0);
    UART::sendMetalData(1, freq1);

    float flowPosX = 0.0f, flowPosY = 0.0f;
    float flowVx = 0.0f, flowVy = 0.0f;
    updateFlowSensor(&flowPosX, &flowPosY, &flowVx, &flowVy);

    // Only send a POSE frame when the flow sensor produced a fresh,
    // SQUAL-gated sample this tick. When invalid (surface too low-texture,
    // out of range, or sensor not ready), the Kalman filter coasted
    // internally but we don't transmit -- the main board simply won't
    // receive a POSE update, which is the correct behavior: stale/coasted
    // estimates shouldn't trigger a position update on the other side.
    //if (flowValid)
    //{
        UART::sendPoseData(flowPosX, flowPosY, 0.0f, flowVx, flowVy, 0.0f);
    //}

    const unsigned long nowMs = millis();

    if (nowMs - lastDebugPrintMs >= DEBUG_PRINT_INTERVAL_MS)
    {
        lastDebugPrintMs = nowMs;

        Serial.printf(
            "[IR] mag1=%u mag10=%u wifi=%d | [Metal] f0=%.2fHz f1=%.2fHz | "
            "[Flow] rdy=%d pos=(%.1f,%.1f)mm v=(%.1f,%.1f)mm/s\n",
            mag1ToSend, mag10ToSend, (mask & WIFI_SWITCH_MASK) != 0,
            freq0, freq1,
            flowReady, flowPosX, flowPosY, flowVx, flowVy
        );
    }

    // Keep the loop responsive enough for smooth servo movement.
    delay(20);
}