#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h>

#include "driver/ledc.h"
#include "driver/adc.h"
#include "UART.h"
#include "MetalDetector.h"

#define SAMPLE_RATE 40000
#define N 256

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

    Serial.printf(
        "[Metal 0] freq=%.2f Hz\n",
        freq0
    );

    Serial.printf(
        "[Metal 1] freq=%.2f Hz\n",
        freq1
    );

    // Keep the loop responsive enough for smooth servo movement.
    delay(5);
}