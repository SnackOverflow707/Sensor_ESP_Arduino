
#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <ESP32Servo.h>

#include  "driver/ledc.h"
#include "driver/adc.h"

#include "UART.h"
#include "Metaldetector.h"
#include "PAA5100JE.h"

#define SAMPLE_RATE 40000
#define N 256

// --------------------------------------------------
// Optical-flow SPI pins
// --------------------------------------------------

#define FLOW_CS_PIN   18
#define FLOW_SCK_PIN  17
#define FLOW_MISO_PIN 15
#define FLOW_MOSI_PIN 16

// Temporary conversion estimate.
// This must eventually be calibrated experimentally.
static constexpr float FLOW_COUNTS_PER_MM = 11.914f;

// --------------------------------------------------
// General configuration
// --------------------------------------------------

#define DEBUG_PRINT_INTERVAL_MS 200

#define WIFI_SWITCH_PIN 2
#define WIFI_SWITCH_MASK 0x04

// --------------------------------------------------
// Servo configuration
// --------------------------------------------------

#define SERVO_CONTROL_PIN 7
#define SERVO_SIGNAL_PIN 8

#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_STEP_TIME_MS 20

Servo sweepServo;

int servoAngle = SERVO_MIN_ANGLE;
int servoDirection = 1;

unsigned long lastServoStepMs = 0;

// --------------------------------------------------
// IR detector data
// --------------------------------------------------

uint16_t samples[N];

float sin1[N];
float cos1[N];

float sin10[N];
float cos10[N];

float actualSampleRate = SAMPLE_RATE;

// --------------------------------------------------
// Optical-flow data
// --------------------------------------------------

PAA5100JE flowSensor(FLOW_CS_PIN);

bool flowReady = false;

unsigned long lastFlowUpdateMs = 0;
unsigned long lastDebugPrintMs = 0;

// Integrated position in millimetres.
float g_flowPosX = 0.0f;
float g_flowPosY = 0.0f;

// --------------------------------------------------
// IR reference generation
// --------------------------------------------------

void generateReference(
    float frequency,
    float *sineReference,
    float *cosineReference
)
{
    for (int i = 0; i < N; i++)
    {
        const float t =
            static_cast<float>(i) /
            actualSampleRate;

        sineReference[i] =
            sinf(2.0f * PI * frequency * t);

        cosineReference[i] =
            cosf(2.0f * PI * frequency * t);
    }
}

// --------------------------------------------------
// ADC sampling
// --------------------------------------------------

void sampleADC()
{
    const uint32_t start = micros();

    for (int i = 0; i < N; i++)
    {
        samples[i] =
            adc1_get_raw(ADC1_CHANNEL_0);
    }

    const uint32_t elapsed =
        micros() - start;

    if (elapsed > 0)
    {
        actualSampleRate =
            static_cast<float>(N) *
            1000000.0f /
            static_cast<float>(elapsed);
    }
}

// --------------------------------------------------
// Lock-in correlation
// --------------------------------------------------

float correlate(
    float *sineReference,
    float *cosineReference
)
{
    float inPhase = 0.0f;
    float quadrature = 0.0f;

    for (int i = 0; i < N; i++)
    {
        const float sample =
            static_cast<float>(samples[i]) -
            2048.0f;

        inPhase +=
            sample * sineReference[i];

        quadrature +=
            sample * cosineReference[i];
    }

    return sqrtf(
        inPhase * inPhase +
        quadrature * quadrature
    );
}

// --------------------------------------------------
// Servo update
// --------------------------------------------------

void updateServo()
{
    const bool shouldSweep =
        digitalRead(SERVO_CONTROL_PIN) == HIGH;

    if (!shouldSweep)
    {
        return;
    }

    const unsigned long now = millis();

    if (
        now - lastServoStepMs <
        SERVO_STEP_TIME_MS
    )
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

// --------------------------------------------------
// Optical-flow update
// --------------------------------------------------
//
// Debug output mapping:
//
// X       = integrated X position in mm
// Y       = integrated Y position in mm
// Heading = 1 if initialized, -1 if initialization failed
// Vx      = raw dx counts
// Vy      = raw dy counts
// Omega   = SQUAL
//
// Motion is temporarily integrated even when valid is false.
// This allows us to determine whether the sensor produces any
// raw dx/dy data at all.
//

void updateFlowSensor(
    float *outPosX,
    float *outPosY,
    float *outVx,
    float *outVy,
    float *outSqual,
    bool *outValid
)
{
    const unsigned long now = millis();

    const float dt =
        static_cast<float>(now - lastFlowUpdateMs) /
        1000.0f;

    lastFlowUpdateMs = now;

    int16_t dx = 0;
    int16_t dy = 0;
    uint8_t squal = 0;

    bool valid = false;

    float vx = 0.0f;
    float vy = 0.0f;

    if (flowReady && dt > 0.0f)
    {
        valid = flowSensor.readMotionBurst(
            &dx,
            &dy,
            &squal
        );

        const float dxMm =
            static_cast<float>(dx) /
            FLOW_COUNTS_PER_MM;

        const float dyMm =
            static_cast<float>(dy) /
            FLOW_COUNTS_PER_MM;

        // Initially integrate whenever the data is reasonable.
        const bool reasonable =
            abs(dx) < 5000 &&
            abs(dy) < 5000;

        if (reasonable)
        {
            g_flowPosX += dxMm;
            g_flowPosY += dyMm;

            vx = dxMm / dt;
            vy = dyMm / dt;
        }
    }

    *outPosX = g_flowPosX;
    *outPosY = g_flowPosY;
    *outVx = static_cast<float>(dx);
    *outVy = static_cast<float>(dy);
    *outSqual = static_cast<float>(squal);
    *outValid = valid;
}

// --------------------------------------------------
// Setup
// --------------------------------------------------

void setup()
{
    Serial.begin(115200);

    UART::begin();

    adc1_config_width(
        ADC_WIDTH_12Bit
    );

    adc1_config_channel_atten(
        ADC1_CHANNEL_0,
        ADC_ATTEN_DB_12
    );

    pinMode(
        WIFI_SWITCH_PIN,
        INPUT_PULLUP
    );

    pinMode(
        SERVO_CONTROL_PIN,
        INPUT_PULLDOWN
    );

    sweepServo.setPeriodHertz(50);

    sweepServo.attach(
        SERVO_SIGNAL_PIN,
        500,
        2500
    );

    sweepServo.write(servoAngle);

    generateReference(
        1000.0f,
        sin1,
        cos1
    );

    generateReference(
        10000.0f,
        sin10,
        cos10
    );

    Metal::begin(0, 14);
    Metal::begin(1, 13);

    SPI.begin(
        FLOW_SCK_PIN,
        FLOW_MISO_PIN,
        FLOW_MOSI_PIN,
        FLOW_CS_PIN
    );

    flowReady =
        flowSensor.begin();

    if (flowReady)
    {
        Serial.println(
            "[PAA5100JE] ready"
        );
    }
    else
    {
        Serial.println(
            "[PAA5100JE] initialization failed"
        );
    }

    lastFlowUpdateMs = millis();

    Serial.println(
        "Frequency detector ready"
    );
}

// --------------------------------------------------
// Main loop
// --------------------------------------------------

void loop()
{
    updateServo();

    uint8_t mask = 0;

    // Switch closed means Wi-Fi is enabled.
    if (
        digitalRead(WIFI_SWITCH_PIN) ==
        LOW
    )
    {
        mask |= WIFI_SWITCH_MASK;
    }

    // --------------------------------------------------
    // IR sampling
    // --------------------------------------------------

    sampleADC();

    generateReference(
        1000.0f,
        sin1,
        cos1
    );

    generateReference(
        10000.0f,
        sin10,
        cos10
    );

    const float mag1 =
        correlate(sin1, cos1);

    const float mag10 =
        correlate(sin10, cos10);

    const uint16_t mag1ToSend =
        static_cast<uint16_t>(
            constrain(
                mag1,
                0.0f,
                65535.0f
            )
        );

    const uint16_t mag10ToSend =
        static_cast<uint16_t>(
            constrain(
                mag10,
                0.0f,
                65535.0f
            )
        );

    UART::sendIRData(
        mag1ToSend,
        mag10ToSend,
        mask
    );

    // --------------------------------------------------
    // Metal-detector data
    // --------------------------------------------------

    const float freq0 =
        Metal::getLatestFreq(0);

    const float freq1 =
        Metal::getLatestFreq(1);

    UART::sendMetalData(
        0,
        freq0
    );

    UART::sendMetalData(
        1,
        freq1
    );

    // --------------------------------------------------
    // Optical-flow data
    // --------------------------------------------------

    float flowPosX = 0.0f;
    float flowPosY = 0.0f;

    float rawDx = 0.0f;
    float rawDy = 0.0f;

    float flowSqual = 0.0f;

    bool flowValid = false;

    updateFlowSensor(
        &flowPosX,
        &flowPosY,
        &rawDx,
        &rawDy,
        &flowSqual,
        &flowValid
    );

    // Debug pose mapping:
    //
    // X       = integrated X
    // Y       = integrated Y
    // Heading = flowReady status
    // Vx      = raw dx
    // Vy      = raw dy
    // Omega   = SQUAL

    UART::sendPoseData(
        flowPosX,
        flowPosY,
        0.0f,      // heading until you add a gyro
        rawDx,
        rawDy,
        0.0f
    );
    // --------------------------------------------------
    // Optional USB serial diagnostics
    // --------------------------------------------------

    const unsigned long nowMs =
        millis();

    if (
        nowMs - lastDebugPrintMs >=
        DEBUG_PRINT_INTERVAL_MS
    )
    {
        lastDebugPrintMs = nowMs;

        Serial.printf(
            "[IR] mag1=%u mag10=%u wifi=%d | "
            "[Metal] f0=%.2fHz f1=%.2fHz | "
            "[Flow] ready=%d valid=%d "
            "raw=(%.0f,%.0f) squal=%.0f "
            "pos=(%.2f,%.2f)mm\n",

            mag1ToSend,
            mag10ToSend,
            (mask & WIFI_SWITCH_MASK) != 0,

            freq0,
            freq1,

            flowReady,
            flowValid,

            rawDx,
            rawDy,
            flowSqual,

            flowPosX,
            flowPosY
        );
    }

    delay(20);
}

