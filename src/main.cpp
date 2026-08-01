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
#define FLOW_SCK_PIN  15
#define FLOW_MISO_PIN 16
#define FLOW_MOSI_PIN 17

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

// --- Constant-velocity Kalman filter, one per axis ---
// State = [position, velocity]. Raw flow dx/dy per sample is jittery
// (confirmed on hardware), so instead of accumulating it straight into
// position, each sample is treated as a noisy velocity measurement and
// blended with a constant-velocity prediction.
//
// This does NOT fix drift or the uncalibrated FLOW_COUNTS_PER_MM scale
// -- it only smooths noise on top of whatever estimate you'd get
// anyway. Garbage scale factor in, smoothed garbage out.
struct KalmanState1D
{
    float pos = 0.0f;
    float vel = 0.0f;
    // 2x2 covariance [[p00,p01],[p10,p11]] -- starts wide-open/uncertain.
    float p00 = 1.0f, p01 = 0.0f, p10 = 0.0f, p11 = 1.0f;
};

// Both placeholders -- tune against the raw-vs-filtered numbers in the
// debug print below. Bigger Q -> trusts new measurements more (less
// smoothing, more responsive). Bigger R -> trusts the constant-velocity
// model more (more smoothing, more lag).
static constexpr float KF_PROCESS_NOISE = 50.0f;      // (mm/s^2)-ish, tune me
static constexpr float KF_MEASUREMENT_NOISE = 400.0f; // (mm/s)^2-ish, tune me

KalmanState1D kfX;
KalmanState1D kfY;

void kalmanPredict(KalmanState1D &s, float dt)
{
    s.pos += s.vel * dt;

    const float p00 = s.p00 + dt * (s.p01 + s.p10) + dt * dt * s.p11 + KF_PROCESS_NOISE * dt;
    const float p01 = s.p01 + dt * s.p11;
    const float p10 = s.p10 + dt * s.p11;
    const float p11 = s.p11 + KF_PROCESS_NOISE * dt;

    s.p00 = p00;
    s.p01 = p01;
    s.p10 = p10;
    s.p11 = p11;
}

// Measurement is a velocity reading (H = [0 1]) -- the flow sensor
// gives displacement/dt each sample, not position directly.
void kalmanUpdateVelocity(KalmanState1D &s, float measuredVel)
{
    const float y = measuredVel - s.vel;
    const float S = s.p11 + KF_MEASUREMENT_NOISE;
    const float k0 = s.p01 / S;
    const float k1 = s.p11 / S;

    s.pos += k0 * y;
    s.vel += k1 * y;

    const float p00 = s.p00 - k0 * s.p01;
    const float p01 = s.p01 - k0 * s.p11;
    const float p10 = s.p10 - k1 * s.p01;
    const float p11 = s.p11 - k1 * s.p11;

    s.p00 = p00;
    s.p01 = p01;
    s.p10 = p10;
    s.p11 = p11;
}

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

// Runs one predict/update cycle of the per-axis Kalman filters and
// hands back the filtered pos/vel plus the raw (unfiltered) velocity
// for that sample -- print both so the filter can actually be tuned
// instead of tuned blind.
void updateFlowSensor(
    float *outPosX, float *outPosY,
    float *outVx, float *outVy,
    float *outRawVx, float *outRawVy,
    bool *outValid)
{
    const unsigned long now = millis();
    const float dt = (now - lastFlowUpdateMs) / 1000.0f;
    lastFlowUpdateMs = now;

    *outRawVx = 0.0f;
    *outRawVy = 0.0f;
    *outValid = false;

    if (flowReady && dt > 0.0f)
    {
        kalmanPredict(kfX, dt);
        kalmanPredict(kfY, dt);

        int16_t dx, dy;
        uint8_t squal;
        bool valid = flowSensor.readMotionBurst(&dx, &dy, &squal);

        if (valid)
        {
            const float dxMm = static_cast<float>(dx) / FLOW_COUNTS_PER_MM;
            const float dyMm = static_cast<float>(dy) / FLOW_COUNTS_PER_MM;

            *outRawVx = dxMm / dt;
            *outRawVy = dyMm / dt;

            kalmanUpdateVelocity(kfX, *outRawVx);
            kalmanUpdateVelocity(kfY, *outRawVy);

            *outValid = true;
        }
        // Invalid sample: predict still ran above, so pos/vel coast
        // forward on the last known velocity rather than freezing or
        // snapping to 0. outValid stays false -- receiver should not
        // update its trust in this pose.
    }

    *outPosX = kfX.pos;
    *outPosY = kfY.pos;
    *outVx = kfX.vel;
    *outVy = kfY.vel;
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
    float flowRawVx = 0.0f, flowRawVy = 0.0f;
    bool  flowValid = false;
    updateFlowSensor(&flowPosX, &flowPosY, &flowVx, &flowVy, &flowRawVx, &flowRawVy, &flowValid);

    // Only send a POSE frame when the flow sensor produced a fresh,
    // SQUAL-gated sample this tick. When invalid (surface too low-texture,
    // out of range, or sensor not ready), the Kalman filter coasted
    // internally but we don't transmit -- the main board simply won't
    // receive a POSE update, which is the correct behavior: stale/coasted
    // estimates shouldn't trigger a position update on the other side.
    if (flowValid)
    {
        UART::sendPoseData(flowPosX, flowPosY, 0.0f, flowVx, flowVy, 0.0f);
    }

    const unsigned long nowMs = millis();

    if (nowMs - lastDebugPrintMs >= DEBUG_PRINT_INTERVAL_MS)
    {
        lastDebugPrintMs = nowMs;

        Serial.printf(
            "[IR] mag1=%u mag10=%u wifi=%d | [Metal] f0=%.2fHz f1=%.2fHz | "
            "[Flow] rdy=%d valid=%d pos=(%.1f,%.1f)mm v_raw=(%.1f,%.1f) v_kf=(%.1f,%.1f)mm/s\n",
            mag1ToSend, mag10ToSend, (mask & WIFI_SWITCH_MASK) != 0,
            freq0, freq1,
            flowReady, flowValid, flowPosX, flowPosY, flowRawVx, flowRawVy, flowVx, flowVy
        );
    }

    // Keep the loop responsive enough for smooth servo movement.
    delay(5);
}