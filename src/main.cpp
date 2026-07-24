#include <Arduino.h>
#include <math.h>
#include "driver/ledc.h"
#include "driver/adc.h"
#include "UART.h"
#include "MetalDetector.h"

// LED pins


#define SAMPLE_RATE  40000   
#define N            256

#define WIFI_SWITCH_PIN 2
#define WIFI_SWITCH_MASK 0x04

uint16_t samples[N];

float sin1[N];
float cos1[N];
float sin10[N];
float cos10[N];


float actualSampleRate = SAMPLE_RATE;

void generateReference(float freq, float *s, float *c)
{
    for (int i = 0; i < N; i++)
    {
        float t = (float)i / actualSampleRate;   
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

    actualSampleRate = (float)N * 1000000.0f / elapsed;

    
}

float correlate(float *s, float *c)
{
    float I = 0;
    float Q = 0;

    for (int i = 0; i < N; i++)
    {
        float x = (float)samples[i] - 2048.0f; // Remove DC offset


        I += x * s[i];
        Q += x * c[i];
    }

    return sqrtf(I * I + Q * Q);
}



void setup()
{
    Serial.begin(115200);
    UART::begin();

    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);

    pinMode(WIFI_SWITCH_PIN, INPUT_PULLUP);

    generateReference(1000.0f, sin1, cos1);
    generateReference(10000.0f, sin10, cos10);

    Metal::begin(0, 14);   // detector 1, confirmed wired
    Metal::begin(1, 13);   // detector 2 -- confirm actual wiring pin

    Serial.println("Frequency detector ready");
}

void loop()
{
    uint8_t mask = 0;
    // Switch closed -> LOW -> Wi-Fi ON
    if (digitalRead(WIFI_SWITCH_PIN) == LOW)
    {
        mask |= WIFI_SWITCH_MASK;
    }
    sampleADC();                                 // updates actualSampleRate

    generateReference(1000.0f, sin1, cos1);       // now matches the real Fs

    generateReference(10000.0f, sin10, cos10);    // now matches the real Fs


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

    Serial.printf("[Metal 0] freq=%.2f Hz\n", freq0);
    Serial.printf("[Metal 1] freq=%.2f Hz\n", freq1);

    delay(100);

    
}