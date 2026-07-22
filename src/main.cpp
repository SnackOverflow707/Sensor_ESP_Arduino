#include <Arduino.h>
#include <math.h>
#include "driver/ledc.h"
#include "driver/adc.h"
#include "UART.h"

// LED pins

#define LED_1KHZ     16
#define LED_10KHZ    17
#define LED_NONE     18

#define SAMPLE_RATE  40000   
#define N            256

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

void setLEDs(bool onek, bool tenk, bool none)
{
    digitalWrite(LED_1KHZ, onek ? HIGH : LOW);
    digitalWrite(LED_10KHZ, tenk ? HIGH : LOW);
    digitalWrite(LED_NONE, none ? HIGH : LOW);
}

void setup()
{
    Serial.begin(115200);

    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);

    pinMode(LED_1KHZ, OUTPUT);
    pinMode(LED_10KHZ, OUTPUT);
    pinMode(LED_NONE, OUTPUT);

    setLEDs(false, false, true);

    generateReference(1000.0f, sin1, cos1);
    generateReference(10000.0f, sin10, cos10);

    Serial.println("Frequency detector ready");
}

void loop()
{
    sampleADC();                                 // updates actualSampleRate

    generateReference(1000.0f, sin1, cos1);       // now matches the real Fs

    generateReference(10000.0f, sin10, cos10);    // now matches the real Fs


    float mag1 = correlate(sin1, cos1);
    float mag10 = correlate(sin10, cos10);
        // Print the values
    /*Serial.print("mag1 = ");
    Serial.print(mag1);

    Serial.print("\tmag10 = ");
    Serial.println(mag10);*/
    UART::sendIRData(mag1, mag10, 0);


    

    
    const float threshold = 8000.0f;

    if (mag1 < threshold && mag10 < threshold)
    {
        setLEDs(false, false, true);
        //Serial.println("Detected: NONE");

    }
    else if (mag1 > mag10)
    {
        setLEDs(true, false, false);
        //Serial.println("Detected: 1 kHz");

    }
    else
    {
        setLEDs(false, true, false);
        //Serial.println("Detected: 10 kHz");

    }
    delay(100);

    
}