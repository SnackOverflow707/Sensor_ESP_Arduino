#include "MetalDetector.h"
#include "driver/pcnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace Metal
{

/* ---------------- tune for your coil/oscillator ---------------- */
static constexpr int   OSC_GPIO_PIN        = 14;    // <-- set to your actual Schmitt-trigger output pin
// Direct pulse counting has a resolution floor of ~1/GATE_S (unavoidable --
// same time/frequency tradeoff for any periodic-signal measurement, not a
// code limitation). At ~150kHz needing 1Hz precision: GATE_S=1.5 gives
// ~0.67Hz nominal resolution with some margin. Drop to 1.0 for exactly ~1Hz,
// go higher (2-3s) for more margin at the cost of a slower update rate.
static constexpr float    GATE_S            = 1.5f;
static constexpr uint32_t SUBWINDOW_MS      = 100;   // must stay well under the ~218ms it'd take a 16-bit
                                                       // hw counter to overflow at ~150kHz -- we clear it
                                                       // this often in software so it never gets close
static constexpr int   CALIB_SAMPLES       = 5;      // gate windows averaged at boot for baseline (~5*GATE_S seconds)
static constexpr float DETECT_THRESHOLD_HZ = 150.0f; // min |delta| to call it "metal" -- re-tune now that the
                                                       // noise floor dropped ~150x; empirically this can likely
                                                       // come down a lot, try real Al/steel samples and see
static constexpr float SMOOTH_ALPHA        = 1.0f;   // EMA smoothing; 1.0 = off. Each sample is already
                                                       // ~1Hz-precise at GATE_S=1.5, so heavy smoothing just adds
                                                       // lag on top of an already-slow ~1.5s native update rate
static constexpr uint16_t FILTER_CLOCK_CYCLES = 10;   // glitch filter, in APB clock cycles (~125ns @ 80MHz);
                                                       // must be << your oscillator's period, max is 1023

static constexpr pcnt_unit_t    PCNT_UNIT    = PCNT_UNIT_0;
static constexpr pcnt_channel_t PCNT_CHANNEL = PCNT_CHANNEL_0;

static Reading s_latest = { 0.0f, 0.0f, METAL_NONE };
static portMUX_TYPE s_readingLock = portMUX_INITIALIZER_UNLOCKED;

static void pcntInit()
{
    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = OSC_GPIO_PIN;
    cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;   // no level-gating input, just count edges
    cfg.unit           = PCNT_UNIT;
    cfg.channel        = PCNT_CHANNEL;
    cfg.pos_mode       = PCNT_COUNT_INC;      // count rising edges
    cfg.neg_mode       = PCNT_COUNT_DIS;      // ignore falling edges
    cfg.lctrl_mode     = PCNT_MODE_KEEP;
    cfg.hctrl_mode     = PCNT_MODE_KEEP;
    cfg.counter_h_lim  = 32767;               // not relied on for overflow -- we clear every SUBWINDOW_MS instead
    cfg.counter_l_lim  = -32768;

    ESP_ERROR_CHECK(pcnt_unit_config(&cfg));

    ESP_ERROR_CHECK(pcnt_set_filter_value(PCNT_UNIT, FILTER_CLOCK_CYCLES));
    ESP_ERROR_CHECK(pcnt_filter_enable(PCNT_UNIT));

    ESP_ERROR_CHECK(pcnt_counter_pause(PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT));
    ESP_ERROR_CHECK(pcnt_counter_resume(PCNT_UNIT));
}

// Blocks the calling task for ~GATE_S seconds, returns measured frequency
// in Hz. Polls + accumulates in SUBWINDOW_MS chunks (clearing the hw counter
// each time) so the 16-bit counter never gets near overflow, and uses the
// ACTUAL elapsed wall-clock time (micros()) rather than trusting the nominal
// vTaskDelay duration -- the delay is only a scheduling request, the real
// elapsed time can differ by several ms, which at ~150kHz would otherwise
// dominate the error far more than the counting itself does.
static float measureFreqOnce()
{
    ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT));

    int64_t accum = 0;
    uint32_t t0 = micros();
    uint32_t targetMs = (uint32_t)(GATE_S * 1000.0f);
    uint32_t elapsedMs = 0;

    while (elapsedMs < targetMs)
    {
        vTaskDelay(pdMS_TO_TICKS(SUBWINDOW_MS));

        int16_t raw = 0;
        ESP_ERROR_CHECK(pcnt_get_counter_value(PCNT_UNIT, &raw));
        ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT));
        accum += raw;

        elapsedMs = (micros() - t0) / 1000;   // unsigned subtraction, safe across micros() rollover
    }

    uint32_t t1 = micros();
    float elapsedSec = (float)(t1 - t0) / 1000000.0f;

    return (float)accum / elapsedSec;
}

static void task(void *arg)
{
    pcntInit();

    Serial.println("[Metal] calibrating baseline, keep coil clear of metal...");
    double sum = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        sum += measureFreqOnce();
    }
    float baselineHz = sum / CALIB_SAMPLES;
    Serial.printf("[Metal] baseline = %.1f Hz\n", baselineHz);

    float smoothedHz = baselineHz;

    for (;;)
    {
        float f = measureFreqOnce();
        smoothedHz += SMOOTH_ALPHA * (f - smoothedHz);   // EMA to knock down jitter

        float delta = smoothedHz - baselineHz;
        Class cls = METAL_NONE;
        if (delta > DETECT_THRESHOLD_HZ)       cls = METAL_NON_FERROUS;
        else if (delta < -DETECT_THRESHOLD_HZ) cls = METAL_FERROUS;

        portENTER_CRITICAL(&s_readingLock);
        s_latest = { smoothedHz, delta, cls };
        portEXIT_CRITICAL(&s_readingLock);
    }
}

void begin()
{
    xTaskCreatePinnedToCore(task, "metal_pcnt", 4096, nullptr, 1, nullptr, 0);
}

Reading getLatest()
{
    portENTER_CRITICAL(&s_readingLock);
    Reading r = s_latest;
    portEXIT_CRITICAL(&s_readingLock);
    return r;
}

} // namespace Metal