#include "MetalDetector.h"
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace Metal
{

/* ---------------- tune for your coil/oscillator ---------------- */
static constexpr int   OSC_GPIO_PIN        = 15;    // <-- set to your actual Schmitt-trigger output pin
// Direct pulse counting has a resolution floor of ~1/GATE_S (unavoidable --
// same time/frequency tradeoff for any periodic-signal measurement, not a
// code limitation). At ~150kHz needing 1Hz precision: GATE_S=1.5 gives
// ~0.67Hz nominal resolution with some margin. Drop to 1.0 for exactly ~1Hz,
// go higher (2-3s) for more margin at the cost of a slower update rate.
static constexpr float GATE_S              = 1.5f;
static constexpr int   PCNT_HIGH_LIMIT     = 30000; // headroom below 16-bit hw rollover (32767)
static constexpr int   CALIB_SAMPLES       = 5;     // gate windows averaged at boot for baseline (~5*GATE_S seconds)
static constexpr float DETECT_THRESHOLD_HZ = 150.0f;// min |delta| to call it "metal" -- re-tune now that the
                                                      // noise floor dropped ~150x; empirically this can likely
                                                      // come down a lot, try real Al/steel samples and see
static constexpr float SMOOTH_ALPHA        = 1.0f;  // EMA smoothing; 1.0 = off. Each sample is already
                                                      // ~1Hz-precise at GATE_S=1.5, so heavy smoothing just adds
                                                      // lag on top of an already-slow ~1.5s native update rate
static constexpr uint32_t GLITCH_FILTER_NS = 100;   // must be << your oscillator's period

static pcnt_unit_handle_t s_unit = nullptr;
static volatile int64_t s_accum = 0;
static portMUX_TYPE s_countLock = portMUX_INITIALIZER_UNLOCKED;

static Reading s_latest = { 0.0f, 0.0f, METAL_NONE };
static portMUX_TYPE s_readingLock = portMUX_INITIALIZER_UNLOCKED;

static bool IRAM_ATTR onReach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *ctx)
{
    portENTER_CRITICAL_ISR(&s_countLock);
    s_accum += edata->watch_point_value;   // always +HIGH_LIMIT (counter only ever increases)
    portEXIT_CRITICAL_ISR(&s_countLock);
    return false;
}

static void pcntInit()
{
    pcnt_unit_config_t unitCfg = {
        .low_limit  = -1,               // required negative; never actually reached
        .high_limit = PCNT_HIGH_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unitCfg, &s_unit));

    pcnt_glitch_filter_config_t filterCfg = { .max_glitch_ns = GLITCH_FILTER_NS };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_unit, &filterCfg));

    pcnt_chan_config_t chanCfg = {
        .edge_gpio_num  = OSC_GPIO_PIN,
        .level_gpio_num = -1,           // no level gating, just count edges
    };
    pcnt_channel_handle_t chan;
    ESP_ERROR_CHECK(pcnt_new_channel(s_unit, &chanCfg, &chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // count rising edges
        PCNT_CHANNEL_EDGE_ACTION_HOLD));    // ignore falling edges

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit, PCNT_HIGH_LIMIT));

    pcnt_event_callbacks_t cbs = { .on_reach = onReach };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(s_unit, &cbs, nullptr));

    ESP_ERROR_CHECK(pcnt_unit_enable(s_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(s_unit));
}

// Blocks the calling task for ~GATE_S seconds, returns measured frequency
// in Hz. Uses the ACTUAL elapsed wall-clock time (micros()), not the nominal
// vTaskDelay duration -- the delay is only a scheduling request, the real
// elapsed time can differ by several ms, which at ~150kHz would otherwise
// dominate the error far more than the counting itself does.
static float measureFreqOnce()
{
    portENTER_CRITICAL(&s_countLock);
    s_accum = 0;
    portEXIT_CRITICAL(&s_countLock);
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_unit));

    uint32_t t0 = micros();
    vTaskDelay(pdMS_TO_TICKS((uint32_t)(GATE_S * 1000.0f)));   // nominal duration -- doesn't need to be precise
    uint32_t t1 = micros();

    int raw = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(s_unit, &raw));

    portENTER_CRITICAL(&s_countLock);
    int64_t total = s_accum + raw;
    portEXIT_CRITICAL(&s_countLock);

    uint32_t elapsedUs = t1 - t0;   // unsigned subtraction, safe across micros() rollover
    float elapsedSec = (float)elapsedUs / 1000000.0f;

    return (float)total / elapsedSec;
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