#include "MetalDetector.h"
#include "driver/pcnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace Metal
{

/* ---------------- tune for your coil/oscillator ---------------- */
// Direct pulse counting has a resolution floor of ~1/GATE_S (unavoidable --
// same time/frequency tradeoff for any periodic-signal measurement, not a
// code limitation). At ~150kHz needing 1Hz precision: GATE_S=1.5 gives
// ~0.67Hz nominal resolution with some margin. Drop to 1.0 for exactly ~1Hz,
// go higher (2-3s) for more margin at the cost of a slower update rate.
static constexpr float    GATE_S            = 1.5f;
static constexpr uint32_t SUBWINDOW_MS      = 100;   // must stay well under the ~218ms it'd take a 16-bit
                                                       // hw counter to overflow at ~150kHz -- we clear it
                                                       // this often in software so it never gets close
static constexpr uint16_t FILTER_CLOCK_CYCLES = 10;   // glitch filter, in APB clock cycles (~125ns @ 80MHz);
                                                       // must be << your oscillator's period, max is 1023

// One PCNT unit per detector -- S3 has 4 (PCNT_UNIT_0..3), we use 0 and 1.
static constexpr pcnt_unit_t    PCNT_UNITS[NUM_DETECTORS] = { PCNT_UNIT_0, PCNT_UNIT_1 };
static constexpr pcnt_channel_t PCNT_CHANNEL = PCNT_CHANNEL_0;

struct Instance
{
    int          oscGpioPin;
    float        latestFreq;
    portMUX_TYPE lock;
};

static Instance s_instances[NUM_DETECTORS] = {
    { -1, 0.0f, portMUX_INITIALIZER_UNLOCKED },
    { -1, 0.0f, portMUX_INITIALIZER_UNLOCKED },
};

static void pcntInit(uint8_t id)
{
    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = s_instances[id].oscGpioPin;
    cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;   // no level-gating input, just count edges
    cfg.unit           = PCNT_UNITS[id];
    cfg.channel        = PCNT_CHANNEL;
    cfg.pos_mode       = PCNT_COUNT_INC;      // count rising edges
    cfg.neg_mode       = PCNT_COUNT_DIS;      // ignore falling edges
    cfg.lctrl_mode     = PCNT_MODE_KEEP;
    cfg.hctrl_mode     = PCNT_MODE_KEEP;
    cfg.counter_h_lim  = 32767;               // not relied on for overflow -- we clear every SUBWINDOW_MS instead
    cfg.counter_l_lim  = -32768;

    ESP_ERROR_CHECK(pcnt_unit_config(&cfg));

    ESP_ERROR_CHECK(pcnt_set_filter_value(PCNT_UNITS[id], FILTER_CLOCK_CYCLES));
    ESP_ERROR_CHECK(pcnt_filter_enable(PCNT_UNITS[id]));

    ESP_ERROR_CHECK(pcnt_counter_pause(PCNT_UNITS[id]));
    ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNITS[id]));
    ESP_ERROR_CHECK(pcnt_counter_resume(PCNT_UNITS[id]));
}

// Blocks the calling task for ~GATE_S seconds, returns measured frequency
// in Hz. Polls + accumulates in SUBWINDOW_MS chunks (clearing the hw counter
// each time) so the 16-bit counter never gets near overflow, and uses the
// ACTUAL elapsed wall-clock time (micros()) rather than trusting the nominal
// vTaskDelay duration -- the delay is only a scheduling request, the real
// elapsed time can differ by several ms, which at ~150kHz would otherwise
// dominate the error far more than the counting itself does.
static float measureFreqOnce(uint8_t id)
{
    pcnt_unit_t unit = PCNT_UNITS[id];
    ESP_ERROR_CHECK(pcnt_counter_clear(unit));

    int64_t accum = 0;
    uint32_t t0 = micros();
    uint32_t targetMs = (uint32_t)(GATE_S * 1000.0f);
    uint32_t elapsedMs = 0;

    while (elapsedMs < targetMs)
    {
        vTaskDelay(pdMS_TO_TICKS(SUBWINDOW_MS));

        int16_t raw = 0;
        ESP_ERROR_CHECK(pcnt_get_counter_value(unit, &raw));
        ESP_ERROR_CHECK(pcnt_counter_clear(unit));
        accum += raw;

        elapsedMs = (micros() - t0) / 1000;   // unsigned subtraction, safe across micros() rollover
    }

    uint32_t t1 = micros();
    float elapsedSec = (float)(t1 - t0) / 1000000.0f;

    return (float)accum / elapsedSec;
}

static void task(void *arg)
{
    uint8_t id = (uint8_t)(uintptr_t)arg;

    pcntInit(id);

    for (;;)
    {
        float f = measureFreqOnce(id);

        portENTER_CRITICAL(&s_instances[id].lock);
        s_instances[id].latestFreq = f;
        portEXIT_CRITICAL(&s_instances[id].lock);
    }
}

void begin(uint8_t id, int oscGpioPin)
{
    if (id >= NUM_DETECTORS) return;

    s_instances[id].oscGpioPin = oscGpioPin;

    char name[16];
    snprintf(name, sizeof(name), "metal_pcnt%u", id);
    xTaskCreatePinnedToCore(task, name, 4096, (void*)(uintptr_t)id, 1, nullptr, 0);
}

float getLatestFreq(uint8_t id)
{
    if (id >= NUM_DETECTORS) return 0.0f;

    portENTER_CRITICAL(&s_instances[id].lock);
    float f = s_instances[id].latestFreq;
    portEXIT_CRITICAL(&s_instances[id].lock);
    return f;
}

} // namespace Metal