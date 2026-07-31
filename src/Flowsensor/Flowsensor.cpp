#include "Flowsensor.h"

#include <SPI.h>
#include <Bitcraze_PMW3901.h>   // lib_deps: bitcraze/Bitcraze PMW3901@^1.2
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace Flow
{

// ESP32-S3 default hardware SPI pins are SCK=12, MOSI=11, MISO=13, SS=10.
// MISO=13 collides with Metal detector 1 (PCNT input), which already owns
// that pin -- so we do NOT use the defaults. Instead we explicitly init
// SPI on these pins before the sensor library gets a chance to call its
// own SPI.begin() with no arguments (which would silently no-op once SPI
// is already started, keeping OUR pin choice -- but only if we go first).
static constexpr int FLOW_SCK_PIN  = 12;
static constexpr int FLOW_MOSI_PIN = 11;
static constexpr int FLOW_MISO_PIN = 16;  // moved off the conflicting default of 13

static Bitcraze_PMW3901 *s_flow = nullptr;

static float s_pixelsPerMeter = 1.0f;

static Pose         s_pose = {0, 0, 0, 0, 0, 0};
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

// How often we poll the sensor. The PMW3901 can go faster than this, but
// ~100 Hz is already well above what a mecanum robot's dynamics need, and
// keeps the integration math (which assumes "dt is small and constant")
// well behaved.
static constexpr uint32_t LOOP_PERIOD_MS = 10;

static void integrateOnce(float dt)
{
    int16_t dx, dy;
    s_flow->readMotionCount(&dx, &dy);

    // TEMP DIAGNOSTIC -- push the robot in one pure direction at a time
    // (straight forward/back only, then pure sideways only) and watch
    // which column actually moves and with what sign, to confirm the axis
    // mapping assumed below (dx = forward, dy = sideways) matches how the
    // sensor is mounted. Remove once axis mapping is confirmed.
    static uint32_t s_debugCounter = 0;
    if ((s_debugCounter++ % 50) == 0)
    {
        Serial.printf(
            "[Flow raw] FWD-axis(dx)=%6d   SIDE-axis(dy)=%6d\n",
            dx, dy
        );
    }

    // Deadband -- kill sensor read noise before it accumulates into a
    // random walk. Tune NOISE_FLOOR_COUNTS from stationary logging.
    static constexpr int16_t NOISE_FLOOR_COUNTS = 2;
    if (abs(dx) < NOISE_FLOOR_COUNTS && abs(dy) < NOISE_FLOOR_COUNTS)
    {
        portENTER_CRITICAL(&s_lock);
        s_pose.vx = 0.0f;
        s_pose.vy = 0.0f;
        portEXIT_CRITICAL(&s_lock);
        return;
    }

    // --- pixels -> meters ---
    const float dx_m = dx / s_pixelsPerMeter;
    const float dy_m = dy / s_pixelsPerMeter;

    portENTER_CRITICAL(&s_lock);

    // NOTE: no heading source yet -- x/y are raw body-frame displacement
    // summed directly, which is only correct while the robot doesn't turn.
    // Once the IMU is wired in, rotate (dx_m, dy_m) by the IMU's heading
    // before accumulating, same as the old two-sensor theta fusion did.
    s_pose.x += dx_m;
    s_pose.y += dy_m;

    if (dt > 0.0f)
    {
        s_pose.vx = dx_m / dt;
        s_pose.vy = dy_m / dt;
    }

    portEXIT_CRITICAL(&s_lock);
}

static void task(void *arg)
{
    (void)arg;

    uint32_t lastMs = millis();

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));

        uint32_t nowMs = millis();
        float dt = (nowMs - lastMs) / 1000.0f;
        lastMs = nowMs;

        integrateOnce(dt);
    }
}

void begin(int csPin, float pixelsPerMeter)
{
    s_pixelsPerMeter = pixelsPerMeter;

    // Must happen before Bitcraze_PMW3901::begin() -- that call internally
    // does SPI.begin() with no args, which only takes effect the FIRST
    // time SPI is started. Going first here is what makes our pin choice
    // (avoiding GPIO 13) actually stick instead of silently reverting to
    // the board default.
    SPI.begin(FLOW_SCK_PIN, FLOW_MISO_PIN, FLOW_MOSI_PIN, -1);

    s_flow = new Bitcraze_PMW3901(csPin);

    // begin() here does the sensor's internal power-up/init register
    // sequence (handled inside the library -- don't hand-roll this, the
    // PixArt init sequence is long and picky about timing).
    if (!s_flow->begin())
    {
        Serial.println("[Flow] sensor init failed -- check wiring/CS pin");
    }

    xTaskCreatePinnedToCore(task, "flow_fusion", 4096, nullptr, 1, nullptr, 0);
}

Pose getPose()
{
    portENTER_CRITICAL(&s_lock);
    Pose snapshot = s_pose;
    portEXIT_CRITICAL(&s_lock);
    return snapshot;
}

void resetPose()
{
    portENTER_CRITICAL(&s_lock);
    s_pose = {0, 0, 0, 0, 0, 0};
    portEXIT_CRITICAL(&s_lock);
}

} // namespace Flow
