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

/* ---------------- geometry / calibration ---------------- */
// Sensor layout assumed: LEFT and RIGHT mounted side by side, both pointing
// straight down, both oriented with their local +x axis pointing toward the
// robot's forward direction. Separated by `baseline` along the robot's
// LEFT-RIGHT (y) axis.
//
//   ω           = (dx_right - dx_left) / baseline     -- rotation rate
//   vx_center   = (dx_right + dx_left) / 2             -- forward velocity
//   vy_center   = (dy_right + dy_left) / 2             -- strafe velocity
//
// If your two sensors are physically mounted with a different relative
// orientation, adjust the sign/axis mapping below to match -- get this
// wrong and theta will drift or run backwards even though the robot looks
// like it's tracking fine in a straight line.

static Bitcraze_PMW3901 *s_flowLeft  = nullptr;
static Bitcraze_PMW3901 *s_flowRight = nullptr;

static float s_baselineMeters  = 0.1375f;
static float s_pixelsPerMeter  = 1.0f;

static Pose         s_pose = {0, 0, 0, 0, 0, 0};
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

// How often we poll both sensors. The PMW3901 can go faster than this, but
// ~100 Hz is already well above what a mecanum robot's dynamics need, and
// keeps the integration math (which assumes "dt is small and constant")
// well behaved.
static constexpr uint32_t LOOP_PERIOD_MS = 10;

static void readSensorsRaw(int16_t &dxL, int16_t &dyL, int16_t &dxR, int16_t &dyR)
{
    // readMotionCount() is a blocking SPI transaction per sensor -- each
    // call pulls that sensor's dedicated CS low, clocks out the burst-read
    // registers, then releases it. Because CS is per-sensor, doing these
    // back-to-back on a shared bus is safe: only one sensor is ever
    // actively driving MISO at a time.
    s_flowLeft->readMotionCount(&dxL, &dyL);
    s_flowRight->readMotionCount(&dxR, &dyR);
}

static void integrateOnce(float dt)
{
    int16_t dxL, dyL, dxR, dyR;
    readSensorsRaw(dxL, dyL, dxR, dyR);

    // Deadband -- kill sensor read noise before it accumulates into a
    // random walk. Tune NOISE_FLOOR_COUNTS from stationary logging.
    static constexpr int16_t NOISE_FLOOR_COUNTS = 2;
    if (abs(dxL) < NOISE_FLOOR_COUNTS && abs(dxR) < NOISE_FLOOR_COUNTS &&
        abs(dyL) < NOISE_FLOOR_COUNTS && abs(dyR) < NOISE_FLOOR_COUNTS)
    {
        portENTER_CRITICAL(&s_lock);
        s_pose.vx = 0.0f;
        s_pose.vy = 0.0f;
        portEXIT_CRITICAL(&s_lock);
        return;
    }

    // --- pixels -> meters, still in each sensor's own local frame ---
    const float dxL_m = dxL / s_pixelsPerMeter;
    const float dyL_m = dyL / s_pixelsPerMeter;
    const float dxR_m = dxR / s_pixelsPerMeter;
    const float dyR_m = dyR / s_pixelsPerMeter;

    // --- two sensors -> body-frame displacement this cycle ---
    // NOTE: no heading/theta tracking. x/y are just body-frame displacement
    // summed directly, i.e. this assumes the robot doesn't turn. If it
    // does turn, x/y will be wrong from that point on -- see comment in
    // Flowsensor.h.
    const float d_forward = (dxR_m + dxL_m) * 0.5f;   // forward displacement this cycle
    const float d_strafe  = (dyR_m + dyL_m) * 0.5f;   // strafe displacement this cycle

    portENTER_CRITICAL(&s_lock);

    s_pose.x += d_forward;
    s_pose.y += d_strafe;

    if (dt > 0.0f)
    {
        s_pose.vx = d_forward / dt;
        s_pose.vy = d_strafe  / dt;
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

void begin(int csLeftPin, int csRightPin, float baselineMeters, float pixelsPerMeter)
{
    s_baselineMeters = baselineMeters;
    s_pixelsPerMeter = pixelsPerMeter;

    // Must happen before Bitcraze_PMW3901::begin() -- that call internally
    // does SPI.begin() with no args, which only takes effect the FIRST
    // time SPI is started. Going first here is what makes our pin choice
    // (avoiding GPIO 13) actually stick instead of silently reverting to
    // the board default.
    SPI.begin(FLOW_SCK_PIN, FLOW_MISO_PIN, FLOW_MOSI_PIN, -1);

    s_flowLeft  = new Bitcraze_PMW3901(csLeftPin);
    s_flowRight = new Bitcraze_PMW3901(csRightPin);

    // begin() here does the sensor's internal power-up/init register
    // sequence (handled inside the library -- don't hand-roll this, the
    // PixArt init sequence is long and picky about timing).
    if (!s_flowLeft->begin())
    {
        Serial.println("[Flow] LEFT sensor init failed -- check wiring/CS pin");
    }
    if (!s_flowRight->begin())
    {
        Serial.println("[Flow] RIGHT sensor init failed -- check wiring/CS pin");
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