#include "diagnostics.h"
#include <math.h>

#define RAD_TO_DEG 57.29577951308232f

void gyro_cal_init(GyroCalibration *cal)
{
    cal->valid_samples = 0U;
    cal->resets = 0U;
    cal->mean_dps = 0.0f;
    cal->m2_dps = 0.0f;
    cal->complete = false;
}

static void gyro_cal_restart(GyroCalibration *cal)
{
    cal->valid_samples = 0U;
    cal->mean_dps = 0.0f;
    cal->m2_dps = 0.0f;
    cal->complete = false;
    ++cal->resets;
}

bool gyro_cal_add(GyroCalibration *cal, float ay, float az, float gx,
                  bool switch_released)
{
    float yz_norm, pitch;
    if (cal->complete) return true;
    /* Only calibrate while the switch is released; don't use a moving robot. */
    if (!switch_released || !isfinite(ay) || !isfinite(az) ||
        !isfinite(gx)) {
        gyro_cal_restart(cal);
        return false;
    }
    yz_norm = sqrtf(ay * ay + az * az);
    pitch = atan2f(ay, az) * RAD_TO_DEG;
    if (!isfinite(yz_norm) ||
        fabsf(yz_norm - 1.0f) > GYRO_CAL_MAX_GRAVITY_ERROR_G ||
        fabsf(pitch) > GYRO_CAL_MAX_TILT_DEG ||
        fabsf(gx) > GYRO_CAL_MAX_DPS) {
        gyro_cal_restart(cal);
        return false;
    }
    ++cal->valid_samples;
    /* Welford: online gyro mean / variance with good numerical stability. */
    const float delta = gx - cal->mean_dps;
    cal->mean_dps += delta / (float)cal->valid_samples;
    cal->m2_dps += delta * (gx - cal->mean_dps);
    if (cal->valid_samples == GYRO_CAL_SAMPLES) {
        float variance = cal->m2_dps / (float)(cal->valid_samples - 1U);
        if (!isfinite(variance) || variance > GYRO_CAL_MAX_STD_DPS *
                                                        GYRO_CAL_MAX_STD_DPS) {
            gyro_cal_restart(cal);
            return false;
        }
        cal->complete = true;
    }
    return cal->complete;
}

bool gyro_cal_ready(const GyroCalibration *cal)
{
    return cal->complete;
}

float gyro_cal_bias_dps(const GyroCalibration *cal)
{
    return cal->complete ? cal->mean_dps : NAN;
}

void loop_timing_init(LoopTiming *timer, uint32_t clock_hz,
                      uint32_t downcounter_value)
{
    timer->last_counter = downcounter_value;
    timer->tick_hz = clock_hz;
    timer->min_ticks = clock_hz / 400U; /* 2.5 ms minimum */
    timer->max_ticks = (clock_hz / 1000U) * 8U; /* 8 ms maximum */
    timer->checked_ticks = 0U;
    timer->invalid_ticks = 0U;
}

bool loop_timing_step(LoopTiming *timer, uint32_t downcounter_value, float *dt_s)
{
    const uint32_t elapsed = timer->last_counter - downcounter_value;
    timer->last_counter = downcounter_value;
    ++timer->checked_ticks;
    *dt_s = 0.0f;
    if (timer->tick_hz == 0U || elapsed < timer->min_ticks ||
        elapsed > timer->max_ticks) {
        ++timer->invalid_ticks;
        return false;
    }
    *dt_s = (float)elapsed / (float)timer->tick_hz;
    return true;
}
