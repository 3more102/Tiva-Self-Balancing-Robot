#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "diagnostics.h"

static void calibration(void)
{
    GyroCalibration cal;
    gyro_cal_init(&cal);
    assert(!gyro_cal_ready(&cal));
    assert(isnan(gyro_cal_bias_dps(&cal)));
    assert(!gyro_cal_add(&cal, 0.0f, 1.0f, 0.3f, false));
    assert(cal.resets == 1U);
    assert(!gyro_cal_add(&cal, 0.0f, 0.0f, 0.3f, true));
    assert(!gyro_cal_add(&cal, 0.0f, 1.0f, NAN, true));
    assert(!gyro_cal_add(&cal, 0.0f, 1.0f, 6.0f, true));
    assert(!gyro_cal_add(&cal, 0.2f, 0.98f, 0.0f, true));
    assert(!gyro_cal_add(&cal, 0.0f, 1.3f, 0.0f, true));

    for (uint32_t k=0; k<GYRO_CAL_SAMPLES; ++k) {
        float gyro = 0.4f + ((k % 2U) ? 0.05f : -0.05f);
        bool ready = gyro_cal_add(&cal, 0.0f, 1.0f, gyro, true);
        assert(ready == (k == GYRO_CAL_SAMPLES-1U));
    }
    assert(gyro_cal_ready(&cal));
    assert(fabsf(gyro_cal_bias_dps(&cal) - 0.4f) < 0.0001f);
    assert(gyro_cal_add(&cal, 0.0f, 1.0f, NAN, false));
    assert(cal.valid_samples == GYRO_CAL_SAMPLES);

    gyro_cal_init(&cal);
    /* Gyro samples remain inside stationary-rate threshold, but the
       variance is above the calibration bound. */
    for (uint32_t k=0; k<GYRO_CAL_SAMPLES; ++k) {
        bool ready = gyro_cal_add(&cal, 0.0f, 1.0f,
                                  (k % 2U) ? 2.0f : -2.0f, true);
        assert(!ready);
    }
    assert(!gyro_cal_ready(&cal));
    assert(cal.valid_samples == 0U);
    assert(cal.resets == 1U);
}

static void timing(void)
{
    LoopTiming t;
    float dt = NAN;
    loop_timing_init(&t, 80000000U, 1000000U);
    assert(loop_timing_step(&t, 600000U, &dt));
    assert(fabsf(dt - 0.005f) < 0.000001f);
    assert(!loop_timing_step(&t, 599999U, &dt));
    assert(dt == 0.0f);
    assert(t.invalid_ticks == 1U);
    assert(!loop_timing_step(&t, 0xFFF00000U, &dt));
    /* Reinitialize near wrap and check uint32 subtraction explicitly. */
    loop_timing_init(&t, 80000000U, 200000U);
    assert(loop_timing_step(&t, UINT32_MAX - 199999U, &dt));
    assert(fabsf(dt - 0.005f) < 0.000001f);
    assert(!loop_timing_step(&t, 0xFF000000U, &dt));
    assert(t.invalid_ticks == 1U);
}

int main(void)
{
    calibration();
    timing();
    puts("PASS: gyro bias calibration and wrap-safe timer diagnostics");
    return 0;
}
