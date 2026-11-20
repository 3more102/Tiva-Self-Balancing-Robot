#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "ai_range_adapter.h"

static void test_field_mapping(void)
{
    NavigationInput input = {
        true, true, true, true,
        840U, 620U, 910U,
        100U, 120U, 140U
    };
    AiSensorFrame frame;

    assert(ai_sensor_frame_from_navigation(&frame, &input, true, -12.5f));
    assert(frame.inference_enabled);
    assert(frame.front_valid && frame.left_valid && frame.right_valid);
    assert(frame.front_mm == 840U);
    assert(frame.left_mm == 620U);
    assert(frame.right_mm == 910U);
    assert(frame.front_sample_ms == 100U);
    assert(frame.left_sample_ms == 120U);
    assert(frame.right_sample_ms == 140U);
    assert(fabsf(frame.yaw_rate_dps + 12.5f) < 0.0001f);
}

static void test_invalid_ranges_stay_invalid_and_autonomy_is_independent(void)
{
    NavigationInput input = {
        false, true, false, true,
        500U, 1234U, 700U,
        200U, 210U, 220U
    };
    AiSensorFrame frame;

    assert(ai_sensor_frame_from_navigation(&frame, &input, true, 0.0f));
    assert(frame.inference_enabled);
    assert(frame.front_valid && !frame.left_valid && frame.right_valid);
    assert(frame.front_mm == 500U);
    assert(frame.left_mm == 0U);
    assert(frame.right_mm == 700U);
    assert(frame.left_sample_ms == 210U);

    /* Shadow inference can run while the separate navigation autonomy gate
       remains false; disabling the inference gate is an independent choice. */
    assert(ai_sensor_frame_from_navigation(&frame, &input, false, 0.0f));
    assert(!frame.inference_enabled);
}

static void test_null_arguments(void)
{
    NavigationInput input = {0};
    AiSensorFrame frame;
    assert(!ai_sensor_frame_from_navigation(NULL, &input, true, 0.0f));
    assert(!ai_sensor_frame_from_navigation(&frame, NULL, true, 0.0f));
}

int main(void)
{
    test_field_mapping();
    test_invalid_ranges_stay_invalid_and_autonomy_is_independent();
    test_null_arguments();
    puts("PASS: navigation-input to TinyML sensor-frame adapter tests");
    return 0;
}
