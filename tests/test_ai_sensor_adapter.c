#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ai_sensor_adapter.h"

static void test_range_snapshot_maps_to_model_input_frame(void)
{
    RangeSnapshot ranges;
    AiSensorFrame frame;
    memset(&ranges, 0, sizeof(ranges));
    ranges.front.valid = true;
    ranges.front.distance_mm = 840U;
    ranges.front.sample_ms = 100U;
    ranges.left.valid = false;
    ranges.left.distance_mm = 300U;
    ranges.left.sample_ms = 90U;
    ranges.right.valid = true;
    ranges.right.distance_mm = 1250U;
    ranges.right.sample_ms = 95U;

    assert(ai_sensor_frame_from_ranges(&frame, &ranges, false, -12.5f));
    assert(!frame.autonomy_enabled);
    assert(frame.front_valid && !frame.left_valid && frame.right_valid);
    assert(frame.front_mm == 840U && frame.left_mm == 300U &&
           frame.right_mm == 1250U);
    assert(frame.front_sample_ms == 100U);
    assert(frame.left_sample_ms == 90U);
    assert(frame.right_sample_ms == 95U);
    assert(fabsf(frame.yaw_rate_dps + 12.5f) < 0.001f);
}

static void test_null_inputs_are_rejected(void)
{
    RangeSnapshot ranges;
    AiSensorFrame frame;
    assert(!ai_sensor_frame_from_ranges(NULL, &ranges, true, 0.0f));
    assert(!ai_sensor_frame_from_ranges(&frame, NULL, true, 0.0f));
}

int main(void)
{
    test_range_snapshot_maps_to_model_input_frame();
    test_null_inputs_are_rejected();
    puts("PASS: range snapshot to TinyML sensor-frame adapter tests");
    return 0;
}
