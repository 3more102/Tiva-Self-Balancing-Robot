#include "ai_sensor_adapter.h"

#include <stddef.h>
#include <string.h>

bool ai_sensor_frame_from_ranges(AiSensorFrame *frame,
                                 const RangeSnapshot *ranges,
                                 bool autonomy_enabled,
                                 float yaw_rate_dps)
{
    if (frame == NULL || ranges == NULL) return false;

    memset(frame, 0, sizeof(*frame));
    frame->autonomy_enabled = autonomy_enabled;
    frame->front_valid = ranges->front.valid;
    frame->left_valid = ranges->left.valid;
    frame->right_valid = ranges->right.valid;
    frame->front_mm = ranges->front.distance_mm;
    frame->left_mm = ranges->left.distance_mm;
    frame->right_mm = ranges->right.distance_mm;
    frame->front_sample_ms = ranges->front.sample_ms;
    frame->left_sample_ms = ranges->left.sample_ms;
    frame->right_sample_ms = ranges->right.sample_ms;
    frame->yaw_rate_dps = yaw_rate_dps;
    return true;
}
