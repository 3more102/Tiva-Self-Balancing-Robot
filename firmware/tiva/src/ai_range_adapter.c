#include "ai_range_adapter.h"

#include <stddef.h>
#include <string.h>

bool ai_sensor_frame_from_navigation(AiSensorFrame *frame,
                                     const NavigationInput *input,
                                     bool inference_enabled,
                                     float yaw_rate_dps)
{
    if (frame == NULL || input == NULL) return false;

    memset(frame, 0, sizeof(*frame));
    frame->inference_enabled = inference_enabled;

    frame->front_valid = input->front_valid;
    frame->left_valid = input->left_valid;
    frame->right_valid = input->right_valid;

    frame->front_mm = input->front_valid ? input->front_mm : 0U;
    frame->left_mm = input->left_valid ? input->left_mm : 0U;
    frame->right_mm = input->right_valid ? input->right_mm : 0U;

    frame->front_sample_ms = input->front_sample_ms;
    frame->left_sample_ms = input->left_sample_ms;
    frame->right_sample_ms = input->right_sample_ms;
    frame->yaw_rate_dps = yaw_rate_dps;
    return true;
}
