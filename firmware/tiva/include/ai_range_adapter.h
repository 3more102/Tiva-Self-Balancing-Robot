#ifndef GYROGUARD_AI_RANGE_ADAPTER_H
#define GYROGUARD_AI_RANGE_ADAPTER_H

#include <stdbool.h>

#include "ai_navigation.h"
#include "navigation.h"

/* Adapt the canonical rule-based navigation input to the AI shadow model.
 * Autonomy enable remains an independent input; invalid ranges stay invalid.
 * Yaw rate is supplied separately from the calibrated IMU path. */
bool ai_sensor_frame_from_navigation(AiSensorFrame *frame,
                                     const NavigationInput *input,
                                     float yaw_rate_dps);

#endif
