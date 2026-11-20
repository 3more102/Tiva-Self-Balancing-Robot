#ifndef GYROGUARD_AI_RANGE_ADAPTER_H
#define GYROGUARD_AI_RANGE_ADAPTER_H

#include <stdbool.h>

#include "ai_navigation.h"
#include "navigation.h"

/* Adapt the canonical rule-based navigation input to the AI shadow model.
 * The inference-enable gate is independent of NavigationInput.autonomy_enabled;
 * this allows shadow evaluation while actuator autonomy remains disabled.
 * Invalid ranges stay invalid. Yaw rate is supplied by calibrated IMU code. */
bool ai_sensor_frame_from_navigation(AiSensorFrame *frame,
                                     const NavigationInput *input,
                                     bool inference_enabled,
                                     float yaw_rate_dps);

#endif
