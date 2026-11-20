#ifndef GYROGUARD_AI_SENSOR_ADAPTER_H
#define GYROGUARD_AI_SENSOR_ADAPTER_H

#include <stdbool.h>

#include "ai_navigation.h"
#include "range_sensing.h"

/* Map a freshness-checked range snapshot and measured yaw rate into the
 * model's documented sensor-frame order. Does not enable autonomy. */
bool ai_sensor_frame_from_ranges(AiSensorFrame *frame,
                                 const RangeSnapshot *ranges,
                                 bool autonomy_enabled,
                                 float yaw_rate_dps);

#endif /* GYROGUARD_AI_SENSOR_ADAPTER_H */
