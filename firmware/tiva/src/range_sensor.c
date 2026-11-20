#include "range_sensor.h"

#include <stddef.h>
#include <string.h>

void range_sensor_bank_init(RangeSensorBank *bank)
{
    if (bank == NULL) return;
    memset(bank, 0, sizeof(*bank));
}

bool range_sensor_publish(RangeSensorBank *bank, RangeSensorId id,
                          bool measurement_valid, uint16_t distance_mm,
                          uint32_t sample_ms)
{
    RangeSample *sample;
    bool accepted;

    if (bank == NULL || (uint32_t)id >= (uint32_t)RANGE_SENSOR_COUNT)
        return false;

    sample = &bank->samples[(uint32_t)id];
    accepted = measurement_valid && distance_mm > 0U &&
               distance_mm <= RANGE_SENSOR_MAX_VALID_MM;

    sample->valid = accepted;
    sample->distance_mm = accepted ? distance_mm : 0U;
    sample->sample_ms = sample_ms;
    ++sample->sequence;

    return accepted;
}

bool range_sensor_read_fresh(const RangeSensorBank *bank, RangeSensorId id,
                             uint32_t now_ms, uint32_t max_age_ms,
                             RangeSample *sample)
{
    const RangeSample *source;

    if (sample == NULL) return false;
    memset(sample, 0, sizeof(*sample));

    if (bank == NULL || (uint32_t)id >= (uint32_t)RANGE_SENSOR_COUNT)
        return false;

    source = &bank->samples[(uint32_t)id];
    if (!source->valid ||
        (uint32_t)(now_ms - source->sample_ms) > max_age_ms)
        return false;

    *sample = *source;
    return true;
}
