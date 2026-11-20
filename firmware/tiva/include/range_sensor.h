#ifndef TIVA_RANGE_SENSOR_H
#define TIVA_RANGE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

/* Hardware-independent range-sample contract.
 * A sensor backend publishes completed measurements in millimetres.
 * Publish from the foreground loop, not an ISR, unless the caller adds
 * appropriate critical-section protection.
 */
#define RANGE_SENSOR_MAX_VALID_MM 6000U
#define RANGE_FRONT_MAX_AGE_MS 250U
#define RANGE_SIDE_MAX_AGE_MS 750U

typedef enum {
    RANGE_FRONT = 0,
    RANGE_LEFT = 1,
    RANGE_RIGHT = 2,
    RANGE_SENSOR_COUNT = 3
} RangeSensorId;

typedef struct {
    bool valid;
    uint16_t distance_mm;
    uint32_t sample_ms;
    uint32_t sequence;
} RangeSample;

typedef struct {
    RangeSample samples[RANGE_SENSOR_COUNT];
} RangeSensorBank;

void range_sensor_bank_init(RangeSensorBank *bank);

/* A failed/invalid measurement invalidates the previous value for that sensor.
 * Returns true only when a valid distance in [1, 6000] mm was published.
 */
bool range_sensor_publish(RangeSensorBank *bank, RangeSensorId id,
                          bool measurement_valid, uint16_t distance_mm,
                          uint32_t sample_ms);

/* Age arithmetic is wrap-safe for uint32_t millisecond timestamps. */
bool range_sensor_read_fresh(const RangeSensorBank *bank, RangeSensorId id,
                             uint32_t now_ms, uint32_t max_age_ms,
                             RangeSample *sample);

#endif /* TIVA_RANGE_SENSOR_H */
