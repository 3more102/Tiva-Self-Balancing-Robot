#include "range_sensing.h"

#include <stddef.h>
#include <string.h>

static bool time_reached(uint32_t now, uint32_t deadline)
{
    /* Valid for deadlines less than 2^31 ticks into the future. */
    return (uint32_t)(now - deadline) < 0x80000000U;
}

static void clear_active(RangeArray *array, RangeChannel *channel)
{
    channel->state = RANGE_CAPTURE_IDLE;
    channel->rise_us = 0U;
    channel->deadline_us = 0U;
    array->active_sensor = RANGE_SENSOR_NONE;
}

void range_array_init(RangeArray *array)
{
    uint32_t i;
    if (array == NULL) return;
    memset(array, 0, sizeof(*array));
    for (i = 0U; i < RANGE_SENSOR_COUNT; ++i)
        array->channel[i].state = RANGE_CAPTURE_IDLE;
    array->next_sensor = (uint8_t)RANGE_FRONT;
    array->active_sensor = RANGE_SENSOR_NONE;
}

void range_array_poll(RangeArray *array, uint32_t now_us)
{
    RangeChannel *channel;
    if (array == NULL || array->active_sensor >= RANGE_SENSOR_COUNT) return;

    channel = &array->channel[array->active_sensor];
    if (channel->state != RANGE_CAPTURE_IDLE &&
        time_reached(now_us, channel->deadline_us)) {
        ++channel->timeout_count;
        clear_active(array, channel);
    }
}

uint8_t range_array_next_trigger(RangeArray *array,
                                 uint32_t now_ms,
                                 uint32_t now_us)
{
    uint8_t sensor;
    RangeChannel *channel;

    if (array == NULL) return RANGE_SENSOR_NONE;
    range_array_poll(array, now_us);

    if (array->active_sensor != RANGE_SENSOR_NONE) return RANGE_SENSOR_NONE;
    if (array->has_triggered &&
        (uint32_t)(now_ms - array->last_trigger_ms) <
            RANGE_INTER_PING_GAP_MS) {
        return RANGE_SENSOR_NONE;
    }

    sensor = array->next_sensor;
    channel = &array->channel[sensor];
    channel->state = RANGE_CAPTURE_WAIT_RISE;
    channel->rise_us = 0U;
    channel->deadline_us = now_us + RANGE_ECHO_TIMEOUT_US;
    array->active_sensor = sensor;
    array->last_trigger_ms = now_ms;
    array->has_triggered = true;
    ++array->trigger_count;
    array->next_sensor =
        (uint8_t)((sensor + 1U) % RANGE_SENSOR_COUNT);
    return sensor;
}

void range_array_cancel_trigger(RangeArray *array, uint8_t sensor)
{
    RangeChannel *channel;
    if (array == NULL || sensor >= RANGE_SENSOR_COUNT ||
        array->active_sensor != sensor) return;

    channel = &array->channel[sensor];
    ++array->trigger_failure_count;
    clear_active(array, channel);
}

void range_array_echo_edge(RangeArray *array, uint8_t sensor,
                           bool echo_high, uint32_t timestamp_us,
                           uint32_t now_ms)
{
    RangeChannel *channel;
    uint32_t pulse_us;
    uint32_t distance_mm;

    if (array == NULL || sensor >= RANGE_SENSOR_COUNT ||
        array->active_sensor != sensor) return;

    channel = &array->channel[sensor];
    if (channel->state == RANGE_CAPTURE_IDLE) return;

    if (time_reached(timestamp_us, channel->deadline_us)) {
        ++channel->timeout_count;
        clear_active(array, channel);
        return;
    }

    if (echo_high) {
        if (channel->state == RANGE_CAPTURE_WAIT_RISE) {
            channel->rise_us = timestamp_us;
            channel->deadline_us = timestamp_us + RANGE_ECHO_TIMEOUT_US;
            channel->state = RANGE_CAPTURE_WAIT_FALL;
        }
        return;
    }

    if (channel->state != RANGE_CAPTURE_WAIT_FALL) return;

    pulse_us = (uint32_t)(timestamp_us - channel->rise_us);
    if (pulse_us < RANGE_ECHO_MIN_US || pulse_us > RANGE_ECHO_MAX_US) {
        ++channel->invalid_pulse_count;
        clear_active(array, channel);
        return;
    }

    /* Speed of sound ~= 343 m/s at 20 C; echo is a round trip.
     * mm = pulse_us * 343 / 2000. Round to nearest mm and cap to model range. */
    distance_mm = (pulse_us * 343U + 1000U) / 2000U;
    if (distance_mm > RANGE_MAX_DISTANCE_MM)
        distance_mm = RANGE_MAX_DISTANCE_MM;
    if (distance_mm == 0U) {
        ++channel->invalid_pulse_count;
        clear_active(array, channel);
        return;
    }

    channel->distance_mm = (uint16_t)distance_mm;
    channel->last_sample_ms = now_ms;
    channel->has_sample = true;
    ++channel->valid_sample_count;
    clear_active(array, channel);
}

static void copy_reading(const RangeChannel *channel, uint32_t now_ms,
                         uint32_t max_age_ms, RangeReading *reading)
{
    uint32_t age_ms;
    reading->valid = false;
    reading->distance_mm = 0U;
    reading->sample_ms = 0U;
    if (channel == NULL || reading == NULL || !channel->has_sample) return;

    age_ms = (uint32_t)(now_ms - channel->last_sample_ms);
    reading->distance_mm = channel->distance_mm;
    reading->sample_ms = channel->last_sample_ms;
    reading->valid = age_ms <= max_age_ms;
}

void range_array_snapshot(const RangeArray *array, uint32_t now_ms,
                          uint32_t front_max_age_ms,
                          uint32_t side_max_age_ms,
                          RangeSnapshot *snapshot)
{
    if (snapshot == NULL) return;
    memset(snapshot, 0, sizeof(*snapshot));
    if (array == NULL) return;

    copy_reading(&array->channel[RANGE_FRONT], now_ms,
                 front_max_age_ms, &snapshot->front);
    copy_reading(&array->channel[RANGE_LEFT], now_ms,
                 side_max_age_ms, &snapshot->left);
    copy_reading(&array->channel[RANGE_RIGHT], now_ms,
                 side_max_age_ms, &snapshot->right);
}
