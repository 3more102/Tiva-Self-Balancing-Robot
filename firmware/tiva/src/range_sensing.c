#include "range_sensing.h"

#include <stddef.h>
#include <string.h>

static bool time_reached(uint32_t now, uint32_t deadline)
{
    /* Valid for deadlines less than 2^31 ticks into the future. */
    return (uint32_t)(now - deadline) < 0x80000000U;
}

static uint32_t echo_timeout_ticks(const RangeArray *array)
{
    return RANGE_ECHO_TIMEOUT_US * array->ticks_per_us;
}

static void clear_active(RangeArray *array, RangeChannel *channel)
{
    channel->state = RANGE_CAPTURE_IDLE;
    channel->rise_ticks = 0U;
    channel->deadline_ticks = 0U;
    array->active_sensor = RANGE_SENSOR_NONE;
}

void range_array_init(RangeArray *array, uint32_t ticks_per_us)
{
    uint32_t i;
    if (array == NULL) return;
    memset(array, 0, sizeof(*array));
    /* Invalid scales disable capture; never silently assume 1 MHz. */
    array->ticks_per_us =
        (ticks_per_us == 0U ||
         ticks_per_us > (0x7FFFFFFFU / RANGE_ECHO_TIMEOUT_US)) ?
        0U : ticks_per_us;
    for (i = 0U; i < RANGE_SENSOR_COUNT; ++i)
        array->channel[i].state = RANGE_CAPTURE_IDLE;
    array->next_sensor = (uint8_t)RANGE_FRONT;
    array->active_sensor = RANGE_SENSOR_NONE;
}

void range_array_poll(RangeArray *array, uint32_t now_ticks)
{
    RangeChannel *channel;
    if (array == NULL || array->active_sensor >= RANGE_SENSOR_COUNT) return;

    channel = &array->channel[array->active_sensor];
    if (channel->state != RANGE_CAPTURE_IDLE &&
        time_reached(now_ticks, channel->deadline_ticks)) {
        ++channel->timeout_count;
        clear_active(array, channel);
    }
}

uint8_t range_array_next_trigger(RangeArray *array,
                                 uint32_t now_ms,
                                 uint32_t now_ticks)
{
    uint8_t sensor;
    RangeChannel *channel;

    if (array == NULL || array->ticks_per_us == 0U)
        return RANGE_SENSOR_NONE;
    range_array_poll(array, now_ticks);

    if (array->active_sensor != RANGE_SENSOR_NONE) return RANGE_SENSOR_NONE;
    if (array->has_triggered &&
        (uint32_t)(now_ms - array->last_trigger_ms) <
            RANGE_INTER_PING_GAP_MS) {
        return RANGE_SENSOR_NONE;
    }

    sensor = array->next_sensor;
    channel = &array->channel[sensor];
    channel->state = RANGE_CAPTURE_WAIT_RISE;
    channel->rise_ticks = 0U;
    channel->deadline_ticks = now_ticks + echo_timeout_ticks(array);
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
                           bool echo_high, uint32_t timestamp_ticks,
                           uint32_t now_ms)
{
    RangeChannel *channel;
    uint32_t pulse_ticks;
    uint32_t pulse_us;
    uint32_t distance_mm;
    uint32_t timeout_ticks;

    if (array == NULL || sensor >= RANGE_SENSOR_COUNT ||
        array->active_sensor != sensor) return;

    channel = &array->channel[sensor];
    if (channel->state == RANGE_CAPTURE_IDLE) return;

    timeout_ticks = echo_timeout_ticks(array);
    if (time_reached(timestamp_ticks, channel->deadline_ticks)) {
        ++channel->timeout_count;
        clear_active(array, channel);
        return;
    }

    if (echo_high) {
        if (channel->state == RANGE_CAPTURE_WAIT_RISE) {
            channel->rise_ticks = timestamp_ticks;
            channel->deadline_ticks = timestamp_ticks + timeout_ticks;
            channel->state = RANGE_CAPTURE_WAIT_FALL;
        }
        return;
    }

    if (channel->state != RANGE_CAPTURE_WAIT_FALL) return;

    pulse_ticks = (uint32_t)(timestamp_ticks - channel->rise_ticks);
    if (pulse_ticks < RANGE_ECHO_MIN_US * array->ticks_per_us ||
        pulse_ticks > RANGE_ECHO_MAX_US * array->ticks_per_us) {
        ++channel->invalid_pulse_count;
        clear_active(array, channel);
        return;
    }

    pulse_us = (pulse_ticks + (array->ticks_per_us / 2U)) /
               array->ticks_per_us;

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
    if (reading == NULL) return;
    reading->valid = false;
    reading->distance_mm = 0U;
    reading->sample_ms = 0U;
    if (channel == NULL || !channel->has_sample) return;

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
