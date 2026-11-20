#include "range_scanner.h"

#include <stddef.h>

#define RANGE_SCANNER_MAX_INTERVAL_TICKS 0x7FFFFFFFU
#define RANGE_SCANNER_MAX_DISTANCE_MM UINT16_MAX

static bool sensor_id_valid(RangeSensorId sensor)
{
    return (unsigned int)sensor < RANGE_SCANNER_SENSOR_COUNT;
}

bool range_scanner_init(RangeScanner *scanner,
                        uint32_t timeout_ticks,
                        uint32_t min_pulse_ticks,
                        uint32_t max_pulse_ticks,
                        uint32_t inter_ping_ticks,
                        uint32_t timer_hz,
                        uint32_t max_distance_mm)
{
    unsigned int i;

    if (scanner == NULL || inter_ping_ticks == 0U ||
        inter_ping_ticks > RANGE_SCANNER_MAX_INTERVAL_TICKS ||
        timer_hz == 0U || max_distance_mm == 0U ||
        max_distance_mm > RANGE_SCANNER_MAX_DISTANCE_MM) {
        return false;
    }

    for (i = 0U; i < RANGE_SCANNER_SENSOR_COUNT; ++i) {
        if (!range_capture_init(&scanner->capture[i], timeout_ticks,
                                min_pulse_ticks, max_pulse_ticks)) {
            return false;
        }
        scanner->samples[i].distance_mm = 0U;
        scanner->samples[i].sample_ms = 0U;
        scanner->samples[i].valid = false;
        scanner->samples[i].has_sample = false;
    }

    scanner->inter_ping_ticks = inter_ping_ticks;
    scanner->max_distance_mm = max_distance_mm;
    scanner->last_trigger_ticks = 0U;
    scanner->next_sensor = (uint8_t)RANGE_SENSOR_FRONT;
    scanner->active_sensor = (uint8_t)RANGE_SENSOR_FRONT;
    scanner->has_triggered = false;
    scanner->active = false;
    return true;
}

bool range_scanner_next_trigger(const RangeScanner *scanner,
                                uint32_t now_ticks,
                                RangeSensorId *sensor)
{
    if (scanner == NULL || sensor == NULL || scanner->active) return false;

    if (scanner->has_triggered &&
        (uint32_t)(now_ticks - scanner->last_trigger_ticks) <
            scanner->inter_ping_ticks) {
        return false;
    }

    *sensor = (RangeSensorId)scanner->next_sensor;
    return true;
}

bool range_scanner_begin(RangeScanner *scanner, RangeSensorId sensor,
                         uint32_t now_ticks)
{
    if (scanner == NULL || !sensor_id_valid(sensor) || scanner->active ||
        (uint8_t)sensor != scanner->next_sensor) {
        return false;
    }

    if (scanner->has_triggered &&
        (uint32_t)(now_ticks - scanner->last_trigger_ticks) <
            scanner->inter_ping_ticks) {
        return false;
    }

    if (!range_capture_begin(&scanner->capture[(unsigned int)sensor],
                             now_ticks)) {
        return false;
    }

    scanner->active_sensor = (uint8_t)sensor;
    scanner->active = true;
    scanner->last_trigger_ticks = now_ticks;
    scanner->has_triggered = true;
    scanner->next_sensor = (uint8_t)(((unsigned int)sensor + 1U) %
                                     RANGE_SCANNER_SENSOR_COUNT);
    return true;
}

void range_scanner_echo_edge(RangeScanner *scanner, RangeSensorId sensor,
                             bool echo_high, uint32_t now_ticks)
{
    if (scanner == NULL || !sensor_id_valid(sensor) || !scanner->active ||
        (uint8_t)sensor != scanner->active_sensor) {
        return;
    }

    range_capture_echo_edge(&scanner->capture[(unsigned int)sensor],
                            echo_high, now_ticks);
}

bool range_scanner_poll(RangeScanner *scanner, uint32_t now_ticks,
                        uint32_t now_ms, RangeSensorId *completed_sensor)
{
    uint32_t pulse_width_ticks = 0U;
    uint32_t distance_mm = 0U;
    bool pulse_valid = false;
    bool distance_valid = false;
    unsigned int sensor_index;

    if (scanner == NULL || completed_sensor == NULL || !scanner->active) {
        return false;
    }

    sensor_index = scanner->active_sensor;
    if (!range_capture_poll(&scanner->capture[sensor_index], now_ticks,
                            &pulse_width_ticks, &pulse_valid)) {
        return false;
    }

    if (pulse_valid) {
        distance_valid = range_capture_distance_mm(
            pulse_width_ticks, scanner->timer_hz, &distance_mm);
    }

    if (distance_valid && distance_mm <= scanner->max_distance_mm &&
        distance_mm <= RANGE_SCANNER_MAX_DISTANCE_MM) {
        scanner->samples[sensor_index].distance_mm = (uint16_t)distance_mm;
        scanner->samples[sensor_index].valid = true;
    } else {
        scanner->samples[sensor_index].distance_mm = 0U;
        scanner->samples[sensor_index].valid = false;
    }

    scanner->samples[sensor_index].sample_ms = now_ms;
    scanner->samples[sensor_index].has_sample = true;
    scanner->active = false;
    *completed_sensor = (RangeSensorId)sensor_index;
    return true;
}

bool range_scanner_get_sample(const RangeScanner *scanner,
                              RangeSensorId sensor, RangeSample *sample)
{
    if (scanner == NULL || sample == NULL || !sensor_id_valid(sensor)) {
        return false;
    }

    *sample = scanner->samples[(unsigned int)sensor];
    return true;
}
