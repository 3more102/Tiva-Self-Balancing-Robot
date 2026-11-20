#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "range_sensing.h"

static void complete_echo(RangeArray *array, uint8_t sensor,
                          uint32_t rise_us, uint32_t pulse_us,
                          uint32_t sample_ms)
{
    range_array_echo_edge(array, sensor, true, rise_us, sample_ms);
    range_array_echo_edge(array, sensor, false,
                          rise_us + pulse_us, sample_ms);
}

static void test_front_measurement_and_conversion(void)
{
    RangeArray array;
    RangeSnapshot snapshot;
    uint8_t sensor;
    range_array_init(&array, 1U);
    sensor = range_array_next_trigger(&array, 100U, 100000U);
    assert(sensor == RANGE_FRONT);
    complete_echo(&array, sensor, 100100U, 2000U, 102U);
    range_array_snapshot(&array, 120U, 250U, 750U, &snapshot);
    assert(snapshot.front.valid);
    assert(snapshot.front.distance_mm == 343U);
    assert(snapshot.front.sample_ms == 102U);
    assert(!snapshot.left.valid && !snapshot.right.valid);
    assert(array.channel[RANGE_FRONT].valid_sample_count == 1U);
}

static void test_three_sensors_are_sequential_with_quiet_gap(void)
{
    RangeArray array;
    uint8_t sensor;
    range_array_init(&array, 1U);

    sensor = range_array_next_trigger(&array, 0U, 0U);
    assert(sensor == RANGE_FRONT);
    complete_echo(&array, sensor, 100U, 1000U, 1U);
    assert(range_array_next_trigger(&array, 59U, 59000U) == RANGE_SENSOR_NONE);

    sensor = range_array_next_trigger(&array, 60U, 60000U);
    assert(sensor == RANGE_LEFT);
    complete_echo(&array, sensor, 60100U, 1000U, 61U);

    sensor = range_array_next_trigger(&array, 120U, 120000U);
    assert(sensor == RANGE_RIGHT);
    complete_echo(&array, sensor, 120100U, 1000U, 121U);

    sensor = range_array_next_trigger(&array, 180U, 180000U);
    assert(sensor == RANGE_FRONT);
    assert(array.trigger_count == 4U);
}

static void test_timeout_when_no_echo_arrives(void)
{
    RangeArray array;
    range_array_init(&array, 1U);
    assert(range_array_next_trigger(&array, 10U, 10000U) == RANGE_FRONT);
    range_array_poll(&array, 22999U);
    assert(array.active_sensor == RANGE_FRONT);
    range_array_poll(&array, 23000U);
    assert(array.active_sensor == RANGE_SENSOR_NONE);
    assert(array.channel[RANGE_FRONT].timeout_count == 1U);
    assert(range_array_next_trigger(&array, 70U, 70000U) == RANGE_LEFT);
}

static void test_stuck_high_echo_times_out(void)
{
    RangeArray array;
    range_array_init(&array, 1U);
    assert(range_array_next_trigger(&array, 10U, 10000U) == RANGE_FRONT);
    range_array_echo_edge(&array, RANGE_FRONT, true, 10500U, 10U);
    range_array_poll(&array, 23500U);
    assert(array.active_sensor == RANGE_SENSOR_NONE);
    assert(array.channel[RANGE_FRONT].timeout_count == 1U);
}

static void test_invalid_pulse_does_not_replace_last_good_sample(void)
{
    RangeArray array;
    RangeSnapshot snapshot;
    range_array_init(&array, 1U);
    assert(range_array_next_trigger(&array, 0U, 0U) == RANGE_FRONT);
    complete_echo(&array, RANGE_FRONT, 100U, 2000U, 2U);

    assert(range_array_next_trigger(&array, 60U, 60000U) == RANGE_LEFT);
    complete_echo(&array, RANGE_LEFT, 60100U, 20U, 61U);
    assert(array.channel[RANGE_LEFT].invalid_pulse_count == 1U);
    range_array_snapshot(&array, 62U, 250U, 750U, &snapshot);
    assert(snapshot.front.valid && snapshot.front.distance_mm == 343U);
    assert(!snapshot.left.valid);
}

static void test_timer_rollover_and_snapshot_staleness(void)
{
    RangeArray array;
    RangeSnapshot snapshot;
    uint32_t rise = 0xFFFFFF00U;
    range_array_init(&array, 1U);
    assert(range_array_next_trigger(&array, 100U, rise) == RANGE_FRONT);
    complete_echo(&array, RANGE_FRONT, rise, 2000U, 101U);
    range_array_snapshot(&array, 352U, 250U, 750U, &snapshot);
    assert(!snapshot.front.valid);
    assert(snapshot.front.distance_mm == 343U);
    range_array_snapshot(&array, 200U, 250U, 750U, &snapshot);
    assert(snapshot.front.valid);
}


static void test_minimum_echo_and_model_range_cap(void)
{
    RangeArray array;
    RangeSnapshot snapshot;
    range_array_init(&array, 1U);
    assert(range_array_next_trigger(&array, 0U, 0U) == RANGE_FRONT);
    complete_echo(&array, RANGE_FRONT, 100U, RANGE_ECHO_MIN_US, 1U);
    range_array_snapshot(&array, 1U, 250U, 750U, &snapshot);
    assert(snapshot.front.valid);
    assert(snapshot.front.distance_mm >= 17U &&
           snapshot.front.distance_mm <= 18U);

    assert(range_array_next_trigger(&array, 60U, 60000U) == RANGE_LEFT);
    complete_echo(&array, RANGE_LEFT, 60100U, RANGE_ECHO_MAX_US, 61U);
    range_array_snapshot(&array, 61U, 250U, 750U, &snapshot);
    assert(snapshot.left.valid);
    assert(snapshot.left.distance_mm == RANGE_MAX_DISTANCE_MM);
}

static void test_failed_hardware_trigger_can_be_cancelled(void)
{
    RangeArray array;
    range_array_init(&array, 1U);
    assert(range_array_next_trigger(&array, 0U, 0U) == RANGE_FRONT);
    range_array_cancel_trigger(&array, RANGE_FRONT);
    assert(array.active_sensor == RANGE_SENSOR_NONE);
    assert(array.trigger_failure_count == 1U);
    assert(array.channel[RANGE_FRONT].state == RANGE_CAPTURE_IDLE);
}

int main(void)
{
    test_front_measurement_and_conversion();
    test_three_sensors_are_sequential_with_quiet_gap();
    test_timeout_when_no_echo_arrives();
    test_stuck_high_echo_times_out();
    test_invalid_pulse_does_not_replace_last_good_sample();
    test_timer_rollover_and_snapshot_staleness();
    test_minimum_echo_and_model_range_cap();
    test_failed_hardware_trigger_can_be_cancelled();
    puts("PASS: asynchronous range capture, timeout, scheduling and freshness tests");
    return 0;
}
