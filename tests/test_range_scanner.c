#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "range_scanner.h"

#define TIMER_HZ 80000000U
#define ECHO_TIMEOUT_TICKS 2000000U
#define ECHO_MIN_TICKS 9360U
#define ECHO_MAX_TICKS 1400000U
#define INTER_PING_TICKS 4800000U

static void init_scanner(RangeScanner *scanner)
{
    assert(range_scanner_init(scanner, ECHO_TIMEOUT_TICKS, ECHO_MIN_TICKS,
                              ECHO_MAX_TICKS, INTER_PING_TICKS,
                              TIMER_HZ, 4000U));
}

static void complete_valid_echo(RangeScanner *scanner, RangeSensorId sensor,
                                uint32_t trigger_ticks, uint32_t pulse_ticks,
                                uint32_t sample_ms)
{
    RangeSensorId completed = RANGE_SENSOR_FRONT;

    assert(range_scanner_begin(scanner, sensor, trigger_ticks));
    range_scanner_echo_edge(scanner, sensor, true, trigger_ticks + 100U);
    range_scanner_echo_edge(scanner, sensor, false,
                            trigger_ticks + 100U + pulse_ticks);
    assert(range_scanner_poll(scanner,
                              trigger_ticks + 100U + pulse_ticks,
                              sample_ms, &completed));
    assert(completed == sensor);
}

static void test_initial_round_robin_and_valid_sample(void)
{
    RangeScanner scanner;
    RangeSensorId next = RANGE_SENSOR_RIGHT;
    RangeSample sample;

    init_scanner(&scanner);
    assert(range_scanner_next_trigger(&scanner, 0U, &next));
    assert(next == RANGE_SENSOR_FRONT);
    complete_valid_echo(&scanner, RANGE_SENSOR_FRONT, 0U, 72000U, 10U);

    assert(range_scanner_get_sample(&scanner, RANGE_SENSOR_FRONT, &sample));
    assert(sample.has_sample && sample.valid);
    assert(sample.distance_mm == 154U);
    assert(sample.sample_ms == 10U);

    assert(!range_scanner_next_trigger(&scanner,
                                       INTER_PING_TICKS - 1U, &next));
    assert(range_scanner_next_trigger(&scanner, INTER_PING_TICKS, &next));
    assert(next == RANGE_SENSOR_LEFT);
}

static void test_round_robin_three_sensors(void)
{
    RangeScanner scanner;
    RangeSensorId next;
    RangeSample sample;

    init_scanner(&scanner);

    assert(range_scanner_next_trigger(&scanner, 0U, &next));
    assert(next == RANGE_SENSOR_FRONT);
    complete_valid_echo(&scanner, next, 0U, 24000U, 10U);

    assert(range_scanner_next_trigger(&scanner, INTER_PING_TICKS, &next));
    assert(next == RANGE_SENSOR_LEFT);
    complete_valid_echo(&scanner, next, INTER_PING_TICKS, 48000U, 70U);

    assert(range_scanner_next_trigger(&scanner, 2U * INTER_PING_TICKS, &next));
    assert(next == RANGE_SENSOR_RIGHT);
    complete_valid_echo(&scanner, next, 2U * INTER_PING_TICKS,
                        72000U, 130U);

    assert(range_scanner_get_sample(&scanner, RANGE_SENSOR_FRONT, &sample));
    assert(sample.valid && sample.distance_mm == 51U);
    assert(range_scanner_get_sample(&scanner, RANGE_SENSOR_LEFT, &sample));
    assert(sample.valid && sample.distance_mm == 103U);
    assert(range_scanner_get_sample(&scanner, RANGE_SENSOR_RIGHT, &sample));
    assert(sample.valid && sample.distance_mm == 154U);

    assert(range_scanner_next_trigger(&scanner, 3U * INTER_PING_TICKS, &next));
    assert(next == RANGE_SENSOR_FRONT);
}

static void test_sensor_cannot_be_retriggered_while_active(void)
{
    RangeScanner scanner;
    RangeSensorId next;
    RangeSensorId completed;

    init_scanner(&scanner);
    assert(range_scanner_next_trigger(&scanner, 0U, &next));
    assert(range_scanner_begin(&scanner, next, 0U));
    assert(!range_scanner_next_trigger(&scanner, INTER_PING_TICKS, &next));
    assert(!range_scanner_begin(&scanner, RANGE_SENSOR_LEFT,
                                INTER_PING_TICKS));
    assert(!range_scanner_poll(&scanner, 100U, 1U, &completed));
}

static void test_wrong_sensor_echo_is_ignored(void)
{
    RangeScanner scanner;
    RangeSensorId completed;
    RangeSample sample;

    init_scanner(&scanner);
    assert(range_scanner_begin(&scanner, RANGE_SENSOR_FRONT, 0U));
    range_scanner_echo_edge(&scanner, RANGE_SENSOR_LEFT, true, 100U);
    range_scanner_echo_edge(&scanner, RANGE_SENSOR_LEFT, false, 20000U);
    assert(!range_scanner_poll(&scanner, 20000U, 1U, &completed));

    range_scanner_echo_edge(&scanner, RANGE_SENSOR_FRONT, true, 30000U);
    range_scanner_echo_edge(&scanner, RANGE_SENSOR_FRONT, false, 42000U);
    assert(range_scanner_poll(&scanner, 42000U, 2U, &completed));
    assert(completed == RANGE_SENSOR_FRONT);
    assert(range_scanner_get_sample(&scanner, RANGE_SENSOR_FRONT, &sample));
    assert(sample.valid && sample.distance_mm == 26U);
}

static void test_timeout_publishes_invalid_sample(void)
{
    RangeScanner scanner;
    RangeSensorId completed;
    RangeSample sample;

    init_scanner(&scanner);
    assert(range_scanner_begin(&scanner, RANGE_SENSOR_FRONT, 1000U));
    assert(!range_scanner_poll(&scanner,
                               1000U + ECHO_TIMEOUT_TICKS - 1U,
                               20U, &completed));
    assert(range_scanner_poll(&scanner, 1000U + ECHO_TIMEOUT_TICKS,
                              21U, &completed));
    assert(completed == RANGE_SENSOR_FRONT);
    assert(range_scanner_get_sample(&scanner, RANGE_SENSOR_FRONT, &sample));
    assert(sample.has_sample && !sample.valid);
    assert(sample.distance_mm == 0U && sample.sample_ms == 21U);
}

static void test_out_of_range_distance_is_invalid(void)
{
    RangeScanner scanner;
    RangeSensorId completed;
    RangeSample sample;

    assert(range_scanner_init(&scanner, ECHO_TIMEOUT_TICKS, ECHO_MIN_TICKS,
                              ECHO_MAX_TICKS, INTER_PING_TICKS,
                              TIMER_HZ, 100U));
    assert(range_scanner_begin(&scanner, RANGE_SENSOR_FRONT, 0U));
    range_scanner_echo_edge(&scanner, RANGE_SENSOR_FRONT, true, 100U);
    range_scanner_echo_edge(&scanner, RANGE_SENSOR_FRONT, false, 100U + 72000U);
    assert(range_scanner_poll(&scanner, 72100U, 5U, &completed));
    assert(range_scanner_get_sample(&scanner, RANGE_SENSOR_FRONT, &sample));
    assert(sample.has_sample && !sample.valid && sample.distance_mm == 0U);
}

static void test_invalid_pulse_is_published_invalid(void)
{
    RangeScanner scanner;
    RangeSensorId completed;
    RangeSample sample;

    init_scanner(&scanner);
    assert(range_scanner_begin(&scanner, RANGE_SENSOR_FRONT, 0U));
    range_scanner_echo_edge(&scanner, RANGE_SENSOR_FRONT, true, 100U);
    range_scanner_echo_edge(&scanner, RANGE_SENSOR_FRONT, false,
                            100U + ECHO_MIN_TICKS - 1U);
    assert(range_scanner_poll(&scanner, 100U + ECHO_MIN_TICKS,
                              8U, &completed));
    assert(range_scanner_get_sample(&scanner, RANGE_SENSOR_FRONT, &sample));
    assert(sample.has_sample && !sample.valid);
}

static void test_inter_ping_interval_wraparound(void)
{
    RangeScanner scanner;
    RangeSensorId next;
    const uint32_t trigger = UINT32_MAX - 100U;

    init_scanner(&scanner);
    assert(range_scanner_next_trigger(&scanner, trigger, &next));
    assert(next == RANGE_SENSOR_FRONT);
    assert(range_scanner_begin(&scanner, next, trigger));
    range_scanner_echo_edge(&scanner, next, true, trigger + 100U);
    range_scanner_echo_edge(&scanner, next, false, trigger + 12100U);
    {
        RangeSensorId completed;
        assert(range_scanner_poll(&scanner, trigger + 12100U, 1U,
                                  &completed));
    }
    assert(!range_scanner_next_trigger(&scanner,
                                       trigger + INTER_PING_TICKS - 1U,
                                       &next));
    assert(range_scanner_next_trigger(&scanner,
                                      trigger + INTER_PING_TICKS, &next));
    assert(next == RANGE_SENSOR_LEFT);
}

static void test_wrong_order_and_invalid_arguments(void)
{
    RangeScanner scanner;
    RangeSensorId next;
    RangeSample sample;

    init_scanner(&scanner);
    assert(!range_scanner_begin(&scanner, RANGE_SENSOR_LEFT, 0U));
    assert(!range_scanner_next_trigger(NULL, 0U, &next));
    assert(!range_scanner_next_trigger(&scanner, 0U, NULL));
    assert(!range_scanner_begin(NULL, RANGE_SENSOR_FRONT, 0U));
    range_scanner_echo_edge(NULL, RANGE_SENSOR_FRONT, true, 0U);
    range_scanner_echo_edge(&scanner, (RangeSensorId)9, true, 0U);
    assert(!range_scanner_poll(NULL, 0U, 0U, &next));
    assert(!range_scanner_poll(&scanner, 0U, 0U, NULL));
    assert(!range_scanner_get_sample(NULL, RANGE_SENSOR_FRONT, &sample));
    assert(!range_scanner_get_sample(&scanner, (RangeSensorId)9, &sample));
    assert(!range_scanner_get_sample(&scanner, RANGE_SENSOR_FRONT, NULL));

    assert(!range_scanner_init(NULL, ECHO_TIMEOUT_TICKS, ECHO_MIN_TICKS,
                               ECHO_MAX_TICKS, INTER_PING_TICKS,
                               TIMER_HZ, 4000U));
    assert(!range_scanner_init(&scanner, ECHO_TIMEOUT_TICKS, ECHO_MIN_TICKS,
                               ECHO_MAX_TICKS, 0U, TIMER_HZ, 4000U));
    assert(!range_scanner_init(&scanner, ECHO_TIMEOUT_TICKS, ECHO_MIN_TICKS,
                               ECHO_MAX_TICKS, INTER_PING_TICKS,
                               0U, 4000U));
    assert(!range_scanner_init(&scanner, ECHO_TIMEOUT_TICKS, ECHO_MIN_TICKS,
                               ECHO_MAX_TICKS, INTER_PING_TICKS,
                               TIMER_HZ, 0U));
    assert(!range_scanner_init(&scanner, ECHO_TIMEOUT_TICKS, ECHO_MIN_TICKS,
                               ECHO_MAX_TICKS, INTER_PING_TICKS,
                               TIMER_HZ, 65536U));
}

int main(void)
{
    test_initial_round_robin_and_valid_sample();
    test_round_robin_three_sensors();
    test_sensor_cannot_be_retriggered_while_active();
    test_wrong_sensor_echo_is_ignored();
    test_timeout_publishes_invalid_sample();
    test_out_of_range_distance_is_invalid();
    test_invalid_pulse_is_published_invalid();
    test_inter_ping_interval_wraparound();
    test_wrong_order_and_invalid_arguments();
    puts("PASS: three-sensor sequential range scanner and freshness publication");
    return 0;
}
