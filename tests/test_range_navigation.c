#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "range_navigation.h"

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

static void publish_valid(RangeScanner *scanner, RangeSensorId sensor,
                          uint32_t trigger_ticks, uint32_t pulse_ticks,
                          uint32_t sample_ms)
{
    RangeSensorId completed;

    assert(range_scanner_begin(scanner, sensor, trigger_ticks));
    range_scanner_echo_edge(scanner, sensor, true, trigger_ticks + 100U);
    range_scanner_echo_edge(scanner, sensor, false,
                            trigger_ticks + 100U + pulse_ticks);
    assert(range_scanner_poll(scanner,
                              trigger_ticks + 100U + pulse_ticks,
                              sample_ms, &completed));
    assert(completed == sensor);
}

static void publish_timeout(RangeScanner *scanner, RangeSensorId sensor,
                            uint32_t trigger_ticks, uint32_t sample_ms)
{
    RangeSensorId completed;

    assert(range_scanner_begin(scanner, sensor, trigger_ticks));
    assert(range_scanner_poll(scanner, trigger_ticks + ECHO_TIMEOUT_TICKS,
                              sample_ms, &completed));
    assert(completed == sensor);
}

static void test_missing_samples_are_invalid(void)
{
    RangeScanner scanner;
    NavigationInput input = {0};

    init_scanner(&scanner);
    assert(range_navigation_build_input(&scanner, true, &input));
    assert(input.autonomy_enabled);
    assert(!input.front_valid && input.front_mm == 0U);
    assert(!input.left_valid && input.left_mm == 0U);
    assert(!input.right_valid && input.right_mm == 0U);
    assert(input.front_sample_ms == 0U);
    assert(input.left_sample_ms == 0U);
    assert(input.right_sample_ms == 0U);
}

static void test_valid_samples_map_to_navigation_fields(void)
{
    RangeScanner scanner;
    NavigationInput input = {0};

    init_scanner(&scanner);
    publish_valid(&scanner, RANGE_SENSOR_FRONT, 0U, 72000U, 10U);
    publish_valid(&scanner, RANGE_SENSOR_LEFT, INTER_PING_TICKS,
                  48000U, 70U);
    publish_valid(&scanner, RANGE_SENSOR_RIGHT, 2U * INTER_PING_TICKS,
                  96000U, 130U);

    assert(range_navigation_build_input(&scanner, true, &input));
    assert(input.autonomy_enabled);
    assert(input.front_valid && input.front_mm == 154U);
    assert(input.front_sample_ms == 10U);
    assert(input.left_valid && input.left_mm == 103U);
    assert(input.left_sample_ms == 70U);
    assert(input.right_valid && input.right_mm == 206U);
    assert(input.right_sample_ms == 130U);
}

static void test_timeout_is_not_mapped_as_clear_space(void)
{
    RangeScanner scanner;
    NavigationInput input = {0};

    init_scanner(&scanner);
    publish_valid(&scanner, RANGE_SENSOR_FRONT, 0U, 72000U, 10U);
    publish_timeout(&scanner, RANGE_SENSOR_LEFT, INTER_PING_TICKS, 100U);
    publish_valid(&scanner, RANGE_SENSOR_RIGHT, 2U * INTER_PING_TICKS,
                  48000U, 190U);

    assert(range_navigation_build_input(&scanner, true, &input));
    assert(input.front_valid && input.front_mm == 154U);
    assert(!input.left_valid && input.left_mm == 0U);
    assert(input.left_sample_ms == 100U);
    assert(input.right_valid && input.right_mm == 103U);
}

static void test_autonomy_enable_is_separate_from_sensor_validity(void)
{
    RangeScanner scanner;
    NavigationInput input = {0};

    init_scanner(&scanner);
    publish_valid(&scanner, RANGE_SENSOR_FRONT, 0U, 72000U, 10U);

    assert(range_navigation_build_input(&scanner, false, &input));
    assert(!input.autonomy_enabled);
    assert(input.front_valid && input.front_mm == 154U);
    assert(!input.left_valid && !input.right_valid);
}

static void test_navigation_policy_receives_fresh_front_sample(void)
{
    RangeScanner scanner;
    NavigationInput input = {0};
    Navigation navigation;
    NavigationCommand command;

    init_scanner(&scanner);
    publish_valid(&scanner, RANGE_SENSOR_FRONT, 0U, 280000U, 10U);
    assert(range_navigation_build_input(&scanner, true, &input));

    navigation_init(&navigation);
    command = navigation_update(&navigation, &input, 10U);
    assert(command.mode == NAV_HOLD);
    assert(!command.travel_permitted);
    assert(command.pitch_target_deg == 0.0f);
    assert(command.yaw_bias == 0.0f);
}

static void test_null_arguments(void)
{
    RangeScanner scanner;
    NavigationInput input = {0};

    init_scanner(&scanner);
    assert(!range_navigation_build_input(NULL, true, &input));
    assert(!range_navigation_build_input(&scanner, true, NULL));
}

int main(void)
{
    test_missing_samples_are_invalid();
    test_valid_samples_map_to_navigation_fields();
    test_timeout_is_not_mapped_as_clear_space();
    test_autonomy_enable_is_separate_from_sensor_validity();
    test_navigation_policy_receives_fresh_front_sample();
    test_null_arguments();
    puts("PASS: range scanner to navigation input adapter tests");
    return 0;
}
