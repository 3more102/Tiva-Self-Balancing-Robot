#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "range_capture.h"

static void test_valid_echo_and_distance(void)
{
    RangeCapture capture;
    uint32_t width_us = 0U;
    uint32_t distance_mm = 0U;
    bool valid = false;

    assert(range_capture_init(&capture, 25000U, 117U, 17500U));
    assert(range_capture_begin(&capture, 1000U));
    range_capture_echo_edge(&capture, true, 1100U);
    range_capture_echo_edge(&capture, false, 2000U);

    assert(range_capture_poll(&capture, 2000U, &width_us, &valid));
    assert(valid && width_us == 900U);
    assert(range_capture_distance_mm(width_us, &distance_mm));
    assert(distance_mm == 154U);
    assert(!range_capture_poll(&capture, 2001U, &width_us, &valid));
}

static void test_unmatched_falling_edge_is_ignored(void)
{
    RangeCapture capture;
    uint32_t width_us = 0U;
    bool valid = false;

    assert(range_capture_init(&capture, 25000U, 117U, 17500U));
    assert(range_capture_begin(&capture, 0U));
    range_capture_echo_edge(&capture, false, 50U);
    assert(!range_capture_poll(&capture, 50U, &width_us, &valid));

    range_capture_echo_edge(&capture, true, 100U);
    range_capture_echo_edge(&capture, false, 500U);
    assert(range_capture_poll(&capture, 500U, &width_us, &valid));
    assert(valid && width_us == 400U);
}

static void test_duplicate_rising_edge_does_not_restart_pulse(void)
{
    RangeCapture capture;
    uint32_t width_us = 0U;
    bool valid = false;

    assert(range_capture_init(&capture, 25000U, 117U, 17500U));
    assert(range_capture_begin(&capture, 0U));
    range_capture_echo_edge(&capture, true, 100U);
    range_capture_echo_edge(&capture, true, 200U);
    range_capture_echo_edge(&capture, false, 500U);
    assert(range_capture_poll(&capture, 500U, &width_us, &valid));
    assert(valid && width_us == 400U);
}

static void assert_pulse_validity(uint32_t pulse_width_us,
                                  bool expected_valid)
{
    RangeCapture capture;
    uint32_t width_us = 0U;
    bool valid = false;

    assert(range_capture_init(&capture, 25000U, 117U, 17500U));
    assert(range_capture_begin(&capture, 100U));
    range_capture_echo_edge(&capture, true, 200U);
    range_capture_echo_edge(&capture, false, 200U + pulse_width_us);
    assert(range_capture_poll(&capture, 200U + pulse_width_us,
                              &width_us, &valid));
    assert(valid == expected_valid);
    assert(width_us == (expected_valid ? pulse_width_us : 0U));
}

static void test_pulse_width_boundaries(void)
{
    assert_pulse_validity(117U, true);
    assert_pulse_validity(17500U, true);
    assert_pulse_validity(116U, false);
    assert_pulse_validity(17501U, false);
}

static void test_no_echo_times_out(void)
{
    RangeCapture capture;
    uint32_t width_us = 99U;
    bool valid = true;

    assert(range_capture_init(&capture, 25000U, 117U, 17500U));
    assert(range_capture_begin(&capture, 1000U));
    assert(!range_capture_poll(&capture, 25999U, &width_us, &valid));
    assert(range_capture_poll(&capture, 26000U, &width_us, &valid));
    assert(!valid && width_us == 0U);
}

static void test_stuck_high_times_out(void)
{
    RangeCapture capture;
    uint32_t width_us = 99U;
    bool valid = true;

    assert(range_capture_init(&capture, 25000U, 117U, 17500U));
    assert(range_capture_begin(&capture, 0U));
    range_capture_echo_edge(&capture, true, 100U);
    assert(range_capture_poll(&capture, 25000U, &width_us, &valid));
    assert(!valid && width_us == 0U);
}

static void test_late_edge_cannot_resurrect_timed_out_capture(void)
{
    RangeCapture capture;
    uint32_t width_us = 99U;
    bool valid = true;

    assert(range_capture_init(&capture, 25000U, 117U, 17500U));
    assert(range_capture_begin(&capture, 0U));
    range_capture_echo_edge(&capture, true, 25000U);
    range_capture_echo_edge(&capture, false, 26000U);
    assert(range_capture_poll(&capture, 26000U, &width_us, &valid));
    assert(!valid && width_us == 0U);
}

static void test_busy_capture_and_result_consumption(void)
{
    RangeCapture capture;
    uint32_t width_us = 0U;
    bool valid = false;

    assert(range_capture_init(&capture, 25000U, 117U, 17500U));
    assert(range_capture_begin(&capture, 0U));
    assert(!range_capture_begin(&capture, 1U));

    range_capture_echo_edge(&capture, true, 100U);
    range_capture_echo_edge(&capture, false, 300U);
    assert(!range_capture_begin(&capture, 301U));
    assert(range_capture_poll(&capture, 300U, &width_us, &valid));
    assert(valid && width_us == 200U);
    assert(range_capture_begin(&capture, 400U));
}

static void test_timer_wraparound(void)
{
    RangeCapture capture;
    const uint32_t start_us = UINT32_MAX - 50U;
    uint32_t width_us = 0U;
    bool valid = false;

    assert(range_capture_init(&capture, 500U, 100U, 200U));
    assert(range_capture_begin(&capture, start_us));
    range_capture_echo_edge(&capture, true, UINT32_MAX - 10U);
    range_capture_echo_edge(&capture, false, 100U);
    assert(range_capture_poll(&capture, 100U, &width_us, &valid));
    assert(valid && width_us == 111U);
}

static void test_timeout_across_timer_wraparound(void)
{
    RangeCapture capture;
    uint32_t width_us = 99U;
    bool valid = true;

    assert(range_capture_init(&capture, 25U, 10U, 20U));
    assert(range_capture_begin(&capture, UINT32_MAX - 10U));
    assert(range_capture_poll(&capture, 13U, &width_us, &valid) == false);
    assert(range_capture_poll(&capture, 14U, &width_us, &valid));
    assert(!valid && width_us == 0U);
}

static void test_three_sensor_instances_are_independent(void)
{
    RangeCapture sensors[3];
    uint32_t widths[3] = {0U, 0U, 0U};
    bool valid[3] = {false, false, false};
    const uint32_t pulse[3] = {300U, 600U, 900U};
    unsigned i;

    for (i = 0U; i < 3U; ++i) {
        assert(range_capture_init(&sensors[i], 25000U, 117U, 17500U));
        assert(range_capture_begin(&sensors[i], 1000U + i * 100U));
        range_capture_echo_edge(&sensors[i], true, 1100U + i * 100U);
        range_capture_echo_edge(&sensors[i], false,
                                1100U + i * 100U + pulse[i]);
    }

    for (i = 0U; i < 3U; ++i) {
        assert(range_capture_poll(&sensors[i], 3000U,
                                  &widths[i], &valid[i]));
        assert(valid[i] && widths[i] == pulse[i]);
    }
}

static void test_invalid_configuration_and_null_arguments(void)
{
    RangeCapture capture;
    uint32_t width_us = 0U;
    bool valid = false;

    assert(!range_capture_init(&capture, 0U, 1U, 2U));
    assert(!range_capture_init(&capture, 100U, 20U, 10U));
    assert(!range_capture_init(&capture, 100U, 10U, 100U));
    assert(!range_capture_init(&capture, 0x80000000U, 1U, 2U));
    assert(!range_capture_init(NULL, 100U, 1U, 2U));
    assert(!range_capture_begin(NULL, 0U));
    range_capture_echo_edge(NULL, true, 0U);
    assert(!range_capture_poll(NULL, 0U, &width_us, &valid));
    assert(!range_capture_poll(&capture, 0U, NULL, &valid));
    assert(!range_capture_poll(&capture, 0U, &width_us, NULL));
    assert(!range_capture_distance_mm(100U, NULL));
    assert(!range_capture_distance_mm(0U, &width_us));
}

static void test_distance_rounding(void)
{
    uint32_t distance_mm = 0U;

    assert(range_capture_distance_mm(1000U, &distance_mm));
    assert(distance_mm == 172U);
    assert(range_capture_distance_mm(5800U, &distance_mm));
    assert(distance_mm == 995U);
}

int main(void)
{
    test_valid_echo_and_distance();
    test_unmatched_falling_edge_is_ignored();
    test_duplicate_rising_edge_does_not_restart_pulse();
    test_pulse_width_boundaries();
    test_no_echo_times_out();
    test_stuck_high_times_out();
    test_late_edge_cannot_resurrect_timed_out_capture();
    test_busy_capture_and_result_consumption();
    test_timer_wraparound();
    test_timeout_across_timer_wraparound();
    test_three_sensor_instances_are_independent();
    test_invalid_configuration_and_null_arguments();
    test_distance_rounding();
    puts("PASS: ultrasonic range capture timing, timeout, wrap and three-instance tests");
    return 0;
}
