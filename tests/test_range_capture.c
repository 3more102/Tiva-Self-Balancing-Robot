#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "range_capture.h"

#define TIMER_HZ 80000000U
#define HC_TIMEOUT_TICKS 2000000U
#define HC_MIN_PULSE_TICKS 9360U
#define HC_MAX_PULSE_TICKS 1400000U

static void test_valid_echo_and_distance(void)
{
    RangeCapture capture;
    uint32_t width_ticks = 0U;
    uint32_t distance_mm = 0U;
    bool valid = false;

    assert(range_capture_init(&capture, HC_TIMEOUT_TICKS,
                              HC_MIN_PULSE_TICKS, HC_MAX_PULSE_TICKS));
    assert(range_capture_begin(&capture, 1000U));
    range_capture_echo_edge(&capture, true, 1100U);
    range_capture_echo_edge(&capture, false, 73100U);

    assert(range_capture_poll(&capture, 73100U, &width_ticks, &valid));
    assert(valid && width_ticks == 72000U);
    assert(range_capture_distance_mm(width_ticks, TIMER_HZ, &distance_mm));
    assert(distance_mm == 154U);
    assert(!range_capture_poll(&capture, 73101U, &width_ticks, &valid));
}

static void test_unmatched_falling_edge_is_ignored(void)
{
    RangeCapture capture;
    uint32_t width_ticks = 0U;
    bool valid = false;

    assert(range_capture_init(&capture, HC_TIMEOUT_TICKS,
                              HC_MIN_PULSE_TICKS, HC_MAX_PULSE_TICKS));
    assert(range_capture_begin(&capture, 0U));
    range_capture_echo_edge(&capture, false, 4000U);
    assert(!range_capture_poll(&capture, 4000U, &width_ticks, &valid));

    range_capture_echo_edge(&capture, true, 8000U);
    range_capture_echo_edge(&capture, false, 20000U);
    assert(range_capture_poll(&capture, 20000U, &width_ticks, &valid));
    assert(valid && width_ticks == 12000U);
}

static void test_duplicate_rising_edge_does_not_restart_pulse(void)
{
    RangeCapture capture;
    uint32_t width_ticks = 0U;
    bool valid = false;

    assert(range_capture_init(&capture, HC_TIMEOUT_TICKS,
                              HC_MIN_PULSE_TICKS, HC_MAX_PULSE_TICKS));
    assert(range_capture_begin(&capture, 0U));
    range_capture_echo_edge(&capture, true, 8000U);
    range_capture_echo_edge(&capture, true, 9000U);
    range_capture_echo_edge(&capture, false, 20000U);
    assert(range_capture_poll(&capture, 20000U, &width_ticks, &valid));
    assert(valid && width_ticks == 12000U);
}

static void assert_pulse_validity(uint32_t pulse_width_ticks,
                                  bool expected_valid)
{
    RangeCapture capture;
    uint32_t width_ticks = 0U;
    bool valid = false;

    assert(range_capture_init(&capture, HC_TIMEOUT_TICKS,
                              HC_MIN_PULSE_TICKS, HC_MAX_PULSE_TICKS));
    assert(range_capture_begin(&capture, 100U));
    range_capture_echo_edge(&capture, true, 200U);
    range_capture_echo_edge(&capture, false, 200U + pulse_width_ticks);
    assert(range_capture_poll(&capture, 200U + pulse_width_ticks,
                              &width_ticks, &valid));
    assert(valid == expected_valid);
    assert(width_ticks == (expected_valid ? pulse_width_ticks : 0U));
}

static void test_pulse_width_boundaries(void)
{
    assert_pulse_validity(HC_MIN_PULSE_TICKS, true);
    assert_pulse_validity(HC_MAX_PULSE_TICKS, true);
    assert_pulse_validity(HC_MIN_PULSE_TICKS - 1U, false);
    assert_pulse_validity(HC_MAX_PULSE_TICKS + 1U, false);
}

static void test_no_echo_times_out(void)
{
    RangeCapture capture;
    uint32_t width_ticks = 99U;
    bool valid = true;

    assert(range_capture_init(&capture, HC_TIMEOUT_TICKS,
                              HC_MIN_PULSE_TICKS, HC_MAX_PULSE_TICKS));
    assert(range_capture_begin(&capture, 1000U));
    assert(!range_capture_poll(&capture,
                               1000U + HC_TIMEOUT_TICKS - 1U,
                               &width_ticks, &valid));
    assert(range_capture_poll(&capture, 1000U + HC_TIMEOUT_TICKS,
                              &width_ticks, &valid));
    assert(!valid && width_ticks == 0U);
}

static void test_stuck_high_times_out(void)
{
    RangeCapture capture;
    uint32_t width_ticks = 99U;
    bool valid = true;

    assert(range_capture_init(&capture, HC_TIMEOUT_TICKS,
                              HC_MIN_PULSE_TICKS, HC_MAX_PULSE_TICKS));
    assert(range_capture_begin(&capture, 0U));
    range_capture_echo_edge(&capture, true, 8000U);
    assert(range_capture_poll(&capture, HC_TIMEOUT_TICKS,
                              &width_ticks, &valid));
    assert(!valid && width_ticks == 0U);
}

static void test_late_edge_cannot_resurrect_timed_out_capture(void)
{
    RangeCapture capture;
    uint32_t width_ticks = 99U;
    bool valid = true;

    assert(range_capture_init(&capture, HC_TIMEOUT_TICKS,
                              HC_MIN_PULSE_TICKS, HC_MAX_PULSE_TICKS));
    assert(range_capture_begin(&capture, 0U));
    range_capture_echo_edge(&capture, true, HC_TIMEOUT_TICKS);
    range_capture_echo_edge(&capture, false, HC_TIMEOUT_TICKS + 1000U);
    assert(range_capture_poll(&capture, HC_TIMEOUT_TICKS + 1000U,
                              &width_ticks, &valid));
    assert(!valid && width_ticks == 0U);
}

static void test_busy_capture_and_result_consumption(void)
{
    RangeCapture capture;
    uint32_t width_ticks = 0U;
    bool valid = false;

    assert(range_capture_init(&capture, HC_TIMEOUT_TICKS,
                              HC_MIN_PULSE_TICKS, HC_MAX_PULSE_TICKS));
    assert(range_capture_begin(&capture, 0U));
    assert(!range_capture_begin(&capture, 1U));

    range_capture_echo_edge(&capture, true, 8000U);
    range_capture_echo_edge(&capture, false, 20000U);
    assert(!range_capture_begin(&capture, 20001U));
    assert(range_capture_poll(&capture, 20000U, &width_ticks, &valid));
    assert(valid && width_ticks == 12000U);
    assert(range_capture_begin(&capture, 30000U));
}

static void test_timer_wraparound(void)
{
    RangeCapture capture;
    const uint32_t start_ticks = UINT32_MAX - 50U;
    uint32_t width_ticks = 0U;
    bool valid = false;

    assert(range_capture_init(&capture, 500U, 100U, 200U));
    assert(range_capture_begin(&capture, start_ticks));
    range_capture_echo_edge(&capture, true, UINT32_MAX - 10U);
    range_capture_echo_edge(&capture, false, 100U);
    assert(range_capture_poll(&capture, 100U, &width_ticks, &valid));
    assert(valid && width_ticks == 111U);
}

static void test_timeout_across_timer_wraparound(void)
{
    RangeCapture capture;
    uint32_t width_ticks = 99U;
    bool valid = true;

    assert(range_capture_init(&capture, 25U, 10U, 20U));
    assert(range_capture_begin(&capture, UINT32_MAX - 10U));
    assert(!range_capture_poll(&capture, 13U, &width_ticks, &valid));
    assert(range_capture_poll(&capture, 14U, &width_ticks, &valid));
    assert(!valid && width_ticks == 0U);
}

static void test_three_sensor_instances_are_independent(void)
{
    RangeCapture sensors[3];
    uint32_t widths[3] = {0U, 0U, 0U};
    bool valid[3] = {false, false, false};
    const uint32_t pulse[3] = {24000U, 48000U, 72000U};
    unsigned i;

    for (i = 0U; i < 3U; ++i) {
        assert(range_capture_init(&sensors[i], HC_TIMEOUT_TICKS,
                                  HC_MIN_PULSE_TICKS, HC_MAX_PULSE_TICKS));
        assert(range_capture_begin(&sensors[i], 1000U + i * 100U));
        range_capture_echo_edge(&sensors[i], true, 1100U + i * 100U);
        range_capture_echo_edge(&sensors[i], false,
                                1100U + i * 100U + pulse[i]);
    }

    for (i = 0U; i < 3U; ++i) {
        assert(range_capture_poll(&sensors[i], 80000U,
                                  &widths[i], &valid[i]));
        assert(valid[i] && widths[i] == pulse[i]);
    }
}

static void test_invalid_configuration_and_null_arguments(void)
{
    RangeCapture capture;
    uint32_t width_ticks = 0U;
    bool valid = false;

    assert(!range_capture_init(&capture, 0U, 1U, 2U));
    assert(!range_capture_init(&capture, 100U, 20U, 10U));
    assert(!range_capture_init(&capture, 100U, 10U, 100U));
    assert(!range_capture_init(&capture, 0x80000000U, 1U, 2U));
    assert(!range_capture_init(NULL, 100U, 1U, 2U));
    assert(!range_capture_begin(NULL, 0U));
    range_capture_echo_edge(NULL, true, 0U);
    assert(!range_capture_poll(NULL, 0U, &width_ticks, &valid));
    assert(!range_capture_poll(&capture, 0U, NULL, &valid));
    assert(!range_capture_poll(&capture, 0U, &width_ticks, NULL));
    assert(!range_capture_distance_mm(100U, TIMER_HZ, NULL));
    assert(!range_capture_distance_mm(0U, TIMER_HZ, &width_ticks));
    assert(!range_capture_distance_mm(100U, 0U, &width_ticks));
}

static void test_distance_rounding(void)
{
    uint32_t distance_mm = 0U;

    assert(range_capture_distance_mm(80000U, TIMER_HZ, &distance_mm));
    assert(distance_mm == 172U);
    assert(range_capture_distance_mm(464000U, TIMER_HZ, &distance_mm));
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
