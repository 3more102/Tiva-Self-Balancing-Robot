#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "obstacle_avoidance.h"
static void test_invalid_samples_do_not_steer(void)
{
    ObstacleAvoidance nav;
    obstacle_avoidance_init(&nav);
    assert(obstacle_avoidance_step(&nav, 0.005f, true, false, NAN) == 0.0f);
    assert(nav.mode == OBSTACLE_CLEAR);
    assert(nav.invalid_sample_count == 1U);
    assert(obstacle_avoidance_step(&nav, 0.005f, true, true, NAN) == 0.0f);
    assert(nav.mode == OBSTACLE_CLEAR);
    assert(nav.invalid_sample_count == 2U);
}
static void test_obstacle_turn_hysteresis_and_alternation(void)
{
    ObstacleAvoidance nav;
    float steering;
    obstacle_avoidance_init(&nav);
    steering = obstacle_avoidance_step(&nav, 0.005f, true, true, 80.0f);
    assert(steering == 0.0f);
    assert(nav.mode == OBSTACLE_CLEAR);
    steering = obstacle_avoidance_step(&nav, 0.005f, true, true, 20.0f);
    assert(fabsf(steering - 0.16f) < 0.0001f);
    assert(nav.mode == OBSTACLE_TURNING);
    assert(nav.turn_count == 1U);
    steering = obstacle_avoidance_step(&nav, 0.005f, true, false, NAN);
    assert(fabsf(steering - 0.16f) < 0.0001f);
    assert(nav.mode == OBSTACLE_TURNING);
    for (int i = 0; i < 129; ++i)
        steering = obstacle_avoidance_step(&nav, 0.005f, false, false, NAN);
    assert(nav.mode == OBSTACLE_WAIT_CLEAR);
    assert(steering == 0.0f);
    steering = obstacle_avoidance_step(&nav, 0.005f, true, true, 20.0f);
    assert(steering == 0.0f);
    assert(nav.mode == OBSTACLE_WAIT_CLEAR);
    assert(nav.turn_count == 1U);
    (void)obstacle_avoidance_step(&nav, 0.005f, true, true, 45.0f);
    assert(nav.mode == OBSTACLE_CLEAR);
    steering = obstacle_avoidance_step(&nav, 0.005f, true, true, 18.0f);
    assert(fabsf(steering + 0.16f) < 0.0001f);
    assert(nav.mode == OBSTACLE_TURNING);
    assert(nav.turn_count == 2U);
}
static void test_bad_timing_is_rejected(void)
{
    ObstacleAvoidance nav;
    obstacle_avoidance_init(&nav);
    (void)obstacle_avoidance_step(&nav, 0.005f, true, true, 15.0f);
    assert(nav.mode == OBSTACLE_TURNING);
    assert(obstacle_avoidance_step(&nav, 0.050f, false, false, NAN) == 0.0f);
    assert(nav.turn_elapsed_s < 0.02f);
}
int main(void)
{
    test_invalid_samples_do_not_steer();
    test_obstacle_turn_hysteresis_and_alternation();
    test_bad_timing_is_rejected();
    puts("PASS: obstacle range validation, timed steering and hysteresis");
    return 0;
}
