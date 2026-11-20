#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "navigation.h"

static NavigationInput sample_input(void)
{
    NavigationInput input;
    memset(&input, 0, sizeof(input));
    input.autonomy_enabled = true;
    input.front_valid = true;
    input.left_valid = true;
    input.right_valid = true;
    input.front_mm = 1000U;
    input.left_mm = 900U;
    input.right_mm = 800U;
    return input;
}

static NavigationCommand feed_clearance(Navigation *nav,
                                        NavigationInput *input,
                                        uint32_t *time_ms)
{
    NavigationCommand command;
    input->front_mm = 1000U;
    input->front_valid = true;
    input->left_valid = true;
    input->right_valid = true;
    input->left_mm = 900U;
    input->right_mm = 800U;
    input->front_sample_ms = *time_ms;
    input->left_sample_ms = *time_ms;
    input->right_sample_ms = *time_ms;
    command = navigation_update(nav, input, *time_ms);
    *time_ms += 60U;
    input->front_sample_ms = *time_ms;
    input->left_sample_ms = *time_ms;
    input->right_sample_ms = *time_ms;
    command = navigation_update(nav, input, *time_ms);
    *time_ms += 60U;
    input->front_sample_ms = *time_ms;
    input->left_sample_ms = *time_ms;
    input->right_sample_ms = *time_ms;
    command = navigation_update(nav, input, *time_ms);
    return command;
}

static void test_disabled_and_invalid_fail_safe(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;
    navigation_init(&nav);
    input.autonomy_enabled = false;
    command = navigation_update(&nav, &input, 0U);
    assert(command.mode == NAV_DISABLED);
    assert(!command.travel_permitted);
    assert(command.pitch_target_deg == 0.0f && command.yaw_bias == 0.0f);

    input.autonomy_enabled = true;
    input.front_valid = false;
    command = navigation_update(&nav, &input, 10U);
    assert(command.mode == NAV_HOLD);
    assert(!command.travel_permitted);
    assert(command.pitch_target_deg == 0.0f);
}

static void test_cruise_requires_three_fresh_clear_samples(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;
    uint32_t now = 10U;
    navigation_init(&nav);
    command = feed_clearance(&nav, &input, &now);
    assert(command.mode == NAV_CRUISE);
    assert(command.travel_permitted);
    assert(fabsf(command.pitch_target_deg - NAV_CRUISE_LEAN_DEG) < 0.001f);
    assert(command.yaw_bias == 0.0f);
}

static void test_stale_front_stops_cruise(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;
    uint32_t now = 10U;
    navigation_init(&nav);
    (void)feed_clearance(&nav, &input, &now);
    command = navigation_update(&nav, &input, input.front_sample_ms +
                                NAV_FRONT_MAX_AGE_MS + 1U);
    assert(command.mode == NAV_HOLD);
    assert(!command.travel_permitted);
    assert(command.pitch_target_deg == 0.0f && command.yaw_bias == 0.0f);
}

static void test_obstacle_turns_toward_more_clear_side_then_reverify(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;
    uint32_t now = 10U;
    navigation_init(&nav);
    (void)feed_clearance(&nav, &input, &now);

    input.front_mm = 200U;
    input.front_sample_ms = 200U;
    input.left_mm = 700U;
    input.right_mm = 1100U;
    input.left_sample_ms = 200U;
    input.right_sample_ms = 200U;
    command = navigation_update(&nav, &input, 200U);
    assert(command.mode == NAV_HOLD);
    assert(!command.travel_permitted);

    input.front_sample_ms = 320U;
    input.left_sample_ms = 320U;
    input.right_sample_ms = 320U;
    command = navigation_update(&nav, &input, 320U);
    assert(command.mode == NAV_TURN_RIGHT);
    assert(command.travel_permitted);
    assert(command.pitch_target_deg == 0.0f);
    assert(command.yaw_bias > 0.0f);

    input.front_sample_ms = 500U;
    input.left_sample_ms = 500U;
    input.right_sample_ms = 500U;
    command = navigation_update(&nav, &input, 500U);
    assert(command.mode == NAV_TURN_RIGHT);
    assert(command.yaw_bias > 0.0f);

    input.front_sample_ms = 670U;
    input.left_sample_ms = 670U;
    input.right_sample_ms = 670U;
    command = navigation_update(&nav, &input, 670U);
    assert(command.mode == NAV_HOLD);
    assert(!command.travel_permitted);

    input.front_mm = 1000U;
    input.front_sample_ms = 700U;
    input.left_sample_ms = 700U;
    input.right_sample_ms = 700U;
    command = navigation_update(&nav, &input, 700U);
    assert(command.mode == NAV_HOLD);
    input.front_sample_ms = 760U;
    input.left_sample_ms = 760U;
    input.right_sample_ms = 760U;
    command = navigation_update(&nav, &input, 760U);
    assert(command.mode == NAV_HOLD);
    input.front_sample_ms = 820U;
    input.left_sample_ms = 820U;
    input.right_sample_ms = 820U;
    command = navigation_update(&nav, &input, 820U);
    assert(command.mode == NAV_CRUISE);
    assert(command.travel_permitted);
}

static void test_no_verified_side_means_no_turn(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;
    uint32_t now = 10U;
    navigation_init(&nav);
    (void)feed_clearance(&nav, &input, &now);
    input.front_mm = 200U;
    input.front_sample_ms = 200U;
    input.left_mm = 300U;
    input.right_mm = 450U;
    input.left_sample_ms = 200U;
    input.right_sample_ms = 200U;
    (void)navigation_update(&nav, &input, 200U);
    input.front_sample_ms = 320U;
    input.left_sample_ms = 320U;
    input.right_sample_ms = 320U;
    command = navigation_update(&nav, &input, 320U);
    assert(command.mode == NAV_HOLD);
    assert(!command.travel_permitted);
    assert(command.yaw_bias == 0.0f);
}

static void test_autonomy_disable_cancels_motion(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;
    uint32_t now = 10U;
    navigation_init(&nav);
    (void)feed_clearance(&nav, &input, &now);
    input.autonomy_enabled = false;
    command = navigation_update(&nav, &input, now);
    assert(command.mode == NAV_DISABLED);
    assert(!command.travel_permitted);
    assert(command.pitch_target_deg == 0.0f && command.yaw_bias == 0.0f);
}

static void test_retry_uses_opposite_clear_side_only(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;
    uint32_t now = 10U;
    navigation_init(&nav);
    (void)feed_clearance(&nav, &input, &now);

    input.front_mm = 200U;
    input.front_sample_ms = 200U;
    input.left_mm = 700U;
    input.right_mm = 1100U;
    input.left_sample_ms = 200U;
    input.right_sample_ms = 200U;
    (void)navigation_update(&nav, &input, 200U);
    input.front_sample_ms = 320U;
    input.left_sample_ms = 320U;
    input.right_sample_ms = 320U;
    command = navigation_update(&nav, &input, 320U);
    assert(command.mode == NAV_TURN_RIGHT);

    input.front_sample_ms = 670U;
    input.left_sample_ms = 670U;
    input.right_sample_ms = 670U;
    command = navigation_update(&nav, &input, 670U);
    assert(command.mode == NAV_HOLD);

    input.front_sample_ms = 800U;
    input.left_mm = 900U;
    input.right_mm = 300U;
    input.left_sample_ms = 800U;
    input.right_sample_ms = 800U;
    command = navigation_update(&nav, &input, 800U);
    assert(command.mode == NAV_TURN_LEFT);
    assert(command.yaw_bias < 0.0f);
}

static void test_retry_refuses_original_side_if_alternate_is_blocked(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;
    uint32_t now = 10U;
    navigation_init(&nav);
    (void)feed_clearance(&nav, &input, &now);

    input.front_mm = 200U;
    input.front_sample_ms = 200U;
    input.left_mm = 700U;
    input.right_mm = 1100U;
    input.left_sample_ms = 200U;
    input.right_sample_ms = 200U;
    (void)navigation_update(&nav, &input, 200U);
    input.front_sample_ms = 320U;
    input.left_sample_ms = 320U;
    input.right_sample_ms = 320U;
    command = navigation_update(&nav, &input, 320U);
    assert(command.mode == NAV_TURN_RIGHT);

    input.front_sample_ms = 670U;
    input.left_sample_ms = 670U;
    input.right_sample_ms = 670U;
    command = navigation_update(&nav, &input, 670U);
    assert(command.mode == NAV_HOLD);

    input.front_sample_ms = 800U;
    input.left_mm = 300U;
    input.right_mm = 1100U;
    input.left_sample_ms = 800U;
    input.right_sample_ms = 800U;
    command = navigation_update(&nav, &input, 800U);
    assert(command.mode == NAV_HOLD);
    assert(!command.travel_permitted);
    assert(command.yaw_bias == 0.0f);
}

static void test_stale_side_data_cannot_authorize_turn(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;
    uint32_t now = 10U;
    navigation_init(&nav);
    (void)feed_clearance(&nav, &input, &now);

    input.front_mm = 200U;
    input.front_sample_ms = 1000U;
    input.left_mm = 900U;
    input.right_mm = 1100U;
    input.left_sample_ms = 0U;
    input.right_sample_ms = 0U;
    command = navigation_update(&nav, &input, 1000U);
    assert(command.mode == NAV_HOLD);

    input.front_sample_ms = 1120U;
    command = navigation_update(&nav, &input, 1120U);
    assert(command.mode == NAV_HOLD);
    assert(!command.travel_permitted);
    assert(command.yaw_bias == 0.0f);
}

static void test_future_timestamp_is_rejected(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;
    navigation_init(&nav);
    input.front_sample_ms = 101U;
    command = navigation_update(&nav, &input, 100U);
    assert(command.mode == NAV_HOLD);
    assert(!command.travel_permitted);
}

static void test_timestamp_wraparound_and_lean_ramp(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;
    uint32_t now = UINT32_MAX - 100U;
    navigation_init(&nav);
    command = feed_clearance(&nav, &input, &now);
    assert(command.mode == NAV_CRUISE);

    input.front_mm = 425U;
    input.front_sample_ms = 200U;
    command = navigation_update(&nav, &input, 200U);
    assert(command.mode == NAV_CRUISE);
    assert(fabsf(command.pitch_target_deg - 0.75f) < 0.001f);
}

int main(void)
{
    test_disabled_and_invalid_fail_safe();
    test_cruise_requires_three_fresh_clear_samples();
    test_stale_front_stops_cruise();
    test_obstacle_turns_toward_more_clear_side_then_reverify();
    test_no_verified_side_means_no_turn();
    test_autonomy_disable_cancels_motion();
    test_retry_uses_opposite_clear_side_only();
    test_retry_refuses_original_side_if_alternate_is_blocked();
    test_stale_side_data_cannot_authorize_turn();
    test_future_timestamp_is_rejected();
    test_timestamp_wraparound_and_lean_ramp();
    puts("PASS: obstacle navigation fail-safe, freshness, turn and re-verification tests");
    return 0;
}
