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

static void test_stale_side_ranges_cannot_authorize_turn(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;
    uint32_t now = 10U;
    navigation_init(&nav);
    (void)feed_clearance(&nav, &input, &now);

    input.front_mm = 200U;
    input.front_sample_ms = 200U;
    input.left_mm = 1200U;
    input.right_mm = 1000U;
    input.left_sample_ms = 0U;
    input.right_sample_ms = 0U;
    (void)navigation_update(&nav, &input, 200U);

    input.front_sample_ms = 1000U;
    command = navigation_update(&nav, &input, 1000U);
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


static void test_sample_timestamps_survive_uint32_wrap(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;

    navigation_init(&nav);
    input.front_sample_ms = UINT32_MAX - 15U;
    command = navigation_update(&nav, &input, UINT32_MAX - 15U);
    assert(command.mode == NAV_HOLD);

    input.front_sample_ms = 16U;
    command = navigation_update(&nav, &input, 16U);
    assert(command.mode == NAV_HOLD);

    input.front_sample_ms = 76U;
    command = navigation_update(&nav, &input, 76U);
    assert(command.mode == NAV_CRUISE);
    assert(command.travel_permitted);
}

static void test_stale_side_samples_cannot_authorize_turn(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;
    uint32_t now = 10U;

    navigation_init(&nav);
    (void)feed_clearance(&nav, &input, &now);

    input.front_mm = 200U;
    input.front_sample_ms = 200U;
    input.left_mm = 900U;
    input.right_mm = 1100U;
    input.left_sample_ms = 200U;
    input.right_sample_ms = 200U;
    (void)navigation_update(&nav, &input, 200U);

    input.front_sample_ms = 320U;
    input.left_sample_ms = 320U - (NAV_SIDE_MAX_AGE_MS + 1U);
    input.right_sample_ms = 320U - (NAV_SIDE_MAX_AGE_MS + 1U);
    command = navigation_update(&nav, &input, 320U);
    assert(command.mode == NAV_HOLD);
    assert(!command.travel_permitted);
    assert(command.yaw_bias == 0.0f);
}

static void test_stale_front_sample_aborts_timed_turn(void)
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
    assert(command.travel_permitted);

    command = navigation_update(&nav, &input,
                                320U + NAV_FRONT_MAX_AGE_MS + 1U);
    assert(command.mode == NAV_HOLD);
    assert(!command.travel_permitted);
    assert(command.yaw_bias == 0.0f);
}

static void test_stale_selected_side_aborts_turn(void)
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
    assert(command.yaw_bias > 0.0f);

    /* Front remains fresh, but the selected right-side reading is stale. */
    input.front_sample_ms = 600U;
    input.left_sample_ms = 600U;
    command = navigation_update(&nav, &input, 600U);
    assert(command.mode == NAV_HOLD);
    assert(!command.travel_permitted);
    assert(command.pitch_target_deg == 0.0f && command.yaw_bias == 0.0f);
}

static void test_turn_stops_early_when_front_clears(void)
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
    assert(command.mode == NAV_TURN_RIGHT && command.travel_permitted);

    /* Stop the open-loop turn as soon as a fresh front reading is clear. */
    input.front_mm = 600U;
    input.front_sample_ms = 400U;
    input.left_sample_ms = 400U;
    input.right_sample_ms = 400U;
    command = navigation_update(&nav, &input, 400U);
    assert(command.mode == NAV_HOLD && !command.travel_permitted);
    assert(command.yaw_bias == 0.0f);

    input.front_sample_ms = 460U;
    input.left_sample_ms = 460U;
    input.right_sample_ms = 460U;
    command = navigation_update(&nav, &input, 460U);
    assert(command.mode == NAV_HOLD);
    input.front_sample_ms = 520U;
    input.left_sample_ms = 520U;
    input.right_sample_ms = 520U;
    command = navigation_update(&nav, &input, 520U);
    assert(command.mode == NAV_HOLD);
    input.front_sample_ms = 580U;
    input.left_sample_ms = 580U;
    input.right_sample_ms = 580U;
    command = navigation_update(&nav, &input, 580U);
    assert(command.mode == NAV_CRUISE && command.travel_permitted);
}

static void test_out_of_order_front_timestamps_do_not_confirm_clearance(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;
    navigation_init(&nav);

    input.front_sample_ms = 100U;
    command = navigation_update(&nav, &input, 100U);
    assert(command.mode == NAV_HOLD);

    /* This timestamp is older than the last accepted sample. */
    input.front_sample_ms = 90U;
    command = navigation_update(&nav, &input, 110U);
    assert(command.mode == NAV_HOLD);

    input.front_sample_ms = 120U;
    command = navigation_update(&nav, &input, 120U);
    assert(command.mode == NAV_HOLD);

    input.front_sample_ms = 180U;
    command = navigation_update(&nav, &input, 180U);
    assert(command.mode == NAV_CRUISE && command.travel_permitted);
}

static void test_out_of_range_readings_cannot_authorize_motion(void)
{
    Navigation nav;
    NavigationInput input = sample_input();
    NavigationCommand command;

    navigation_init(&nav);
    input.front_mm = UINT16_MAX; /* Common timeout/sentinel value. */
    input.front_sample_ms = 10U;
    command = navigation_update(&nav, &input, 10U);
    assert(command.mode == NAV_HOLD);
    assert(!command.travel_permitted);
    assert(command.pitch_target_deg == 0.0f);

    input.front_mm = 200U;
    input.front_sample_ms = 200U;
    input.left_mm = UINT16_MAX;
    input.right_mm = UINT16_MAX;
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

int main(void)
{
    test_disabled_and_invalid_fail_safe();
    test_cruise_requires_three_fresh_clear_samples();
    test_stale_front_stops_cruise();
    test_obstacle_turns_toward_more_clear_side_then_reverify();
    test_no_verified_side_means_no_turn();
    test_stale_side_ranges_cannot_authorize_turn();
    test_autonomy_disable_cancels_motion();
    test_out_of_range_readings_cannot_authorize_motion();
    test_sample_timestamps_survive_uint32_wrap();
    test_stale_side_samples_cannot_authorize_turn();
    test_stale_front_sample_aborts_timed_turn();
    test_stale_selected_side_aborts_turn();
    test_turn_stops_early_when_front_clears();
    test_out_of_order_front_timestamps_do_not_confirm_clearance();
    puts("PASS: obstacle navigation fail-safe, freshness, turn and re-verification tests");
    return 0;
}
