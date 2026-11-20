#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ai_navigation.h"

static AiSensorFrame clear_frame(void)
{
    AiSensorFrame frame;
    memset(&frame, 0, sizeof(frame));
    frame.autonomy_enabled = true;
    frame.front_valid = true;
    frame.left_valid = true;
    frame.right_valid = true;
    frame.front_mm = 1500U;
    frame.left_mm = 1000U;
    frame.right_mm = 900U;
    frame.front_sample_ms = 100U;
    frame.left_sample_ms = 100U;
    frame.right_sample_ms = 100U;
    frame.yaw_rate_dps = 0.0f;
    return frame;
}

static void test_model_returns_valid_probabilities(void)
{
    const float features[5] = {0.75f, 0.50f, 0.45f, 0.0f, 0.0f};
    float probabilities[5];
    float sum = 0.0f;
    uint32_t i;
    assert(ai_model_predict(features, probabilities));
    for (i = 0U; i < 5U; ++i) {
        assert(isfinite(probabilities[i]));
        assert(probabilities[i] >= 0.0f && probabilities[i] <= 1.0f);
        sum += probabilities[i];
    }
    assert(fabsf(sum - 1.0f) < 0.001f);
    assert(!ai_model_predict(NULL, probabilities));
}

static void test_open_path_selects_forward(void)
{
    AiNavigationState state;
    AiSensorFrame frame = clear_frame();
    AiNavigationCommand command;
    ai_navigation_init(&state);
    command = ai_navigation_step(&state, &frame, 100U);
    assert(command.action == AI_ACTION_FORWARD);
    assert(command.confidence > 0.80f);
    assert(!command.safety_override);
}

static void test_blocked_front_selects_clear_left(void)
{
    AiNavigationState state;
    AiSensorFrame frame = clear_frame();
    AiNavigationCommand command;
    frame.front_mm = 200U;
    frame.left_mm = 1200U;
    frame.right_mm = 250U;
    ai_navigation_init(&state);
    command = ai_navigation_step(&state, &frame, 100U);
    assert(command.action == AI_ACTION_TURN_LEFT);
    assert(command.confidence > 0.80f);
}

static void test_blocked_front_selects_clear_right(void)
{
    AiNavigationState state;
    AiSensorFrame frame = clear_frame();
    AiNavigationCommand command;
    frame.front_mm = 200U;
    frame.left_mm = 250U;
    frame.right_mm = 1300U;
    ai_navigation_init(&state);
    command = ai_navigation_step(&state, &frame, 100U);
    assert(command.action == AI_ACTION_TURN_RIGHT);
    assert(command.confidence > 0.80f);
}

static void test_no_clear_side_selects_stop(void)
{
    AiNavigationState state;
    AiSensorFrame frame = clear_frame();
    AiNavigationCommand command;
    frame.front_mm = 150U;
    frame.left_mm = 200U;
    frame.right_mm = 250U;
    ai_navigation_init(&state);
    command = ai_navigation_step(&state, &frame, 100U);
    assert(command.action == AI_ACTION_STOP);
}

static void test_stale_or_missing_front_forces_stop(void)
{
    AiNavigationState state;
    AiSensorFrame frame = clear_frame();
    AiNavigationCommand command;
    ai_navigation_init(&state);
    command = ai_navigation_step(&state, &frame, 351U);
    assert(command.action == AI_ACTION_STOP);
    assert(command.safety_override);

    frame.front_sample_ms = 400U;
    frame.front_valid = false;
    command = ai_navigation_step(&state, &frame, 400U);
    assert(command.action == AI_ACTION_STOP);
    assert(command.safety_override);
}

static void test_missing_side_cannot_authorize_turn(void)
{
    AiNavigationState state;
    AiSensorFrame frame = clear_frame();
    AiNavigationCommand command;
    frame.front_mm = 200U;
    frame.left_mm = 1200U;
    frame.left_valid = false;
    frame.right_mm = 200U;
    ai_navigation_init(&state);
    command = ai_navigation_step(&state, &frame, 100U);
    assert(command.action != AI_ACTION_TURN_LEFT);
}

static void test_closing_rate_feature_is_calculated(void)
{
    AiNavigationState state;
    AiSensorFrame frame = clear_frame();
    AiNavigationCommand command;
    ai_navigation_init(&state);
    (void)ai_navigation_step(&state, &frame, 100U);
    frame.front_mm = 1000U;
    frame.front_sample_ms = 600U;
    frame.left_sample_ms = 600U;
    frame.right_sample_ms = 600U;
    command = ai_navigation_step(&state, &frame, 600U);
    assert(fabsf(command.features[3] - 0.5f) < 0.001f);
}

static void test_disabled_autonomy_stops(void)
{
    AiNavigationState state;
    AiSensorFrame frame = clear_frame();
    AiNavigationCommand command;
    frame.autonomy_enabled = false;
    ai_navigation_init(&state);
    command = ai_navigation_step(&state, &frame, 100U);
    assert(command.action == AI_ACTION_STOP);
    assert(!command.safety_override);
}

int main(void)
{
    test_model_returns_valid_probabilities();
    test_open_path_selects_forward();
    test_blocked_front_selects_clear_left();
    test_blocked_front_selects_clear_right();
    test_no_clear_side_selects_stop();
    test_stale_or_missing_front_forces_stop();
    test_missing_side_cannot_authorize_turn();
    test_closing_rate_feature_is_calculated();
    test_disabled_autonomy_stops();
    puts("PASS: GyroGuard TinyML inference and safety-shield tests");
    return 0;
}
