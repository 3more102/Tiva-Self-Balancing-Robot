#include "ai_navigation.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool sample_fresh(bool valid, uint16_t distance_mm,
                         uint32_t sample_ms, uint32_t now_ms,
                         uint32_t max_age_ms)
{
    return valid && distance_mm != 0U &&
           (uint32_t)(now_ms - sample_ms) <= max_age_ms;
}

static float clampf_local(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static AiNavigationCommand safe_stop(bool override)
{
    AiNavigationCommand command;
    memset(&command, 0, sizeof(command));
    command.action = AI_ACTION_STOP;
    command.safety_override = override;
    return command;
}

void ai_navigation_init(AiNavigationState *state)
{
    if (state == NULL) return;
    state->has_previous_front = false;
    state->previous_front_mm = 0U;
    state->previous_front_sample_ms = 0U;
}

AiNavigationCommand ai_navigation_step(AiNavigationState *state,
                                       const AiSensorFrame *frame,
                                       uint32_t now_ms)
{
    AiNavigationCommand command;
    float closing_mps = 0.0f;
    float features[5];
    float max_probability = -1.0f;
    uint32_t best_action = (uint32_t)AI_ACTION_STOP;
    uint32_t i;
    bool front_fresh, left_fresh, right_fresh;

    if (state == NULL || frame == NULL) return safe_stop(true);
    if (!frame->inference_enabled) return safe_stop(false);
    if (!isfinite(frame->yaw_rate_dps)) return safe_stop(true);

    front_fresh = sample_fresh(frame->front_valid, frame->front_mm,
                               frame->front_sample_ms, now_ms,
                               AI_FRONT_MAX_AGE_MS);
    left_fresh = sample_fresh(frame->left_valid, frame->left_mm,
                              frame->left_sample_ms, now_ms,
                              AI_SIDE_MAX_AGE_MS);
    right_fresh = sample_fresh(frame->right_valid, frame->right_mm,
                               frame->right_sample_ms, now_ms,
                               AI_SIDE_MAX_AGE_MS);

    /* A missing or stale forward measurement makes all motion unsafe. */
    if (!front_fresh) return safe_stop(true);

    if (state->has_previous_front &&
        frame->front_sample_ms != state->previous_front_sample_ms) {
        const uint32_t elapsed_ms =
            (uint32_t)(frame->front_sample_ms - state->previous_front_sample_ms);
        if (elapsed_ms >= 20U && elapsed_ms <= 1000U) {
            const float elapsed_s = (float)elapsed_ms / 1000.0f;
            closing_mps =
                ((float)state->previous_front_mm - (float)frame->front_mm) /
                (1000.0f * elapsed_s);
        }
    }
    state->has_previous_front = true;
    state->previous_front_mm = frame->front_mm;
    state->previous_front_sample_ms = frame->front_sample_ms;

    features[0] = clampf_local((float)frame->front_mm /
                               AI_MAX_DISTANCE_MM, 0.0f, 1.0f);
    features[1] = left_fresh ?
        clampf_local((float)frame->left_mm / AI_MAX_DISTANCE_MM, 0.0f, 1.0f) :
        0.0f;
    features[2] = right_fresh ?
        clampf_local((float)frame->right_mm / AI_MAX_DISTANCE_MM, 0.0f, 1.0f) :
        0.0f;
    features[3] = clampf_local(closing_mps / AI_MAX_CLOSING_MPS, -1.0f, 1.0f);
    features[4] = clampf_local(frame->yaw_rate_dps / AI_MAX_YAW_DPS,
                               -1.0f, 1.0f);

    if (!ai_model_predict(features, command.probabilities))
        return safe_stop(true);

    for (i = 0U; i < 5U; ++i) {
        if (command.probabilities[i] > max_probability) {
            max_probability = command.probabilities[i];
            best_action = i;
        }
        command.features[i] = features[i];
    }
    command.confidence = max_probability;
    command.safety_override = false;
    command.action = (AiAction)best_action;

    if (max_probability < AI_MIN_CONFIDENCE)
        command.action = AI_ACTION_REASSESS;

    /* Independent rule-based safety shield: the network cannot authorize
       forward motion into a close obstacle or turn toward an unverified side. */
    if (command.action == AI_ACTION_FORWARD &&
        frame->front_mm < AI_FRONT_CLEAR_MM) {
        command.action = AI_ACTION_STOP;
        command.safety_override = true;
    } else if (command.action == AI_ACTION_TURN_LEFT &&
               (!left_fresh || frame->left_mm < AI_SIDE_CLEAR_MM)) {
        command.action = AI_ACTION_STOP;
        command.safety_override = true;
    } else if (command.action == AI_ACTION_TURN_RIGHT &&
               (!right_fresh || frame->right_mm < AI_SIDE_CLEAR_MM)) {
        command.action = AI_ACTION_STOP;
        command.safety_override = true;
    }

    return command;
}
