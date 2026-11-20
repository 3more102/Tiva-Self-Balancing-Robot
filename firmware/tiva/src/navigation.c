#include "navigation.h"

static bool range_fresh(bool valid, uint16_t distance_mm,
                        uint32_t sample_ms, uint32_t now_ms,
                        uint32_t max_age_ms)
{
    return valid && distance_mm != 0U && distance_mm <= NAV_MAX_RANGE_MM &&
           (uint32_t)(now_ms - sample_ms) <= max_age_ms;
}

/* Accept only forward-moving timestamps; signed subtraction is wrap-safe
 * provided successive samples are separated by less than 2^31 milliseconds. */
static bool timestamp_is_newer(uint32_t candidate, uint32_t previous)
{
    return (int32_t)(candidate - previous) > 0;
}

static NavigationCommand command_for(const Navigation *navigation,
                                     float pitch_target_deg, float yaw_bias)
{
    NavigationCommand command;
    command.mode = navigation->mode;
    command.pitch_target_deg = pitch_target_deg;
    command.yaw_bias = yaw_bias;
    command.travel_permitted = navigation->mode == NAV_CRUISE ||
                               navigation->mode == NAV_TURN_LEFT ||
                               navigation->mode == NAV_TURN_RIGHT;
    return command;
}

static NavigationCommand stopped(const Navigation *navigation)
{
    return command_for(navigation, 0.0f, 0.0f);
}

static NavigationCommand turning(const Navigation *navigation)
{
    /* Positive yaw bias makes the left wheel command exceed the right. */
    const float yaw = navigation->mode == NAV_TURN_RIGHT ?
                      NAV_TURN_BIAS : -NAV_TURN_BIAS;
    return command_for(navigation, 0.0f, yaw);
}

static float cruise_lean(uint16_t front_mm)
{
    float fraction;
    if (front_mm >= NAV_FRONT_CLEAR_MM) return NAV_CRUISE_LEAN_DEG;
    fraction = (float)(front_mm - NAV_FRONT_STOP_MM) /
               (float)(NAV_FRONT_CLEAR_MM - NAV_FRONT_STOP_MM);
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;
    return fraction * NAV_CRUISE_LEAN_DEG;
}

static bool start_turn(Navigation *navigation,
                       const NavigationInput *input,
                       bool left_fresh, bool right_fresh,
                       uint32_t now_ms)
{
    const bool left_clear = left_fresh && input->left_mm >= NAV_SIDE_CLEAR_MM;
    const bool right_clear = right_fresh && input->right_mm >= NAV_SIDE_CLEAR_MM;
    int8_t direction = 0;

    if (navigation->turn_attempts > 0U &&
        navigation->last_turn_direction != 0) {
        /* Retry only on the opposite side, and only while it is freshly clear. */
        direction = (int8_t)-navigation->last_turn_direction;
        if ((direction < 0 && !left_clear) ||
            (direction > 0 && !right_clear)) {
            return false;
        }
    } else {
        if (!left_clear && !right_clear) return false;
        if (left_clear && right_clear) {
            direction = input->right_mm >= input->left_mm ? 1 : -1;
        } else {
            direction = left_clear ? -1 : 1;
        }
    }

    navigation->mode = direction < 0 ? NAV_TURN_LEFT : NAV_TURN_RIGHT;
    navigation->last_turn_direction = direction;
    if (navigation->turn_attempts < UINT8_MAX) ++navigation->turn_attempts;
    navigation->state_since_ms = now_ms;
    navigation->clear_samples = 0U;
    return true;
}

void navigation_init(Navigation *navigation)
{
    if (navigation == 0) return;
    navigation->mode = NAV_DISABLED;
    navigation->state_since_ms = 0U;
    navigation->last_front_sample_ms = 0U;
    navigation->clear_samples = 0U;
    navigation->turn_attempts = 0U;
    navigation->last_turn_direction = 0;
    navigation->has_front_sample = false;
}

NavigationCommand navigation_update(Navigation *navigation,
                                    const NavigationInput *input,
                                    uint32_t now_ms)
{
    bool front_fresh, left_fresh, right_fresh, front_new;

    if (navigation == 0 || input == 0) {
        NavigationCommand safe = {NAV_DISABLED, 0.0f, 0.0f, false};
        return safe;
    }

    if (!input->autonomy_enabled) {
        navigation_init(navigation);
        return stopped(navigation);
    }

    if (navigation->mode == NAV_DISABLED) {
        navigation->mode = NAV_HOLD;
        navigation->state_since_ms = now_ms;
        navigation->clear_samples = 0U;
        navigation->turn_attempts = 0U;
        navigation->last_turn_direction = 0;
        navigation->has_front_sample = false;
    }

    front_fresh = range_fresh(input->front_valid, input->front_mm,
                              input->front_sample_ms, now_ms,
                              NAV_FRONT_MAX_AGE_MS);
    left_fresh = range_fresh(input->left_valid, input->left_mm,
                             input->left_sample_ms, now_ms,
                             NAV_SIDE_MAX_AGE_MS);
    right_fresh = range_fresh(input->right_valid, input->right_mm,
                              input->right_sample_ms, now_ms,
                              NAV_SIDE_MAX_AGE_MS);
    front_new = front_fresh &&
                (!navigation->has_front_sample ||
                 timestamp_is_newer(input->front_sample_ms,
                                    navigation->last_front_sample_ms));
    if (front_new) {
        navigation->last_front_sample_ms = input->front_sample_ms;
        navigation->has_front_sample = true;
    }

    switch (navigation->mode) {
    case NAV_CRUISE:
        if (!front_fresh) {
            navigation->mode = NAV_HOLD;
            navigation->state_since_ms = now_ms;
            navigation->clear_samples = 0U;
            return stopped(navigation);
        }
        if (input->front_mm <= NAV_FRONT_STOP_MM) {
            navigation->mode = NAV_HOLD;
            navigation->state_since_ms = now_ms;
            navigation->clear_samples = 0U;
            return stopped(navigation);
        }
        return command_for(navigation, cruise_lean(input->front_mm), 0.0f);

    case NAV_TURN_LEFT:
        if (!front_fresh || !left_fresh ||
            input->left_mm < NAV_SIDE_CLEAR_MM ||
            input->front_mm >= NAV_FRONT_CLEAR_MM ||
            (uint32_t)(now_ms - navigation->state_since_ms) >=
                NAV_TURN_DURATION_MS) {
            navigation->mode = NAV_HOLD;
            navigation->state_since_ms = now_ms;
            navigation->clear_samples = 0U;
            return stopped(navigation);
        }
        return turning(navigation);

    case NAV_TURN_RIGHT:
        if (!front_fresh || !right_fresh ||
            input->right_mm < NAV_SIDE_CLEAR_MM ||
            input->front_mm >= NAV_FRONT_CLEAR_MM ||
            (uint32_t)(now_ms - navigation->state_since_ms) >=
                NAV_TURN_DURATION_MS) {
            navigation->mode = NAV_HOLD;
            navigation->state_since_ms = now_ms;
            navigation->clear_samples = 0U;
            return stopped(navigation);
        }
        return turning(navigation);

    case NAV_HOLD:
        if (!front_fresh) {
            navigation->clear_samples = 0U;
            return stopped(navigation);
        }
        if (input->front_mm >= NAV_FRONT_CLEAR_MM) {
            if (front_new && navigation->clear_samples < UINT8_MAX)
                ++navigation->clear_samples;
            if (navigation->clear_samples >= NAV_CLEAR_CONFIRMATIONS) {
                navigation->mode = NAV_CRUISE;
                navigation->state_since_ms = now_ms;
                navigation->turn_attempts = 0U;
                navigation->last_turn_direction = 0;
                return command_for(navigation,
                                   cruise_lean(input->front_mm), 0.0f);
            }
            return stopped(navigation);
        }

        navigation->clear_samples = 0U;
        if (input->front_mm <= NAV_FRONT_STOP_MM && front_new &&
            navigation->turn_attempts < NAV_MAX_TURN_ATTEMPTS &&
            (uint32_t)(now_ms - navigation->state_since_ms) >=
                NAV_TURN_RETRY_MS &&
            start_turn(navigation, input, left_fresh, right_fresh, now_ms)) {
            return turning(navigation);
        }
        return stopped(navigation);

    case NAV_DISABLED:
    default:
        navigation->mode = NAV_HOLD;
        navigation->state_since_ms = now_ms;
        return stopped(navigation);
    }
}
