#include "range_capture.h"

#include <stddef.h>

#define RANGE_CAPTURE_MAX_INTERVAL_TICKS 0x7FFFFFFFU

static void complete(RangeCapture *capture, uint32_t pulse_width_ticks,
                     bool valid)
{
    capture->active = false;
    capture->waiting_for_fall = false;
    capture->pulse_width_ticks = valid ? pulse_width_ticks : 0U;
    capture->result_valid = valid;
    capture->result_ready = true;
}

bool range_capture_init(RangeCapture *capture, uint32_t timeout_ticks,
                        uint32_t min_pulse_ticks, uint32_t max_pulse_ticks)
{
    if (capture == NULL || timeout_ticks == 0U ||
        timeout_ticks > RANGE_CAPTURE_MAX_INTERVAL_TICKS ||
        min_pulse_ticks == 0U || min_pulse_ticks > max_pulse_ticks ||
        max_pulse_ticks >= timeout_ticks) {
        return false;
    }

    capture->timeout_ticks = timeout_ticks;
    capture->min_pulse_ticks = min_pulse_ticks;
    capture->max_pulse_ticks = max_pulse_ticks;
    capture->trigger_ticks = 0U;
    capture->rise_ticks = 0U;
    capture->pulse_width_ticks = 0U;
    capture->active = false;
    capture->waiting_for_fall = false;
    capture->result_ready = false;
    capture->result_valid = false;
    return true;
}

bool range_capture_begin(RangeCapture *capture, uint32_t now_ticks)
{
    if (capture == NULL || capture->active || capture->result_ready) {
        return false;
    }

    capture->trigger_ticks = now_ticks;
    capture->rise_ticks = 0U;
    capture->pulse_width_ticks = 0U;
    capture->waiting_for_fall = false;
    capture->result_valid = false;
    capture->active = true;
    return true;
}

void range_capture_echo_edge(RangeCapture *capture, bool echo_high,
                             uint32_t now_ticks)
{
    uint32_t elapsed_ticks;
    uint32_t pulse_width_ticks;

    if (capture == NULL || !capture->active) return;

    elapsed_ticks = now_ticks - capture->trigger_ticks;
    if (elapsed_ticks >= capture->timeout_ticks) {
        complete(capture, 0U, false);
        return;
    }

    if (echo_high) {
        if (!capture->waiting_for_fall) {
            capture->rise_ticks = now_ticks;
            capture->waiting_for_fall = true;
        }
        return;
    }

    if (!capture->waiting_for_fall) return;

    pulse_width_ticks = now_ticks - capture->rise_ticks;
    complete(capture, pulse_width_ticks,
             pulse_width_ticks >= capture->min_pulse_ticks &&
             pulse_width_ticks <= capture->max_pulse_ticks);
}

bool range_capture_poll(RangeCapture *capture, uint32_t now_ticks,
                        uint32_t *pulse_width_ticks, bool *valid)
{
    if (capture == NULL || pulse_width_ticks == NULL || valid == NULL) {
        return false;
    }

    if (capture->active &&
        (uint32_t)(now_ticks - capture->trigger_ticks) >=
            capture->timeout_ticks) {
        complete(capture, 0U, false);
    }

    if (!capture->result_ready) return false;

    *pulse_width_ticks = capture->pulse_width_ticks;
    *valid = capture->result_valid;
    capture->pulse_width_ticks = 0U;
    capture->result_valid = false;
    capture->result_ready = false;
    return true;
}

bool range_capture_distance_mm(uint32_t pulse_width_ticks,
                               uint32_t timer_hz, uint32_t *distance_mm)
{
    uint64_t denominator;
    uint64_t numerator;
    uint64_t rounded_mm;

    if (pulse_width_ticks == 0U || timer_hz == 0U ||
        distance_mm == NULL) {
        return false;
    }

    /* d = pulse_ticks * 343000 mm/s / (2 * timer_hz ticks/s). */
    denominator = (uint64_t)timer_hz * 2U;
    numerator = (uint64_t)pulse_width_ticks * 343000U;
    rounded_mm = (numerator + denominator / 2U) / denominator;
    if (rounded_mm == 0U || rounded_mm > UINT32_MAX) return false;

    *distance_mm = (uint32_t)rounded_mm;
    return true;
}
