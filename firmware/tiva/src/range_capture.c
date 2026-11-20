#include "range_capture.h"

#include <stddef.h>

#define RANGE_CAPTURE_MAX_INTERVAL_US 0x7FFFFFFFU

static void complete(RangeCapture *capture, uint32_t pulse_width_us,
                     bool valid)
{
    capture->active = false;
    capture->waiting_for_fall = false;
    capture->pulse_width_us = valid ? pulse_width_us : 0U;
    capture->result_valid = valid;
    capture->result_ready = true;
}

bool range_capture_init(RangeCapture *capture, uint32_t timeout_us,
                        uint32_t min_pulse_us, uint32_t max_pulse_us)
{
    if (capture == NULL || timeout_us == 0U ||
        timeout_us >= RANGE_CAPTURE_MAX_INTERVAL_US ||
        min_pulse_us == 0U || min_pulse_us > max_pulse_us ||
        max_pulse_us >= timeout_us) {
        return false;
    }

    capture->timeout_us = timeout_us;
    capture->min_pulse_us = min_pulse_us;
    capture->max_pulse_us = max_pulse_us;
    capture->trigger_us = 0U;
    capture->rise_us = 0U;
    capture->pulse_width_us = 0U;
    capture->active = false;
    capture->waiting_for_fall = false;
    capture->result_ready = false;
    capture->result_valid = false;
    return true;
}

bool range_capture_begin(RangeCapture *capture, uint32_t now_us)
{
    if (capture == NULL || capture->active || capture->result_ready) {
        return false;
    }

    capture->trigger_us = now_us;
    capture->rise_us = 0U;
    capture->pulse_width_us = 0U;
    capture->waiting_for_fall = false;
    capture->result_valid = false;
    capture->active = true;
    return true;
}

void range_capture_echo_edge(RangeCapture *capture, bool echo_high,
                             uint32_t now_us)
{
    uint32_t elapsed_us;
    uint32_t pulse_width_us;

    if (capture == NULL || !capture->active) return;

    elapsed_us = now_us - capture->trigger_us;
    if (elapsed_us >= capture->timeout_us) {
        complete(capture, 0U, false);
        return;
    }

    if (echo_high) {
        if (!capture->waiting_for_fall) {
            capture->rise_us = now_us;
            capture->waiting_for_fall = true;
        }
        return;
    }

    if (!capture->waiting_for_fall) return;

    pulse_width_us = now_us - capture->rise_us;
    complete(capture, pulse_width_us,
             pulse_width_us >= capture->min_pulse_us &&
             pulse_width_us <= capture->max_pulse_us);
}

bool range_capture_poll(RangeCapture *capture, uint32_t now_us,
                        uint32_t *pulse_width_us, bool *valid)
{
    if (capture == NULL || pulse_width_us == NULL || valid == NULL) {
        return false;
    }

    if (capture->active &&
        (uint32_t)(now_us - capture->trigger_us) >= capture->timeout_us) {
        complete(capture, 0U, false);
    }

    if (!capture->result_ready) return false;

    *pulse_width_us = capture->pulse_width_us;
    *valid = capture->result_valid;
    capture->pulse_width_us = 0U;
    capture->result_valid = false;
    capture->result_ready = false;
    return true;
}

bool range_capture_distance_mm(uint32_t pulse_width_us,
                               uint32_t *distance_mm)
{
    uint64_t rounded_mm;

    if (pulse_width_us == 0U || distance_mm == NULL) return false;

    /* d = (echo_time * 343000 mm/s) / (2 * 1000000 us/s). */
    rounded_mm = ((uint64_t)pulse_width_us * 343U + 1000U) / 2000U;
    if (rounded_mm == 0U || rounded_mm > UINT32_MAX) return false;

    *distance_mm = (uint32_t)rounded_mm;
    return true;
}
