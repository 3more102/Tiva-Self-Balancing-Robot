#ifndef TIVA_RANGE_CAPTURE_H
#define TIVA_RANGE_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Portable, non-blocking ultrasonic echo-pulse capture.
 *
 * Use one RangeCapture instance per sensor. The hardware adapter supplies
 * rising/falling ECHO edges with timestamps from a free-running 1 MHz timer.
 * Calls from an ISR and the foreground poller must be serialized by the
 * adapter (for example, mask that sensor's GPIO interrupt around poll()).
 */
typedef struct {
    uint32_t timeout_us;
    uint32_t min_pulse_us;
    uint32_t max_pulse_us;
    uint32_t trigger_us;
    uint32_t rise_us;
    uint32_t pulse_width_us;
    bool active;
    bool waiting_for_fall;
    bool result_ready;
    bool result_valid;
} RangeCapture;

/*
 * Configure a capture channel. timeout_us must be nonzero and less than
 * 2^31; min_pulse_us must be nonzero; min_pulse_us <= max_pulse_us <
 * timeout_us. Returns false for invalid arguments.
 */
bool range_capture_init(RangeCapture *capture, uint32_t timeout_us,
                        uint32_t min_pulse_us, uint32_t max_pulse_us);

/* Start one measurement. Returns false while a measurement/result is pending. */
bool range_capture_begin(RangeCapture *capture, uint32_t now_us);

/* Feed one sampled ECHO edge from the GPIO interrupt handler. */
void range_capture_echo_edge(RangeCapture *capture, bool echo_high,
                             uint32_t now_us);

/*
 * Poll without waiting. Returns true exactly once when a valid or timed-out
 * measurement is ready. pulse_width_us is zero for timeout/invalid results.
 */
bool range_capture_poll(RangeCapture *capture, uint32_t now_us,
                        uint32_t *pulse_width_us, bool *valid);

/* Convert a valid round-trip echo pulse to millimetres (343 m/s at ~20 C). */
bool range_capture_distance_mm(uint32_t pulse_width_us,
                               uint32_t *distance_mm);

#endif
