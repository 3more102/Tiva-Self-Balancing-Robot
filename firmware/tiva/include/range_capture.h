#ifndef TIVA_RANGE_CAPTURE_H
#define TIVA_RANGE_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Portable, non-blocking ultrasonic echo-pulse capture.
 *
 * Use one RangeCapture instance per sensor. The hardware adapter supplies
 * rising/falling ECHO edges with timestamps from a free-running, monotonically
 * increasing 32-bit timer. Tiva Timer1A is down-counting; ~TimerValueGet()
 * converts its value to an up-counting timestamp.
 *
 * Calls from an ISR and the foreground poller must be serialized by the
 * adapter (for example, mask that sensor's GPIO interrupt around poll()).
 */
typedef struct {
    uint32_t timeout_ticks;
    uint32_t min_pulse_ticks;
    uint32_t max_pulse_ticks;
    uint32_t trigger_ticks;
    uint32_t rise_ticks;
    uint32_t pulse_width_ticks;
    bool active;
    bool waiting_for_fall;
    bool result_ready;
    bool result_valid;
} RangeCapture;

/*
 * Configure a capture channel. timeout_ticks must be nonzero and <= 2^31-1;
 * min_pulse_ticks must be nonzero; min_pulse_ticks <= max_pulse_ticks <
 * timeout_ticks. Returns false for invalid arguments.
 */
bool range_capture_init(RangeCapture *capture, uint32_t timeout_ticks,
                        uint32_t min_pulse_ticks, uint32_t max_pulse_ticks);

/* Start one measurement. Returns false while a measurement/result is pending. */
bool range_capture_begin(RangeCapture *capture, uint32_t now_ticks);

/* Feed one sampled ECHO edge from the GPIO interrupt handler. */
void range_capture_echo_edge(RangeCapture *capture, bool echo_high,
                             uint32_t now_ticks);

/*
 * Poll without waiting. Returns true exactly once when a valid or timed-out
 * measurement is ready. pulse_width_ticks is zero for timeout/invalid results.
 */
bool range_capture_poll(RangeCapture *capture, uint32_t now_ticks,
                        uint32_t *pulse_width_ticks, bool *valid);

/* Convert round-trip pulse ticks to millimetres using the actual timer rate. */
bool range_capture_distance_mm(uint32_t pulse_width_ticks,
                               uint32_t timer_hz, uint32_t *distance_mm);

#endif
