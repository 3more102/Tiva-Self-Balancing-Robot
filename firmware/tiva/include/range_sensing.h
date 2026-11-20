#ifndef GYROGUARD_RANGE_SENSING_H
#define GYROGUARD_RANGE_SENSING_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Portable asynchronous ultrasonic echo-capture core.
 *
 * The MCU-specific adapter must:
 *   1. Call range_array_next_trigger() from a non-blocking scheduler.
 *   2. Emit the sensor-specific trigger pulse for the returned channel.
 *   3. Forward both echo edges and monotonic hardware-counter timestamps to
 *      range_array_echo_edge() from a GPIO/timer capture ISR.
 *   4. Call range_array_poll() periodically to expire missing echoes.
 *   5. Protect range_array_snapshot() against concurrent ISR writes.
 *
 * Timestamps are raw ticks from a free-running up-counter (or the bitwise
 * inverse of a free-running down-counter). ticks_per_us is supplied at init.
 * This avoids needing a separate absolute microsecond clock on the MCU.
 *
 * No GPIO pins, timer peripherals, trigger pulse width, or sensor model are
 * assumed here. The 60 ms inter-ping gap is a conservative default intended
 * to reduce acoustic cross-talk between three sensors.
 */

#define RANGE_SENSOR_COUNT 3U
#define RANGE_SENSOR_NONE 0xFFU
#define RANGE_INTER_PING_GAP_MS 60U
#define RANGE_ECHO_TIMEOUT_US 13000U
#define RANGE_ECHO_MIN_US 100U
#define RANGE_ECHO_MAX_US 12000U
#define RANGE_MAX_DISTANCE_MM 2000U

typedef enum {
    RANGE_FRONT = 0,
    RANGE_LEFT = 1,
    RANGE_RIGHT = 2
} RangeSensorId;

typedef enum {
    RANGE_CAPTURE_IDLE = 0,
    RANGE_CAPTURE_WAIT_RISE = 1,
    RANGE_CAPTURE_WAIT_FALL = 2
} RangeCaptureState;

typedef struct {
    bool valid;
    uint16_t distance_mm;
    uint32_t sample_ms;
} RangeReading;

typedef struct {
    RangeReading front;
    RangeReading left;
    RangeReading right;
} RangeSnapshot;

typedef struct {
    RangeCaptureState state;
    uint32_t rise_ticks;
    uint32_t deadline_ticks;
    uint32_t last_sample_ms;
    uint16_t distance_mm;
    uint32_t valid_sample_count;
    uint32_t timeout_count;
    uint32_t invalid_pulse_count;
    bool has_sample;
} RangeChannel;

typedef struct {
    RangeChannel channel[RANGE_SENSOR_COUNT];
    uint8_t next_sensor;
    uint8_t active_sensor;
    uint32_t last_trigger_ms;
    uint32_t trigger_count;
    uint32_t trigger_failure_count;
    uint32_t ticks_per_us;
    bool has_triggered;
} RangeArray;

void range_array_init(RangeArray *array, uint32_t ticks_per_us);

/*
 * Reserve the next channel for a ping and return RANGE_FRONT/LEFT/RIGHT.
 * Returns RANGE_SENSOR_NONE while a capture is active or the inter-ping
 * interval has not elapsed. The caller must emit the physical trigger pulse
 * immediately after this function returns a channel.
 */
uint8_t range_array_next_trigger(RangeArray *array,
                                 uint32_t now_ms,
                                 uint32_t now_ticks);

/* Call if the hardware adapter could not emit the trigger pulse. */
void range_array_cancel_trigger(RangeArray *array, uint8_t sensor);

/* Feed a rising/falling echo edge. timestamp_ticks and now_ticks must use the
 * same free-running counter configured through ticks_per_us. Unsigned
 * subtraction makes pulse widths safe across counter rollover. */
void range_array_echo_edge(RangeArray *array, uint8_t sensor,
                           bool echo_high, uint32_t timestamp_ticks,
                           uint32_t now_ms);

/* Expire a capture that did not receive a complete echo pulse. */
void range_array_poll(RangeArray *array, uint32_t now_ticks);

/* max-age arguments are sensor-specific freshness limits in milliseconds.
 * Call inside a short critical section if an echo ISR can update the array. */
void range_array_snapshot(const RangeArray *array, uint32_t now_ms,
                          uint32_t front_max_age_ms,
                          uint32_t side_max_age_ms,
                          RangeSnapshot *snapshot);

#endif /* GYROGUARD_RANGE_SENSING_H */
