#ifndef TIVA_RANGE_SCANNER_H
#define TIVA_RANGE_SCANNER_H

#include <stdbool.h>
#include <stdint.h>

#include "range_capture.h"

#define RANGE_SCANNER_SENSOR_COUNT 3U

typedef enum {
    RANGE_SENSOR_FRONT = 0,
    RANGE_SENSOR_LEFT = 1,
    RANGE_SENSOR_RIGHT = 2
} RangeSensorId;

typedef struct {
    uint16_t distance_mm;
    uint32_t sample_ms;
    bool valid;
    bool has_sample;
} RangeSample;

typedef struct {
    RangeCapture capture[RANGE_SCANNER_SENSOR_COUNT];
    RangeSample samples[RANGE_SCANNER_SENSOR_COUNT];
    uint32_t inter_ping_ticks;
    uint32_t max_distance_mm;
    uint32_t timer_hz;
    uint32_t last_trigger_ticks;
    uint8_t next_sensor;
    uint8_t active_sensor;
    bool has_triggered;
    bool active;
} RangeScanner;

/*
 * Configure a three-sensor round-robin scanner. All timing values use the
 * same free-running 32-bit timer tick domain as RangeCapture.
 *
 * inter_ping_ticks is the minimum time between trigger starts for any two
 * sensors. Use a conservative acoustic quiet interval for the selected module.
 * max_distance_mm rejects readings outside the application's trusted range.
 */
bool range_scanner_init(RangeScanner *scanner,
                        uint32_t timeout_ticks,
                        uint32_t min_pulse_ticks,
                        uint32_t max_pulse_ticks,
                        uint32_t inter_ping_ticks,
                        uint32_t timer_hz,
                        uint32_t max_distance_mm);

/* Return the next sensor that may be triggered. Does not reserve it. */
bool range_scanner_next_trigger(const RangeScanner *scanner,
                                uint32_t now_ticks,
                                RangeSensorId *sensor);

/*
 * Begin capture immediately before the hardware TRIG pulse. The caller must
 * issue the short hardware pulse immediately after this returns true.
 */
bool range_scanner_begin(RangeScanner *scanner, RangeSensorId sensor,
                         uint32_t now_ticks);

/* Forward an ECHO edge from the GPIO ISR for the specified physical sensor. */
void range_scanner_echo_edge(RangeScanner *scanner, RangeSensorId sensor,
                             bool echo_high, uint32_t now_ticks);

/*
 * Consume one completed capture without waiting. sample_ms is the foreground
 * time at which the completed result is published. Invalid/timeout readings
 * are published with valid=false and distance_mm=0.
 */
bool range_scanner_poll(RangeScanner *scanner, uint32_t now_ticks,
                        uint32_t now_ms, RangeSensorId *completed_sensor);

/* Copy the latest published sample for one sensor. */
bool range_scanner_get_sample(const RangeScanner *scanner,
                              RangeSensorId sensor, RangeSample *sample);

#endif
