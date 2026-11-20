#ifndef TIVA_RANGE_ACQUISITION_H
#define TIVA_RANGE_ACQUISITION_H

#include <stdbool.h>
#include <stdint.h>

#include "range_navigation.h"
#include "range_scanner.h"

/*
 * Hardware boundary for the foreground range-acquisition coordinator.
 *
 * timer_ticks must return the same monotonically increasing 32-bit tick
 * domain used by RangeCapture. For Timer1A's down-counter, use
 * ~TimerValueGet(TIMER1_BASE, TIMER_A).
 *
 * now_ms must be a wrap-safe millisecond clock in the same epoch used by
 * navigation_update().
 *
 * set_echo_irq_enabled must mask/unmask only the range-sensor ECHO GPIO
 * interrupts. It must not clear pending edge flags when re-enabling them.
 *
 * trigger_pulse must emit one short hardware TRIG pulse for the selected
 * sensor and return immediately after the pulse; it must not wait for ECHO.
 */
typedef struct {
    void *context;
    uint32_t (*timer_ticks)(void *context);
    uint32_t (*now_ms)(void *context);
    void (*set_echo_irq_enabled)(void *context, bool enabled);
    void (*trigger_pulse)(void *context, RangeSensorId sensor);
} RangeHardwareOps;

typedef struct {
    RangeScanner scanner;
    RangeHardwareOps hardware;
    bool initialized;
} RangeAcquisition;

/*
 * Initialize the portable acquisition coordinator. All sensor timing values
 * are passed through to range_scanner_init(). The ECHO interrupts are left
 * disabled until the first range_acquisition_step() call.
 */
bool range_acquisition_init(RangeAcquisition *acquisition,
                            const RangeHardwareOps *hardware,
                            uint32_t timeout_ticks,
                            uint32_t min_pulse_ticks,
                            uint32_t max_pulse_ticks,
                            uint32_t inter_ping_ticks,
                            uint32_t timer_hz,
                            uint32_t max_distance_mm);

/*
 * Call once per foreground control iteration. It masks ECHO interrupts only
 * for the short scanner state update, publishes any completed result, starts
 * at most one next sensor, emits its TRIG pulse, and restores ECHO interrupts.
 *
 * Returns true when a sample was published. completed_sensor is optional.
 */
bool range_acquisition_step(RangeAcquisition *acquisition,
                            RangeSensorId *completed_sensor);

/* Call from the GPIO ECHO ISR with the physical sensor ID and pin level. */
void range_acquisition_echo_edge(RangeAcquisition *acquisition,
                                 RangeSensorId sensor, bool echo_high);

/* Copy latest range samples into the existing navigation input contract. */
bool range_acquisition_build_navigation_input(
    const RangeAcquisition *acquisition, bool autonomy_enabled,
    NavigationInput *input);

#endif
