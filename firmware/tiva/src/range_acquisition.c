#include "range_acquisition.h"

#include <stddef.h>

bool range_acquisition_init(RangeAcquisition *acquisition,
                            const RangeHardwareOps *hardware,
                            uint32_t timeout_ticks,
                            uint32_t min_pulse_ticks,
                            uint32_t max_pulse_ticks,
                            uint32_t inter_ping_ticks,
                            uint32_t timer_hz,
                            uint32_t max_distance_mm)
{
    if (acquisition == NULL) return false;
    acquisition->initialized = false;

    if (hardware == NULL || hardware->timer_ticks == NULL ||
        hardware->now_ms == NULL ||
        hardware->set_echo_irq_enabled == NULL ||
        hardware->trigger_pulse == NULL) {
        return false;
    }

    if (!range_scanner_init(&acquisition->scanner, timeout_ticks,
                            min_pulse_ticks, max_pulse_ticks,
                            inter_ping_ticks, timer_hz, max_distance_mm)) {
        acquisition->initialized = false;
        return false;
    }

    acquisition->hardware = *hardware;
    acquisition->initialized = true;
    acquisition->hardware.set_echo_irq_enabled(
        acquisition->hardware.context, false);
    return true;
}

bool range_acquisition_step(RangeAcquisition *acquisition,
                            RangeSensorId *completed_sensor)
{
    RangeSensorId completed = RANGE_SENSOR_FRONT;
    RangeSensorId next_sensor;
    uint32_t now_ticks;
    uint32_t now_ms;
    bool sample_published;

    if (acquisition == NULL || !acquisition->initialized) return false;

    /*
     * The ECHO ISR mutates RangeCapture while the foreground polls it.
     * Mask only these GPIO interrupts around the short shared-state section.
     * Timer0A/Timer1A and the balance loop are not masked or delayed.
     */
    acquisition->hardware.set_echo_irq_enabled(
        acquisition->hardware.context, false);

    now_ticks = acquisition->hardware.timer_ticks(
        acquisition->hardware.context);
    now_ms = acquisition->hardware.now_ms(
        acquisition->hardware.context);

    sample_published = range_scanner_poll(&acquisition->scanner,
                                          now_ticks, now_ms, &completed);

    if (range_scanner_next_trigger(&acquisition->scanner, now_ticks,
                                   &next_sensor) &&
        range_scanner_begin(&acquisition->scanner, next_sensor, now_ticks)) {
        /* The hardware callback emits TRIG only; it must not wait for ECHO. */
        acquisition->hardware.trigger_pulse(
            acquisition->hardware.context, next_sensor);
    }

    acquisition->hardware.set_echo_irq_enabled(
        acquisition->hardware.context, true);

    if (sample_published && completed_sensor != NULL) {
        *completed_sensor = completed;
    }
    return sample_published;
}

void range_acquisition_echo_edge(RangeAcquisition *acquisition,
                                 RangeSensorId sensor, bool echo_high)
{
    uint32_t now_ticks;

    if (acquisition == NULL || !acquisition->initialized) return;

    now_ticks = acquisition->hardware.timer_ticks(
        acquisition->hardware.context);
    range_scanner_echo_edge(&acquisition->scanner, sensor, echo_high,
                            now_ticks);
}

bool range_acquisition_build_navigation_input(
    const RangeAcquisition *acquisition, bool autonomy_enabled,
    NavigationInput *input)
{
    if (acquisition == NULL || !acquisition->initialized || input == NULL) {
        return false;
    }

    return range_navigation_build_input(&acquisition->scanner,
                                        autonomy_enabled, input);
}
