#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "range_acquisition.h"

#define TIMER_HZ 80000000U
#define ECHO_TIMEOUT_TICKS 2000000U
#define ECHO_MIN_TICKS 9360U
#define ECHO_MAX_TICKS 1400000U
#define INTER_PING_TICKS 4800000U

typedef struct {
    uint32_t ticks;
    uint32_t ms;
    bool echo_irq_enabled;
    unsigned int disable_count;
    unsigned int enable_count;
    RangeSensorId triggered[8];
    unsigned int trigger_count;
} FakeHardware;

static uint32_t fake_timer_ticks(void *context)
{
    return ((FakeHardware *)context)->ticks;
}

static uint32_t fake_now_ms(void *context)
{
    return ((FakeHardware *)context)->ms;
}

static void fake_set_echo_irq_enabled(void *context, bool enabled)
{
    FakeHardware *fake = (FakeHardware *)context;
    fake->echo_irq_enabled = enabled;
    if (enabled) ++fake->enable_count;
    else ++fake->disable_count;
}

static void fake_trigger_pulse(void *context, RangeSensorId sensor)
{
    FakeHardware *fake = (FakeHardware *)context;
    assert(!fake->echo_irq_enabled);
    assert(fake->trigger_count < 8U);
    fake->triggered[fake->trigger_count++] = sensor;
}

static void init_acquisition(RangeAcquisition *acquisition,
                             FakeHardware *fake)
{
    RangeHardwareOps hardware;

    memset(fake, 0, sizeof(*fake));
    hardware.context = fake;
    hardware.timer_ticks = fake_timer_ticks;
    hardware.now_ms = fake_now_ms;
    hardware.set_echo_irq_enabled = fake_set_echo_irq_enabled;
    hardware.trigger_pulse = fake_trigger_pulse;

    assert(range_acquisition_init(acquisition, &hardware,
                                  ECHO_TIMEOUT_TICKS, ECHO_MIN_TICKS,
                                  ECHO_MAX_TICKS, INTER_PING_TICKS,
                                  TIMER_HZ, 4000U));
    assert(!fake->echo_irq_enabled);
    assert(fake->trigger_count == 0U);
}

static void send_echo(RangeAcquisition *acquisition, FakeHardware *fake,
                      RangeSensorId sensor, uint32_t rise_offset,
                      uint32_t pulse_ticks)
{
    assert(fake->echo_irq_enabled);
    fake->ticks += rise_offset;
    range_acquisition_echo_edge(acquisition, sensor, true);
    fake->ticks += pulse_ticks;
    range_acquisition_echo_edge(acquisition, sensor, false);
}

static void test_initialization_and_first_trigger(void)
{
    RangeAcquisition acquisition;
    FakeHardware fake;
    RangeSensorId completed = RANGE_SENSOR_RIGHT;

    init_acquisition(&acquisition, &fake);
    assert(!range_acquisition_step(NULL, NULL));
    assert(!range_acquisition_step(&acquisition, &completed));
    assert(fake.trigger_count == 1U);
    assert(fake.triggered[0] == RANGE_SENSOR_FRONT);
    assert(fake.echo_irq_enabled);
    assert(completed == RANGE_SENSOR_RIGHT);
    assert(fake.disable_count == 2U);
    assert(fake.enable_count == 1U);
}

static void test_three_sensor_round_robin_and_navigation_mapping(void)
{
    RangeAcquisition acquisition;
    FakeHardware fake;
    NavigationInput input = {0};
    RangeSensorId completed;

    init_acquisition(&acquisition, &fake);

    /* First foreground step starts the front sensor. */
    assert(!range_acquisition_step(&acquisition, NULL));
    assert(fake.triggered[0] == RANGE_SENSOR_FRONT);

    /* Complete front at 154 mm and publish it on the next foreground step. */
    send_echo(&acquisition, &fake, RANGE_SENSOR_FRONT, 100U, 72000U);
    fake.ms = 5U;
    assert(range_acquisition_step(&acquisition, &completed));
    assert(completed == RANGE_SENSOR_FRONT);
    assert(fake.trigger_count == 1U);

    /* At 60 ms the next sensor is left. */
    fake.ticks = INTER_PING_TICKS;
    fake.ms = 60U;
    assert(!range_acquisition_step(&acquisition, NULL));
    assert(fake.trigger_count == 2U);
    assert(fake.triggered[1] == RANGE_SENSOR_LEFT);

    send_echo(&acquisition, &fake, RANGE_SENSOR_LEFT, 100U, 48000U);
    fake.ticks += 100U;
    fake.ms = 65U;
    assert(range_acquisition_step(&acquisition, &completed));
    assert(completed == RANGE_SENSOR_LEFT);

    fake.ticks = 2U * INTER_PING_TICKS;
    fake.ms = 120U;
    assert(!range_acquisition_step(&acquisition, NULL));
    assert(fake.trigger_count == 3U);
    assert(fake.triggered[2] == RANGE_SENSOR_RIGHT);

    send_echo(&acquisition, &fake, RANGE_SENSOR_RIGHT, 100U, 96000U);
    fake.ticks += 100U;
    fake.ms = 125U;
    assert(range_acquisition_step(&acquisition, &completed));
    assert(completed == RANGE_SENSOR_RIGHT);

    assert(range_acquisition_build_navigation_input(&acquisition, true,
                                                     &input));
    assert(input.autonomy_enabled);
    assert(input.front_valid && input.front_mm == 154U);
    assert(input.front_sample_ms == 5U);
    assert(input.left_valid && input.left_mm == 103U);
    assert(input.left_sample_ms == 65U);
    assert(input.right_valid && input.right_mm == 206U);
    assert(input.right_sample_ms == 125U);
}

static void test_timeout_is_published_invalid_and_not_clear(void)
{
    RangeAcquisition acquisition;
    FakeHardware fake;
    NavigationInput input = {0};
    RangeSensorId completed;

    init_acquisition(&acquisition, &fake);
    assert(!range_acquisition_step(&acquisition, NULL));

    fake.ticks = ECHO_TIMEOUT_TICKS;
    fake.ms = 25U;
    assert(range_acquisition_step(&acquisition, &completed));
    assert(completed == RANGE_SENSOR_FRONT);
    assert(range_acquisition_build_navigation_input(&acquisition, true,
                                                     &input));
    assert(!input.front_valid);
    assert(input.front_mm == 0U);
    assert(input.front_sample_ms == 25U);
}

static void test_echo_for_non_active_sensor_is_ignored(void)
{
    RangeAcquisition acquisition;
    FakeHardware fake;
    NavigationInput input = {0};
    RangeSensorId completed;

    init_acquisition(&acquisition, &fake);
    assert(!range_acquisition_step(&acquisition, NULL));

    fake.ticks = 100U;
    range_acquisition_echo_edge(&acquisition, RANGE_SENSOR_LEFT, true);
    fake.ticks = 20000U;
    range_acquisition_echo_edge(&acquisition, RANGE_SENSOR_LEFT, false);
    fake.ms = 2U;
    assert(!range_acquisition_step(&acquisition, &completed));

    send_echo(&acquisition, &fake, RANGE_SENSOR_FRONT, 100U, 12000U);
    fake.ms = 3U;
    assert(range_acquisition_step(&acquisition, &completed));
    assert(completed == RANGE_SENSOR_FRONT);

    assert(range_acquisition_build_navigation_input(&acquisition, true,
                                                     &input));
    assert(input.front_valid && input.front_mm == 26U);
    assert(!input.left_valid);
}

static void test_trigger_is_only_started_after_quiet_interval(void)
{
    RangeAcquisition acquisition;
    FakeHardware fake;

    init_acquisition(&acquisition, &fake);
    assert(!range_acquisition_step(&acquisition, NULL));
    assert(fake.trigger_count == 1U);

    fake.ticks = INTER_PING_TICKS - 1U;
    fake.ms = 59U;
    /* The 60 ms quiet-time check is also a 60 ms timeout poll, so the
       first sensor's 25 ms no-echo result is published as invalid here. */
    assert(range_acquisition_step(&acquisition, NULL));
    assert(fake.trigger_count == 1U);

    fake.ticks = INTER_PING_TICKS;
    fake.ms = 60U;
    assert(!range_acquisition_step(&acquisition, NULL));
    assert(fake.trigger_count == 2U);
    assert(fake.triggered[1] == RANGE_SENSOR_LEFT);
}

static void test_invalid_hardware_contract(void)
{
    RangeAcquisition acquisition;
    FakeHardware fake;
    RangeHardwareOps hardware;

    memset(&fake, 0, sizeof(fake));
    hardware.context = &fake;
    hardware.timer_ticks = fake_timer_ticks;
    hardware.now_ms = fake_now_ms;
    hardware.set_echo_irq_enabled = fake_set_echo_irq_enabled;
    hardware.trigger_pulse = fake_trigger_pulse;

    assert(!range_acquisition_init(NULL, &hardware, ECHO_TIMEOUT_TICKS,
                                   ECHO_MIN_TICKS, ECHO_MAX_TICKS,
                                   INTER_PING_TICKS, TIMER_HZ, 4000U));
    assert(!range_acquisition_init(&acquisition, NULL, ECHO_TIMEOUT_TICKS,
                                   ECHO_MIN_TICKS, ECHO_MAX_TICKS,
                                   INTER_PING_TICKS, TIMER_HZ, 4000U));

    hardware.timer_ticks = NULL;
    assert(!range_acquisition_init(&acquisition, &hardware,
                                   ECHO_TIMEOUT_TICKS, ECHO_MIN_TICKS,
                                   ECHO_MAX_TICKS, INTER_PING_TICKS,
                                   TIMER_HZ, 4000U));

    hardware.timer_ticks = fake_timer_ticks;
    hardware.trigger_pulse = NULL;
    assert(!range_acquisition_init(&acquisition, &hardware,
                                   ECHO_TIMEOUT_TICKS, ECHO_MIN_TICKS,
                                   ECHO_MAX_TICKS, INTER_PING_TICKS,
                                   TIMER_HZ, 4000U));

    assert(!range_acquisition_build_navigation_input(&acquisition, true,
                                                      &(NavigationInput){0}));
    range_acquisition_echo_edge(NULL, RANGE_SENSOR_FRONT, true);
}

int main(void)
{
    test_initialization_and_first_trigger();
    test_three_sensor_round_robin_and_navigation_mapping();
    test_timeout_is_published_invalid_and_not_clear();
    test_echo_for_non_active_sensor_is_ignored();
    test_trigger_is_only_started_after_quiet_interval();
    test_invalid_hardware_contract();
    puts("PASS: range acquisition coordinator and mock hardware tests");
    return 0;
}
