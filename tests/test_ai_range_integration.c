#include <assert.h>
#include <stdio.h>

#include "ai_navigation.h"
#include "ai_sensor_adapter.h"
#include "range_sensing.h"

static void echo(RangeArray *array, uint8_t sensor, uint32_t rise_us,
                 uint32_t pulse_us, uint32_t sample_ms)
{
    range_array_echo_edge(array, sensor, true, rise_us, sample_ms);
    range_array_echo_edge(array, sensor, false,
                          rise_us + pulse_us, sample_ms);
}

static void test_measured_range_events_reach_ai_decision(void)
{
    RangeArray ranges;
    RangeSnapshot snapshot;
    AiSensorFrame frame;
    AiNavigationState state;
    AiNavigationCommand command;
    uint8_t sensor;

    range_array_init(&ranges);

    /* Front obstacle ~= 200 mm. */
    sensor = range_array_next_trigger(&ranges, 0U, 0U);
    assert(sensor == RANGE_FRONT);
    echo(&ranges, sensor, 100U, 1166U, 1U);

    /* Left side clear ~= 1,200 mm. */
    sensor = range_array_next_trigger(&ranges, 60U, 60000U);
    assert(sensor == RANGE_LEFT);
    echo(&ranges, sensor, 60100U, 7000U, 61U);

    /* Right side blocked ~= 250 mm. */
    sensor = range_array_next_trigger(&ranges, 120U, 120000U);
    assert(sensor == RANGE_RIGHT);
    echo(&ranges, sensor, 120100U, 1458U, 121U);

    range_array_snapshot(&ranges, 121U, 250U, 750U, &snapshot);
    assert(snapshot.front.valid && snapshot.left.valid &&
           snapshot.right.valid);
    assert(snapshot.front.distance_mm >= 199U &&
           snapshot.front.distance_mm <= 201U);
    assert(snapshot.left.distance_mm >= 1199U &&
           snapshot.left.distance_mm <= 1202U);
    assert(snapshot.right.distance_mm >= 249U &&
           snapshot.right.distance_mm <= 251U);

    assert(ai_sensor_frame_from_ranges(&frame, &snapshot, true, 0.0f));
    ai_navigation_init(&state);
    command = ai_navigation_step(&state, &frame, 121U);
    assert(command.action == AI_ACTION_TURN_LEFT);
    assert(command.confidence > 0.80f);
    assert(!command.safety_override);
}

static void test_stale_front_measurement_blocks_ai_motion(void)
{
    RangeArray ranges;
    RangeSnapshot snapshot;
    AiSensorFrame frame;
    AiNavigationState state;
    AiNavigationCommand command;
    uint8_t sensor;

    range_array_init(&ranges);
    sensor = range_array_next_trigger(&ranges, 0U, 0U);
    assert(sensor == RANGE_FRONT);
    echo(&ranges, sensor, 100U, 6000U, 1U);
    range_array_snapshot(&ranges, 252U, 250U, 750U, &snapshot);
    assert(!snapshot.front.valid);
    assert(ai_sensor_frame_from_ranges(&frame, &snapshot, true, 0.0f));

    ai_navigation_init(&state);
    command = ai_navigation_step(&state, &frame, 252U);
    assert(command.action == AI_ACTION_STOP);
    assert(command.safety_override);
}

int main(void)
{
    test_measured_range_events_reach_ai_decision();
    test_stale_front_measurement_blocks_ai_motion();
    puts("PASS: range capture to TinyML obstacle-decision integration tests");
    return 0;
}
