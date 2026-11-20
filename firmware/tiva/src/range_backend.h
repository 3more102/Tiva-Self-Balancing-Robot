#ifndef TIVA_RANGE_BACKEND_H
#define TIVA_RANGE_BACKEND_H

#include <stdbool.h>
#include <stdint.h>

#include "range_sensing.h"
#include "ai_navigation.h"

/*
 * Optional HC-SR04-compatible adapter for EK-TM4C123GXL.
 *
 * Disabled by default because the actual range-sensor model and wiring have
 * not been confirmed. Enable only after checking the sensor datasheet and
 * wiring.md. Echo inputs must be 3.3 V-safe (use level translation/divider
 * for a 5 V Echo output).
 *
 * Provisional LaunchPad pins when enabled:
 *   Front: TRIG PE0, ECHO PE1
 *   Left:  TRIG PE2, ECHO PE3
 *   Right: TRIG PE4, ECHO PE5
 *
 * The 10 us trigger pulse is ended by Timer2A; Echo edges are captured by
 * Port E GPIO interrupts. Timer1A is already a free-running 80 MHz downcounter
 * in main.c; its bitwise inverse is passed as an increasing raw tick counter.
 * No busy-wait is performed in the 200 Hz balance loop.
 */
#ifndef RANGE_HCSR04_BACKEND_ENABLED
#define RANGE_HCSR04_BACKEND_ENABLED 0
#endif

static RangeArray g_range_array;

#if RANGE_HCSR04_BACKEND_ENABLED

#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "inc/hw_ints.h"

#define RANGE_TRIGGER_MASK (GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_4)
#define RANGE_ECHO_MASK    (GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_5)

static const uint8_t g_range_trigger_pins[RANGE_SENSOR_COUNT] = {
    GPIO_PIN_0, GPIO_PIN_2, GPIO_PIN_4
};
static const uint8_t g_range_echo_pins[RANGE_SENSOR_COUNT] = {
    GPIO_PIN_1, GPIO_PIN_3, GPIO_PIN_5
};

static volatile uint32_t g_range_now_ms;
static volatile bool g_range_trigger_pulse_active;
static uint32_t g_range_clock_hz;

static void range_trigger_timer_isr(void)
{
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    TimerDisable(TIMER2_BASE, TIMER_A);
    GPIOPinWrite(GPIO_PORTE_BASE, RANGE_TRIGGER_MASK, 0U);
    g_range_trigger_pulse_active = false;
}

static void range_echo_port_isr(void)
{
    uint32_t status = GPIOIntStatus(GPIO_PORTE_BASE, true);
    uint32_t edge_ticks;
    uint32_t sample_ms;
    uint32_t i;

    GPIOIntClear(GPIO_PORTE_BASE, status);
    edge_ticks = ~TimerValueGet(TIMER1_BASE, TIMER_A);
    sample_ms = g_range_now_ms;

    for (i = 0U; i < RANGE_SENSOR_COUNT; ++i) {
        const uint8_t pin = g_range_echo_pins[i];
        if ((status & pin) != 0U) {
            const bool high = (GPIOPinRead(GPIO_PORTE_BASE, pin) & pin) != 0U;
            range_array_echo_edge(&g_range_array, (uint8_t)i, high,
                                  edge_ticks, sample_ms);
        }
    }
}

static void range_backend_init(uint32_t clock_hz)
{
    uint32_t ticks_per_us = clock_hz / 1000000U;

    g_range_clock_hz = clock_hz;
    g_range_now_ms = 0U;
    g_range_trigger_pulse_active = false;
    range_array_init(&g_range_array, ticks_per_us);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    wait_for_peripheral(SYSCTL_PERIPH_GPIOE);
    wait_for_peripheral(SYSCTL_PERIPH_TIMER2);

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, RANGE_TRIGGER_MASK);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, RANGE_ECHO_MASK);
    GPIOPadConfigSet(GPIO_PORTE_BASE, RANGE_ECHO_MASK,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    GPIOPinWrite(GPIO_PORTE_BASE, RANGE_TRIGGER_MASK, 0U);

    GPIOIntDisable(GPIO_PORTE_BASE, RANGE_ECHO_MASK);
    GPIOIntTypeSet(GPIO_PORTE_BASE, RANGE_ECHO_MASK, GPIO_BOTH_EDGES);
    GPIOIntRegister(GPIO_PORTE_BASE, range_echo_port_isr);
    GPIOIntClear(GPIO_PORTE_BASE, RANGE_ECHO_MASK);
    GPIOIntEnable(GPIO_PORTE_BASE, RANGE_ECHO_MASK);
    IntEnable(INT_GPIOE);

    TimerConfigure(TIMER2_BASE, TIMER_CFG_ONE_SHOT);
    TimerIntRegister(TIMER2_BASE, TIMER_A, range_trigger_timer_isr);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER2A);

    IntMasterEnable();
}

static void range_backend_service(uint32_t now_ms)
{
    const uint32_t now_ticks = ~TimerValueGet(TIMER1_BASE, TIMER_A);
    uint8_t sensor;

    g_range_now_ms = now_ms;

    /* The echo ISR is the only other writer of g_range_array. Keep this
       critical section short; no acoustic wait or trigger delay occurs here. */
    GPIOIntDisable(GPIO_PORTE_BASE, RANGE_ECHO_MASK);
    range_array_poll(&g_range_array, now_ticks);
    sensor = range_array_next_trigger(&g_range_array, now_ms, now_ticks);

    if (sensor != RANGE_SENSOR_NONE) {
        if (g_range_trigger_pulse_active) {
            range_array_cancel_trigger(&g_range_array, sensor);
        } else {
            const uint32_t pulse_ticks = g_range_clock_hz / 100000U;
            g_range_trigger_pulse_active = true;
            GPIOPinWrite(GPIO_PORTE_BASE, RANGE_TRIGGER_MASK,
                         g_range_trigger_pins[sensor]);
            TimerLoadSet(TIMER2_BASE, TIMER_A,
                         pulse_ticks > 0U ? pulse_ticks - 1U : 0U);
            TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
            TimerEnable(TIMER2_BASE, TIMER_A);
        }
    }
    GPIOIntEnable(GPIO_PORTE_BASE, RANGE_ECHO_MASK);
}

static void range_backend_snapshot(uint32_t now_ms, RangeSnapshot *snapshot)
{
    GPIOIntDisable(GPIO_PORTE_BASE, RANGE_ECHO_MASK);
    range_array_snapshot(&g_range_array, now_ms,
                         AI_FRONT_MAX_AGE_MS, AI_SIDE_MAX_AGE_MS, snapshot);
    GPIOIntEnable(GPIO_PORTE_BASE, RANGE_ECHO_MASK);
}

#else /* RANGE_HCSR04_BACKEND_ENABLED == 0 */

static void range_backend_init(uint32_t clock_hz)
{
    range_array_init(&g_range_array, clock_hz / 1000000U);
}

static void range_backend_service(uint32_t now_ms)
{
    (void)now_ms;
}

static void range_backend_snapshot(uint32_t now_ms, RangeSnapshot *snapshot)
{
    range_array_snapshot(&g_range_array, now_ms,
                         AI_FRONT_MAX_AGE_MS, AI_SIDE_MAX_AGE_MS, snapshot);
}

#endif /* RANGE_HCSR04_BACKEND_ENABLED */

#endif /* TIVA_RANGE_BACKEND_H */
