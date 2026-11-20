/*
 * NANENG 512: TI EK-TM4C123GXL starter firmware.
 * Unverified embedded program: keep motors disconnected for first tests.
 * Source deliberately separates MCU helper routines into local headers,
 * which are included here (not compiled as separate translation units).
 */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "controller.h"
#include "diagnostics.h"
#include "ai_navigation.h"
#include "ai_sensor_adapter.h"

#define MOTOR_OUTPUT_ALLOWED 0  /* MUST remain disabled before hardware qualification. */
#define MOTOR_POLARITY (-1.0f)
#define LEFT_MOTOR_POLARITY 1
#define RIGHT_MOTOR_POLARITY (-1)

#define CONTROL_HZ 200U
#define PWM_HZ 20000U
#define MPU6050_ADDRESS 0x68U
#define I2C_TIMEOUT_LOOPS 50000U
#define MPU6050_WHO_AM_I 0x75U
#define MPU6050_ACCEL_XOUT_H 0x3BU
#define MPU6050_GYRO_CONFIG 0x1BU
#define MPU6050_ACCEL_CONFIG 0x1CU
#define MPU6050_PWR_MGMT_1 0x6BU
#define MPU6050_CONFIG 0x1AU
#define MPU6050_SMPLRT_DIV 0x19U
#define GPIO_MOTOR_PINS (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)

static void wait_for_peripheral(uint32_t id)
{
    while (!SysCtlPeripheralReady(id)) { }
}

#include "imu_backend.h"
#include "motor_backend.h"
#include "range_backend.h"

/* CCS watch expressions for shadow-mode navigation diagnostics. */
static volatile uint32_t g_ai_action_debug = (uint32_t)AI_ACTION_STOP;
static volatile uint32_t g_ai_safety_override_debug = 0U;
static volatile float g_ai_confidence_debug = 0.0f;
static volatile float g_yaw_rate_debug_dps = 0.0f;

static void control_timer_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    wait_for_peripheral(SYSCTL_PERIPH_TIMER0);
    wait_for_peripheral(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / CONTROL_HZ - 1U);
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
    /* Timer1A: continuously running timestamp source independent of polling.
       Unsigned down-count subtraction safely handles the 32-bit rollover. */
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, 0xFFFFFFFFU);
    TimerEnable(TIMER1_BASE, TIMER_A);
}

static void await_tick(void)
{
    while ((TimerIntStatus(TIMER0_BASE, false) & TIMER_TIMA_TIMEOUT) == 0U) { }
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

int main(void)
{
    float ay, az, gx, gz;
    uint32_t i;
    float dt_s = 0.0f;
    GyroCalibration gyro_cal;
    LoopTiming timing;
    bool imu_ok;
    BalanceSafety safety;
    TiltEstimator estimator;
    BalanceController controller;
    AiNavigationState ai_state;
    AiSensorFrame ai_frame;
    AiNavigationCommand ai_command;
    RangeSnapshot range_snapshot;
    uint32_t now_ms = 0U;
    uint8_t ai_divider = 0U;
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ); /* 80 MHz */
    FPUEnable();
    FPULazyStackingEnable();
    outputs_init();
    motors_off();
    indicator(GPIO_PIN_2); /* blue: safe/off */
    imu_ok = imu_init();
    control_timer_init();
    range_backend_init(SysCtlClockGet());
    ai_navigation_init(&ai_state);
    gyro_cal_init(&gyro_cal);
    if (imu_ok) {
        /* Calibration requires 200 consecutive, upright, still samples
           with SW1 RELEASED. Timeout after 10 s of unsuccessful attempts. */
        for (i = 0U; i < 2000U && !gyro_cal_ready(&gyro_cal); ++i) {
            bool released;
            await_tick();
            released = (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) &
                        GPIO_PIN_4) != 0U;
            if (!mpu_sample(&ay, &az, &gx, &gz)) { imu_ok = false; break; }
            (void)gyro_cal_add_axes(&gyro_cal, ay, az, gx, gz, released);
        }
        if (!gyro_cal_ready(&gyro_cal)) imu_ok = false;
    }
    balance_safety_init(&safety);
    tilt_estimator_init(&estimator, 0.5f,
                        imu_ok ? gyro_cal_bias_dps(&gyro_cal) : 0.0f);
    loop_timing_init(&timing, SysCtlClockGet(),
                     TimerValueGet(TIMER1_BASE, TIMER_A));
    balance_controller_init(&controller, 0.016f, 0.0f, 0.0009f);

    for (;;) {
        bool pressed, balance_armed;
        float angle, output;
        await_tick();
        /* Actual elapsed time, not assumed constant Ts. A missed/abnormal
           sample deadline cuts motor output via the software safety gate. */
        const bool on_time = loop_timing_step(
            &timing, TimerValueGet(TIMER1_BASE, TIMER_A), &dt_s);
        pressed = (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) & GPIO_PIN_4) == 0U;
        if (!on_time) {
            g_ai_action_debug = (uint32_t)AI_ACTION_STOP;
            g_ai_safety_override_debug = 1U;
            g_ai_confidence_debug = 0.0f;
            (void)balance_safety_update(&safety, NAN, pressed, false);
            balance_controller_reset(&controller);
            motors_off();
            indicator(GPIO_PIN_1);
            continue;
        }
        if (!imu_ok || !mpu_sample(&ay, &az, &gx, &gz)) {
            imu_ok = false; /* Fatal sensor fault requires a hardware reset. */
            g_ai_action_debug = (uint32_t)AI_ACTION_STOP;
            g_ai_safety_override_debug = 1U;
            g_ai_confidence_debug = 0.0f;
            (void)balance_safety_update(&safety, NAN, pressed, false);
            motors_off();
            indicator(GPIO_PIN_1);
            continue;
        }
        /* Range scheduling is non-blocking. Timer1A raw ticks timestamp echo
           edges; now_ms is advanced from the measured 200 Hz loop interval. */
        now_ms += (uint32_t)(dt_s * 1000.0f + 0.5f);
        range_backend_service(now_ms);
        g_yaw_rate_debug_dps = gz - gyro_cal_yaw_bias_dps(&gyro_cal);
        if (!isfinite(g_yaw_rate_debug_dps)) {
            g_ai_action_debug = (uint32_t)AI_ACTION_STOP;
            g_ai_safety_override_debug = 1U;
            g_ai_confidence_debug = 0.0f;
            (void)balance_safety_update(&safety, NAN, pressed, false);
            motors_off();
            indicator(GPIO_PIN_1);
            continue;
        }
        angle = tilt_estimator_update(&estimator, ay, az, gx, dt_s);
        balance_armed = balance_safety_update(&safety, angle, pressed, imu_ok);

        if (!balance_armed) {
            g_ai_action_debug = (uint32_t)AI_ACTION_STOP;
            g_ai_confidence_debug = 0.0f;
            g_ai_safety_override_debug = 0U;
            balance_controller_reset(&controller);
            motors_off();
            indicator(safety.mode == BALANCE_FAULT ? GPIO_PIN_1 : GPIO_PIN_2);
            continue;
        }

        /* Preserve the balance path's priority: compute and issue its output
           before the lower-priority TinyML shadow inference is executed. */
        output = balance_controller_update(
            &controller, angle, gx - estimator.gyro_bias_dps,
            dt_s);
        if (!isfinite(output)) {
            g_ai_action_debug = (uint32_t)AI_ACTION_STOP;
            g_ai_confidence_debug = 0.0f;
            g_ai_safety_override_debug = 1U;
            (void)balance_safety_update(&safety, NAN, pressed, false);
            motors_off();
            indicator(GPIO_PIN_1);
            continue;
        }
        motor_command(MOTOR_POLARITY * output);
        indicator(MOTOR_OUTPUT_ALLOWED ? GPIO_PIN_3 : GPIO_PIN_2);

        /* Run navigation at 50 Hz after the balance command is issued.
           Shadow decisions never alter the current or next motor command. */
        if (++ai_divider >= 4U) {
            ai_divider = 0U;
            range_backend_snapshot(now_ms, &range_snapshot);
            if (ai_sensor_frame_from_ranges(
                    &ai_frame, &range_snapshot, balance_armed,
                    g_yaw_rate_debug_dps)) {
                ai_command = ai_navigation_step(&ai_state, &ai_frame, now_ms);
                g_ai_action_debug = (uint32_t)ai_command.action;
                g_ai_confidence_debug = ai_command.confidence;
                g_ai_safety_override_debug =
                    ai_command.safety_override ? 1U : 0U;
            } else {
                g_ai_action_debug = (uint32_t)AI_ACTION_STOP;
                g_ai_safety_override_debug = 1U;
                g_ai_confidence_debug = 0.0f;
            }
        }
    }
}
