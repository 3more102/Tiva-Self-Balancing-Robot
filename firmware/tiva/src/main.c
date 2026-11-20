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
#include "driverlib/watchdog.h"
#include "controller.h"
#include "diagnostics.h"
#include "navigation.h"
#include "ai_navigation.h"
#include "ai_range_adapter.h"

#define MOTOR_OUTPUT_ALLOWED 0  /* MUST remain disabled before hardware qualification. */
#define MOTOR_POLARITY (-1.0f)
#define LEFT_MOTOR_POLARITY 1
#define RIGHT_MOTOR_POLARITY (-1)

#define CONTROL_HZ 200U
#define PWM_HZ 20000U
#define WATCHDOG_TIMEOUT_MS 50U
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

/* CCS watch expressions for shadow-mode AI diagnostics. */
static volatile uint32_t g_ai_action_debug = (uint32_t)AI_ACTION_STOP;
static volatile uint32_t g_ai_safety_override_debug = 0U;
static volatile float g_ai_confidence_debug = 0.0f;
static volatile float g_yaw_rate_debug_dps = 0.0f;

static void wait_for_peripheral(uint32_t id)
{
    while (!SysCtlPeripheralReady(id)) { }
}

#include "imu_backend.h"
#include "motor_backend.h"

/* WDOG0 uses the system clock: first timeout sets a flag; a second uncleared
   timeout resets the MCU. At 80 MHz, 4,000,000 counts is nominally 50 ms. */
static void watchdog_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
    wait_for_peripheral(SYSCTL_PERIPH_WDOG0);
    if (WatchdogLockState(WATCHDOG0_BASE)) {
        WatchdogUnlock(WATCHDOG0_BASE);
    }
    WatchdogReloadSet(WATCHDOG0_BASE,
                      (SysCtlClockGet() / 1000U) * WATCHDOG_TIMEOUT_MS);
    WatchdogResetEnable(WATCHDOG0_BASE);
    WatchdogStallEnable(WATCHDOG0_BASE);
    WatchdogEnable(WATCHDOG0_BASE);
}

/* Clearing a pending first-timeout flag reloads the watchdog counter. */
static void watchdog_service(void)
{
    if (WatchdogIntStatus(WATCHDOG0_BASE, false) != 0U) {
        WatchdogIntClear(WATCHDOG0_BASE);
    }
}

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
    float navigation_ms_remainder = 0.0f;
    uint32_t navigation_now_ms = 0U;
    uint8_t ai_divider = 0U;
    float yaw_rate_dps = 0.0f;
    GyroCalibration gyro_cal;
    LoopTiming timing;
    bool imu_ok;
    BalanceSafety safety;
    TiltEstimator estimator;
    BalanceController controller;
    Navigation navigation;
    NavigationInput navigation_input = {0};
    NavigationCommand navigation_command;
    AiNavigationState ai_state;
    AiSensorFrame ai_frame;
    AiNavigationCommand ai_command;

    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ); /* 80 MHz */
    FPUEnable();
    FPULazyStackingEnable();
    outputs_init();
    motors_off();
    indicator(GPIO_PIN_2); /* blue: safe/off */
    watchdog_init();
    imu_ok = imu_init();
    watchdog_service();
    control_timer_init();
    watchdog_service();
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
            watchdog_service();
        }
        if (!gyro_cal_ready(&gyro_cal)) imu_ok = false;
    }

    balance_safety_init(&safety);
    tilt_estimator_init(&estimator, 0.5f,
                        imu_ok ? gyro_cal_bias_dps(&gyro_cal) : 0.0f);
    loop_timing_init(&timing, SysCtlClockGet(),
                     TimerValueGet(TIMER1_BASE, TIMER_A));
    balance_controller_init(&controller, 0.016f, 0.0f, 0.0009f);

    navigation_init(&navigation);
            ai_navigation_init(&ai_state);
    /* Hard-disabled until range acquisition and a separate operator enable
       input have both been implemented and verified. */
    navigation_input.autonomy_enabled = false;

    for (;;) {
        bool pressed;
        float angle, output, pitch_target_deg;

        await_tick();
        /* Actual elapsed time, not assumed constant Ts. A missed/abnormal
           sample deadline cuts motor output via the software safety gate. */
        const bool on_time = loop_timing_step(
            &timing, TimerValueGet(TIMER1_BASE, TIMER_A), &dt_s);
        pressed = (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) & GPIO_PIN_4) == 0U;

        if (!on_time) {
            (void)balance_safety_update(&safety, NAN, pressed, false);
            balance_controller_reset(&controller);
            navigation_init(&navigation);
            ai_navigation_init(&ai_state);
            motors_off();
            indicator(GPIO_PIN_1);
            watchdog_service();
            continue;
        }

        if (!imu_ok || !mpu_sample(&ay, &az, &gx, &gz)) {
            imu_ok = false; /* Fatal sensor fault requires a hardware reset. */
            (void)balance_safety_update(&safety, NAN, pressed, false);
            navigation_init(&navigation);
            ai_navigation_init(&ai_state);
            motors_off();
            indicator(GPIO_PIN_1);
            watchdog_service();
            continue;
        }

        yaw_rate_dps = gz - gyro_cal_yaw_bias_dps(&gyro_cal);
        g_yaw_rate_debug_dps = yaw_rate_dps;
        angle = tilt_estimator_update(&estimator, ay, az, gx, dt_s);
        if (!balance_safety_update(&safety, angle, pressed, imu_ok)) {
            balance_controller_reset(&controller);
            navigation_init(&navigation);
            ai_navigation_init(&ai_state);
            motors_off();
            indicator(safety.mode == BALANCE_FAULT ? GPIO_PIN_1 : GPIO_PIN_2);
            watchdog_service();
            continue;
        }

        /* Build a wrap-safe millisecond clock from the measured control dt. */
        navigation_ms_remainder += dt_s * 1000.0f;
        if (navigation_ms_remainder >= 1.0f) {
            const uint32_t elapsed_ms = (uint32_t)navigation_ms_remainder;
            navigation_now_ms += elapsed_ms;
            navigation_ms_remainder -= (float)elapsed_ms;
        }

        navigation_command = navigation_update(
            &navigation, &navigation_input, navigation_now_ms);
        pitch_target_deg = navigation_command.travel_permitted
                         ? navigation_command.pitch_target_deg : 0.0f;
        output = balance_controller_update_target(
            &controller, angle, gx - estimator.gyro_bias_dps,
            pitch_target_deg, dt_s);

        if (!isfinite(output)) {
            (void)balance_safety_update(&safety, NAN, pressed, false);
            balance_controller_reset(&controller);
            navigation_init(&navigation);
            ai_navigation_init(&ai_state);
            motors_off();
            indicator(GPIO_PIN_1);
            watchdog_service();
            continue;
        }

        motor_command_differential(
            MOTOR_POLARITY * output,
            navigation_command.travel_permitted ?
                navigation_command.yaw_bias : 0.0f);
        indicator(MOTOR_OUTPUT_ALLOWED ? GPIO_PIN_3 : GPIO_PIN_2);

        /* Lower-priority 50 Hz AI inference runs after the balance command.
           It consumes the same range contract as the rule-based policy and
           remains diagnostic-only: no AI output changes motor PWM or targets. */
        if (++ai_divider >= 4U) {
            ai_divider = 0U;
            if (ai_sensor_frame_from_navigation(&ai_frame, &navigation_input,
                                                yaw_rate_dps)) {
                ai_command = ai_navigation_step(&ai_state, &ai_frame,
                                                navigation_now_ms);
                g_ai_action_debug = (uint32_t)ai_command.action;
                g_ai_confidence_debug = ai_command.confidence;
                g_ai_safety_override_debug =
                    ai_command.safety_override ? 1U : 0U;
            } else {
                g_ai_action_debug = (uint32_t)AI_ACTION_STOP;
                g_ai_confidence_debug = 0.0f;
                g_ai_safety_override_debug = 1U;
            }
        }
        watchdog_service();
    }
}
