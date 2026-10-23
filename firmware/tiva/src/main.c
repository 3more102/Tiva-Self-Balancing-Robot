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

static void control_timer_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    wait_for_peripheral(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / CONTROL_HZ - 1U);
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

static void await_tick(void)
{
    while ((TimerIntStatus(TIMER0_BASE, false) & TIMER_TIMA_TIMEOUT) == 0U) { }
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

int main(void)
{
    float ay, az, gx;
    float bias_sum = 0.0f;
    uint32_t i;
    bool imu_ok;
    BalanceSafety safety;
    TiltEstimator estimator;
    BalanceController controller;
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ); /* 80 MHz */
    FPUEnable();
    FPULazyStackingEnable();
    outputs_init();
    motors_off();
    indicator(GPIO_PIN_2); /* blue: safe/off */
    imu_ok = imu_init();
    control_timer_init();
    if (imu_ok) {
        /* Keep robot stationary for first 200 gyro calibration samples. */
        for (i = 0U; i < 200U; ++i) {
            await_tick();
            if (!mpu_sample(&ay, &az, &gx)) { imu_ok = false; break; }
            bias_sum += gx;
        }
    }
    balance_safety_init(&safety);
    tilt_estimator_init(&estimator, 0.5f, bias_sum / 200.0f);
    balance_controller_init(&controller, 0.016f, 0.0f, 0.0009f);

    for (;;) {
        bool pressed;
        float angle, output;
        await_tick();
        pressed = (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) & GPIO_PIN_4) == 0U;
        if (!imu_ok || !mpu_sample(&ay, &az, &gx)) {
            imu_ok = false; /* Fatal sensor fault requires a hardware reset. */
            (void)balance_safety_update(&safety, NAN, pressed, false);
            motors_off();
            indicator(GPIO_PIN_1);
            continue;
        }
        angle = tilt_estimator_update(&estimator, ay, az, gx, 1.0f / CONTROL_HZ);
        if (!balance_safety_update(&safety, angle, pressed, imu_ok)) {
            balance_controller_reset(&controller);
            motors_off();
            indicator(safety.mode == BALANCE_FAULT ? GPIO_PIN_1 : GPIO_PIN_2);
            continue;
        }
        output = balance_controller_update(
            &controller, angle, gx - estimator.gyro_bias_dps,
            1.0f / CONTROL_HZ);
        if (!isfinite(output)) {
            (void)balance_safety_update(&safety, NAN, pressed, false);
            motors_off();
            indicator(GPIO_PIN_1);
            continue;
        }
        motor_command(MOTOR_POLARITY * output);
        indicator(MOTOR_OUTPUT_ALLOWED ? GPIO_PIN_3 : GPIO_PIN_2);
    }
}
