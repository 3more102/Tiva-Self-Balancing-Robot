#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/* All angular values use degrees and degrees per second.
 * Actuator command is normalized to [-output_limit, +output_limit].
 * No particular motor direction is implied by this portable module. */
#define BALANCE_TRIP_ANGLE_DEG 25.0f
#define BALANCE_ARM_ANGLE_DEG 8.0f
#define BALANCE_ARM_SAMPLE_COUNT 40U /* 200 ms at the planned 200 Hz rate */
#define BALANCE_MAX_DT_S 0.020f

typedef struct {
    float angle_deg;
    float gyro_bias_dps;
    float time_constant_s;
    bool initialized;
} TiltEstimator;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float integral_limit;
    float output_limit;
} BalanceController;

typedef enum {
    BALANCE_DISARMED = 0,
    BALANCE_ARMING = 1,
    BALANCE_ARMED = 2,
    BALANCE_FAULT = 3
} BalanceMode;

typedef struct {
    BalanceMode mode;
    uint16_t upright_samples;
    bool release_seen;
} BalanceSafety;

/* Invalid samples and invalid dt return NAN, which must trigger safe shutdown. */
void tilt_estimator_init(TiltEstimator *est, float time_constant_s, float gyro_bias_dps);
float tilt_estimator_update(TiltEstimator *est, float accel_y_g,
                            float accel_z_g, float gyro_x_dps, float dt_s);
void balance_controller_init(BalanceController *pid, float kp, float ki, float kd);
void balance_controller_reset(BalanceController *pid);
float balance_controller_update(BalanceController *pid, float angle_deg,
                                float gyro_x_dps, float dt_s);
bool balance_is_safe(float angle_deg, bool enable_pressed, bool sensor_ok);

/* Arm only after a release, then 40 consecutive upright samples.
 * A fault cannot rearm while the button is held.
 * If sensor_ok becomes false, this state machine returns disabled; the
 * application additionally latches fatal sensor faults until hardware reset. */
void balance_safety_init(BalanceSafety *safety);
bool balance_safety_update(BalanceSafety *safety, float angle_deg,
                           bool enable_pressed, bool sensor_ok);

#endif
