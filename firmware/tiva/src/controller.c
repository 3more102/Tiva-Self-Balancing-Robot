#include "controller.h"
#include <math.h>

#define DEG_PER_RAD 57.29577951308232f
#define SAFE_ANGLE_DEG 25.0f

static float clampf(float x, float min_value, float max_value)
{
    if (x < min_value) return min_value;
    if (x > max_value) return max_value;
    return x;
}

void tilt_estimator_init(TiltEstimator *est, float time_constant_s, float gyro_bias_dps)
{
    est->angle_deg = 0.0f;
    est->gyro_bias_dps = gyro_bias_dps;
    est->time_constant_s = time_constant_s > 0.0f ? time_constant_s : 0.5f;
    est->initialized = false;
}

float tilt_estimator_update(TiltEstimator *est, float accel_y_g,
                            float accel_z_g, float gyro_x_dps, float dt_s)
{
    if (!isfinite(accel_y_g) || !isfinite(accel_z_g) ||
        !isfinite(gyro_x_dps) || !isfinite(dt_s) || dt_s <= 0.0f) {
        return NAN;
    }
    const float accel_deg = atan2f(accel_y_g, accel_z_g) * DEG_PER_RAD;
    if (!est->initialized) {
        est->angle_deg = accel_deg;
        est->initialized = true;
        return est->angle_deg;
    }
    const float alpha = est->time_constant_s / (est->time_constant_s + dt_s);
    const float gyro_deg_s = gyro_x_dps - est->gyro_bias_dps;
    est->angle_deg = alpha * (est->angle_deg + gyro_deg_s * dt_s) +
                     (1.0f - alpha) * accel_deg;
    return est->angle_deg;
}

void balance_controller_init(BalanceController *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = 20.0f;
    pid->output_limit = 0.7f;
    pid->integral = 0.0f;
}

void balance_controller_reset(BalanceController *pid)
{
    pid->integral = 0.0f;
}

float balance_controller_update(BalanceController *pid, float angle_deg,
                                float gyro_x_dps, float dt_s)
{
    if (!isfinite(angle_deg) || !isfinite(gyro_x_dps) ||
        !isfinite(dt_s) || dt_s <= 0.0f) return NAN;
    const float error = -angle_deg;
    const float previous_integral = pid->integral;
    const float proposed_integral = clampf(previous_integral + error * dt_s,
                                            -pid->integral_limit,
                                            pid->integral_limit);
    const float pd = pid->kp * error - pid->kd * gyro_x_dps;
    const float requested = pd + pid->ki * proposed_integral;
    /* Conditional integration: do not wind up against a saturated actuator. */
    if (fabsf(requested) <= pid->output_limit ||
        (requested > pid->output_limit && error < 0.0f) ||
        (requested < -pid->output_limit && error > 0.0f)) {
        pid->integral = proposed_integral;
    }
    return clampf(pd + pid->ki * pid->integral,
                  -pid->output_limit, pid->output_limit);
}

bool balance_is_safe(float angle_deg, bool enable_pressed, bool sensor_ok)
{
    return sensor_ok && enable_pressed && isfinite(angle_deg) &&
           fabsf(angle_deg) < SAFE_ANGLE_DEG;
}
