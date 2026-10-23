#include "controller.h"
#include <math.h>

#define DEG_PER_RAD 57.29577951308232f

static float clampf(float x, float low, float high)
{
    if (x < low) return low;
    if (x > high) return high;
    return x;
}

void tilt_estimator_init(TiltEstimator *est, float time_constant_s, float gyro_bias_dps)
{
    est->angle_deg = 0.0f;
    est->gyro_bias_dps = isfinite(gyro_bias_dps) ? gyro_bias_dps : 0.0f;
    est->time_constant_s = isfinite(time_constant_s) && time_constant_s > 0.0f
                         ? time_constant_s : 0.5f;
    est->initialized = false;
}

float tilt_estimator_update(TiltEstimator *est, float ay, float az,
                            float gyro_x_dps, float dt_s)
{
    if (!isfinite(ay) || !isfinite(az) || !isfinite(gyro_x_dps) ||
        !isfinite(dt_s) || dt_s <= 0.0f || dt_s > BALANCE_MAX_DT_S ||
        ay * ay + az * az < 0.04f || ay * ay + az * az > 4.0f) {
        return NAN; /* Unusable accel direction or missed control deadline. */
    }
    const float accel_deg = atan2f(ay, az) * DEG_PER_RAD;
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
    pid->integral_limit = 20.0f; /* deg*s */
    pid->output_limit = 0.7f; /* fraction of available PWM command */
    pid->integral = 0.0f;
}

void balance_controller_reset(BalanceController *pid) { pid->integral = 0.0f; }

float balance_controller_update(BalanceController *pid, float angle_deg,
                                float gyro_x_dps, float dt_s)
{
    if (!isfinite(angle_deg) || !isfinite(gyro_x_dps) ||
        !isfinite(dt_s) || dt_s <= 0.0f || dt_s > BALANCE_MAX_DT_S ||
        !isfinite(pid->kp) || !isfinite(pid->ki) || !isfinite(pid->kd) ||
        !isfinite(pid->integral)) return NAN;

    /* Derivative-on-measurement suppresses setpoint derivative kick.
     * Sign and effective motor torque depend on physical installation. */
    const float error = -angle_deg;
    const float proposed = clampf(pid->integral + error * dt_s,
                                  -pid->integral_limit, pid->integral_limit);
    const float pd = pid->kp * error - pid->kd * gyro_x_dps;
    const float requested = pd + pid->ki * proposed;
    /* Integrate if unsaturated or if error would drive it out of saturation. */
    if (fabsf(requested) <= pid->output_limit ||
        (requested > pid->output_limit && error < 0.0f) ||
        (requested < -pid->output_limit && error > 0.0f)) {
        pid->integral = proposed;
    }
    return clampf(pd + pid->ki * pid->integral,
                  -pid->output_limit, pid->output_limit);
}

bool balance_is_safe(float angle_deg, bool enable_pressed, bool sensor_ok)
{
    return sensor_ok && enable_pressed && isfinite(angle_deg) &&
           fabsf(angle_deg) < BALANCE_TRIP_ANGLE_DEG;
}

void balance_safety_init(BalanceSafety *safety)
{
    safety->mode = BALANCE_DISARMED;
    safety->upright_samples = 0U;
    safety->release_seen = false;
}

bool balance_safety_update(BalanceSafety *safety, float angle_deg,
                           bool enable_pressed, bool sensor_ok)
{
    if (!sensor_ok || !isfinite(angle_deg)) {
        safety->mode = BALANCE_FAULT;
        safety->upright_samples = 0U;
        return false;
    }

    if (!enable_pressed) {
        safety->mode = BALANCE_DISARMED;
        safety->upright_samples = 0U;
        safety->release_seen = true;
        return false;
    }

    if (safety->mode == BALANCE_FAULT) return false;

    if (fabsf(angle_deg) >= BALANCE_TRIP_ANGLE_DEG) {
        safety->mode = BALANCE_FAULT;
        safety->upright_samples = 0U;
        return false;
    }

    if (safety->mode == BALANCE_ARMED) return true;

    if (!safety->release_seen || fabsf(angle_deg) > BALANCE_ARM_ANGLE_DEG) {
        safety->mode = BALANCE_DISARMED;
        safety->upright_samples = 0U;
        return false;
    }
    if (safety->upright_samples < BALANCE_ARM_SAMPLE_COUNT)
        ++safety->upright_samples;

    if (safety->upright_samples >= BALANCE_ARM_SAMPLE_COUNT) {
        safety->mode = BALANCE_ARMED;
        return true;
    }
    safety->mode = BALANCE_ARMING;
    return false;
}
