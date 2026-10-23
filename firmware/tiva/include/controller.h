#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdbool.h>

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

void tilt_estimator_init(TiltEstimator *est, float time_constant_s, float gyro_bias_dps);
float tilt_estimator_update(TiltEstimator *est, float accel_y_g,
                            float accel_z_g, float gyro_x_dps, float dt_s);
void balance_controller_init(BalanceController *pid, float kp, float ki, float kd);
void balance_controller_reset(BalanceController *pid);
float balance_controller_update(BalanceController *pid, float angle_deg,
                                float gyro_x_dps, float dt_s);
bool balance_is_safe(float angle_deg, bool enable_pressed, bool sensor_ok);

#endif
