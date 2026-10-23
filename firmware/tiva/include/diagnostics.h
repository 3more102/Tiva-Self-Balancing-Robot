#ifndef TIVA_BALANCE_DIAGNOSTICS_H
#define TIVA_BALANCE_DIAGNOSTICS_H

#include <stdbool.h>
#include <stdint.h>

/* Pure-C calibration and clock diagnostics: no TivaWare dependency.
 * The frame used is X along wheel axle, +Y forwards, +Z upwards.
 * All physical thresholds are conservative PROVISIONAL defaults:
 * measured noise and calibration repeatability must be verified on hardware. */
#define GYRO_CAL_SAMPLES 200U
#define GYRO_CAL_MAX_DPS 5.0f
#define GYRO_CAL_MAX_TILT_DEG 6.0f
#define GYRO_CAL_MAX_GRAVITY_ERROR_G 0.15f
#define GYRO_CAL_MAX_STD_DPS 0.75f

typedef struct {
    uint32_t valid_samples;
    uint32_t resets;
    float mean_dps;
    float m2_dps;
    bool complete;
} GyroCalibration;

void gyro_cal_init(GyroCalibration *cal);
bool gyro_cal_add(GyroCalibration *cal, float accel_y_g,
                  float accel_z_g, float gyro_x_dps, bool switch_released);
bool gyro_cal_ready(const GyroCalibration *cal);
float gyro_cal_bias_dps(const GyroCalibration *cal);

/* Timer1A free-running 32-bit DOWN counter, tick_hz = MCU SysCtlClockGet().
 * Unsigned subtraction handles wrap when elapsed < 2^32 ticks.
 * MIN/MAX periods are *provisional* and must be verified experimentally.
 * Returns false for a late/early tick and leaves dt_s=0 in that case. */
typedef struct {
    uint32_t last_counter;
    uint32_t min_ticks;
    uint32_t max_ticks;
    uint32_t tick_hz;
    uint32_t checked_ticks;
    uint32_t invalid_ticks;
} LoopTiming;

void loop_timing_init(LoopTiming *timer, uint32_t clock_hz,
                      uint32_t downcounter_value);
bool loop_timing_step(LoopTiming *timer, uint32_t downcounter_value, float *dt_s);

#endif
