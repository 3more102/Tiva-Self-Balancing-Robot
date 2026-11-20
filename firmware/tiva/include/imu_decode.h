#ifndef TIVA_IMU_DECODE_H
#define TIVA_IMU_DECODE_H

#include <stdbool.h>
#include <stdint.h>

/* MPU6050 sample decoded for the configured +/-2 g and +/-250 deg/s ranges.
 * Robot frame documented in hardware/wiring.md:
 * +X along the axle, +Y forward, +Z upward.
 * Therefore gyro X is the pitch-rate input and gyro Z is the yaw-rate input.
 */
typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float temperature_c;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} Mpu6050Sample;

/* Decode the 14-byte burst beginning at ACCEL_XOUT_H. */
bool mpu6050_decode_sample(const uint8_t raw[14], Mpu6050Sample *sample);

#endif /* TIVA_IMU_DECODE_H */
