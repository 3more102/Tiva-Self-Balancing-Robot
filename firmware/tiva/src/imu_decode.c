#include "imu_decode.h"

#include <math.h>
#include <stddef.h>

static int16_t signed_be16(const uint8_t *bytes)
{
    int32_t value = ((int32_t)bytes[0] << 8) | (int32_t)bytes[1];
    if ((value & 0x8000) != 0) value -= 0x10000;
    return (int16_t)value;
}

bool mpu6050_decode_sample(const uint8_t raw[14], Mpu6050Sample *sample)
{
    if (raw == NULL || sample == NULL) return false;

    sample->accel_x_g = (float)signed_be16(&raw[0]) / 16384.0f;
    sample->accel_y_g = (float)signed_be16(&raw[2]) / 16384.0f;
    sample->accel_z_g = (float)signed_be16(&raw[4]) / 16384.0f;
    sample->temperature_c =
        ((float)signed_be16(&raw[6]) / 340.0f) + 36.53f;
    sample->gyro_x_dps = (float)signed_be16(&raw[8]) / 131.0f;
    sample->gyro_y_dps = (float)signed_be16(&raw[10]) / 131.0f;
    sample->gyro_z_dps = (float)signed_be16(&raw[12]) / 131.0f;

    return isfinite(sample->accel_x_g) && isfinite(sample->accel_y_g) &&
           isfinite(sample->accel_z_g) && isfinite(sample->temperature_c) &&
           isfinite(sample->gyro_x_dps) && isfinite(sample->gyro_y_dps) &&
           isfinite(sample->gyro_z_dps);
}
