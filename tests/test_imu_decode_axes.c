#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "imu_decode.h"

static void put_be16(uint8_t *raw, unsigned index, int16_t value)
{
    const uint16_t bits = (uint16_t)value;
    raw[index] = (uint8_t)(bits >> 8U);
    raw[index + 1U] = (uint8_t)bits;
}

int main(void)
{
    uint8_t raw[14] = {0};
    Mpu6050Sample sample;

    put_be16(raw, 0U, 8192);    /* +0.5 g X */
    put_be16(raw, 2U, -4096);   /* -0.25 g Y */
    put_be16(raw, 4U, 16384);   /* +1.0 g Z */
    put_be16(raw, 6U, 0);       /* 36.53 C */
    put_be16(raw, 8U, 1310);    /* +10 deg/s pitch gyro X */
    put_be16(raw, 10U, -655);   /* -5 deg/s gyro Y */
    put_be16(raw, 12U, 2620);   /* +20 deg/s yaw gyro Z */

    assert(mpu6050_decode_sample(raw, &sample));
    assert(fabsf(sample.accel_x_g - 0.5f) < 0.0001f);
    assert(fabsf(sample.accel_y_g + 0.25f) < 0.0001f);
    assert(fabsf(sample.accel_z_g - 1.0f) < 0.0001f);
    assert(fabsf(sample.temperature_c - 36.53f) < 0.001f);
    assert(fabsf(sample.gyro_x_dps - 10.0f) < 0.001f);
    assert(fabsf(sample.gyro_y_dps + 5.0f) < 0.001f);
    assert(fabsf(sample.gyro_z_dps - 20.0f) < 0.001f);
    assert(!mpu6050_decode_sample(NULL, &sample));
    assert(!mpu6050_decode_sample(raw, NULL));

    puts("PASS: MPU6050 signed-axis decoding and gyro-Z yaw mapping");
    return 0;
}
