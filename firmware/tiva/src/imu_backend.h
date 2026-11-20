/* Internal implementation included only by main.c. Requires TivaWare includes/macros. */
#include <stddef.h>
#include "imu_decode.h"
static bool i2c_finish(uint32_t command)
{
    uint32_t tries = I2C_TIMEOUT_LOOPS;
    I2CMasterControl(I2C0_BASE, command);
    while (I2CMasterBusy(I2C0_BASE)) {
        if (--tries == 0U) return false;
    }
    return I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE;
}

static bool mpu_write(uint8_t reg, uint8_t value)
{
    I2CMasterSlaveAddrSet(I2C0_BASE, MPU6050_ADDRESS, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    if (!i2c_finish(I2C_MASTER_CMD_BURST_SEND_START)) return false;
    I2CMasterDataPut(I2C0_BASE, value);
    return i2c_finish(I2C_MASTER_CMD_BURST_SEND_FINISH);
}

static bool mpu_read(uint8_t reg, uint8_t *buffer, uint32_t n)
{
    uint32_t i;
    if (n == 0U || buffer == 0) return false;
    I2CMasterSlaveAddrSet(I2C0_BASE, MPU6050_ADDRESS, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    if (!i2c_finish(I2C_MASTER_CMD_SINGLE_SEND)) return false;
    I2CMasterSlaveAddrSet(I2C0_BASE, MPU6050_ADDRESS, true);
    for (i = 0U; i < n; ++i) {
        uint32_t cmd = (n == 1U) ? I2C_MASTER_CMD_SINGLE_RECEIVE :
                      (i == 0U ? I2C_MASTER_CMD_BURST_RECEIVE_START :
                      (i == n - 1U ? I2C_MASTER_CMD_BURST_RECEIVE_FINISH :
                                    I2C_MASTER_CMD_BURST_RECEIVE_CONT));
        if (!i2c_finish(cmd)) return false;
        buffer[i] = (uint8_t)I2CMasterDataGet(I2C0_BASE);
    }
    return true;
}

/* +X axis parallel to axle, +Y forward and +Z up when robot upright.
 * The portable decoder keeps signed big-endian parsing host-testable. */
static bool mpu_sample(float *ay_g, float *az_g, float *gx_dps,
                       float *gz_dps)
{
    uint8_t bytes[14];
    Mpu6050Sample sample;

    if (ay_g == NULL || az_g == NULL || gx_dps == NULL || gz_dps == NULL)
        return false;
    if (!mpu_read(MPU6050_ACCEL_XOUT_H, bytes, 14U) ||
        !mpu6050_decode_sample(bytes, &sample))
        return false;

    *ay_g = sample.accel_y_g;
    *az_g = sample.accel_z_g;
    *gx_dps = sample.gyro_x_dps; /* pitch rate */
    *gz_dps = sample.gyro_z_dps; /* vertical-axis yaw rate */
    return true;
}

static bool imu_init(void)
{
    uint8_t who = 0U;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    wait_for_peripheral(SYSCTL_PERIPH_GPIOB);
    wait_for_peripheral(SYSCTL_PERIPH_I2C0);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
    if (!mpu_read(MPU6050_WHO_AM_I, &who, 1U) || who != 0x68U) return false;
    if (!mpu_write(MPU6050_PWR_MGMT_1, 0x01U)) return false;
    SysCtlDelay(SysCtlClockGet() / 300U);
    return mpu_write(MPU6050_CONFIG, 0x03U) &&
           mpu_write(MPU6050_SMPLRT_DIV, 0x04U) &&
           mpu_write(MPU6050_GYRO_CONFIG, 0x00U) &&
           mpu_write(MPU6050_ACCEL_CONFIG, 0x00U);
}
