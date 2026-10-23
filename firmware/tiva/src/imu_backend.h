/* Internal implementation included only by main.c. Requires TivaWare includes/macros. */
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

static int16_t i16_be(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[0] << 8U) | bytes[1]);
}

/* +X axis parallel to axle, +Y forward and +Z up when robot upright. */
static bool mpu_sample(float *ay_g, float *az_g, float *gx_dps)
{
    uint8_t bytes[14];
    if (!mpu_read(MPU6050_ACCEL_XOUT_H, bytes, 14U)) return false;
    *ay_g = (float)i16_be(&bytes[2]) / 16384.0f;
    *az_g = (float)i16_be(&bytes[4]) / 16384.0f;
    *gx_dps = (float)i16_be(&bytes[8]) / 131.0f;
    return isfinite(*ay_g) && isfinite(*az_g) && isfinite(*gx_dps);
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
