#include "mpu6050.h"
#include <string.h>

volatile MPU6050_CalibrationReport mpu_calibration_report;

/* 写单个寄存器 */
static uint8_t write_reg(I2C_HandleTypeDef *h, uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(h, MPU6050_ADDR, reg, 1, &val, 1, 5);
}

/* 唤醒并配置量程。返回 0 成功, 非 0 失败(见错误码) */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t id = 0;
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &id, 1, 5) != HAL_OK)
        return 1;                       /* 总线无应答 */
    if (id != 0x68)
        return 2;                       /* 地址/接线不对 */

    if (write_reg(hi2c, MPU6050_PWR_MGMT_1, 0x00) != HAL_OK) return 3; /* 退出睡眠 */
    if (write_reg(hi2c, MPU6050_SMPLRT_DIV, 0x04) != HAL_OK) return 4; /* 1kHz/(4+1)=200Hz */
    if (write_reg(hi2c, MPU6050_CONFIG,       0x03) != HAL_OK) return 5; /* 低通滤波 */
    if (write_reg(hi2c, MPU6050_GYRO_CONFIG,  0x08) != HAL_OK) return 6; /* ±500°/s */
    if (write_reg(hi2c, MPU6050_ACCEL_CONFIG, 0x00) != HAL_OK) return 7; /* ±2g */
    return 0;
}

/* 连续读 14 字节(加速度+温度+陀螺), 转成物理量 */
uint8_t MPU6050_Read(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *d) {
    uint8_t buf[14];
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, buf, 14, 5) != HAL_OK)
        return 1;
    int16_t raw[7];
    for (int i = 0; i < 7; i++)
        raw[i] = (int16_t)((buf[2 * i] << 8) | buf[2 * i + 1]);

    d->ax = raw[0] / MPU6050_ACCEL_LSB;
    d->ay = raw[1] / MPU6050_ACCEL_LSB;
    d->az = raw[2] / MPU6050_ACCEL_LSB;
    d->gx = raw[4] / MPU6050_GYRO_LSB;   /* 注意: 陀螺数据从 raw[4] 开始 */
    d->gy = raw[5] / MPU6050_GYRO_LSB;
    d->gz = raw[6] / MPU6050_GYRO_LSB;
    return 0;
}

/* 静止时采样求零偏, 消除陀螺零点漂移 */
uint8_t MPU6050_CalibrateBias(I2C_HandleTypeDef *hi2c,
                           float *bgx, float *bgy, float *bgz, uint16_t n) {
    float sx = 0, sy = 0, sz = 0;
    MPU6050_Data_t d;
    uint16_t ok = 0;
    float low[3] = {500.0f, 500.0f, 500.0f};
    float high[3] = {-500.0f, -500.0f, -500.0f};
    uint8_t moving = 0u;
    memset((void *)&mpu_calibration_report, 0, sizeof mpu_calibration_report);
    mpu_calibration_report.requested = n;
    mpu_calibration_report.gravity_min = 4.0f;
    if (n < 20u) { mpu_calibration_report.flags = 1u; return 1u; }
    for (uint16_t i = 0; i < n; i++) {
        if (MPU6050_Read(hi2c, &d) == 0) {
            float rates[3] = {d.gx, d.gy, d.gz};
            float gravity = d.ax * d.ax + d.ay * d.ay + d.az * d.az;
            if (gravity < mpu_calibration_report.gravity_min) mpu_calibration_report.gravity_min = gravity;
            if (gravity > mpu_calibration_report.gravity_max) mpu_calibration_report.gravity_max = gravity;
            if (!(gravity >= 0.64f && gravity <= 1.44f)) {
                moving = 1u; ++mpu_calibration_report.gravity_failures;
            }
            for (unsigned axis = 0; axis < 3u; ++axis) {
                if (!(rates[axis] >= -10.0f && rates[axis] <= 10.0f)) {
                    moving = 1u; ++mpu_calibration_report.rate_failures;
                }
                if (rates[axis] < low[axis]) low[axis] = rates[axis];
                if (rates[axis] > high[axis]) high[axis] = rates[axis];
            }
            sx += d.gx; sy += d.gy; sz += d.gz; ok++;
        } else ++mpu_calibration_report.io_failures;
        HAL_Delay(5);
    }
    mpu_calibration_report.samples = ok;
    if ((uint32_t)ok * 10u < (uint32_t)n * 9u) mpu_calibration_report.flags |= 2u;
    if (mpu_calibration_report.gravity_failures) mpu_calibration_report.flags |= 4u;
    if (mpu_calibration_report.rate_failures) mpu_calibration_report.flags |= 8u;
    for (unsigned axis = 0; axis < 3u; ++axis) {
        mpu_calibration_report.low[axis] = low[axis];
        mpu_calibration_report.high[axis] = high[axis];
        if (high[axis] - low[axis] > 2.0f) mpu_calibration_report.flags |= 16u;
    }
    if ((uint32_t)ok * 10u < (uint32_t)n * 9u || moving || mpu_calibration_report.flags) return 1u;
    *bgx = sx / ok;
    *bgy = sy / ok;
    *bgz = sz / ok;
    return 0u;
}
