#include "mpu6050.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static unsigned reads;
static int missing, failures, moving, vibration, no_gravity, write_error, bad_id;
uint32_t HAL_GetTick(void) { return 0; }
void HAL_Delay(uint32_t ms) { (void)ms; }
int HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t addr, uint16_t reg, uint16_t width,
                     uint8_t *data, uint16_t n, uint32_t timeout)
{
    (void)h; (void)width; assert(addr == MPU6050_ADDR && timeout == 5);
    if (reg == MPU6050_WHO_AM_I) { *data = bad_id ? 0 : 0x68; return missing ? HAL_ERROR : HAL_OK; }
    assert(reg == MPU6050_ACCEL_XOUT_H && n == 14);
    ++reads;
    if (missing || (int)reads <= failures) return HAL_ERROR;
    int16_t raw[7] = {0, 0, 16384, 0, 65, 131, -65};
    if (moving) raw[5] = 800;
    if (vibration) raw[5] = reads % 2 ? 131 : -131;
    if (no_gravity) raw[2] = 0;
    for (unsigned i = 0; i < 7; ++i) {
        data[2*i] = (uint8_t)((uint16_t)raw[i] >> 8);
        data[2*i+1] = (uint8_t)raw[i];
    }
    return HAL_OK;
}
int HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t addr, uint16_t reg, uint16_t width,
                      uint8_t *data, uint16_t n, uint32_t timeout)
{ (void)h; (void)addr; (void)reg; (void)width; (void)data; (void)n; (void)timeout; return write_error ? HAL_ERROR : HAL_OK; }
int main(int argc, char **argv)
{
    I2C_HandleTypeDef bus = 0;
    float x = 99, y = 99, z = 99;
    assert(argc == 2);
    if (!strcmp(argv[1], "missing")) missing = 1;
    else if (!strcmp(argv[1], "partial")) failures = 3;
    else if (!strcmp(argv[1], "boundary")) failures = 2;
    else if (!strcmp(argv[1], "moving")) moving = 1;
    else if (!strcmp(argv[1], "vibration")) vibration = 1;
    else if (!strcmp(argv[1], "gravity")) no_gravity = 1;
    else if (!strcmp(argv[1], "init")) {
        MPU6050_Data_t mapping = {0.0f, 0.0f, 0.0f, 11.0f, 22.0f, 33.0f};
        assert(MPU_YAW_AXIS == 0u && MPU_YAW_SIGN == 1 && MPU6050_RawYaw(&mapping) == 11);
        assert(MPU6050_Init(&bus) == 0);
        bad_id = 1; assert(MPU6050_Init(&bus) == 2);
        bad_id = 0; write_error = 1; assert(MPU6050_Init(&bus) == 3);
        puts("mpu_init: PASS"); return 0;
    } else assert(!strcmp(argv[1], "valid") || !strcmp(argv[1], "too_short"));
    uint16_t n = !strcmp(argv[1], "too_short") ? 0 : 20;
    uint8_t result = MPU6050_CalibrateBias(&bus, &x, &y, &z, n);
    assert(sizeof(MPU6050_CalibrationReport) == 56 && mpu_calibration_report.requested == n);
    if (!strcmp(argv[1], "valid") || !strcmp(argv[1], "boundary")) {
        assert(result == 0 && fabsf(x - 65.0f/65.5f) < .0001f && fabsf(y - 2) < .0001f);
        assert(z < 0);
        assert(mpu_calibration_report.flags == 0);
    } else assert(result != 0 && x == 99 && y == 99 && z == 99);
    if (moving) assert(mpu_calibration_report.flags & 8u);
    if (vibration) assert(mpu_calibration_report.flags & 16u);
    if (no_gravity) assert(mpu_calibration_report.flags & 4u);
    if (failures == 3) assert(mpu_calibration_report.flags & 2u);
    printf("mpu_%s: PASS\n", argv[1]); return 0;
}
