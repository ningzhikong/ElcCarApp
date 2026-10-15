#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"

/* HAL 的 DevAddress 参数使用左移一位的地址；AD0=0 时 7-bit 地址为 0x68。 */
#define MPU6050_ADDR         (0x68U << 1)
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B

/* 量程: 陀螺 ±500°/s -> 65.5 LSB/(°/s); 加速度 ±2g -> 16384 LSB/g */
#define MPU6050_GYRO_LSB   65.5f
#define MPU6050_ACCEL_LSB 16384.0f

/* 竖放(绕 X 轴转 90°)时, 偏航(yaw)对应芯片 Y 轴。
   若实车发现转向方向反了, 把这里改成 -1.0f 即可翻转。 */
#define MPU_YAW_SIGN  (+1.0f)

typedef struct {
    float ax, ay, az;   /* 单位 g */
    float gx, gy, gz;   /* 单位 °/s */
} MPU6050_Data_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
uint8_t MPU6050_Read(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *d);
void    MPU6050_CalibrateBias(I2C_HandleTypeDef *hi2c,
                              float *bgx, float *bgy, float *bgz, uint16_t n);

#endif
