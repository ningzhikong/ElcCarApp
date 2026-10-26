#ifndef CAR_IMU_DIAGNOSTIC_H
#define CAR_IMU_DIAGNOSTIC_H
#include "mpu6050.h"

/* Motor-disabled axis/sign check. No key or serial command enables motion. */
typedef struct {
    uint32_t magic, sequence, state, error, tick_ms, samples, period_ms;
    float gyro[3], angle[3], bias[3], accel[3];
} ImuDiagnostic;
extern volatile ImuDiagnostic imu_diagnostic;
void ImuDiagnostic_Init(I2C_HandleTypeDef *bus);
void ImuDiagnostic_Poll(I2C_HandleTypeDef *bus);
#endif
