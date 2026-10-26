#include "imu_diagnostic.h"
#include "control.h"
#include <string.h>

volatile ImuDiagnostic imu_diagnostic;
static uint32_t previous_tick;
static float previous_rate[3];

static void failed(uint32_t error)
{
    ++imu_diagnostic.sequence;
    imu_diagnostic.state = 3u;
    imu_diagnostic.error = error;
    ++imu_diagnostic.sequence;
    Control_Stop();
}

void ImuDiagnostic_Init(I2C_HandleTypeDef *bus)
{
    float bias[3];
    Control_Stop();
    memset((void *)&imu_diagnostic, 0, sizeof imu_diagnostic);
    memset(previous_rate, 0, sizeof previous_rate);
    imu_diagnostic.magic = 0x494d5531u;
    imu_diagnostic.state = 1u;
    if (MPU6050_Init(bus)) { failed(1u); return; }
    if (MPU6050_CalibrateBias(bus, &bias[0], &bias[1], &bias[2], 200u)) {
        failed(2u); return;
    }
    for (unsigned i = 0; i < 3u; ++i) {
        if (!(bias[i] >= -10.0f && bias[i] <= 10.0f)) { failed(2u); return; }
        imu_diagnostic.bias[i] = bias[i];
    }
    previous_tick = HAL_GetTick();
    imu_diagnostic.tick_ms = previous_tick;
    imu_diagnostic.state = 2u;
}

void ImuDiagnostic_Poll(I2C_HandleTypeDef *bus)
{
    MPU6050_Data_t sample;
    uint32_t now, dt;
    float rate[3], accel[3];
    Control_Stop();
    if (imu_diagnostic.state != 2u) return;
    now = HAL_GetTick(); dt = now - previous_tick;
    if (dt < 20u) return;
    if (dt > 50u) { failed(3u); return; }
    if (MPU6050_Read(bus, &sample)) { failed(4u); return; }
    rate[0] = sample.gx; rate[1] = sample.gy; rate[2] = sample.gz;
    accel[0] = sample.ax; accel[1] = sample.ay; accel[2] = sample.az;
    for (unsigned i = 0; i < 3u; ++i) {
        if (!(rate[i] >= -500.0f && rate[i] <= 500.0f &&
              accel[i] >= -2.0f && accel[i] <= 2.0f)) { failed(5u); return; }
        rate[i] -= imu_diagnostic.bias[i];
    }
    ++imu_diagnostic.sequence;
    for (unsigned i = 0; i < 3u; ++i) {
        imu_diagnostic.gyro[i] = rate[i];
        imu_diagnostic.angle[i] += .5f * (previous_rate[i] + rate[i]) * ((float)dt * .001f);
        imu_diagnostic.accel[i] = accel[i];
        previous_rate[i] = rate[i];
    }
    previous_tick = now;
    imu_diagnostic.tick_ms = now;
    imu_diagnostic.period_ms = dt;
    ++imu_diagnostic.samples;
    ++imu_diagnostic.sequence;
}
