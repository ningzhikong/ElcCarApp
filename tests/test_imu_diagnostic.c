#include "imu_diagnostic.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t now, stops, reads;
static int init_error, calibration_error, read_error;
static float rates[3];
uint32_t HAL_GetTick(void) { return now; }
void Control_Stop(void) { ++stops; }
uint8_t MPU6050_Init(I2C_HandleTypeDef *bus) { (void)bus; return (uint8_t)init_error; }
uint8_t MPU6050_CalibrateBias(I2C_HandleTypeDef *bus, float *x, float *y, float *z, uint16_t n)
{
    (void)bus; assert(n == 200u); now += 1000u;
    *x = 1; *y = 2; *z = -1; return (uint8_t)calibration_error;
}
uint8_t MPU6050_Read(I2C_HandleTypeDef *bus, MPU6050_Data_t *sample)
{
    (void)bus; ++reads; memset(sample, 0, sizeof *sample);
    sample->ay = 1; sample->gx = 1 + rates[0]; sample->gy = 2 + rates[1]; sample->gz = -1 + rates[2];
    return (uint8_t)read_error;
}
int main(int argc, char **argv)
{
    I2C_HandleTypeDef bus = 0;
    assert(argc == 2);
    if (!strcmp(argv[1], "wrap")) now = 0xfffffc00u;
    init_error = !strcmp(argv[1], "init_error");
    calibration_error = !strcmp(argv[1], "calibration_error");
    ImuDiagnostic_Init(&bus);
    assert(sizeof(ImuDiagnostic) == 76 && stops == (init_error || calibration_error ? 2u : 1u));
    if (init_error || calibration_error) {
        assert(imu_diagnostic.state == 3 && imu_diagnostic.error == (init_error ? 1u : 2u));
        ImuDiagnostic_Poll(&bus); assert(reads == 0 && stops == 3);
    } else {
        assert(imu_diagnostic.state == 2 && imu_diagnostic.sequence % 2 == 0);
        ImuDiagnostic_Poll(&bus); assert(reads == 0 && stops == 2);
        if (!strcmp(argv[1], "gap")) {
            now += 51; ImuDiagnostic_Poll(&bus);
            assert(imu_diagnostic.state == 3 && imu_diagnostic.error == 3 && reads == 0);
        } else if (!strcmp(argv[1], "read_error") || !strcmp(argv[1], "nonfinite")) {
            read_error = !strcmp(argv[1], "read_error");
            if (!read_error) rates[1] = NAN;
            now += 20; ImuDiagnostic_Poll(&bus);
            assert(imu_diagnostic.state == 3 && imu_diagnostic.error == (read_error ? 4u : 5u));
        } else {
            int axis = !strcmp(argv[1], "left_x") ? 0 : 1;
            float expected = !strcmp(argv[1], "negative") ? -30.0f : 30.0f;
            if (!strcmp(argv[1], "stationary") || !strcmp(argv[1], "wrap")) expected = 0;
            rates[axis] = expected;
            for (unsigned i = 0; i < 50; ++i) { now += 20; ImuDiagnostic_Poll(&bus); }
            rates[axis] = 0; now += 20; ImuDiagnostic_Poll(&bus);
            assert(imu_diagnostic.state == 2 && imu_diagnostic.samples == 51);
            assert(imu_diagnostic.period_ms == 20 && imu_diagnostic.sequence % 2 == 0);
            for (int i = 0; i < 3; ++i)
                assert(fabsf(imu_diagnostic.angle[i] - (i == axis ? expected : 0)) < .0001f);
            assert(stops == 53);
        }
    }
    printf("imu_diagnostic_%s: PASS\n", argv[1]); return 0;
}
