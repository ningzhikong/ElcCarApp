#include "ground_test.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "wheel_calibration.h"
#include "ir_filter.h"
#include "drive_config.h"
#include "mpu6050.h"

TestTimer test_tim1, test_tim2, test_tim3, test_tim4;
TestRcc test_rcc;
TestCoreDebug test_debug;
TestDwt test_dwt;
uint32_t SystemCoreClock = 72000000u;
int htim2, htim3, htim4;
static uint32_t now, mask;
static GPIO_PinState start_key = GPIO_PIN_SET, stop_key = GPIO_PIN_SET;
static unsigned mux;
I2C_HandleTypeDef hi2c2;
static int imu_init_error, imu_calibration_error, imu_read_error;
static float imu_rate;
void HAL_Delay(uint32_t ms) { now += ms; }
uint8_t MPU6050_Init(I2C_HandleTypeDef *bus) { (void)bus; return (uint8_t)imu_init_error; }
uint8_t MPU6050_CalibrateBias(I2C_HandleTypeDef *bus, float *x, float *y, float *z, uint16_t n)
{ (void)bus; assert(n == 200); *x = *y = *z = 0; now += 1000; return (uint8_t)imu_calibration_error; }
uint8_t MPU6050_Read(I2C_HandleTypeDef *bus, MPU6050_Data_t *data)
{ (void)bus; memset(data, 0, sizeof *data); data->gx = imu_rate; return (uint8_t)imu_read_error; }
static uint32_t ir_levels = 0xe7u; /* two centered black sensors */
void HAL_GPIO_Init(void *p, GPIO_InitTypeDef *c)
{ assert(p == GPIOA && c->Pin == GPIO_PIN_2 && c->Pull == GPIO_PULLUP); }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask = 1u; }
void __set_PRIMASK(uint32_t m) { mask = m; }
uint32_t HAL_GetTick(void) { return now; }
int HAL_TIM_Encoder_Start(void *h, unsigned c) { (void)h; (void)c; return HAL_OK; }
int HAL_TIM_PWM_Start(void *h, unsigned c)
{ (void)h; (void)c; RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E; return HAL_OK; }
void HAL_GPIO_WritePin(void *p, uint16_t pin, GPIO_PinState s)
{
    unsigned bit = p == GPIOB && pin == GPIO_PIN_5 ? 1u : p == GPIOA && pin == GPIO_PIN_12 ? 2u : p == GPIOA && pin == GPIO_PIN_11 ? 4u : 0u;
    mux = s ? mux | bit : mux & ~bit;
}
GPIO_PinState HAL_GPIO_ReadPin(void *p, uint16_t pin)
{
    if (p == GPIOA && pin == GPIO_PIN_2) return ir_levels & (1u << mux) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    return pin == Start_Pin ? start_key : stop_key;
}
uint32_t HAL_RCC_GetPCLK2Freq(void) { return 72000000u; }
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *c, uint32_t *l) { c->APB2CLKDivider = RCC_HCLK_DIV1; *l = 2u; }
void HAL_NVIC_SetPriority(int i, unsigned p, unsigned s) { (void)i; (void)p; (void)s; }
void HAL_NVIC_EnableIRQ(int i) { (void)i; }
void Error_Handler(void) { assert(0); }
int Autotune_IsEnabled(void) { return 0; }
void Autotune_BeforeControl(uint32_t t) { (void)t; }
void Autotune_AfterControl(const ControlView *v) { (void)v; }

static void tick(unsigned ms, int l, int r, int heartbeat)
{
    now += ms; DWT->CYCCNT += ms * 72000u;
    TIM2->CNT = (uint16_t)(TIM2->CNT + l); TIM4->CNT = (uint16_t)(TIM4->CNT + r);
    if (heartbeat) Ground_Heartbeat();
    TIM1->SR |= TIM_SR_UIF; Control_TimerIRQ(); assert(mask == 0);
}
static void idle(unsigned ticks)
{ for (unsigned i = 0; i < ticks; ++i) tick(10, 0, 0, 1); }
static void arm(void)
{
    idle(40); start_key = GPIO_PIN_RESET; idle(105);
    assert(ground_result.state == GROUND_READY && TIM3->CCR3 == 0);
    start_key = GPIO_PIN_SET; idle(4);
    assert(ground_result.state == GROUND_COUNTDOWN && TIM3->CCR3 == 0);
}
static void run(void)
{
    arm(); idle(299);
    assert(ground_result.state == GROUND_COUNTDOWN && TIM3->CCR3 == 0);
    idle(1); assert(ground_result.state == GROUND_RUNNING && TIM3->CCR3 > 0);
    ControlView v;
    Control_ReadView(&v);
    assert(ground_target_rpm == GROUND_RPM);
    assert(fabsf(v.target_l - Wheel_RPMToCounts(GROUND_START_RPM, CAR_WHEEL_CPR_LEFT)) < .00001f);
    assert(fabsf(v.target_r - Wheel_RPMToCounts(GROUND_START_RPM, CAR_WHEEL_CPR_RIGHT)) < .00001f);
    assert(pid_L.max_output == CAR_DRIVE_PWM_LIMIT && pid_R.max_output == CAR_DRIVE_PWM_LIMIT);
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    imu_init_error = !strcmp(argv[1], "imu_init_error");
    imu_calibration_error = !strcmp(argv[1], "imu_calibration_error");
    if (!strcmp(argv[1], "wrap")) now = 0xfffff000u;
    Motor_Init(); Ground_Init(); Control_StartTimer();
    assert(ground_result.magic == 0x47524434u);
#ifndef CAR_GROUND_LINE
    if (imu_init_error || imu_calibration_error) {
        assert(ground_result.state == GROUND_FAULT && ground_result.reason == 110 && !ground_heading.ready);
        idle(200); assert(TIM3->CCR3 == 0 && TIM3->CCR4 == 0);
        puts("ground_imu_init_fault: PASS"); return 0;
    }
    if (!strcmp(argv[1], "heading_reference")) ground_heading.angle_deg = 45.0f;
#endif
    if (!strcmp(argv[1], "filter")) {
        IRFilter f;
        IRFilter_Reset(&f);
        IRFilter_Step(&f, 0xe7u, 0u); IRFilter_Step(&f, 0xe7u, 10u);
        assert(f.initialized == 0u);
        IRFilter_Step(&f, 0xe7u, 20u); assert(f.initialized == 255u && f.levels == 0xe7u);
        IRFilter_Step(&f, 0xe3u, 30u); IRFilter_Step(&f, 0xe3u, 40u);
        assert(f.levels == 0xe7u); /* two-frame spike rejected */
        IRFilter_Step(&f, 0xe7u, 50u); assert(f.levels == 0xe7u);
        IRFilter_Step(&f, 0xc7u, 60u); IRFilter_Step(&f, 0xc7u, 70u); IRFilter_Step(&f, 0xc7u, 80u);
        assert(f.levels == 0xc7u); /* sustained real edge accepted */
        for (unsigned i = 0; i < 5; ++i) IRFilter_Step(&f, i % 2 ? 0xc7u : 0xc3u, 90u + i * 10u);
        assert(f.noisy);
        for (unsigned t = 140u; t <= 340u; t += 10u) IRFilter_Step(&f, 0xc7u, t);
        assert(!f.noisy && f.levels == 0xc7u);
        IRFilter_Step(&f, 0xc7u, 400u); assert(f.initialized == 0u); /* gap invalidates old samples */
        IRFilter_Reset(&f);
        IRFilter_Step(&f, 0xe7u, 0xfffffff0u); IRFilter_Step(&f, 0xe7u, 0xfffffffau); IRFilter_Step(&f, 0xe7u, 4u);
        assert(f.initialized == 255u && f.levels == 0xe7u && !f.noisy);
        puts("ground_filter: PASS"); return 0;
    }
#ifdef CAR_GROUND_LINE
    if (!strcmp(argv[1], "mapping_blocked") || !strcmp(argv[1], "white_start") || !strcmp(argv[1], "black_start")) {
        if (!strcmp(argv[1], "white_start")) ir_levels = 0xffu;
        if (!strcmp(argv[1], "black_start")) ir_levels = 0u;
        idle(40); start_key = GPIO_PIN_RESET; idle(105); start_key = GPIO_PIN_SET; idle(400);
        assert(ground_result.state == GROUND_READY && TIM3->CCR3 == 0 && TIM3->CCR4 == 0);
        puts("ground_line_start_guard: PASS"); return 0;
    }
#endif
    if (!strcmp(argv[1], "boot_held")) {
        start_key = GPIO_PIN_RESET; idle(600);
        start_key = GPIO_PIN_SET; idle(400);
        assert(ground_result.state == GROUND_READY && TIM3->CCR3 == 0);
    } else if (!strcmp(argv[1], "short_press")) {
        idle(40); start_key = GPIO_PIN_RESET; idle(50); start_key = GPIO_PIN_SET; idle(400);
        assert(ground_result.state == GROUND_READY && TIM3->CCR3 == 0);
    } else if (!strcmp(argv[1], "cancel")) {
        arm(); stop_key = GPIO_PIN_RESET; idle(1);
        assert(ground_result.state == GROUND_STOPPED && ground_result.count == 0);
    } else if (!strcmp(argv[1], "countdown_start")) {
        arm(); start_key = GPIO_PIN_RESET; idle(1);
        assert(ground_result.state == GROUND_STOPPED);
#ifdef CAR_GROUND_LINE
    } else if (!strcmp(argv[1], "countdown_line_lost")) {
        arm(); ir_levels = 0xffu; idle(300);
        assert(ground_result.state == GROUND_FAULT && ground_result.reason == 104 && ground_result.count == 0);
#endif
    } else {
        run();
#ifndef CAR_GROUND_LINE
        if (!strcmp(argv[1], "heading_scaling")) {
            for (unsigned i = 0; i < 220; ++i) tick(10, 2, 2, 1);
            ground_heading.angle_deg = ground_heading.target_deg + 10.0f;
            for (unsigned i = 0; i < 40; ++i) tick(10, 2, 2, 1);
            GroundSample sample = ground_trace[ground_result.count - 1u];
            float left = sample.target_base_rpm - sample.turn_rpm;
            float right = sample.target_base_rpm + sample.turn_rpm;
            assert(left > right && left <= 130.0001f && right <= 130.0001f);
            assert(fabsf(left - 130.0f) < .0001f && fabsf(right - 118.529412f) < .0001f);
            assert(sample.sensor.yaw_error_deg == 10 && sample.sensor_age_ms <= 50);
            stop_key = GPIO_PIN_RESET; idle(1);
            assert(TIM3->CCR3 == 0 && TIM3->CCR4 == 0);
            puts("ground_heading_scaling: PASS"); return 0;
        }
        if (!strcmp(argv[1], "heading_direction")) {
            ControlView v;
            ground_heading.angle_deg = ground_heading.target_deg + 5.0f;
            tick(10, 2, 2, 1); Control_ReadView(&v);
            assert(Wheel_CountsToRPM(v.target_l, CAR_WHEEL_CPR_LEFT) > Wheel_CountsToRPM(v.target_r, CAR_WHEEL_CPR_RIGHT));
            ground_heading.angle_deg = ground_heading.target_deg - 5.0f;
            tick(10, 2, 2, 1); tick(10, 2, 2, 1);
            Control_ReadView(&v);
            assert(Wheel_CountsToRPM(v.target_l, CAR_WHEEL_CPR_LEFT) < Wheel_CountsToRPM(v.target_r, CAR_WHEEL_CPR_RIGHT));
            stop_key = GPIO_PIN_RESET; idle(1);
            assert(ground_result.state == GROUND_STOPPED && TIM3->CCR3 == 0 && TIM3->CCR4 == 0);
            puts("ground_heading_direction: PASS"); return 0;
        }
        if (!strcmp(argv[1], "imu_read_error") || !strcmp(argv[1], "imu_nonfinite") ||
            !strcmp(argv[1], "imu_stale") || !strcmp(argv[1], "heading_diverge")) {
            if (!strcmp(argv[1], "imu_read_error")) imu_read_error = 1;
            if (!strcmp(argv[1], "imu_nonfinite")) imu_rate = NAN;
            if (!strcmp(argv[1], "imu_stale")) ground_heading.tick_ms = now - 51u;
            if (!strcmp(argv[1], "heading_diverge")) ground_heading.angle_deg = ground_heading.target_deg + 26.0f;
            for (unsigned i = 0; i < 3; ++i) tick(10, 2, 2, strcmp(argv[1], "imu_stale") != 0);
            assert(ground_result.state == GROUND_FAULT && ground_result.reason ==
                (!strcmp(argv[1], "heading_diverge") ? 111u : 110u));
            assert(TIM3->CCR3 == 0 && TIM3->CCR4 == 0);
            puts("ground_heading_fault: PASS"); return 0;
        }
        if (!strcmp(argv[1], "ramp")) {
            ControlView v;
            for (unsigned i = 1; i <= 250; ++i) {
                tick(10, 2, 2, 1); Control_ReadView(&v);
                float expected = i < 200 ? 20.0f + 0.55f * (float)i : 130.0f;
                assert(fabsf(Wheel_CountsToRPM(v.target_l, CAR_WHEEL_CPR_LEFT) - expected) < .0001f);
                assert(fabsf(Wheel_CountsToRPM(v.target_r, CAR_WHEEL_CPR_RIGHT) - expected) < .0001f);
                assert(ground_result.state == GROUND_RUNNING && v.pwm_l <= 450 && v.pwm_r <= 450);
            }
            stop_key = GPIO_PIN_RESET; idle(1);
            assert(ground_result.state == GROUND_STOPPED && TIM3->CCR3 == 0 && TIM3->CCR4 == 0);
            assert(ground_motion_stats.ticks == 0); /* 2.5s: still inside ramp+settle exclusion. */
            puts("ground_ramp: PASS"); return 0;
        }
#endif
#ifdef CAR_GROUND_LINE
        if (!strcmp(argv[1], "speed_plan")) {
            /* Tick counts are derived so the ramp actually saturates: a fixed
             * count only reached the ceiling for the original 10->15 range. */
            const unsigned ramp_ticks =
                (unsigned)((ROUTE_LINE_FAST_RPM - ROUTE_LINE_SLOW_RPM) / ROUTE_LINE_ACCEL_RPM_S * 100.0f) + 50u;
            const unsigned brake_ticks =
                (unsigned)((ROUTE_LINE_FAST_RPM - ROUTE_LINE_SLOW_RPM) / ROUTE_LINE_DECEL_RPM_S * 100.0f) + 20u;
            for (unsigned i = 0; i < ramp_ticks; ++i) tick(10, 2, 2, 1);
            ControlView v; Control_ReadView(&v);
            assert(fabsf(Wheel_CountsToRPM(v.target_l, CAR_WHEEL_CPR_LEFT) - ROUTE_LINE_FAST_RPM) < .0001f);
            assert(fabsf(Wheel_CountsToRPM(v.target_r, CAR_WHEEL_CPR_RIGHT) - ROUTE_LINE_FAST_RPM) < .0001f);
            ir_levels = ground_ir_direction == 1 ? 0xfeu : 0x7fu;
            for (unsigned i = 0; i < brake_ticks; ++i) tick(10, 2, 2, 1);
            Control_ReadView(&v);
            /* Saturated at the floor with the turn clamped to its bound. */
            assert(fabsf(Wheel_CountsToRPM(v.target_l, CAR_WHEEL_CPR_LEFT) -
                         (ROUTE_LINE_SLOW_RPM - ROUTE_MAX_TURN_RPM)) < .0001f);
            assert(fabsf(Wheel_CountsToRPM(v.target_r, CAR_WHEEL_CPR_RIGHT) -
                         (ROUTE_LINE_SLOW_RPM + ROUTE_MAX_TURN_RPM)) < .0001f);
            stop_key = GPIO_PIN_RESET; idle(1);
        } else if (!strcmp(argv[1], "noise")) {
            for (unsigned i = 0; i < 6; ++i) { ir_levels = i % 2 ? 0xe7u : 0xe3u; tick(10, 2, 2, 1); }
            assert(ground_result.state == GROUND_FAULT && ground_result.reason == 104 && ground_ir.noisy);
        } else if (!strcmp(argv[1], "line_lost") || !strcmp(argv[1], "line_wide") || !strcmp(argv[1], "line_stale")) {
            if (!strcmp(argv[1], "line_lost")) ir_levels = 0xffu;
            else if (!strcmp(argv[1], "line_wide")) ir_levels = 0u;
            else ground_ir.tick_ms = now - 50u;
            tick(10, 2, 2, strcmp(argv[1], "line_stale") != 0);
            assert(ground_result.state == GROUND_FAULT && ground_result.reason == 104);
        } else if (!strcmp(argv[1], "steering")) {
            ControlView v;
            ir_levels = ground_ir_direction == 1 ? 0xfeu : 0x7fu;
            for (unsigned i = 0; i < 3; ++i) tick(10, 2, 2, 1);
            Control_ReadView(&v);
            assert(fabsf(v.target_l - Wheel_RPMToCounts(6, CAR_WHEEL_CPR_LEFT)) < .00001f);
            assert(fabsf(v.target_r - Wheel_RPMToCounts(14, CAR_WHEEL_CPR_RIGHT)) < .00001f);
            ir_levels = ground_ir_direction == 1 ? 0x7fu : 0xfeu;
            for (unsigned i = 0; i < 3; ++i) tick(10, 2, 2, 1);
            Control_ReadView(&v);
            assert(fabsf(v.target_l - Wheel_RPMToCounts(14, CAR_WHEEL_CPR_LEFT)) < .00001f);
            assert(fabsf(v.target_r - Wheel_RPMToCounts(6, CAR_WHEEL_CPR_RIGHT)) < .00001f);
            stop_key = GPIO_PIN_RESET; idle(1);
        } else
#endif
        if (!strcmp(argv[1], "heartbeat")) {
            for (unsigned i = 0; i < 11; ++i) tick(10, 2, 2, 0);
            assert(ground_result.state == GROUND_FAULT);
#ifdef CAR_GROUND_LINE
            assert(ground_result.reason == 104); /* IR freshness stops before heartbeat deadline. */
#else
            assert(ground_result.reason == 110); /* IMU freshness fails before foreground watchdog. */
#endif
        } else if (!strcmp(argv[1], "stall")) {
            idle(101); assert(ground_result.state == GROUND_FAULT && ground_result.reason == CONTROL_FAULT_STALL);
        } else if (!strcmp(argv[1], "direction")) {
            for (unsigned i = 0; i < 3; ++i) tick(10, -4, 2, 1);
            assert(ground_result.state == GROUND_FAULT && ground_result.reason == CONTROL_FAULT_DIRECTION);
        } else if (!strcmp(argv[1], "timing")) {
            tick(20, 2, 2, 1); assert(ground_result.state == GROUND_FAULT && ground_result.reason == CONTROL_FAULT_TIMING);
        } else if (!strcmp(argv[1], "stop")) {
            stop_key = GPIO_PIN_RESET; tick(10, 2, 2, 1);
            assert(ground_result.state == GROUND_STOPPED);
        } else {
            for (unsigned i = 0; i < GROUND_RUN_MS / 10u - 1u; ++i) tick(10, 2, 2, 1);
            assert(ground_result.state == GROUND_RUNNING);
            tick(10, 2, 2, 1); assert(ground_result.state == GROUND_SETTLING && TIM3->CCR3 == 0);
            if (!strcmp(argv[1], "coasting")) {
                for (unsigned i = 0; i < 200; ++i) tick(10, 1, 1, 1);
                assert(ground_result.state == GROUND_FAULT && ground_result.reason == 102);
            } else {
                idle(100);
                assert(ground_result.state == GROUND_DONE && ground_result.count == (GROUND_RUN_MS + 1000u) / GROUND_SAMPLE_MS);
                assert(ground_result.count <= GROUND_TRACE_CAPACITY);
#ifndef CAR_GROUND_LINE
                assert(ground_motion_stats.ticks == 700);
                assert(ground_motion_stats.counts_l == 1400 && ground_motion_stats.counts_r == 1400);
                assert(ground_motion_stats.period_us == 7000000);
                assert(ground_motion_stats.pwm_sum_l == 450u * 700u && ground_motion_stats.pwm_sum_r == 450u * 700u);
                assert(ground_motion_stats.capped_l == 700 && ground_motion_stats.capped_r == 700);
                if (!strcmp(argv[1], "heading_reference")) {
                    assert(ground_heading.target_deg == 45.0f && Ground_HeadingError() == 0);
                }
#endif
                assert(ground_trace[0].elapsed_ms == 0 && ground_trace[GROUND_RUN_MS / GROUND_SAMPLE_MS].elapsed_ms == GROUND_RUN_MS);
                for (unsigned i = GROUND_RUN_MS / GROUND_SAMPLE_MS; i < ground_result.count; ++i)
                    assert(ground_trace[i].pwm_l == 0 && ground_trace[i].pwm_r == 0);
                uint32_t count = ground_result.count;
                start_key = GPIO_PIN_RESET; idle(110); start_key = GPIO_PIN_SET; idle(400);
                assert(ground_result.state == GROUND_DONE && ground_result.count == count);
            }
        }
    }
    assert(TIM3->CCR3 == 0 && TIM3->CCR4 == 0);
    printf("ground_%s: PASS\n", argv[1]); return 0;
}
