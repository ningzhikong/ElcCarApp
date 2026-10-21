#include "ground_test.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "wheel_calibration.h"
#include "ir_filter.h"

TestTimer test_tim1, test_tim2, test_tim3, test_tim4;
TestRcc test_rcc;
TestCoreDebug test_debug;
TestDwt test_dwt;
uint32_t SystemCoreClock = 72000000u;
int htim2, htim3, htim4;
static uint32_t now, mask;
static GPIO_PinState start_key = GPIO_PIN_SET, stop_key = GPIO_PIN_SET;
static unsigned mux;
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
    assert(fabsf(v.target_l - Wheel_RPMToCounts(GROUND_RPM, CAR_WHEEL_CPR_LEFT)) < .00001f);
    assert(fabsf(v.target_r - Wheel_RPMToCounts(GROUND_RPM, CAR_WHEEL_CPR_RIGHT)) < .00001f);
    assert(pid_L.max_output == 150 && pid_R.max_output == 150);
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    if (!strcmp(argv[1], "wrap")) now = 0xfffff000u;
    Motor_Init(); Ground_Init(); Control_StartTimer();
    assert(ground_result.magic == 0x47524434u);
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
#ifdef CAR_GROUND_LINE
        if (!strcmp(argv[1], "speed_plan")) {
            for (unsigned i = 0; i < 300; ++i) tick(10, 2, 2, 1);
            ControlView v; Control_ReadView(&v);
            assert(fabsf(Wheel_CountsToRPM(v.target_l, CAR_WHEEL_CPR_LEFT) - 15) < .0001f);
            assert(fabsf(Wheel_CountsToRPM(v.target_r, CAR_WHEEL_CPR_RIGHT) - 15) < .0001f);
            ir_levels = ground_ir_direction == 1 ? 0xfeu : 0x7fu;
            for (unsigned i = 0; i < 30; ++i) tick(10, 2, 2, 1);
            Control_ReadView(&v);
            assert(fabsf(Wheel_CountsToRPM(v.target_l, CAR_WHEEL_CPR_LEFT) - 6) < .0001f);
            assert(fabsf(Wheel_CountsToRPM(v.target_r, CAR_WHEEL_CPR_RIGHT) - 14) < .0001f);
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
            assert(ground_result.reason == 100);
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
