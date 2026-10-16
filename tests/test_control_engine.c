#include "control.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

TestTimer test_tim1, test_tim2, test_tim3, test_tim4;
TestRcc test_rcc;
TestCoreDebug test_debug;
TestDwt test_dwt;
uint32_t SystemCoreClock = 72000000u;
int htim2, htim3, htim4;
static uint32_t now, mask;
static int test_mode;

uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask = 1u; }
void __set_PRIMASK(uint32_t value) { mask = value; }
uint32_t HAL_GetTick(void) { return now; }
int HAL_TIM_Encoder_Start(void *timer, unsigned channel)
{ (void)timer; (void)channel; return HAL_OK; }
int HAL_TIM_PWM_Start(void *timer, unsigned channel)
{
    (void)timer; (void)channel;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
    return HAL_OK;
}
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state)
{ (void)port; (void)pin; (void)state; }
uint32_t HAL_RCC_GetPCLK2Freq(void) { return 72000000u; }
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *clocks, uint32_t *latency)
{ clocks->APB2CLKDivider = RCC_HCLK_DIV1; *latency = 2u; }
void HAL_NVIC_SetPriority(int irq, unsigned priority, unsigned subpriority)
{ assert(irq == TIM1_UP_IRQn && priority == 0u && subpriority == 0u); }
void HAL_NVIC_EnableIRQ(int irq) { assert(irq == TIM1_UP_IRQn); }
void Error_Handler(void) { assert(0); }
int Autotune_IsEnabled(void) { return test_mode; }
void Autotune_BeforeControl(uint32_t tick) { (void)tick; }
void Autotune_AfterControl(const ControlView *view) { (void)view; }

static ControlView tick(uint32_t period_ms, int delta_l, int delta_r)
{
    ControlView view;
    now += period_ms;
    DWT->CYCCNT += period_ms * 72000u;
    TIM2->CNT = (uint16_t)(TIM2->CNT + delta_l);
    TIM4->CNT = (uint16_t)(TIM4->CNT + delta_r);
    TIM1->SR |= TIM_SR_UIF;
    Control_TimerIRQ();
    Control_ReadView(&view);
    assert(!mask);
    return view;
}

int main(int argc, char **argv)
{
    ControlView view;
    assert(argc == 2);
    Motor_EmergencyStop(); /* May be called before the peripherals are ready. */
    Motor_Init();
    Control_StartTimer();
    assert(TIM1->PSC == 7199u && TIM1->ARR == 99u && TIM1->DIER == TIM_DIER_UIE);
    if (!strcmp(argv[1], "sampling")) {
        view = tick(10, -5, 3);
        assert(view.raw_l == -5 && view.raw_r == 3 && view.position_l == -5);
        assert(view.period_us == 10000u && TIM2->CNT == 65531u);
        Control_SetEncoderSigns(-1, 1);
        view = tick(10, -5, 3);
        assert(view.raw_l == 5 && view.raw_r == 3);
    } else if (!strcmp(argv[1], "stale")) {
        Control_SetTargets(10.0f, 10.0f);
        for (unsigned i = 0; i < 11u; ++i) view = tick(10, 0, 0);
        assert(view.fault == CONTROL_FAULT_STALE_TARGET);
        assert(TIM3->CCR3 == 0u && TIM3->CCR4 == 0u);
        assert(!(TIM3->CCER & (TIM_CCER_CC3E | TIM_CCER_CC4E)));
        Control_SetTargets(10.0f, 10.0f);
        view = tick(10, 0, 0);
        assert(view.pwm_l == 0.0f); /* A new target cannot silently clear a fault. */
    } else if (!strcmp(argv[1], "timing")) {
        Control_SetTargets(10.0f, 10.0f);
        view = tick(20, 0, 0);
        assert(view.fault == CONTROL_FAULT_TIMING && view.late_ticks == 1u);
        assert(TIM3->CCR3 == 0u);
    } else if (!strcmp(argv[1], "stop")) {
        Control_SetTargets(10.0f, 10.0f);
        view = tick(10, 0, 0);
        assert(view.pwm_l > 0.0f);
        Control_Stop();
        assert(TIM3->CCR3 == 0u && TIM3->CCR4 == 0u);
        view = tick(10, 0, 0);
        assert(view.pwm_l == 0.0f && view.target_l == 0.0f);
    } else if (!strcmp(argv[1], "test_zero")) {
        test_mode = 1;
        Control_SetTargets(0.0f, 0.0f);
        view = tick(10, 5, 5);
        assert(view.pwm_l == 0.0f && pid_L.output == 0.0f);
    } else if (!strcmp(argv[1], "cycle_wrap")) {
        DWT->CYCCNT = 0xfff00000u;
        view = tick(10, 0, 0); /* Stopped timing anomaly is diagnostic only. */
        Control_SetTargets(1.0f, 1.0f);
        view = tick(10, 0, 0);
        assert(view.period_us == 10000u && view.fault == 0u);
    } else if (!strcmp(argv[1], "invalid_target")) {
        Control_SetTargets(NAN, 10.0f);
        view = tick(10, 0, 0);
        assert(view.fault == CONTROL_FAULT_EMERGENCY && view.pwm_l == 0.0f);
    } else assert(0);
    printf("control_%s: PASS\n", argv[1]);
    return 0;
}
