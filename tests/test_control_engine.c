#include "control.h"
#include "oled.h"
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
int hi2c1, test_i2c_busy;
static uint32_t now, mask;
static int test_mode;
static GPIO_PinState pin_states[7];
static unsigned i2c_calls;
static int i2c_error, i2c_race, check_pixels;
void HAL_Delay(uint32_t ms) { now += ms; }
int HAL_I2C_Master_Transmit(void *h, uint16_t address, uint8_t *data, uint16_t n, uint32_t timeout)
{
    (void)h; assert(address == 0x78 && timeout == 3 && n <= 17);
    ++i2c_calls;
    if (check_pixels && data[0] == 0x40) {
        assert(n == 17);
        for (unsigned i = 1; i < n; ++i) assert(data[i] == 0xa5);
    }
    /* Emulate BUSY appearing after the precheck: at most one 26ms wait. */
    if (i2c_race) {
        for (unsigned i = 0; i < 26; ++i) {
            ++now; DWT->CYCCNT += 72000;
            if (now % 10 == 0) { TIM1->SR |= TIM_SR_UIF; Control_TimerIRQ(); }
        }
        return HAL_BUSY;
    }
    return i2c_error ? HAL_ERROR : HAL_OK;
}

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
{ (void)port; assert(pin < 7u); pin_states[pin] = state; }
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
#if CAR_TUNED_PI_ENABLED
    if (!strcmp(argv[1], "tuned_profile")) {
        assert(pid_L.kp == 2.25f && pid_R.kp == 2.5f && pid_L.ki == .35f && pid_R.ki == .4f);
        assert(pid_L.kd == 0 && pid_R.kd == 0 && pid_L.max_output == 60);
        ClosedLoop_Drive(25, 18, 0, 0); view = tick(10, -2, 2);
        assert(view.raw_l == 2 && view.raw_r == 2);
        assert(fabsf(view.target_l - 35.0f/43.0f) < .00001f && view.target_r == 5);
        for (unsigned base = 1; base < 100; ++base) {
            ClosedLoop_Drive((float)base, (float)base * .8f, 0, 0);
            view = tick(10, 0, 0);
            assert(view.fault == 0 && view.target_l <= 5 && view.target_r <= 5);
            assert(view.pwm_l >= 0 && view.pwm_l <= 60 && view.pwm_r >= 0 && view.pwm_r <= 60);
        }
        Control_SetTargets(0, 0); view = tick(10, -2, 2);
        assert(view.pwm_l == 0 && view.pwm_r == 0);
        Control_SetTargets(-1, 1); assert(Control_GetFault() == CONTROL_FAULT_EMERGENCY);
        puts("control_tuned_profile: PASS"); return 0;
    }
#endif
    if (!strcmp(argv[1], "rpm_feedback_slow")) {
        test_mode = 1; Control_SetRPMFeedback(1);
        for (unsigned i = 0; i < 60; ++i) {
            int delta = i % 20 < 7 ? 5 : 0;
            Control_SetTargets(1.8f, 2.2f); view = tick(10, delta, delta);
            assert(view.raw_l == delta);
            if (i >= 19) assert(view.speed_l == 1.75f);
        }
        Control_Stop(); Control_SetRPMFeedback(0);
        view = tick(10, 8, 8); assert(view.speed_l == 1);
    } else if (!strcmp(argv[1], "rpm_feedback")) {
        test_mode = 1;
        Control_SetRPMFeedback(1);
        for (unsigned i = 0; i < 20; ++i) {
            int delta = i % 10 < 7 ? 5 : 0;
            Control_SetTargets(3.5f, 3.5f); view = tick(10, delta, delta);
            assert(view.raw_l == delta && view.raw_r == delta);
            if (i >= 9) assert(view.speed_l == 3.5f && view.speed_r == 3.5f);
        }
        assert(view.position_l == 70 && view.position_r == 70);
        Control_Stop(); Control_SetRPMFeedback(0);
        view = tick(10, 8, 8);
        assert(view.speed_l == 1 && view.speed_r == 1);
    } else if (!strcmp(argv[1], "rpm_feedback_guard")) {
        Control_SetRPMFeedback(1);
        assert(Control_GetFault() == CONTROL_FAULT_EMERGENCY);
        Control_ClearFault(); test_mode = 1;
        Control_SetTargets(1, 1); tick(10, 0, 0);
        Control_SetRPMFeedback(1);
        assert(Control_GetFault() == CONTROL_FAULT_EMERGENCY && TIM3->CCR3 == 0);
    } else if (!strcmp(argv[1], "jog_output")) {
        test_mode = 1;
        Control_SetJogPWM(150, 0); view = tick(10, -4, 2);
        assert(view.pwm_l == 150 && view.pwm_r == 0 && view.target_l == 0);
        assert(pin_states[motorLeftUp_Pin] == GPIO_PIN_SET);
        Control_Stop(); view = tick(10, 0, 0);
        assert(view.pwm_l == 0 && TIM3->CCR3 == 0);
        Control_SetJogPWM(0, -100); view = tick(10, 0, 0);
        assert(view.pwm_l == 0 && view.pwm_r == -100);
        assert(pin_states[motorRightUp_Pin] == GPIO_PIN_RESET);
        Control_SetTargets(0, 0); view = tick(10, 0, 0);
        assert(view.pwm_l == 0 && view.pwm_r == 0);
        Control_SetJogPWM(100, 0); tick(10, 0, 0);
        for (unsigned i = 0; i < 10; ++i) view = tick(10, 0, 0);
        assert(view.fault == CONTROL_FAULT_STALE_TARGET && TIM3->CCR3 == 0);
    } else if (!strcmp(argv[1], "jog_invalid")) {
        test_mode = 1; Control_SetJogPWM(100, 100);
        assert(Control_GetFault() == CONTROL_FAULT_EMERGENCY);
        Control_ClearFault(); Control_SetJogPWM(151, 0);
        assert(Control_GetFault() == CONTROL_FAULT_EMERGENCY);
        Control_ClearFault(); test_mode = 0; Control_SetJogPWM(100, 0);
        assert(Control_GetFault() == CONTROL_FAULT_EMERGENCY && TIM3->CCR3 == 0);
    } else if (!strcmp(argv[1], "motor_polarity")) {
        Motor_SetPWM(150, 100);
        assert(TIM3->CCR3 == 150 && TIM3->CCR4 == 100);
        assert(pin_states[motorLeftUp_Pin] == GPIO_PIN_SET);
        assert(pin_states[motorLeftBack_Pin] == GPIO_PIN_RESET);
        assert(pin_states[motorRightUp_Pin] == GPIO_PIN_SET);
        assert(pin_states[motorRightBack_Pin] == GPIO_PIN_RESET);
        Motor_SetPWM(-150, -100);
        assert(TIM3->CCR3 == 150 && TIM3->CCR4 == 100);
        assert(pin_states[motorLeftUp_Pin] == GPIO_PIN_RESET);
        assert(pin_states[motorLeftBack_Pin] == GPIO_PIN_SET);
        assert(pin_states[motorRightUp_Pin] == GPIO_PIN_RESET);
        assert(pin_states[motorRightBack_Pin] == GPIO_PIN_SET);
        Motor_SetPWM(-32768, 32767);
        assert(TIM3->CCR3 == 999 && TIM3->CCR4 == 999);
        Control_Stop();
        assert(TIM3->CCR3 == 0 && TIM3->CCR4 == 0);
    } else if (!strcmp(argv[1], "quiet")) {
        for (unsigned i = 0; i < 30; ++i) view = tick(10, 0, 0);
        assert(view.quiet_ticks == 30);
        view = tick(10, 1, 0); assert(view.quiet_ticks == 0);
    } else if (!strcmp(argv[1], "oled_busy")) {
        Control_SetTargets(10, 10); test_i2c_busy = 1;
        OLED_ShowFrame(); OLED_Poll();
        assert(i2c_calls == 0 && now == 0 && Control_GetFault() == 0);
    } else if (!strcmp(argv[1], "oled_race")) {
        now = 6; DWT->CYCCNT += 6 * 72000; Control_SetTargets(10, 10);
        i2c_race = 1; OLED_ShowFrame(); OLED_Poll();
        assert(i2c_calls == 1 && now == 32 && Control_GetFault() == 0);
        Control_SetTargets(10, 10); OLED_ShowFrame(); OLED_Poll();
        assert(i2c_calls == 1);
    } else if (!strcmp(argv[1], "oled_frame")) {
        extern uint8_t OLED_GRAM[8][128];
        memset(OLED_GRAM, 0xa5, sizeof OLED_GRAM); OLED_ShowFrame(); OLED_NewFrame();
        check_pixels = 1;
        for (unsigned i = 0; i < 88; ++i) {
            unsigned before = i2c_calls; OLED_Poll(); assert(i2c_calls == before + 1);
        }
        OLED_Poll(); assert(i2c_calls == 88);
    } else if (!strcmp(argv[1], "oled_error")) {
        i2c_error = 1; OLED_ShowFrame(); OLED_Poll(); assert(i2c_calls == 1);
        i2c_error = 0; OLED_ShowFrame(); OLED_Poll(); assert(i2c_calls == 1);
        now += 50; OLED_Poll(); assert(i2c_calls == 2);
    } else if (!strcmp(argv[1], "sampling")) {
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
    } else if (!strcmp(argv[1], "sign_change")) {
        Control_SetTargets(10.0f, 10.0f);
        tick(10, 0, 0);
        Control_SetEncoderSigns(-1, 1);
        assert(Control_GetFault() == CONTROL_FAULT_EMERGENCY && TIM3->CCR3 == 0u);
    } else if (!strcmp(argv[1], "invalid_gains")) {
        Control_ConfigurePI(NAN, .3f, 2.0f, .3f, 100.0f);
        assert(Control_GetFault() == CONTROL_FAULT_EMERGENCY && TIM3->CCR3 == 0u);
    } else if (!strcmp(argv[1], "fractional_limit")) {
        Control_ConfigurePI(2.0f, .3f, 2.0f, .3f, 100.5f);
        assert(Control_GetFault() == CONTROL_FAULT_EMERGENCY && TIM3->CCR3 == 0u);
    } else assert(0);
    printf("control_%s: PASS\n", argv[1]);
    return 0;
}
