#include "control.h"
#ifndef CAR_HOST_TEST
#include "tim.h"
#endif
#include "autotune.h"
#include <string.h>

Speed_PID_t pid_L, pid_R;
Turn_PID_t pid_turn;
static volatile ControlView current;
static uint16_t previous_l, previous_r;
static float filtered_l, filtered_r;
static EncoderWindow10 window_l, window_r;
static uint8_t rpm_feedback;
static float requested_l, requested_r;
static int encoder_sign_l = CAR_ENCODER_SIGN_LEFT, encoder_sign_r = CAR_ENCODER_SIGN_RIGHT;
static volatile uint8_t drive_enabled;
static uint8_t jog_enabled;
static int16_t jog_l, jog_r;
static uint32_t target_tick, previous_cycles;

static uint32_t lock(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}

void Motor_EmergencyStop(void)
{
    /* No HAL calls or delays: safe even before PWM initialization/in faults. */
    if (RCC->APB1ENR & RCC_APB1ENR_TIM3EN) {
        TIM3->CCR3 = 0u;
        TIM3->CCR4 = 0u;
        TIM3->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC4E);
    }
}

void Motor_SetPWM(int16_t left, int16_t right)
{
    if (left > 999) left = 999;
    if (left < -999) left = -999;
    if (right > 999) right = 999;
    if (right < -999) right = -999;
    left = (int16_t)(left * CAR_MOTOR_SIGN_LEFT);
    right = (int16_t)(right * CAR_MOTOR_SIGN_RIGHT);
    /* One writer in the control ISR. GPIO labels preserve PCB wiring. */
    HAL_GPIO_WritePin(motorLeftUp_GPIO_Port, motorLeftUp_Pin,
                      left < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motorLeftBack_GPIO_Port, motorLeftBack_Pin,
                      left < 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(motorRightUp_GPIO_Port, motorRightUp_Pin,
                      right < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motorRightBack_GPIO_Port, motorRightBack_Pin,
                      right < 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    TIM3->CCR3 = (uint16_t)(left < 0 ? -left : left);
    TIM3->CCR4 = (uint16_t)(right < 0 ? -right : right);
}

void Speed_PID_Init(void)
{
    memset(&pid_L, 0, sizeof pid_L);
    memset(&pid_R, 0, sizeof pid_R);
    memset(&pid_turn, 0, sizeof pid_turn);
    pid_L.kp = pid_R.kp = 2.0f;
    pid_L.ki = pid_R.ki = 0.3f;
    pid_L.kd = pid_R.kd = 0.05f;
    pid_L.max_output = pid_R.max_output = 400.0f;
#if CAR_TUNED_PI_ENABLED
    /* Match the bench controller and actuator envelope, including Kd=0. */
    if (!(CAR_TUNED_KP_LEFT >= 0.0f && CAR_TUNED_KP_LEFT <= 10.0f &&
          CAR_TUNED_KP_RIGHT >= 0.0f && CAR_TUNED_KP_RIGHT <= 10.0f &&
          CAR_TUNED_KI_LEFT >= 0.0f && CAR_TUNED_KI_LEFT <= 2.0f &&
          CAR_TUNED_KI_RIGHT >= 0.0f && CAR_TUNED_KI_RIGHT <= 2.0f &&
          CAR_TUNED_PWM_LIMIT >= 20 && CAR_TUNED_PWM_LIMIT <= 200 &&
          (float)(unsigned)CAR_TUNED_PWM_LIMIT == (float)CAR_TUNED_PWM_LIMIT &&
          CAR_TUNED_PEAK >= 1.0f && CAR_TUNED_PEAK <= 25.0f)) Error_Handler();
    pid_L.kp = CAR_TUNED_KP_LEFT; pid_L.ki = CAR_TUNED_KI_LEFT;
    pid_R.kp = CAR_TUNED_KP_RIGHT; pid_R.ki = CAR_TUNED_KI_RIGHT;
    pid_L.kd = pid_R.kd = 0.0f;
    pid_L.max_output = pid_R.max_output = CAR_TUNED_PWM_LIMIT;
#endif
    pid_turn.kp = 8.0f;
    pid_turn.kd = 2.0f;
    pid_turn.max_output = 5.0f;
}

void Motor_Init(void)
{
    Speed_PID_Init();
    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK ||
        HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
    Motor_SetPWM(0, 0);
    previous_l = (uint16_t)TIM2->CNT;
    previous_r = (uint16_t)TIM4->CNT;
}

void Control_Stop(void)
{
    uint32_t saved = lock();
    drive_enabled = 0u;
    jog_enabled = 0u;
    requested_l = requested_r = 0.0f;
    PID_Reset(&pid_L);
    PID_Reset(&pid_R);
    current.target_l = current.target_r = 0.0f;
    current.pwm_l = current.pwm_r = 0.0f;
    TIM3->CCR3 = TIM3->CCR4 = 0u;
    __set_PRIMASK(saved);
}

void Control_LatchFault(uint32_t fault)
{
    uint32_t saved = lock();
    if (current.fault == CONTROL_FAULT_NONE) current.fault = fault;
    Control_Stop();
    Motor_EmergencyStop();
    __set_PRIMASK(saved);
}

uint32_t Control_GetFault(void) { return current.fault; }

void Control_ClearFault(void)
{
    uint32_t saved = lock();
    Control_Stop();
    current.fault = CONTROL_FAULT_NONE;
    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
    __set_PRIMASK(saved);
}

void Control_ReadView(ControlView *view)
{
    uint32_t saved = lock();
    *view = current;
    __set_PRIMASK(saved);
}

void Control_SetEncoderSigns(int left, int right)
{
    uint32_t saved = lock();
    if (drive_enabled || (left != 1 && left != -1) || (right != 1 && right != -1)) {
        Control_LatchFault(CONTROL_FAULT_EMERGENCY);
        __set_PRIMASK(saved);
        return;
    }
    encoder_sign_l = left;
    encoder_sign_r = right;
    previous_l = (uint16_t)TIM2->CNT;
    previous_r = (uint16_t)TIM4->CNT;
    filtered_l = filtered_r = 0.0f;
    Encoder_WindowReset(&window_l); Encoder_WindowReset(&window_r);
    current.speed_l = current.speed_r = 0.0f;
    __set_PRIMASK(saved);
}

void Control_ConfigurePI(float kp_l, float ki_l, float kp_r, float ki_r, float limit)
{
    uint32_t saved = lock();
    if (drive_enabled || !(kp_l >= 0.0f && kp_l <= 10.0f && ki_l >= 0.0f && ki_l <= 2.0f &&
        kp_r >= 0.0f && kp_r <= 10.0f && ki_r >= 0.0f && ki_r <= 2.0f && limit >= 20.0f && limit <= 200.0f) ||
        limit != (float)(uint32_t)limit) {
        Control_LatchFault(CONTROL_FAULT_EMERGENCY);
        __set_PRIMASK(saved);
        return;
    }
    Control_Stop();
    pid_L.kp = kp_l; pid_L.ki = ki_l; pid_L.kd = 0.0f;
    pid_R.kp = kp_r; pid_R.ki = ki_r; pid_R.kd = 0.0f;
    pid_L.max_output = pid_R.max_output = limit;
    filtered_l = filtered_r = 0.0f;
    current.speed_l = current.speed_r = 0.0f;
    __set_PRIMASK(saved);
}

void Control_SetJogPWM(int16_t left, int16_t right)
{
    uint32_t saved = lock();
    if (!Autotune_IsEnabled() || left < -150 || left > 150 ||
        right < -150 || right > 150 || (left != 0 && right != 0)) {
        Control_LatchFault(CONTROL_FAULT_EMERGENCY);
    } else if (current.fault == CONTROL_FAULT_NONE) {
        jog_l = left; jog_r = right;
        requested_l = requested_r = 0.0f;
        target_tick = HAL_GetTick();
        jog_enabled = drive_enabled = 1u;
    }
    __set_PRIMASK(saved);
}

void Control_SetTargets(float left, float right)
{
    uint32_t saved = lock();
#if CAR_TUNED_PI_ENABLED
    if (!Autotune_IsEnabled() && !(left >= 0.0f && left <= CAR_TUNED_PEAK &&
                                   right >= 0.0f && right <= CAR_TUNED_PEAK)) {
        Control_LatchFault(CONTROL_FAULT_EMERGENCY);
        __set_PRIMASK(saved);
        return;
    }
#endif
    if (!(left >= -100.0f && left <= 100.0f && right >= -100.0f && right <= 100.0f)) {
        Control_LatchFault(CONTROL_FAULT_EMERGENCY);
        __set_PRIMASK(saved);
        return;
    }
    if (current.fault == CONTROL_FAULT_NONE) {
        jog_enabled = 0u;
        requested_l = left;
        requested_r = right;
        target_tick = HAL_GetTick();
        drive_enabled = 1u;
    }
    __set_PRIMASK(saved);
}

void Control_SetRPMFeedback(int enabled)
{
    uint32_t saved = lock();
    if (drive_enabled || (enabled && !Autotune_IsEnabled())) {
        Control_LatchFault(CONTROL_FAULT_EMERGENCY);
    } else {
        rpm_feedback = enabled != 0;
        Encoder_WindowReset(&window_l); Encoder_WindowReset(&window_r);
        filtered_l = filtered_r = 0.0f;
        current.speed_l = current.speed_r = 0.0f;
    }
    __set_PRIMASK(saved);
}

void ClosedLoop_Drive(float base, float turn, int16_t actual_l, int16_t actual_r)
{
    (void)actual_l;
    (void)actual_r;
    float left = base - turn, right = base + turn;
#if CAR_TUNED_PI_ENABLED
    if (!Autotune_IsEnabled()) {
        float peak = left > right ? left : right;
        if (peak > CAR_TUNED_PEAK) {
            left *= CAR_TUNED_PEAK / peak;
            right *= CAR_TUNED_PEAK / peak;
            if (left > CAR_TUNED_PEAK) left = CAR_TUNED_PEAK;
            if (right > CAR_TUNED_PEAK) right = CAR_TUNED_PEAK;
        }
    }
#endif
    Control_SetTargets(left, right);
}

void Control_StartTimer(void)
{
    uint32_t clock = HAL_RCC_GetPCLK2Freq();
    RCC_ClkInitTypeDef clocks;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clocks, &latency);
    if (clocks.APB2CLKDivider != RCC_HCLK_DIV1) clock *= 2u;
    if (clock % 10000u != 0u || SystemCoreClock % 1000000u != 0u)
        Error_Handler();
    __HAL_RCC_TIM1_CLK_ENABLE();
    TIM1->CR1 = 0u;
    TIM1->PSC = clock / 10000u - 1u;
    TIM1->ARR = 99u;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0u;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    previous_cycles = DWT->CYCCNT;
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0u, 0u);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    TIM1->DIER = TIM_DIER_UIE;
    TIM1->CR1 = TIM_CR1_CEN;
}

void Control_TimerIRQ(void)
{
    uint16_t count_l, count_r;
    uint32_t cycles, now;
    float out_l, out_r;
    if (!(TIM1->SR & TIM_SR_UIF)) return;
    TIM1->SR &= ~TIM_SR_UIF;
    cycles = DWT->CYCCNT;
    current.period_us = (cycles - previous_cycles) / (SystemCoreClock / 1000000u);
    previous_cycles = cycles;
    now = HAL_GetTick();
    count_l = (uint16_t)TIM2->CNT;
    count_r = (uint16_t)TIM4->CNT;
    current.raw_l = encoder_sign_l * Encoder_Delta16(count_l, previous_l);
    current.raw_r = encoder_sign_r * Encoder_Delta16(count_r, previous_r);
    if (current.raw_l == 0 && current.raw_r == 0) {
        if (current.quiet_ticks < 1000u) ++current.quiet_ticks;
    } else current.quiet_ticks = 0u;
    previous_l = count_l;
    previous_r = count_r;
    /* Unsigned arithmetic gives defined wrap; consumers use modular deltas. */
    current.position_l = (int32_t)((uint32_t)current.position_l + (uint32_t)current.raw_l);
    current.position_r = (int32_t)((uint32_t)current.position_r + (uint32_t)current.raw_r);
    filtered_l = rpm_feedback ? Encoder_WindowStep(&window_l, current.raw_l) : Encoder_Filter(filtered_l, current.raw_l);
    filtered_r = rpm_feedback ? Encoder_WindowStep(&window_r, current.raw_r) : Encoder_Filter(filtered_r, current.raw_r);
    current.speed_l = filtered_l;
    current.speed_r = filtered_r;
    current.tick_ms = now;
    if (current.period_us > 15000u || current.period_us < 5000u) {
        ++current.late_ticks;
        if (drive_enabled) Control_LatchFault(CONTROL_FAULT_TIMING);
    }
    if (Autotune_IsEnabled()) Autotune_BeforeControl(now);
    if (drive_enabled && now - target_tick > 100u)
        Control_LatchFault(CONTROL_FAULT_STALE_TARGET);
    current.target_l = requested_l;
    current.target_r = requested_r;
    if (drive_enabled && current.fault == CONTROL_FAULT_NONE) {
        out_l = jog_enabled ? (float)jog_l : PID_Incremental_Calc(&pid_L, requested_l, filtered_l);
        out_r = jog_enabled ? (float)jog_r : PID_Incremental_Calc(&pid_R, requested_r, filtered_r);
        /* Bench/tuned PI is forward only; zero target cancels PWM. */
        if (!jog_enabled && (Autotune_IsEnabled() || CAR_TUNED_PI_ENABLED)) {
            if (out_l < 0.0f || requested_l == 0.0f) out_l = pid_L.output = 0.0f;
            if (out_r < 0.0f || requested_r == 0.0f) out_r = pid_R.output = 0.0f;
        }
        Motor_SetPWM((int16_t)out_l, (int16_t)out_r);
        current.pwm_l = (float)(int16_t)out_l;
        current.pwm_r = (float)(int16_t)out_r;
    } else {
        TIM3->CCR3 = TIM3->CCR4 = 0u;
        current.pwm_l = current.pwm_r = 0.0f;
    }
    if (Autotune_IsEnabled()) {
        ControlView snapshot = current;
        Autotune_AfterControl(&snapshot);
    }
}
