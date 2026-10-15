#include "control.h"
#include "tim.h"
#include <stdlib.h>

Speed_PID_t pid_L, pid_R;
Turn_PID_t  pid_turn;

/**
 * @brief 编码器定时器初始化与启动
 */
void Motor_Init(void)
{
    // 启动 TIM3 PWM (PB0, PB1)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    // 启动 TIM2 (PA0/PA1, 左轮编码器) 和 TIM4 (PB6/PB7, 右轮编码器)
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    Speed_PID_Init();
}

/**
 * @brief 初始化 PID 参数 
 */
void Speed_PID_Init(void)
{
    // 大幅降低 Kp 和 Ki，彻底消除高频抖动和过大电流冲击
    pid_L.kp = 2.0f;   pid_L.ki = 0.3f;   pid_L.kd = 0.05f;  pid_L.max_output = 400.0f;
    pid_R.kp = 2.0f;   pid_R.ki = 0.3f;   pid_R.kd = 0.05f;  pid_R.max_output = 400.0f;
    pid_L.output = 0;  pid_R.output = 0;

    // 外环转向限幅也压低，防止差速直接把单侧轮子拉停或反转
    pid_turn.kp = 8.0f; pid_turn.kd = 2.0f; pid_turn.max_output = 5.0f;
}

/**
 * @brief 在 10ms 中断内读取两轮速度并清空计数值
 */
// 编码器速度读取 — 每次清零计数器 + 绝对值
static int16_t speed_l_prev = 0;
static int16_t speed_r_prev = 0;

void Encoder_ReadSpeeds(volatile int16_t *speed_l, volatile int16_t *speed_r)
{
    // 读取并清零计数器
    int32_t raw_l = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    
    int32_t raw_r = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    
    // 取绝对值（只关心速度大小）
    if (raw_l < 0) raw_l = -raw_l;
    if (raw_r < 0) raw_r = -raw_r;
    
    // 限制最大值
    if (raw_l > 32767) raw_l = 32767;
    if (raw_r > 32767) raw_r = 32767;
    
    // 8 次采样取平均滤波
    static int32_t avg_l = 0;
    static int32_t avg_r = 0;
    avg_l = (avg_l * 7 + raw_l) >> 3;
    avg_r = (avg_r * 7 + raw_r) >> 3;
    
    *speed_l = (int16_t)avg_l;
    *speed_r = (int16_t)avg_r;
}
/**
 * @brief 底层直接操作 TB6612 方向与 PWM
 */
void Motor_SetPWM(int16_t l_pwm, int16_t r_pwm)
{
    if (l_pwm > 999)  l_pwm = 999;
    if (l_pwm < -999) l_pwm = -999;
    if (r_pwm > 999)  r_pwm = 999;
    if (r_pwm < -999) r_pwm = -999;

    // 左电机 (PA3, PA4, PB0-PWM) — 原子操作防止桥臂直通
    uint32_t tmp_cr = __get_PRIMASK();
    __disable_irq();
    if (l_pwm >= 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, l_pwm);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, -l_pwm);
    }
    __set_PRIMASK(tmp_cr);

    // 右电机 (PA5, PA6, PB1-PWM) — 原子操作防止桥臂直通
    tmp_cr = __get_PRIMASK();
    __disable_irq();
    if (r_pwm >= 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, r_pwm);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -r_pwm);
    }
    __set_PRIMASK(tmp_cr);
}

/**
 * @brief 增量式 PID 计算 (针对电机速度闭环)
 */
float PID_Incremental_Calc(Speed_PID_t *pid, float target, float actual)
{
    pid->target = target;
    pid->actual = actual;
    pid->err = target - actual;

    float delta = pid->kp * (pid->err - pid->err_last)
                + pid->ki * pid->err
                + pid->kd * (pid->err - 2.0f * pid->err_last + pid->err_prev);

    // 关键保护：限制单步增量，单次 10ms 最大只允许变化 ±30 PWM
    if (delta > 30.0f)  delta = 30.0f;
    if (delta < -30.0f) delta = -30.0f;

    pid->output += delta;

    pid->err_prev = pid->err_last;
    pid->err_last = pid->err;

    // 限幅输出
    if (pid->output > pid->max_output)  pid->output = pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output;

    return pid->output;
}

/**
 * @brief 位置式 PID 计算 (针对外环转向调节)
 */
float PID_Positional_Calc(Turn_PID_t *pid, float target, float actual)
{
    pid->err = target - actual;
    pid->output = pid->kp * pid->err + pid->kd * (pid->err - pid->err_last);
    pid->err_last = pid->err;

    if (pid->output > pid->max_output)  pid->output = pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output;

    return pid->output;
}

/**
 * @brief 串级双闭环总调度函数 (每 10ms 调用一次)
 * @param base_speed       基准期望速度
 * @param turn_adjustment  外环算出的转向偏差量 (ΔV)
 * @param actual_l         左轮实际测得的编码器脉冲
 * @param actual_r         右轮实际测得的编码器脉冲
 */
void ClosedLoop_Drive(float base_speed, float turn_adjustment, int16_t actual_l, int16_t actual_r)
{
    // 1. 计算左右轮各自的期望速度
    float target_speed_l = base_speed - turn_adjustment;
    float target_speed_r = base_speed + turn_adjustment;

    // 2. 左右轮分别通过增量式 PID 计算出各自需要的实际硬件 PWM 占空比
    float pwm_out_l = PID_Incremental_Calc(&pid_L, target_speed_l, (float)actual_l);
    float pwm_out_r = PID_Incremental_Calc(&pid_R, target_speed_r, (float)actual_r);

    // 3. 施加给 TB6612 驱动芯片
    Motor_SetPWM((int16_t)pwm_out_l, (int16_t)pwm_out_r);
}
