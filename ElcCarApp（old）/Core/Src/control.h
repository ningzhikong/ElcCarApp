#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"

/* 闭环增量式 PID 结构体 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float target;      // 目标速度 
    float actual;      // 实际编码器测得速度
    float err;         // 当前偏差
    float err_last;    // 上次偏差
    float err_prev;    // 上上次偏差
    float output;      // 最终输出给 PWM 的值
    float max_output;  // 最大 PWM 限幅
} Speed_PID_t;

/* 外环位置式 PID  */
typedef struct {
    float kp;
    float kd;
    float err;
    float err_last;
    float output;
    float max_output;  // 转向限幅
} Turn_PID_t;

extern Speed_PID_t pid_L, pid_R;
extern Turn_PID_t  pid_turn;

/* 接口函数 */
void Motor_Init(void);
void Motor_SetPWM(int16_t l_pwm, int16_t r_pwm);
void Encoder_ReadSpeeds(int16_t *speed_l, int16_t *speed_r);
void Speed_PID_Init(void);
float PID_Incremental_Calc(Speed_PID_t *pid, float target, float actual);
float PID_Positional_Calc(Turn_PID_t *pid, float target, float actual);
void ClosedLoop_Drive(float base_speed, float turn_adjustment, int16_t actual_l, int16_t actual_r);

#endif /* __CONTROL_H__ */
