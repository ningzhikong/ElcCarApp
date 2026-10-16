#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H
#ifdef CAR_HOST_TEST
#include "host_hal.h"
#else
#include "main.h"
#endif
#include "control_math.h"

#define CONTROL_PERIOD_MS 10u
#ifndef CAR_ENCODER_SIGN_LEFT
#define CAR_ENCODER_SIGN_LEFT 1
#endif
#ifndef CAR_ENCODER_SIGN_RIGHT
#define CAR_ENCODER_SIGN_RIGHT 1
#endif
#if (CAR_ENCODER_SIGN_LEFT != 1 && CAR_ENCODER_SIGN_LEFT != -1) || \
    (CAR_ENCODER_SIGN_RIGHT != 1 && CAR_ENCODER_SIGN_RIGHT != -1)
#error Encoder signs must be +1 or -1
#endif
#define CONTROL_FAULT_NONE 0u
#define CONTROL_FAULT_TIMING 1u
#define CONTROL_FAULT_STALE_TARGET 2u
#define CONTROL_FAULT_UART 3u
#define CONTROL_FAULT_EMERGENCY 4u

typedef struct {
    uint32_t tick_ms, period_us, late_ticks, fault;
    int32_t raw_l, raw_r, position_l, position_r;
    float speed_l, speed_r, target_l, target_r, pwm_l, pwm_r;
} ControlView;

extern Speed_PID_t pid_L, pid_R;
extern Turn_PID_t pid_turn;
void Motor_Init(void);
void Motor_SetPWM(int16_t left, int16_t right);
void Motor_EmergencyStop(void);
void Speed_PID_Init(void);
void Control_StartTimer(void);
void Control_TimerIRQ(void);
void Control_ReadView(ControlView *view);
void Control_Stop(void);
void Control_LatchFault(uint32_t fault);
uint32_t Control_GetFault(void);
void Control_ClearFault(void);
void Control_SetTargets(float left, float right);
void Control_SetEncoderSigns(int left, int right);
void Control_ConfigurePI(float kp_l, float ki_l, float kp_r, float ki_r, float limit);
void ClosedLoop_Drive(float base, float turn, int16_t actual_l, int16_t actual_r);
#endif
