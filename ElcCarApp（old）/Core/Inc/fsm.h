#ifndef __CAR_FSM_H__
#define __CAR_FSM_H__

#include "main.h"

typedef enum {
    SYS_STATE_STANDBY = 0, 
    SYS_STATE_VISION,      
    SYS_STATE_RUN_ABCD,    
    SYS_STATE_RUN_ACBD,    
    SYS_STATE_BRAKE        
} SystemState_t;

/* ACBD 轨迹细分子状态 */
typedef enum {
    ACBD_TO_C = 0,
    ACBD_LINE_TO_B,
    ACBD_TO_D,
    ACBD_LINE_TO_A
} ACBD_SubPhase_t;

/* 颜色枚举 */
typedef enum {
    COLOR_UNKNOWN = 0,
    COLOR_BLUE,
    COLOR_RED
} VisionColor_t;

/* PID 控制器结构体 */
typedef struct {
    float kp, ki, kd;
    float err, err_last;
    float output;
    float max_output;
} PID_t;

/* 全局可导出的系统变量 */
extern SystemState_t g_system_state;
extern uint8_t g_target_laps;
extern uint8_t g_current_laps;
extern VisionColor_t g_vision_color;
extern int16_t speed_L;
extern int16_t speed_R;

/* 核心接口函数 */
void Car_System_Init(void);
void FSM_KeyScan(void);
void FSM_Timer1s_Tick(void);     // 1s 中断：处理视觉蜂鸣器/LED闪烁
void FSM_ControlLoop_10ms(void); // 10ms 中断：核心控制闭环
void FSM_OLED_Update(void);      // 主循环调用：非阻塞刷新显存
void FSM_UART_RxCallback(uint8_t *data, uint16_t len);

#endif /* __CAR_FSM_H__ */
