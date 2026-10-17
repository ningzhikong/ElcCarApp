#ifndef CAR_FSM_H
#define CAR_FSM_H
#ifdef CAR_HOST_TEST
#include "host_hal.h"
#else
#include "main.h"
#endif
typedef enum {
    SYS_STATE_STANDBY = 0, SYS_STATE_VISION, SYS_STATE_RUN_ABCD,
    SYS_STATE_RUN_ACBD, SYS_STATE_BRAKE, SYS_STATE_FAULT
} SystemState_t;
typedef enum { ACBD_TO_C = 0, ACBD_LINE_TO_B, ACBD_TO_D, ACBD_LINE_TO_A } ACBD_SubPhase_t;
typedef enum { COLOR_UNKNOWN = 0, COLOR_BLUE, COLOR_RED } VisionColor_t;
typedef enum {
    FSM_FAULT_NONE = 0, FSM_FAULT_CONTROL, FSM_FAULT_IMU,
    FSM_FAULT_ADC, FSM_FAULT_LINE_LOST, FSM_FAULT_ROUTE_TIMEOUT,
    FSM_FAULT_LAP_SEQUENCE
} FSM_Fault_t;
extern volatile SystemState_t g_system_state;
extern ACBD_SubPhase_t g_acbd_phase;
extern volatile FSM_Fault_t g_fsm_fault;
extern volatile uint8_t g_target_laps, g_current_laps;
extern volatile VisionColor_t g_vision_color;
extern volatile float speed_L, speed_R; /* Wheel RPM. */
void Car_System_Init(void);
void FSM_KeyScan(void);
void FSM_Timer1s_Tick(void);
void FSM_ControlLoop_10ms(void);
void FSM_OLED_Update(void);
void FSM_VisionRxByte(uint8_t byte);
#endif
