#ifndef CAR_GROUND_TEST_H
#define CAR_GROUND_TEST_H
#include "control.h"
#include "route_config.h"
#define GROUND_READY 0u
#define GROUND_COUNTDOWN 1u
#define GROUND_RUNNING 2u
#define GROUND_SETTLING 3u
#define GROUND_DONE 4u
#define GROUND_STOPPED 5u
#define GROUND_FAULT 6u
#define GROUND_TRACE_CAPACITY 121u
#define GROUND_DELAY_MS 3000u
#ifdef CAR_GROUND_LINE
#define GROUND_RUN_MS 10000u
#define GROUND_SAMPLE_MS 100u
#define GROUND_RPM 10.0f
/* +1: channel zero is physically left; -1: right; 0: mapping only, no motion. */
#ifndef GROUND_IR_DIRECTION
#define GROUND_IR_DIRECTION IR_CHANNEL_DIRECTION
#endif
#if GROUND_IR_DIRECTION != -1 && GROUND_IR_DIRECTION != 0 && GROUND_IR_DIRECTION != 1
#error Invalid infrared channel direction
#endif
#else
#define GROUND_RUN_MS 4000u
#define GROUND_SAMPLE_MS 50u
#define GROUND_RPM 15.0f
#endif
typedef struct {
    uint32_t elapsed_ms;
    int32_t raw_l, raw_r;
    float rpm_l, rpm_r, pwm_l, pwm_r;
    uint32_t period_us, fault, ir_levels, ir_age_ms;
    float turn_rpm;
    uint32_t ir_raw_levels, ir_noisy;
    float target_base_rpm;
} GroundSample;
typedef struct {
    uint32_t magic, state, reason, count, elapsed_ms, quiet_ticks;
} GroundResult;
extern volatile GroundResult ground_result;
extern volatile GroundSample ground_trace[GROUND_TRACE_CAPACITY];
extern const volatile float ground_target_rpm;
extern const volatile uint32_t ground_run_ms, ground_sample_ms;
typedef struct {
    uint32_t levels, black_count, tick_ms, valid;
    float error;
    uint32_t raw_levels, noisy;
} GroundIR;
extern volatile GroundIR ground_ir;
extern const volatile int32_t ground_ir_direction;
void Ground_Init(void);
void Ground_Heartbeat(void);
void Ground_BeforeControl(uint32_t now);
void Ground_AfterControl(const ControlView *v);
#endif
