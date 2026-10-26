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
#define GROUND_START_RPM GROUND_RPM
#define GROUND_RAMP_MS 0u
/* +1: channel zero is physically left; -1: right; 0: mapping only, no motion. */
#ifndef GROUND_IR_DIRECTION
#define GROUND_IR_DIRECTION IR_CHANNEL_DIRECTION
#endif
#if GROUND_IR_DIRECTION != -1 && GROUND_IR_DIRECTION != 0 && GROUND_IR_DIRECTION != 1
#error Invalid infrared channel direction
#endif
#else
#define GROUND_RUN_MS 10000u
#define GROUND_SAMPLE_MS 100u
#define GROUND_RPM 130.0f
#define GROUND_START_RPM 20.0f
#define GROUND_RAMP_MS 2000u
#endif
typedef struct {
    uint32_t elapsed_ms;
    int32_t raw_l, raw_r;
    float rpm_l, rpm_r, pwm_l, pwm_r;
    uint32_t period_us, fault;
    union { uint32_t ir_levels; float yaw_error_deg; } sensor;
    uint32_t sensor_age_ms;
    float turn_rpm;
    uint32_t ir_raw_levels, ir_noisy;
    float target_base_rpm;
} GroundSample;
typedef struct {
    uint32_t tick_ms, ready, error;
    float angle_deg, rate_dps, bias_dps, target_deg;
} GroundHeading;
extern volatile GroundHeading ground_heading;
typedef struct {
    uint32_t magic, state, reason, count, elapsed_ms, quiet_ticks;
} GroundResult;
/* All 10ms control ticks in the full-speed interval, not sparse display data. */
typedef struct {
    uint32_t ticks;
    int32_t counts_l, counts_r;
    uint32_t period_us, pwm_sum_l, pwm_sum_r, capped_l, capped_r;
} GroundMotionStats;
extern volatile GroundMotionStats ground_motion_stats;
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
#ifndef CAR_GROUND_LINE
float Ground_HeadingError(void);
#endif
#endif
