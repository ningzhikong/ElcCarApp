#include "ground_test.h"
#include "wheel_calibration.h"
#include "route_config.h"
#include "ir_filter.h"
#include "line_follow.h"
#include <string.h>

/* One shot per reset. Results survive debugger attach, but not reset/power loss. */
volatile GroundResult ground_result;
volatile GroundSample ground_trace[GROUND_TRACE_CAPACITY];
/* Retained in the image for readback tools to report the actual test target. */
const volatile float ground_target_rpm = GROUND_RPM;
const volatile uint32_t ground_run_ms = GROUND_RUN_MS;
const volatile uint32_t ground_sample_ms = GROUND_SAMPLE_MS;
volatile GroundIR ground_ir;
#ifdef CAR_GROUND_LINE
const volatile int32_t ground_ir_direction = GROUND_IR_DIRECTION;
#else
const volatile int32_t ground_ir_direction = 0;
#endif
static float steering;
static volatile uint32_t heartbeat;
static uint32_t phase_tick, run_tick, next_sample, press_tick, key_tick;
static uint8_t released, pressing, last_key, stable_key;
#ifdef CAR_GROUND_LINE
static LineSensor line_sensor;
static LineController line_controller;
const volatile float ground_max_rpm = ROUTE_LINE_FAST_RPM;
#endif

static void finish(uint32_t state, uint32_t reason)
{
    Control_Stop();
    ground_result.state = state;
    ground_result.reason = reason;
}

void Ground_Init(void)
{
    Control_Stop();
    Control_ConfigureDriveRPM();
    memset((void *)&ground_result, 0, sizeof ground_result);
    memset((void *)ground_trace, 0, sizeof ground_trace);
    memset((void *)&ground_ir, 0, sizeof ground_ir);
    ground_result.magic = 0x47524434u;
    steering = 0.0f;
#ifdef CAR_GROUND_LINE
    LineSensor_Reset(&line_sensor);
    LineController_Reset(&line_controller);
    GPIO_InitTypeDef input = {0};
    input.Pin = GPIO_PIN_2; input.Mode = GPIO_MODE_INPUT; input.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &input);
#endif
    released = pressing = 0u;
    last_key = stable_key = 0u; /* Must first observe a debounced release. */
    key_tick = phase_tick = heartbeat = HAL_GetTick();
    run_tick = next_sample = press_tick = 0u;
}

void Ground_Heartbeat(void)
{
    heartbeat = HAL_GetTick();
#ifdef CAR_GROUND_LINE
    if (ground_result.state < GROUND_DONE && (!line_sensor.filter.seen || HAL_GetTick() - ground_ir.tick_ms >= 10u)) {
        GroundIR frame = {0};
        for (unsigned i = 0; i < 8; ++i) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (i & 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, (i & 2u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, (i & 4u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#ifndef CAR_HOST_TEST
            uint32_t selected_at = DWT->CYCCNT;
            while ((uint32_t)(DWT->CYCCNT - selected_at) < SystemCoreClock / 10000u) { }
#endif
            uint8_t raw = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) != GPIO_PIN_RESET;
            if (IR_NORMALIZE_LEVEL(raw)) frame.raw_levels |= 1u << i;
        }
        LineSensor_Update(&line_sensor, (uint8_t)frame.raw_levels, HAL_GetTick());
        frame.levels = line_sensor.frame.levels;
        frame.valid = line_sensor.frame.valid;
        frame.noisy = line_sensor.frame.noisy;
        frame.black_count = line_sensor.frame.black;
        frame.error = line_sensor.frame.error;
        frame.tick_ms = HAL_GetTick();
        uint32_t saved = __get_PRIMASK();
        __disable_irq(); ground_ir = frame; __set_PRIMASK(saved);
    }
#endif
    heartbeat = HAL_GetTick();
}

static int line_ready(uint32_t now)
{
#ifdef CAR_GROUND_LINE
    LineFrame frame = Line_Describe((uint8_t)ground_ir.raw_levels, (uint8_t)ground_ir.levels, ground_ir.valid, ground_ir.noisy, ground_ir.tick_ms);
    return (ground_ir_direction == 1 || ground_ir_direction == -1) && Line_Trackable(&frame, now, 0);
#else
    (void)now; return 1;
#endif
}

void Ground_BeforeControl(uint32_t now)
{
    ControlView v;
    uint8_t key = HAL_GPIO_ReadPin(Start_GPIO_Port, Start_Pin) != GPIO_PIN_RESET;
    uint32_t state = ground_result.state;
    Control_ReadView(&v);
    ground_result.quiet_ticks = v.quiet_ticks;
    if (state >= GROUND_DONE) { Control_Stop(); return; }
    if (v.fault) { finish(GROUND_FAULT, v.fault); return; }
    /* Raw local stop checked every 10ms, including countdown and settling. */
    if (HAL_GPIO_ReadPin(ModeChange_GPIO_Port, ModeChange_Pin) == GPIO_PIN_RESET) {
        finish(GROUND_STOPPED, 0u); return;
    }
    if (now - heartbeat > 100u) { finish(GROUND_FAULT, 100u); return; }
    if (key != last_key) { last_key = key; key_tick = now; }
    if (key != stable_key && now - key_tick >= 30u) {
        stable_key = key;
        if (key) {
            if (pressing && now - press_tick >= 1000u && state == GROUND_READY && v.quiet_ticks >= 30u && line_ready(now)) {
                ground_result.state = GROUND_COUNTDOWN;
                phase_tick = now;
            }
            released = 1u;
            pressing = 0u;
        } else if (released && state == GROUND_READY) {
            pressing = 1u;
            press_tick = now;
        }
    }
    state = ground_result.state;
    if (state == GROUND_COUNTDOWN) {
        Control_Stop();
        ground_result.elapsed_ms = now - phase_tick;
        if (!key) { finish(GROUND_STOPPED, 0u); return; }
        if (now - phase_tick >= GROUND_DELAY_MS) {
            if (v.quiet_ticks < 30u || now - v.tick_ms > 30u) {
                finish(GROUND_FAULT, 101u); return;
            }
            if (!line_ready(now)) { finish(GROUND_FAULT, 104u); return; }
            ground_result.state = GROUND_RUNNING;
            run_tick = now;
            next_sample = 0u;
            ground_result.elapsed_ms = 0u;
        }
    }
    if (ground_result.state == GROUND_RUNNING) {
        ground_result.elapsed_ms = now - run_tick;
        if (!key) { finish(GROUND_STOPPED, 0u); return; }
        if (now - run_tick >= ground_run_ms) {
            Control_Stop();
            ground_result.state = GROUND_SETTLING;
            phase_tick = now;
        } else {
            if (!line_ready(now)) { finish(GROUND_FAULT, 104u); return; }
            steering = 0.0f;
#ifdef CAR_GROUND_LINE
            LineCommand command = LineController_Step(&line_controller, ground_ir.error, now, ground_ir_direction, ground_max_rpm);
            steering = command.turn_rpm;
            Control_DriveRPM(command.base_rpm, steering);
#else
            Control_DriveRPM(ground_target_rpm, steering);
#endif
        }
    }
    if (ground_result.state == GROUND_SETTLING) {
        Control_Stop();
        ground_result.elapsed_ms = now - run_tick;
        if (now - phase_tick >= 1000u && v.quiet_ticks >= 30u)
            finish(GROUND_DONE, 0u);
        else if (now - phase_tick >= 2000u) finish(GROUND_FAULT, 102u);
    }
}

void Ground_AfterControl(const ControlView *v)
{
    uint32_t state = ground_result.state, n = ground_result.count;
    if (v->fault && state < GROUND_DONE) finish(GROUND_FAULT, v->fault);
    if (state != GROUND_RUNNING && state != GROUND_SETTLING) return;
    if (v->tick_ms - run_tick < next_sample) return;
    if (n >= GROUND_TRACE_CAPACITY) { finish(GROUND_FAULT, 103u); return; }
    GroundSample sample;
    sample.elapsed_ms = v->tick_ms - run_tick;
    sample.raw_l = v->raw_l; sample.raw_r = v->raw_r;
    sample.rpm_l = Wheel_CountsToRPM(v->speed_l, CAR_WHEEL_CPR_LEFT);
    sample.rpm_r = Wheel_CountsToRPM(v->speed_r, CAR_WHEEL_CPR_RIGHT);
    sample.pwm_l = v->pwm_l; sample.pwm_r = v->pwm_r;
    sample.period_us = v->period_us; sample.fault = v->fault;
    sample.ir_levels = ground_ir.levels;
    sample.ir_age_ms = ground_ir.valid ? v->tick_ms - ground_ir.tick_ms : 0xffffffffu;
    sample.turn_rpm = ground_result.state == GROUND_RUNNING ? steering : 0.0f;
    sample.ir_raw_levels = ground_ir.raw_levels;
    sample.ir_noisy = ground_ir.noisy;
    sample.target_base_rpm = .5f * (Wheel_CountsToRPM(v->target_l, CAR_WHEEL_CPR_LEFT) + Wheel_CountsToRPM(v->target_r, CAR_WHEEL_CPR_RIGHT));
    ground_trace[n] = sample;
    ground_result.count = n + 1u;
    next_sample = sample.elapsed_ms + ground_sample_ms;
}
