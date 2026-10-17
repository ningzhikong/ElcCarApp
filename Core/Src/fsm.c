#include "fsm.h"
#ifndef CAR_HOST_TEST
#include "adc.h"
#include "usart.h"
#include "oled.h"
#include "i2c.h"
#include "mpu6050.h"
#else
#include "host_services.h"
#endif
#include "control.h"
#include "serial_router.h"
#include "route_config.h"
#include "wheel_calibration.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

volatile SystemState_t g_system_state = SYS_STATE_STANDBY;
ACBD_SubPhase_t g_acbd_phase = ACBD_TO_C;
volatile FSM_Fault_t g_fsm_fault;
volatile uint8_t g_target_laps = 1, g_current_laps;
volatile VisionColor_t g_vision_color = COLOR_BLUE;
volatile float speed_L, speed_R; /* Wheel RPM, for display only. */
static uint32_t vision_start_time, brake_start_time, phase_tick;
static uint32_t peak_lockout_tick, beep_led_off_tick, line_seen_tick;
static uint32_t previous_l, previous_r, yaw_tick;
static bool beep_active, blink_on, suppress_initial_marker;
static volatile uint8_t blink_total_count, blink_cur_count;
static uint8_t g_beep_event_count;
static int64_t total_encoder_twice, lap_start_twice;
static uint32_t peak_maintain_pulse;
static bool is_on_peak, peak_event, peak_locked_out, is_straight_locked;
static uint16_t gray_adc[8];
static bool gray_valid;
static uint32_t gray_tick;
static float gray_error, current_yaw, target_straight_yaw, mpu_bias[3];
static bool line_detected, mpu_ready;
static Turn_PID_t line_pid, heading_pid;
static int previous_steering = -1;
static volatile uint8_t uart_rx_idx;
static char uart_rx_first;

static bool running(void)
{
    return g_system_state == SYS_STATE_RUN_ABCD || g_system_state == SYS_STATE_RUN_ACBD;
}
static void Device_Beep_Led_Set(bool on)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
static void fault_stop(FSM_Fault_t reason)
{
    Control_Stop();
    if (g_fsm_fault == FSM_FAULT_NONE) g_fsm_fault = reason;
    g_system_state = SYS_STATE_FAULT;
    Serial_StopVision();
    Device_Beep_Led_Set(true);
    beep_active = false;
}
static void brake(uint32_t now)
{
    Control_Stop(); /* Cancel immediately, including on the operator key. */
    g_system_state = SYS_STATE_BRAKE;
    brake_start_time = now;
    Device_Beep_Led_Set(false);
    beep_active = false;
}
static void reset_route(uint32_t now)
{
    ControlView v;
    Control_Stop();
    Control_ReadView(&v);
    previous_l = (uint32_t)v.position_l;
    previous_r = (uint32_t)v.position_r;
    total_encoder_twice = lap_start_twice = 0;
    g_current_laps = g_beep_event_count = 0;
    peak_maintain_pulse = 0;
    is_on_peak = peak_event = peak_locked_out = is_straight_locked = false;
    suppress_initial_marker = false;
    previous_steering = -1;
    line_pid.err = line_pid.err_last = line_pid.output = 0.0f;
    heading_pid.err = heading_pid.err_last = heading_pid.output = 0.0f;
    phase_tick = line_seen_tick = now;
    g_acbd_phase = ACBD_TO_C;
}
static void Enter_Standby_State(void)
{
    reset_route(HAL_GetTick());
    g_system_state = SYS_STATE_STANDBY;
    Serial_StopVision();
    Device_Beep_Led_Set(false);
    beep_active = false;
    blink_total_count = blink_cur_count = 0;
    blink_on = false;
}
static bool initialize_imu(void)
{
    mpu_ready = MPU6050_Init(&hi2c2) == 0 &&
        MPU6050_CalibrateBias(&hi2c2, &mpu_bias[0], &mpu_bias[1], &mpu_bias[2], 200) == 0;
    yaw_tick = HAL_GetTick();
    current_yaw = 0.0f;
    return mpu_ready;
}
void Car_System_Init(void)
{
    Motor_Init();
    Control_ConfigureDriveRPM();
    line_pid.kp = ROUTE_LINE_KP;
    line_pid.kd = ROUTE_LINE_KD;
    line_pid.max_output = ROUTE_MAX_TURN_RPM;
    heading_pid.kp = ROUTE_HEADING_KP;
    heading_pid.kd = ROUTE_HEADING_KD;
    heading_pid.max_output = ROUTE_MAX_TURN_RPM;
    initialize_imu();
    Enter_Standby_State();
}
static bool GraySensor_Scan(void)
{
    uint16_t frame[8];
    uint32_t sum = 0;
    float weighted = 0.0f;
    unsigned black = 0;
    gray_valid = false;
    for (unsigned i = 0; i < 8; ++i) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (i & 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, (i & 2u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, (i & 4u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        for (volatile unsigned d = 0; d < 30; ++d) { }
        if (HAL_ADC_Start(&hadc1) != HAL_OK) return false;
        if (HAL_ADC_PollForConversion(&hadc1, 1u) != HAL_OK) {
            HAL_ADC_Stop(&hadc1);
            return false;
        }
        frame[i] = (uint16_t)HAL_ADC_GetValue(&hadc1);
        if (HAL_ADC_Stop(&hadc1) != HAL_OK) return false;
        /* The module already thresholds gray internally: black is a low
         * digital output, white is high. Do not weight the white voltage. */
        if (frame[i] <= ROUTE_GRAY_LINE) {
            ++sum;
            weighted += (float)i - 3.5f;
            ++black;
        }
    }
    /* Publish only complete frames. Failed scans cannot create marker edges. */
    memcpy(gray_adc, frame, sizeof frame);
    gray_valid = true;
    gray_tick = HAL_GetTick();
    line_detected = black > 0u;
    gray_error = line_detected ? weighted / (float)sum : 0.0f;
    is_on_peak = black >= 5u || (frame[3] <= ROUTE_GRAY_LINE && frame[4] <= ROUTE_GRAY_LINE);
    return true;
}
static float wrap_yaw(float value)
{
    while (value > 180.0f) value -= 360.0f;
    while (value < -180.0f) value += 360.0f;
    return value;
}
static bool update_yaw(uint32_t now)
{
    MPU6050_Data_t data;
    uint32_t dt = now - yaw_tick;
    if (!mpu_ready || dt > ROUTE_IMU_MAX_GAP_MS || MPU6050_Read(&hi2c2, &data) != 0 ||
        !(data.gy >= -500.0f && data.gy <= 500.0f)) return false;
    current_yaw = wrap_yaw(current_yaw + MPU_YAW_SIGN * (data.gy - mpu_bias[1]) * ((float)dt * 0.001f));
    yaw_tick = now;
    return true;
}
static void update_marker(bool previous_peak, uint32_t distance, uint32_t now)
{
    peak_event = false;
    if (suppress_initial_marker) {
        if (!is_on_peak) suppress_initial_marker = false;
        peak_maintain_pulse = 0;
        return;
    }
    if (is_on_peak) {
        if (!previous_peak) peak_maintain_pulse = 0;
        if (peak_maintain_pulse < ROUTE_MARK_MIN_COUNTS * 2u)
            peak_maintain_pulse += distance;
    } else if (previous_peak) {
        if (peak_maintain_pulse >= ROUTE_MARK_MIN_COUNTS * 2u &&
            (!peak_locked_out || now - peak_lockout_tick >= ROUTE_MARK_COOLDOWN_MS)) {
            peak_event = true;
            peak_lockout_tick = now;
            peak_locked_out = true;
        }
        peak_maintain_pulse = 0;
    }
}
static bool record_marker(uint32_t now)
{
    ++g_beep_event_count;
    phase_tick = now;
    Device_Beep_Led_Set(true);
    beep_active = true;
    beep_led_off_tick = now;
    if (g_beep_event_count % 4u == 0u) {
        if (total_encoder_twice - lap_start_twice < (int64_t)ROUTE_LAP_MIN_COUNTS * 2) {
            fault_stop(FSM_FAULT_LAP_SEQUENCE);
            return false;
        }
        lap_start_twice = total_encoder_twice;
        ++g_current_laps;
        if (g_current_laps >= g_target_laps) {
            brake(now);
            return false;
        }
    }
    return true;
}
static void drive(float base, bool heading)
{
    Turn_PID_t *pid = heading ? &heading_pid : &line_pid;
    float turn, cap = base * 0.8f;
    if (previous_steering != (int)heading) {
        pid->err = pid->err_last = pid->output = 0.0f;
        previous_steering = heading;
    }
    turn = heading ? PID_Positional_Calc(pid, 0.0f, -wrap_yaw(target_straight_yaw - current_yaw)) :
                     PID_Positional_Calc(pid, 0.0f, -gray_error);
    if (turn > cap) turn = cap;
    if (turn < -cap) turn = -cap;
    Control_DriveRPM(base, turn);
}
static void begin_run(uint32_t now)
{
    bool starts_on_mark;
    yaw_tick = now; /* Standby intentionally does not integrate yaw. */
    if (!update_yaw(now)) { fault_stop(FSM_FAULT_IMU); return; }
    if (!GraySensor_Scan()) { fault_stop(FSM_FAULT_ADC); return; }
    starts_on_mark = is_on_peak;
    reset_route(HAL_GetTick());
    suppress_initial_marker = starts_on_mark;
    g_system_state = g_vision_color == COLOR_BLUE ? SYS_STATE_RUN_ACBD : SYS_STATE_RUN_ABCD;
    target_straight_yaw = wrap_yaw(current_yaw + 45.0f);
    blink_total_count = blink_cur_count = 0;
    blink_on = false;
    Device_Beep_Led_Set(false);
}
void FSM_KeyScan(void)
{
    static GPIO_PinState last_raw = GPIO_PIN_SET, stable = GPIO_PIN_SET;
    static uint32_t changed_at;
    uint32_t now = HAL_GetTick();
    GPIO_PinState raw = HAL_GPIO_ReadPin(ModeChange_GPIO_Port, ModeChange_Pin);
    if (raw != last_raw) { last_raw = raw; changed_at = now; }
    if (raw == stable || now - changed_at < 20u) return;
    stable = raw;
    if (stable != GPIO_PIN_RESET) return;
    switch (g_system_state) {
    case SYS_STATE_STANDBY: {
        uint8_t request[] = "START\r\n";
        g_system_state = SYS_STATE_VISION;
        vision_start_time = now;
        g_vision_color = COLOR_BLUE;
        g_target_laps = 1;
        uart_rx_idx = 0;
        blink_total_count = blink_cur_count = 0;
        blink_on = false;
        Serial_StartVision();
        if (HAL_UART_Transmit(&huart1, request, sizeof request - 1u, 20u) != HAL_OK)
            Enter_Standby_State();
        break;
    }
    case SYS_STATE_VISION:
        Serial_StopVision();
        begin_run(now);
        break;
    case SYS_STATE_RUN_ABCD:
    case SYS_STATE_RUN_ACBD:
        brake(now);
        break;
    case SYS_STATE_FAULT: {
        ControlView v;
        Control_ReadView(&v);
        if (v.quiet_ticks < 30u || now - v.tick_ms > 30u ||
            v.raw_l != 0 || v.raw_r != 0) break;
        if (!initialize_imu() || !GraySensor_Scan()) break;
        /* Calibration can take a second: verify uninterrupted quiet again. */
        Control_ReadView(&v);
        if (v.quiet_ticks < 30u || HAL_GetTick() - v.tick_ms > 30u) break;
        Control_ClearFault();
        g_fsm_fault = FSM_FAULT_NONE;
        Enter_Standby_State();
        break;
    }
    case SYS_STATE_BRAKE:
        break;
    }
}
void FSM_VisionRxByte(uint8_t byte)
{
    if (g_system_state != SYS_STATE_VISION) return;
    if (byte >= '1' && byte <= '5') { uart_rx_first = (char)byte; uart_rx_idx = 1; }
    else if (uart_rx_idx && (byte == 'B' || byte == 'b' || byte == 'R' || byte == 'r')) {
        g_target_laps = (uint8_t)(uart_rx_first - '0');
        g_vision_color = (byte == 'B' || byte == 'b') ? COLOR_BLUE : COLOR_RED;
        blink_total_count = g_target_laps;
        blink_cur_count = 0;
        uart_rx_idx = 0;
    } else uart_rx_idx = 0;
}
void FSM_Timer1s_Tick(void)
{
    if (g_system_state != SYS_STATE_VISION) return;
    if (blink_cur_count < blink_total_count) {
        blink_on = !blink_on;
        Device_Beep_Led_Set(blink_on);
        if (!blink_on) ++blink_cur_count;
    } else Device_Beep_Led_Set(false);
}
void FSM_ControlLoop_10ms(void)
{
    ControlView v;
    uint32_t now = HAL_GetTick();
    int32_t dl, dr;
    bool previous_peak = is_on_peak, heading;
    Control_ReadView(&v);
    speed_L = Wheel_CountsToRPM(v.speed_l, CAR_WHEEL_CPR_LEFT);
    speed_R = Wheel_CountsToRPM(v.speed_r, CAR_WHEEL_CPR_RIGHT);
    dl = (int32_t)((uint32_t)v.position_l - previous_l);
    dr = (int32_t)((uint32_t)v.position_r - previous_r);
    previous_l = (uint32_t)v.position_l;
    previous_r = (uint32_t)v.position_r;
    if (v.fault) { fault_stop(FSM_FAULT_CONTROL); return; }
    if (beep_active && now - beep_led_off_tick >= 80u) {
        Device_Beep_Led_Set(false); beep_active = false;
    }
    if (g_system_state == SYS_STATE_FAULT) { Control_Stop(); return; }
    if (g_system_state == SYS_STATE_STANDBY || g_system_state == SYS_STATE_VISION) {
        Control_Stop();
        yaw_tick = now;
        if (g_system_state == SYS_STATE_VISION && now - vision_start_time > 10000u && !blink_total_count)
            Enter_Standby_State();
        return;
    }
    if (g_system_state == SYS_STATE_BRAKE) {
        Control_Stop();
        if (now - brake_start_time >= 500u || v.quiet_ticks >= 30u)
            Enter_Standby_State();
        return;
    }
    if (!running()) { fault_stop(FSM_FAULT_CONTROL); return; }
    if (now - v.tick_ms > 30u) { fault_stop(FSM_FAULT_CONTROL); return; }
    total_encoder_twice += (int64_t)dl + dr;
    if (!update_yaw(now)) { fault_stop(FSM_FAULT_IMU); return; }
    if (!GraySensor_Scan()) { fault_stop(FSM_FAULT_ADC); return; }
    now = HAL_GetTick();
    update_marker(previous_peak, (uint32_t)(((dl < 0 ? -(int64_t)dl : dl) +
                                           (dr < 0 ? -(int64_t)dr : dr))), now);
    if (now - phase_tick >= ROUTE_PHASE_TIMEOUT_MS) { fault_stop(FSM_FAULT_ROUTE_TIMEOUT); return; }
    if (g_system_state == SYS_STATE_RUN_ABCD) {
        if (peak_event) {
            if (!record_marker(now)) return;
            target_straight_yaw = current_yaw;
            is_straight_locked = true;
        }
        if (is_straight_locked && now - peak_lockout_tick >= ROUTE_MARK_COOLDOWN_MS)
            is_straight_locked = false;
        heading = is_straight_locked;
    } else {
        if (peak_event && (g_acbd_phase != ACBD_TO_C || now - phase_tick >= 300u)) {
            if (!record_marker(now)) return;
            g_acbd_phase = (ACBD_SubPhase_t)((g_acbd_phase + 1u) % 4u);
            previous_steering = -1;
            if (g_acbd_phase == ACBD_TO_C) target_straight_yaw = wrap_yaw(current_yaw + 45.0f);
            if (g_acbd_phase == ACBD_TO_D) target_straight_yaw = wrap_yaw(current_yaw - 90.0f);
            line_seen_tick = now;
        }
        heading = g_acbd_phase == ACBD_TO_C || g_acbd_phase == ACBD_TO_D;
    }
    if (line_detected || heading) line_seen_tick = now;
    else if (now - line_seen_tick >= ROUTE_LINE_LOST_MS) { fault_stop(FSM_FAULT_LINE_LOST); return; }
    drive(g_system_state == SYS_STATE_RUN_ABCD ? ROUTE_ABCD_RPM : ROUTE_ACBD_RPM, heading);
}
void FSM_OLED_Update(void)
{
    char text[32];
    /* Running uses its existing scan; idle diagnostics must also be live.
     * A failed idle scan is shown as unknown, never as a valid old pattern. */
    if (!running()) GraySensor_Scan();
    OLED_NewFrame();
    if (g_system_state == SYS_STATE_FAULT) {
        snprintf(text, sizeof text, "FAULT %u CTL:%lu", (unsigned)g_fsm_fault, (unsigned long)Control_GetFault());
        OLED_PrintString(0, 0, text, &font16x16, OLED_COLOR_NORMAL);
        OLED_PrintString(0, 16, "STOP / ACK KEY", &font16x16, OLED_COLOR_NORMAL);
    } else if (g_system_state == SYS_STATE_STANDBY)
        OLED_PrintString(0, 0, "STANDBY", &font16x16, OLED_COLOR_NORMAL);
    else if (g_system_state == SYS_STATE_VISION)
        OLED_PrintString(0, 0, "VISION", &font16x16, OLED_COLOR_NORMAL);
    else if (g_system_state == SYS_STATE_BRAKE)
        OLED_PrintString(0, 0, "BRAKE", &font16x16, OLED_COLOR_NORMAL);
    else {
        snprintf(text, sizeof text, "RPM %.1f %.1f", (double)speed_L, (double)speed_R);
        OLED_PrintString(0, 0, text, &font16x16, OLED_COLOR_NORMAL);
        snprintf(text, sizeof text, "lap:%u/%u p:%u", g_current_laps, g_target_laps, (unsigned)g_acbd_phase);
        OLED_PrintString(0, 16, text, &font16x16, OLED_COLOR_NORMAL);
    }
    if (!running() && g_system_state != SYS_STATE_FAULT) {
        snprintf(text, sizeof text, "RPM %.1f %.1f", (double)speed_L, (double)speed_R);
        OLED_PrintString(0, 16, text, &font16x16, OLED_COLOR_NORMAL);
    }
    OLED_PrintString(0, 32, "IR:12345678", &font16x16, OLED_COLOR_NORMAL);
    memcpy(text, "   ????????", 12u);
    if (gray_valid && HAL_GetTick() - gray_tick <= 100u) {
        for (unsigned i = 0; i < 8; ++i) text[i + 3u] = gray_adc[i] > ROUTE_GRAY_LINE ? '1' : '0';
    }
    OLED_PrintString(0, 48, text, &font16x16, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
}
