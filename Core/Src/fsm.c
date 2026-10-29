#include "fsm.h"
#ifndef CAR_HOST_TEST
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
#include "line_follow.h"
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
static uint32_t peak_lockout_tick, beep_led_off_tick;
static uint32_t previous_l, previous_r, yaw_tick;
static bool beep_active, blink_on;
static volatile uint8_t blink_total_count, blink_cur_count;
static uint8_t g_beep_event_count;
static int64_t total_encoder_twice, lap_start_twice;
static bool is_on_peak, was_on_peak, is_straight_locked;
static float current_yaw, target_straight_yaw, mpu_bias[3];
static bool mpu_ready;
static Turn_PID_t heading_pid;
static LineSensor line_sensor;
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
static void brake(uint32_t now)
{
    Control_Stop();
    g_system_state = SYS_STATE_BRAKE;
    brake_start_time = now;
    Device_Beep_Led_Set(false);
    beep_active = false;
}
static void Enter_Standby_State(void)
{
    Control_Stop();
    g_system_state = SYS_STATE_STANDBY;
    Device_Beep_Led_Set(false);
    Serial_StopVision();
    beep_active = false;
    blink_on = false;
    g_current_laps = g_beep_event_count = 0;
    total_encoder_twice = lap_start_twice = 0;
    is_on_peak = was_on_peak = is_straight_locked = false;
    heading_pid.err = heading_pid.err_last = heading_pid.output = 0.0f;
    phase_tick = HAL_GetTick();
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
    GPIO_InitTypeDef ir_pin = {0};
    Motor_Init();
    Control_ConfigureDriveRPM();
    heading_pid.kp = ROUTE_HEADING_KP;
    heading_pid.kd = ROUTE_HEADING_KD;
    heading_pid.max_output = ROUTE_MAX_TURN_RPM;
    /* MX_ADC1_Init leaves PA2 in analog mode, where IDR never updates and every
     * sensor read returns low. Restore the digital pull-up input the mux output
     * actually needs; this runs after the ADC setup so it wins. */
    ir_pin.Pin = GPIO_PIN_2;
    ir_pin.Mode = GPIO_MODE_INPUT;
    ir_pin.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &ir_pin);
    /* A missing IMU only costs the straight-line heading hold; the route still
     * runs on line following. */
    initialize_imu();
    Enter_Standby_State();
}
static void GraySensor_Scan(void)
{
    uint8_t raw_levels = 0u;
    for (unsigned i = 0; i < 8; ++i) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (i & 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, (i & 2u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, (i & 4u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#ifndef CAR_HOST_TEST
        /* The mux output needs far longer than the 100us a bare 4051 would
         * imply: the sensor's pull-up and cable capacitance dominate. */
        uint32_t selected_at = DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT - selected_at) < SystemCoreClock / 1000u) { }
#endif
        uint8_t raw = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET ? 1u : 0u;
        if (IR_NORMALIZE_LEVEL(raw)) raw_levels |= (uint8_t)(1u << i);
    }
    LineSensor_Update(&line_sensor, raw_levels, HAL_GetTick());
    was_on_peak = is_on_peak;
    is_on_peak = line_sensor.frame.marker;
}
static float wrap_yaw(float value)
{
    while (value > 180.0f) value -= 360.0f;
    while (value < -180.0f) value += 360.0f;
    return value;
}
static void update_yaw(uint32_t now)
{
    MPU6050_Data_t data;
    uint32_t dt = now - yaw_tick;
    if (!mpu_ready || dt > ROUTE_IMU_MAX_GAP_MS || MPU6050_Read(&hi2c2, &data) != 0) return;
    float raw_yaw = MPU6050_RawYaw(&data);
    if (!(raw_yaw >= -500.0f && raw_yaw <= 500.0f)) return;
    current_yaw = wrap_yaw(current_yaw + MPU_YAW_SIGN * (raw_yaw - mpu_bias[MPU_YAW_AXIS]) * ((float)dt * 0.001f));
    yaw_tick = now;
}
/* One beep per marker. Every fourth marker is a lap, but only once the car has
 * actually travelled far enough since the previous one. */
static void record_marker(uint32_t now)
{
    ++g_beep_event_count;
    phase_tick = now;
    Device_Beep_Led_Set(true);
    beep_active = true;
    beep_led_off_tick = now;
    if (g_beep_event_count % 4u != 0u) return;
    if (total_encoder_twice - lap_start_twice < (int64_t)ROUTE_LAP_MIN_COUNTS * 2) return;
    lap_start_twice = total_encoder_twice;
    if (++g_current_laps >= g_target_laps) brake(now);
}
static void drive(float base)
{
    float turn = 0.0f;
    if (is_straight_locked && mpu_ready)
        turn = PID_Positional_Calc(&heading_pid, 0.0f, -wrap_yaw(target_straight_yaw - current_yaw));
    else if (!is_on_peak)
        turn = PID_Positional_Calc(&heading_pid, 0.0f, -line_sensor.frame.error);
    Control_DriveRPM(base, turn);
}
static void begin_run(uint32_t now)
{
    yaw_tick = now;
    update_yaw(now);
    GraySensor_Scan();
    total_encoder_twice = lap_start_twice = 0;
    g_current_laps = g_beep_event_count = 0;
    phase_tick = now;
    is_straight_locked = false;
    g_acbd_phase = ACBD_TO_C;
    /* Per the route spec: blue selects ABCD, red selects ACBD. */
    g_system_state = g_vision_color == COLOR_RED ? SYS_STATE_RUN_ACBD : SYS_STATE_RUN_ABCD;
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
        /* No vision reply keeps COLOR_UNKNOWN, which selects RUN_ABCD.
         * A 'B'/'R' byte during VISION overrides this default. */
        g_vision_color = COLOR_UNKNOWN;
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
    case SYS_STATE_FAULT:
        Control_ClearFault();
        g_fsm_fault = FSM_FAULT_NONE;
        Enter_Standby_State();
        break;
    case SYS_STATE_BRAKE:
        break;
    }
}
void FSM_Timer1s_Tick(void)
{
    if (g_system_state != SYS_STATE_VISION || blink_total_count == 0u) return;
    if (blink_cur_count >= blink_total_count) { Device_Beep_Led_Set(false); return; }
    blink_on = !blink_on;
    Device_Beep_Led_Set(blink_on);
    if (!blink_on) ++blink_cur_count;
}
void FSM_ControlLoop_10ms(void)
{
    ControlView v;
    int32_t dl, dr;
    uint32_t now = HAL_GetTick();
    Control_ReadView(&v);
    speed_L = Wheel_CountsToRPM(v.speed_l, CAR_WHEEL_CPR_LEFT);
    speed_R = Wheel_CountsToRPM(v.speed_r, CAR_WHEEL_CPR_RIGHT);
    dl = (int32_t)((uint32_t)v.position_l - previous_l);
    dr = (int32_t)((uint32_t)v.position_r - previous_r);
    previous_l = (uint32_t)v.position_l;
    previous_r = (uint32_t)v.position_r;
    if (v.fault) {
        Control_Stop();
        if (g_fsm_fault == FSM_FAULT_NONE) g_fsm_fault = FSM_FAULT_CONTROL;
        g_system_state = SYS_STATE_FAULT;
        Device_Beep_Led_Set(true);
        beep_active = false;
        return;
    }
    if (beep_active && now - beep_led_off_tick >= 80u) {
        Device_Beep_Led_Set(false);
        beep_active = false;
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
        if (now - brake_start_time >= 500u || v.quiet_ticks >= 30u) Enter_Standby_State();
        return;
    }
    if (!running()) { Control_Stop(); return; }
    total_encoder_twice += (int64_t)dl + dr;
    update_yaw(now);
    GraySensor_Scan();
    now = HAL_GetTick();
    if (g_system_state == SYS_STATE_RUN_ABCD) {
        /* A peak edge starts a 500ms IMU heading hold; the beep marks it. */
        if (is_on_peak != was_on_peak && now - peak_lockout_tick >= ROUTE_MARK_COOLDOWN_MS) {
            peak_lockout_tick = now;
            record_marker(now);
            target_straight_yaw = current_yaw;
            is_straight_locked = true;
        }
        if (is_straight_locked && now - peak_lockout_tick >= ROUTE_MARK_COOLDOWN_MS)
            is_straight_locked = false;
        drive(ROUTE_ABCD_RPM);
    } else {
        /* The C leg is driven blind on yaw until a sensor finds the line. */
        if (g_acbd_phase == ACBD_TO_C) {
            if (now - phase_tick >= ROUTE_PHASE_TIMEOUT_MS) { brake(now); return; }
            if (now - phase_tick > 300u && line_sensor.frame.raw_black) {
                record_marker(now);
                g_acbd_phase = ACBD_LINE_TO_B;
            }
        } else if (is_on_peak != was_on_peak && now - peak_lockout_tick >= ROUTE_MARK_COOLDOWN_MS) {
            peak_lockout_tick = now;
            record_marker(now);
            g_acbd_phase = (ACBD_SubPhase_t)((g_acbd_phase + 1u) % 4u);
            /* Each blind leg re-anchors on the heading the car has right now,
             * so a second lap does not reuse the first lap's target. */
            if (g_acbd_phase == ACBD_TO_C) {
                target_straight_yaw = wrap_yaw(current_yaw + 45.0f);
                phase_tick = now;
            }
            if (g_acbd_phase == ACBD_TO_D) target_straight_yaw = wrap_yaw(current_yaw - 90.0f);
        }
        is_straight_locked = g_acbd_phase == ACBD_TO_C || g_acbd_phase == ACBD_TO_D;
        drive(ROUTE_ACBD_RPM);
    }
}
void FSM_OLED_Update(void)
{
    /* 128x64 panel. font16x16 renders ASCII 8px wide and 16px tall, so a row
     * holds 16 characters. Three rows at 20px spacing (0/20/40) leave a 4px
     * gap; 16px spacing made consecutive rows touch. */
    char text[20];
    OLED_NewFrame();
    if (g_system_state == SYS_STATE_FAULT) {
        snprintf(text, sizeof text, "FAULT %u", (unsigned)g_fsm_fault);
        OLED_PrintString(0, 0, text, &font16x16, OLED_COLOR_NORMAL);
        snprintf(text, sizeof text, "CTL: %lu", (unsigned long)Control_GetFault());
        OLED_PrintString(0, 20, text, &font16x16, OLED_COLOR_NORMAL);
        OLED_PrintString(0, 40, "STOP/ACK", &font16x16, OLED_COLOR_NORMAL);
    } else if (g_system_state == SYS_STATE_STANDBY) {
        OLED_PrintString(0, 0, "cur: stay", &font16x16, OLED_COLOR_NORMAL);
        OLED_PrintString(0, 20, "motor: off", &font16x16, OLED_COLOR_NORMAL);
        OLED_PrintString(0, 40, "vs: off", &font16x16, OLED_COLOR_NORMAL);
    } else if (g_system_state == SYS_STATE_VISION) {
        OLED_PrintString(0, 0, "cur: vs", &font16x16, OLED_COLOR_NORMAL);
        snprintf(text, sizeof text, "run: %u", (unsigned)g_target_laps);
        OLED_PrintString(0, 20, text, &font16x16, OLED_COLOR_NORMAL);
        /* Blue selects ABCD and red selects ACBD, matching begin_run. */
        if (g_vision_color == COLOR_BLUE)
            OLED_PrintString(0, 40, "way:(BLUE)ABCD", &font16x16, OLED_COLOR_NORMAL);
        else if (g_vision_color == COLOR_RED)
            OLED_PrintString(0, 40, "way:(RED)ACBD", &font16x16, OLED_COLOR_NORMAL);
        else
            OLED_PrintString(0, 40, "nosign", &font16x16, OLED_COLOR_NORMAL);
    } else if (g_system_state == SYS_STATE_BRAKE) {
        OLED_PrintString(0, 0, "current: stop", &font16x16, OLED_COLOR_NORMAL);
    } else {
        OLED_PrintString(0, 0,
                         g_system_state == SYS_STATE_RUN_ABCD ? "cur: ABCDway" : "cur: ACBDway",
                         &font16x16, OLED_COLOR_NORMAL);
        snprintf(text, sizeof text, "end: %u/%u", (unsigned)g_current_laps,
                 (unsigned)g_target_laps);
        OLED_PrintString(0, 20, text, &font16x16, OLED_COLOR_NORMAL);
        snprintf(text, sizeof text, "rpm: %.0f %.0f", (double)speed_L, (double)speed_R);
        OLED_PrintString(0, 40, text, &font16x16, OLED_COLOR_NORMAL);
    }
    OLED_ShowFrame();
}
void FSM_VisionRxByte(uint8_t byte)
{
    if (g_system_state != SYS_STATE_VISION) return;
    if (uart_rx_idx == 0u) uart_rx_first = (char)byte;
    if (++uart_rx_idx < 2u) return;
    uart_rx_idx = 0;
    if (uart_rx_first < '1' || uart_rx_first > '5') return;
    char cmd = (char)byte;
    if (cmd == 'B' || cmd == 'b') g_vision_color = COLOR_BLUE;
    else if (cmd == 'R' || cmd == 'r') g_vision_color = COLOR_RED;
    else return;
    g_target_laps = (uint8_t)(uart_rx_first - '0');
    blink_total_count = g_target_laps;
    blink_cur_count = 0;
    blink_on = false;
}
