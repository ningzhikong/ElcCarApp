#include "fsm.h"
#include "tim.h"
#include "adc.h"
#include "usart.h"
#include "oled.h"
#include "control.h"
#include "i2c.h"
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

/* 系统状态与全局变量 */
volatile SystemState_t g_system_state = SYS_STATE_STANDBY;
ACBD_SubPhase_t g_acbd_phase = ACBD_TO_C;
volatile uint8_t g_target_laps = 1;
volatile uint8_t g_current_laps = 0;
volatile VisionColor_t g_vision_color = COLOR_BLUE;



/* 计时与时间戳 */
static uint32_t vision_start_time = 0;
static uint32_t brake_start_time = 0;
static uint32_t peak_lockout_tick = 0;
static uint32_t blind_start_tick = 0;
static uint32_t beep_led_off_tick = 0; // 独立蜂鸣器/LED关闭倒计时

/* 编码器与脉冲累计 — volatile 防止编译器优化 */
 volatile int16_t speed_L = 0;
 volatile int16_t speed_R = 0;
static int32_t total_encoder_ticks = 0;
static int32_t last_lap_encoder_tick = 0;
static int32_t peak_maintain_pulse = 0;

/* 蜂鸣器与LED */
static uint8_t g_beep_event_count = 0;
static volatile uint8_t blink_total_count = 0;
static volatile uint8_t blink_cur_count = 0;

/* 传感器与姿态变量 */
static uint16_t gray_adc[8] = {0};
static float gray_error = 0.0f;
static bool is_on_peak = false;
static bool was_on_peak = false;
static float current_yaw = 0.0f;      // 实际 MPU6050 航向角
static float target_straight_yaw = 0.0f;
static bool is_straight_locked = false;

/* MPU6050 零偏与就绪标志 */
static float mpu_bias[3] = {0.0f, 0.0f, 0.0f}; // gx, gy, gz 零偏
static bool mpu_ready = false;

/* 串口接收与握手缓冲区 */
static uint8_t uart_rx_byte;
static char uart_rx_buf[16];
static volatile uint8_t uart_rx_idx = 0;

/* --- 硬件底层驱动接口 --- */

// 控制 PA7 蜂鸣器与 LED (低电平导通点亮)
static void Device_Beep_Led_Set(bool enable) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

// 触发一次通电 (固定 80ms 自动关闭)
static void Trigger_Beep_Led_80ms(void) {
    Device_Beep_Led_Set(true);
    beep_led_off_tick = HAL_GetTick() + 80;
}

// 74HC4051 通道切换 (PB5:S0, PA12:S1, PA11:S2)
static void Mux_Select(uint8_t ch) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,  (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// 采集 8 路灰度模拟量
static void GraySensor_Scan(void) {
    uint32_t total_sum = 0;
    float weighted_sum = 0.0f;
    uint8_t black_count = 0;

    for (uint8_t i = 0; i < 8; i++) {
        Mux_Select(i);
        for (volatile int d = 0; d < 30; d++); // 延时稳定模拟开关
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            gray_adc[i] = HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);

        total_sum += gray_adc[i];
        weighted_sum += gray_adc[i] * ((float)i - 3.5f);

        if (gray_adc[i] > 2800) { // 阈值判定为黑线
            black_count++;
        }
    }

    if (total_sum > 400) {
        gray_error = weighted_sum / (float)total_sum;
    }

    was_on_peak = is_on_peak;
    is_on_peak = (black_count >= 5 || (gray_adc[3] > 3000 && gray_adc[4] > 3000));
}

// 检测是否有任一探头扫到黑线
static bool GraySensor_AnyLineDetected(void) {
    for (uint8_t i = 0; i < 8; i++) {
        if (gray_adc[i] > 2600) return true;
    }
    return false;
}

/* --- 状态切换与初始化 --- */

static void Enter_Standby_State(void) {
    g_system_state = SYS_STATE_STANDBY;

    // 参数清零
    Motor_SetPWM(0, 0);
    Device_Beep_Led_Set(false);
    HAL_UART_AbortReceive(&huart1);

    // 彻底清空电机闭环 PID 状态
    pid_L.output = 0; pid_L.err = 0; pid_L.err_last = 0; pid_L.err_prev = 0;
    pid_R.output = 0; pid_R.err = 0; pid_R.err_last = 0; pid_R.err_prev = 0;

    g_current_laps = 0;
    g_beep_event_count = 0;
    total_encoder_ticks = 0;
    last_lap_encoder_tick = 0;
    peak_maintain_pulse = 0;
    peak_lockout_tick = 0;
    is_on_peak = false;
    was_on_peak = false;
    is_straight_locked = false;
    blind_start_tick = 0;
    beep_led_off_tick = 0;
    pid_turn.err = 0.0f;
    pid_turn.err_last = 0.0f;
    pid_turn.output = 0.0f;
}

void Car_System_Init(void) {
    // 启动底盘定时器、PWM 与编码器
    Motor_Init();

    // MPU6050 初始化 + 静止零偏标定（竖放: yaw 取芯片 Y 轴）
    if (MPU6050_Init(&hi2c2) == 0) {
        mpu_ready = true;
        MPU6050_CalibrateBias(&hi2c2, &mpu_bias[0], &mpu_bias[1], &mpu_bias[2], 200);
    }

    // 默认上电进入待机
    Enter_Standby_State();

    // 纠正外环转向参数：限幅降到 18.0f 脉冲，防止暴力飞车
    pid_turn.kp = 22.0f;
    pid_turn.kd = 8.0f;
    pid_turn.max_output = 18.0f; 
}

/* --- 按键状态机跳转逻辑 --- */

void FSM_KeyScan(void) {
    static GPIO_PinState last_key = GPIO_PIN_SET;
    GPIO_PinState cur_key = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);

    // MODECHANGE (PB15) 下降沿
    if (last_key == GPIO_PIN_SET && cur_key == GPIO_PIN_RESET) {
        HAL_Delay(20); // 消抖
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) {
            switch (g_system_state) {
                case SYS_STATE_STANDBY: {
                    g_system_state = SYS_STATE_VISION;
                    vision_start_time = HAL_GetTick();
                    g_vision_color = COLOR_BLUE;
                    g_target_laps = 1;
                    blink_total_count = 0;
                    blink_cur_count = 0;
                    uart_rx_idx = 0;
                    uint8_t req_data[] = "START\r\n";
                    HAL_UART_Transmit(&huart1, req_data, strlen((char*)req_data), 100);
                    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
                    break;
                }

                case SYS_STATE_VISION:
                    // 兜底发车参数
                    if (g_vision_color == COLOR_UNKNOWN) {
                        g_vision_color = COLOR_RED; // 默认红色 ABCD
                        g_target_laps  = 1;
                    }

                    if (g_vision_color == COLOR_RED) {
                        g_system_state = SYS_STATE_RUN_ABCD;
                    } else if (g_vision_color == COLOR_BLUE) {
                        g_system_state = SYS_STATE_RUN_ACBD;
                        g_acbd_phase   = ACBD_TO_C;
                    }

                    // 关键：进入 RUN 状态前清零 PID 累积状态，防止左右不对称
                    pid_L.output = 0; pid_L.err = 0; pid_L.err_last = 0; pid_L.err_prev = 0;
                    pid_R.output = 0; pid_R.err = 0; pid_R.err_last = 0; pid_R.err_prev = 0;

                    Device_Beep_Led_Set(false);
                    blink_total_count     = 0;
                    blink_cur_count       = 0;
                    total_encoder_ticks   = 0;
                    last_lap_encoder_tick = 0;
                    g_beep_event_count    = 0;
                    g_current_laps        = 0;
                    break;

                case SYS_STATE_RUN_ABCD:
                case SYS_STATE_RUN_ACBD:
                    // 循迹中按按键强制切入刹车
                    g_system_state = SYS_STATE_BRAKE;
                    brake_start_time = HAL_GetTick();
                    break;

                case SYS_STATE_BRAKE:
                    break;
            }
        }
    }
    last_key = cur_key;
}

/* --- 串口接收 --- */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1 && g_system_state == SYS_STATE_VISION) {
        if (uart_rx_idx < sizeof(uart_rx_buf) - 1) {
            uart_rx_buf[uart_rx_idx++] = uart_rx_byte;
        }

        if (uart_rx_idx >= 2) {
            char num_char = uart_rx_buf[0];
            char cmd_char = uart_rx_buf[1];

            if (num_char >= '1' && num_char <= '5') {
                if (cmd_char == 'B' || cmd_char == 'b') {
                    g_target_laps = num_char - '0';
                    g_vision_color = COLOR_BLUE;
                    blink_total_count = g_target_laps;
                    blink_cur_count = 0;
                    uart_rx_idx = 0;
                } else if (cmd_char == 'R' || cmd_char == 'r') {
                    g_target_laps = num_char - '0';
                    g_vision_color = COLOR_RED;
                    blink_total_count = g_target_laps;
                    blink_cur_count = 0;
                    uart_rx_idx = 0;
                }
            }
        }

        if (uart_rx_idx >= sizeof(uart_rx_buf) - 1 || uart_rx_byte == '\n') {
            uart_rx_idx = 0;
        }

        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
}

/* --- 1s 中断处理视觉识别闪烁 --- */
void FSM_Timer1s_Tick(void) {
    if (g_system_state == SYS_STATE_VISION && blink_total_count > 0) {
        if (blink_cur_count < blink_total_count) {
            static bool toggle = false;
            toggle = !toggle;
            Device_Beep_Led_Set(toggle);
            if (!toggle) {
                blink_cur_count++;
            }
        } else {
            Device_Beep_Led_Set(false);
        }
    }
}

/* --- 10ms 中断核心闭环控制逻辑 --- */

void FSM_ControlLoop_10ms(void) {
    uint32_t now = HAL_GetTick();

    // 1. 每 10ms 读取左右轮真实编码器速度脉冲
    Encoder_ReadSpeeds(&speed_L, &speed_R);
    total_encoder_ticks += (speed_L + speed_R) / 2;
    

    // 1.5 MPU6050 航向积分（竖放 yaw=GYRO_Y；待机不累计，避免长期漂移）
    if (mpu_ready && g_system_state != SYS_STATE_STANDBY) {
        MPU6050_Data_t mpu;
        if (MPU6050_Read(&hi2c2, &mpu) == 0) {
            current_yaw += MPU_YAW_SIGN * (mpu.gy - mpu_bias[1]) * 0.01f; // dt=10ms
        }
    }

    // 2. 独立蜂鸣器/LED 80ms 自动断电倒计时
    if (beep_led_off_tick > 0 && now >= beep_led_off_tick) {
        Device_Beep_Led_Set(false);
        beep_led_off_tick = 0;
    }

    switch (g_system_state) {
        case SYS_STATE_STANDBY:
            pid_L.output = 0;
            pid_R.output = 0;
            Motor_SetPWM(0, 0);
            Device_Beep_Led_Set(false);
            break;

        case SYS_STATE_VISION:
            pid_L.output = 0;
            pid_R.output = 0;
            Motor_SetPWM(0, 0);

            if (now - vision_start_time > 10000 && blink_total_count == 0) {
                Enter_Standby_State();
            }
            break;

        case SYS_STATE_RUN_ABCD: {
            GraySensor_Scan();

            // 顶峰脉冲累计防抖
            if (is_on_peak) {
                peak_maintain_pulse += abs((speed_L + speed_R) / 2);
            } else {
                peak_maintain_pulse = 0;
            }

            // 顶峰判定：脉冲超 20 且离开 500ms 锁定区
            if ((now - peak_lockout_tick > 500) && (peak_maintain_pulse > 20)) {
                if ((!was_on_peak && is_on_peak) || (was_on_peak && !is_on_peak)) {
                    peak_lockout_tick = now;
                    is_straight_locked = true;
                    target_straight_yaw = current_yaw; // MPU 锁直线
                    Trigger_Beep_Led_80ms();           // 触发通电 80ms
                    g_beep_event_count++;

                    if (g_beep_event_count % 4 == 0) {
                        g_current_laps++;
                    }

                    // 4*N 终点判定 (必须超过 3000 脉冲)
                    if (g_beep_event_count >= (4 * g_target_laps)) {
                        if (total_encoder_ticks - last_lap_encoder_tick > 3000) {
                            g_system_state = SYS_STATE_BRAKE;
                            brake_start_time = now;
                            return;
                        }
                    }
                    last_lap_encoder_tick = total_encoder_ticks;
                }
            }

            // 500ms 锁定结束交回灰度循迹
            if (is_straight_locked && (now - peak_lockout_tick > 500)) {
                is_straight_locked = false;
            }

            // 外环转向计算
            float turn_diff = 0.0f;
            if (is_straight_locked) {
                turn_diff = PID_Positional_Calc(&pid_turn, target_straight_yaw, current_yaw);
            } else {
                turn_diff = PID_Positional_Calc(&pid_turn, 0.0f, -gray_error);
            }

            // BUG FIX: 之前这里把 turn_diff 算出来却传了 0.0f，导致 ABCD 循迹完全没有转向修正。
            // 现在把 turn_diff 真正喂给闭环。注意：转向下发前的正负号（朝线偏哪边转）需在实车上校准，
            // 若发现车往背离黑线方向跑，把上面的 -gray_error 改成 gray_error（或给 turn_diff 取反）。
            ClosedLoop_Drive(10.0f, turn_diff, speed_L, speed_R);
            break;
        }

        case SYS_STATE_RUN_ACBD: {
            GraySensor_Scan();

            switch (g_acbd_phase) {
                case ACBD_TO_C:
                    if (blind_start_tick == 0) {
                        blind_start_tick = now;
                        target_straight_yaw = current_yaw + 45.0f; // 转向 C 点
                    }
                    if (now - blind_start_tick > 300) {
                        if (GraySensor_AnyLineDetected()) {
                            if (was_on_peak && !is_on_peak && (now - peak_lockout_tick > 500)) {
                                peak_lockout_tick = now;
                                Trigger_Beep_Led_80ms();
                                g_beep_event_count++;
                                g_acbd_phase = ACBD_LINE_TO_B;
                            }
                        }
                    }
                    if (now - blind_start_tick > 10000) {
                        g_system_state = SYS_STATE_BRAKE;
                        brake_start_time = now;
                        return;
                    }
                    break;

                case ACBD_LINE_TO_B:
                    if (!was_on_peak && is_on_peak && (now - peak_lockout_tick > 500)) {
                        peak_lockout_tick = now;
                        Trigger_Beep_Led_80ms();
                        g_beep_event_count++;
                        blind_start_tick = 0;
                        g_acbd_phase = ACBD_TO_D;
                    }
                    break;

                case ACBD_TO_D:
                    if (blind_start_tick == 0) {
                        blind_start_tick = now;
                        target_straight_yaw = current_yaw - 90.0f; // 转向 D 点
                    }
                    if (was_on_peak && !is_on_peak && (now - peak_lockout_tick > 500)) {
                        peak_lockout_tick = now;
                        Trigger_Beep_Led_80ms();
                        g_beep_event_count++;
                        g_acbd_phase = ACBD_LINE_TO_A;
                    }
                    break;

                case ACBD_LINE_TO_A:
                    if (!was_on_peak && is_on_peak && (now - peak_lockout_tick > 500)) {
                        peak_lockout_tick = now;
                        Trigger_Beep_Led_80ms();
                        g_beep_event_count++;
                        g_current_laps++;

                        if (g_beep_event_count >= (4 * g_target_laps)) {
                            if (total_encoder_ticks - last_lap_encoder_tick > 3000) {
                                g_system_state = SYS_STATE_BRAKE;
                                brake_start_time = now;
                                return;
                            }
                        }
                        last_lap_encoder_tick = total_encoder_ticks;
                        blind_start_tick = 0;
                        g_acbd_phase = ACBD_TO_C;
                    }
                    break;
            }

            float turn_diff = PID_Positional_Calc(&pid_turn, target_straight_yaw, current_yaw);
            ClosedLoop_Drive(25.0f, turn_diff, speed_L, speed_R);
            break;
        }

        case SYS_STATE_BRAKE: {
            Device_Beep_Led_Set(false);

            // 500ms 超时强退待机
            if (now - brake_start_time > 500) {
                Enter_Standby_State();
                break;
            }

            // 刹车防抽搐：降至低速直接彻底刹停切断
            if (abs(speed_L) > 4 || abs(speed_R) > 4) {
                ClosedLoop_Drive(0.0f, 0.0f, speed_L, speed_R);
            } else {
                Motor_SetPWM(0, 0);
                Enter_Standby_State();
            }
            break;
        }
    }
}

/* --- OLED 状态文本显示任务 --- */

void FSM_OLED_Update(void) {
    char disp_buf[32];
    OLED_NewFrame();

    switch (g_system_state) {
        case SYS_STATE_STANDBY:
            // 在屏幕中央显示 STANDBY
            OLED_PrintString(20, 20, "STANDBY", &font16x16, OLED_COLOR_NORMAL);
            break;

        case SYS_STATE_VISION:
            OLED_PrintString(20, 20, "VISION", &font16x16, OLED_COLOR_NORMAL);
            break;

        case SYS_STATE_RUN_ABCD:
            GraySensor_Scan();
            // 调试：显示灰度值和左右轮速度
            snprintf(disp_buf, sizeof(disp_buf), "g:%d %d %d %d", gray_adc[0], gray_adc[1], gray_adc[2], gray_adc[3]);
            OLED_PrintString(10, 0, disp_buf, &font16x16, OLED_COLOR_NORMAL);
            snprintf(disp_buf, sizeof(disp_buf), "L:%d R:%d", speed_L, speed_R);
            OLED_PrintString(10, 16, disp_buf, &font16x16, OLED_COLOR_NORMAL);
            snprintf(disp_buf, sizeof(disp_buf), "e:%.1f", gray_error);
            OLED_PrintString(10, 32, disp_buf, &font16x16, OLED_COLOR_NORMAL);
            snprintf(disp_buf, sizeof(disp_buf), "g:%d %d %d %d", gray_adc[4], gray_adc[5], gray_adc[6], gray_adc[7]);
            OLED_PrintString(10, 48, disp_buf, &font16x16, OLED_COLOR_NORMAL);
            break;

        case SYS_STATE_RUN_ACBD:
            // 调试：显示左右轮速度和IMU航向角
            snprintf(disp_buf, sizeof(disp_buf), "L:%d R:%d", speed_L, speed_R);
            OLED_PrintString(10, 16, disp_buf, &font16x16, OLED_COLOR_NORMAL);
            snprintf(disp_buf, sizeof(disp_buf), "yaw:%.1f", current_yaw);
            OLED_PrintString(10, 32, disp_buf, &font16x16, OLED_COLOR_NORMAL);
            break;

        case SYS_STATE_BRAKE:
            OLED_DrawRectangle(0, 0, 127, 63, OLED_COLOR_REVERSED);
            OLED_PrintString(10, 24, "BRAKE", &font16x16, OLED_COLOR_NORMAL);
            break;
    }

    OLED_ShowFrame();
}
