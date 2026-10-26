/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fsm.h"
#include "oled.h"
#include "string.h"
#include "control.h"
#include "autotune.h"
#include "serial_router.h"
#include "route_config.h"
#include <stdio.h>
#ifdef CAR_IMU_TEST
#include "imu_diagnostic.h"
#endif
#ifdef CAR_GROUND_TEST
#include "ground_test.h"
#include "line_follow.h"
#include "wheel_calibration.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef CAR_IMU_TEST
static void Imu_Test_Run(void)
{
  uint32_t display_tick = 0u;
  Motor_Init(); Control_Stop(); OLED_Init();
  ImuDiagnostic_Init(&hi2c2);
  while (1) {
    ImuDiagnostic_Poll(&hi2c2);
    if (HAL_GetTick() - display_tick >= 200u) {
      char text[24];
      display_tick = HAL_GetTick();
      OLED_NewFrame();
      if (imu_diagnostic.state == 2u) snprintf(text, sizeof text, "IMU MOTOR LOCK");
      else snprintf(text, sizeof text, "IMU FAULT:%lu", (unsigned long)imu_diagnostic.error);
      OLED_PrintString(0, 0, text, &font16x16, OLED_COLOR_NORMAL);
      for (unsigned axis = 0; axis < 3u; ++axis) {
        snprintf(text, sizeof text, "%c:%+.1f DEG", (int)('X' + axis), (double)imu_diagnostic.angle[axis]);
        OLED_PrintString(0, (uint8_t)(16u * (axis + 1u)), text, &font16x16, OLED_COLOR_NORMAL);
      }
      OLED_ShowFrame();
    }
    OLED_Poll();
  }
}
#endif
#ifdef CAR_GROUND_TEST
static void Ground_Run(void)
{
  uint32_t display_tick = 0u;
  const char *labels[] = {"GROUND READY", "WAIT 3 SEC", "RUN STRAIGHT", "STOPPING", "DONE STOP", "CANCEL STOP", "FAULT STOP"};
  Motor_Init();
  OLED_Init();
  Ground_Init();
  Control_StartTimer();
  while (1) {
    Ground_Heartbeat();
    if (HAL_GetTick() - display_tick >= 200u) {
      GroundResult r;
      ControlView v;
      char text[24];
      uint32_t saved = __get_PRIMASK();
      __disable_irq();
      r = ground_result;
      __set_PRIMASK(saved);
      display_tick = HAL_GetTick();
      Control_ReadView(&v);
      OLED_NewFrame();
#ifdef CAR_GROUND_LINE
      if (r.state == GROUND_READY) {
        LineFrame f = Line_Describe((uint8_t)ground_ir.raw_levels, (uint8_t)ground_ir.levels, ground_ir.valid, ground_ir.noisy, ground_ir.tick_ms);
        char *status = !ground_ir_direction ? "LINE MAP CHECK" : !f.valid ? "IR WARMUP" : f.noisy ? "IR NOISY" :
                       !f.raw_black ? "NEED BLACK LINE" : f.raw_black > 4u ? "LINE TOO WIDE" :
                       Line_Trackable(&f, HAL_GetTick(), 0) ? "LINE READY" : "WAIT IR STABLE";
        OLED_PrintString(0, 0, status, &font16x16, OLED_COLOR_NORMAL);
      } else if (r.state == GROUND_RUNNING)
        OLED_PrintString(0, 0, "RUN LINE", &font16x16, OLED_COLOR_NORMAL);
      else
#endif
      OLED_PrintString(0, 0, (char *)labels[r.state <= GROUND_FAULT ? r.state : GROUND_FAULT], &font16x16, OLED_COLOR_NORMAL);
      if (r.state == GROUND_READY) snprintf(text, sizeof text, "HOLD START 1S");
      else {
        float target = .5f * (Wheel_CountsToRPM(v.target_l, CAR_WHEEL_CPR_LEFT) + Wheel_CountsToRPM(v.target_r, CAR_WHEEL_CPR_RIGHT));
        snprintf(text, sizeof text, "RPM:%u MODE:STOP", (unsigned)(target + .5f));
      }
#ifdef CAR_GROUND_LINE
      if (r.state == GROUND_READY) {
        memcpy(text, "R:11111111 N:0", 15);
        for (unsigned i = 0; i < 8; ++i) text[2+i] = (ground_ir.raw_levels & (1u << i)) ? '1' : '0';
        text[13] = ground_ir.noisy ? '1' : '0';
      }
#endif
      OLED_PrintString(0, 16, text, &font16x16, OLED_COLOR_NORMAL);
      snprintf(text, sizeof text, "T:%lu N:%lu", (unsigned long)r.elapsed_ms, (unsigned long)r.count);
#ifdef CAR_GROUND_LINE
      memcpy(text, "IR:11111111", 12);
      for (unsigned i = 0; i < 8; ++i) text[3+i] = ground_ir.valid ? ((ground_ir.levels & (1u << i)) ? '1' : '0') : '?';
#endif
      OLED_PrintString(0, 32, text, &font16x16, OLED_COLOR_NORMAL);
      snprintf(text, sizeof text, "F:%lu Q:%lu", (unsigned long)r.reason, (unsigned long)v.quiet_ticks);
      OLED_PrintString(0, 48, text, &font16x16, OLED_COLOR_NORMAL);
#ifndef CAR_GROUND_LINE
      if (r.state == GROUND_READY) {
        snprintf(text, sizeof text, "HEAD READY X+");
        OLED_PrintString(0, 32, text, &font16x16, OLED_COLOR_NORMAL);
      }
      if (r.state >= GROUND_DONE && ground_motion_stats.ticks) {
        GroundMotionStats stats;
        saved = __get_PRIMASK(); __disable_irq();
        stats = ground_motion_stats; __set_PRIMASK(saved);
        if (stats.period_us) {
          float rpm_l = (float)stats.counts_l * 60000000.0f / ((float)CAR_WHEEL_CPR_LEFT * (float)stats.period_us);
          float rpm_r = (float)stats.counts_r * 60000000.0f / ((float)CAR_WHEEL_CPR_RIGHT * (float)stats.period_us);
          OLED_NewFrame();
          if (r.state == GROUND_DONE) {
            snprintf(text, sizeof text, "DONE H:%+.1f", (double)Ground_HeadingError());
            OLED_PrintString(0, 0, text, &font16x16, OLED_COLOR_NORMAL);
          } else OLED_PrintString(0, 0, (char *)labels[r.state <= GROUND_FAULT ? r.state : GROUND_FAULT], &font16x16, OLED_COLOR_NORMAL);
          snprintf(text, sizeof text, "L:%.1f R:%.1f", (double)rpm_l, (double)rpm_r);
          OLED_PrintString(0, 16, text, &font16x16, OLED_COLOR_NORMAL);
          snprintf(text, sizeof text, "PWM:%lu/%lu", (unsigned long)(stats.pwm_sum_l / stats.ticks),
              (unsigned long)(stats.pwm_sum_r / stats.ticks));
          OLED_PrintString(0, 32, text, &font16x16, OLED_COLOR_NORMAL);
          snprintf(text, sizeof text, "C:%lu/%lu F:%lu", (unsigned long)(100u * stats.capped_l / stats.ticks),
              (unsigned long)(100u * stats.capped_r / stats.ticks), (unsigned long)r.reason);
          OLED_PrintString(0, 48, text, &font16x16, OLED_COLOR_NORMAL);
        }
      }
#endif
      OLED_ShowFrame();
    }
    OLED_Poll();
  }
}
#endif
#ifdef CAR_IR_TEST
/* Standalone digital test: no FSM, IMU, serial commands or control timer. */
volatile uint8_t ir_test_levels[8];
volatile uint8_t ir_test_raw[8];
volatile uint8_t ir_test_changed;
volatile uint32_t ir_test_frames, ir_test_select_errors;
static void IR_Test_Run(void)
{
  GPIO_InitTypeDef input = {0};
  char bits[9] = "11111111", text[24];
  uint32_t display_tick = 0;
  uint8_t previous[8] = {0};
  input.Pin = GPIO_PIN_2;
  input.Mode = GPIO_MODE_INPUT;
  input.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &input);
  Motor_EmergencyStop();
  OLED_Init();
  while (1) {
    for (unsigned i = 0; i < 8; ++i) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (i & 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, (i & 2u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, (i & 4u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      /* Slow diagnostic scan; keep display transfers progressing while held. */
      uint32_t selected_at = HAL_GetTick();
      while (HAL_GetTick() - selected_at < 100u) OLED_Poll();
      unsigned selected = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) ? 1u : 0u)
        | (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) ? 2u : 0u)
        | (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) ? 4u : 0u);
      if (selected != i) ++ir_test_select_errors;
      ir_test_raw[i] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) ? 1u : 0u;
      ir_test_levels[i] = IR_NORMALIZE_LEVEL(ir_test_raw[i]);
      bits[i] = ir_test_levels[i] ? '1' : '0';
      if (ir_test_frames && previous[i] != ir_test_levels[i]) ir_test_changed |= (uint8_t)(1u << i);
      previous[i] = ir_test_levels[i];
      OLED_Poll();
    }
    ++ir_test_frames;
    if (HAL_GetTick() - display_tick >= 200u) {
      display_tick = HAL_GetTick();
      OLED_NewFrame();
      OLED_PrintString(0, 0, "IR TEST STOP", &font16x16, OLED_COLOR_NORMAL);
      OLED_PrintString(0, 16, "CH:12345678", &font16x16, OLED_COLOR_NORMAL);
      snprintf(text, sizeof text, "IN:%s", bits);
      OLED_PrintString(0, 32, text, &font16x16, OLED_COLOR_NORMAL);
      snprintf(text, sizeof text, "T:%02X E:%lu", (unsigned)ir_test_changed, (unsigned long)ir_test_select_errors);
      OLED_PrintString(0, 48, text, &font16x16, OLED_COLOR_NORMAL);
      OLED_ShowFrame();
    }
    OLED_Poll();
  }
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
#ifdef CAR_IMU_TEST
  Imu_Test_Run();
#endif
#ifdef CAR_GROUND_TEST
  Ground_Run();
#endif
#ifdef CAR_IR_TEST
  IR_Test_Run();
#endif
  Autotune_SelectBootMode();
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) Error_Handler();
  if (Autotune_IsEnabled()) Motor_Init();
  else {
    OLED_Init();
    Car_System_Init();
  }
  Control_StartTimer();
  if (Autotune_IsEnabled()) Autotune_Init();
  uint32_t tick_10ms = HAL_GetTick();
  uint32_t tick_100ms = tick_10ms;
  uint32_t tick_1s = tick_10ms;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    Serial_Poll();
    if (Autotune_IsEnabled()) {
      Autotune_Poll();
    } else {
      FSM_KeyScan();
      if (now - tick_10ms >= CONTROL_PERIOD_MS) {
        tick_10ms = now;
        FSM_ControlLoop_10ms(); /* Foreground sensors/route; PWM runs on TIM1. */
      }
      if (now - tick_100ms >= 200u) {
        tick_100ms = now;
        FSM_OLED_Update();
      }
      if (now - tick_1s >= 1000u) {
        tick_1s = now;
        FSM_Timer1s_Tick();
      }
      OLED_Poll(); /* One page command or 16-byte data chunk per iteration. */
    }
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  Motor_EmergencyStop();
  /* 错误指示: LED 快速闪烁 (PA7 低电平导通) */
  while (1) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
    for (volatile uint32_t i = 0; i < 200000; i++);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
