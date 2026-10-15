/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LeftEncoder1_Pin GPIO_PIN_0
#define LeftEncoder1_GPIO_Port GPIOA
#define LeftEncoder2_Pin GPIO_PIN_1
#define LeftEncoder2_GPIO_Port GPIOA
#define motorLeftBack_Pin GPIO_PIN_3
#define motorLeftBack_GPIO_Port GPIOA
#define motorLeftUp_Pin GPIO_PIN_4
#define motorLeftUp_GPIO_Port GPIOA
#define motorRightUp_Pin GPIO_PIN_5
#define motorRightUp_GPIO_Port GPIOA
#define motorRightBack_Pin GPIO_PIN_6
#define motorRightBack_GPIO_Port GPIOA
#define BuzzerLight_Pin GPIO_PIN_7
#define BuzzerLight_GPIO_Port GPIOA
#define motorLeftPWM_Pin GPIO_PIN_0
#define motorLeftPWM_GPIO_Port GPIOB
#define motorRightPWM_Pin GPIO_PIN_1
#define motorRightPWM_GPIO_Port GPIOB
#define MPU_SCL_Pin GPIO_PIN_10
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_11
#define MPU_SDA_GPIO_Port GPIOB
#define Start_Pin GPIO_PIN_14
#define Start_GPIO_Port GPIOB
#define ModeChange_Pin GPIO_PIN_15
#define ModeChange_GPIO_Port GPIOB
#define SensorS2_Pin GPIO_PIN_11
#define SensorS2_GPIO_Port GPIOA
#define SensorS1_Pin GPIO_PIN_12
#define SensorS1_GPIO_Port GPIOA
#define SensorS0_Pin GPIO_PIN_5
#define SensorS0_GPIO_Port GPIOB
#define RightEncoder1_Pin GPIO_PIN_6
#define RightEncoder1_GPIO_Port GPIOB
#define RightEncoder2_Pin GPIO_PIN_7
#define RightEncoder2_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
