#ifndef CAR_TEST_HAL_H
#define CAR_TEST_HAL_H
#include <stdint.h>
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
#define Start_GPIO_Port ((void *)1)
#define Start_Pin 14u
#define ModeChange_GPIO_Port ((void *)1)
#define ModeChange_Pin 15u
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_BUSY 2
typedef int I2C_HandleTypeDef;
#define GPIOA ((void *)1)
#define GPIOB ((void *)2)
#define GPIO_PIN_2 4u
#define GPIO_MODE_INPUT 0u
#define GPIO_PULLUP 1u
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *config);
#define GPIO_PIN_5 32u
#define GPIO_PIN_7 128u
#define GPIO_PIN_11 2048u
#define GPIO_PIN_12 4096u
#define I2C_FLAG_BUSY 1u
extern int test_i2c_busy;
#define __HAL_I2C_GET_FLAG(h, flag) ((void)(h), (void)(flag), test_i2c_busy)
int HAL_ADC_Start(void *h);
int HAL_ADC_Stop(void *h);
int HAL_ADC_PollForConversion(void *h, uint32_t timeout);
uint32_t HAL_ADC_GetValue(void *h);
int HAL_I2C_Master_Transmit(void *h, uint16_t address, uint8_t *data, uint16_t n, uint32_t timeout);
int HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t address, uint16_t reg, uint16_t width,
                     uint8_t *data, uint16_t n, uint32_t timeout);
int HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t address, uint16_t reg, uint16_t width,
                      uint8_t *data, uint16_t n, uint32_t timeout);
extern int huart1;
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin);
int HAL_UART_Transmit(void *uart, uint8_t *data, uint16_t length, uint32_t timeout);

typedef struct {
    uint32_t CR1, PSC, ARR, EGR, SR, DIER, CNT, CCR3, CCR4, CCER;
} TestTimer;
typedef struct { uint32_t APB1ENR; } TestRcc;
typedef struct { uint32_t DEMCR; } TestCoreDebug;
typedef struct { uint32_t CYCCNT, CTRL; } TestDwt;
typedef struct { uint32_t APB2CLKDivider; } RCC_ClkInitTypeDef;
extern TestTimer test_tim1, test_tim2, test_tim3, test_tim4;
extern TestRcc test_rcc;
extern TestCoreDebug test_debug;
extern TestDwt test_dwt;
extern uint32_t SystemCoreClock;
extern int htim2, htim3, htim4;
#define TIM1 (&test_tim1)
#define TIM2 (&test_tim2)
#define TIM3 (&test_tim3)
#define TIM4 (&test_tim4)
#define RCC (&test_rcc)
#define CoreDebug (&test_debug)
#define DWT (&test_dwt)
#define RCC_APB1ENR_TIM3EN 1u
#define TIM_CCER_CC3E 256u
#define TIM_CCER_CC4E 4096u
#define TIM_CHANNEL_ALL 0xffffu
#define TIM_CHANNEL_3 3u
#define TIM_CHANNEL_4 4u
#define TIM_EGR_UG 1u
#define TIM_DIER_UIE 1u
#define TIM_CR1_CEN 1u
#define TIM_SR_UIF 1u
#define RCC_HCLK_DIV1 1u
#define CoreDebug_DEMCR_TRCENA_Msk 1u
#define DWT_CTRL_CYCCNTENA_Msk 1u
#define TIM1_UP_IRQn 25
#define __HAL_RCC_TIM1_CLK_ENABLE() ((void)0)
#define motorLeftUp_GPIO_Port ((void *)1)
#define motorLeftUp_Pin 4u
#define motorLeftBack_GPIO_Port ((void *)1)
#define motorLeftBack_Pin 3u
#define motorRightUp_GPIO_Port ((void *)1)
#define motorRightUp_Pin 5u
#define motorRightBack_GPIO_Port ((void *)1)
#define motorRightBack_Pin 6u
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state);
int HAL_TIM_Encoder_Start(void *timer, unsigned channel);
int HAL_TIM_PWM_Start(void *timer, unsigned channel);
uint32_t HAL_RCC_GetPCLK2Freq(void);
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *clocks, uint32_t *latency);
void HAL_NVIC_SetPriority(int irq, unsigned priority, unsigned subpriority);
void HAL_NVIC_EnableIRQ(int irq);
void Error_Handler(void);
#endif
