#include "serial_router.h"
#include "autotune.h"
#include "fsm.h"
#include "usart.h"

static uint8_t rx_byte;
static volatile uint8_t vision_error;

static void start_receive(void)
{
    if (HAL_UART_Receive_IT(&huart1, &rx_byte, 1u) != HAL_OK) {
        if (Autotune_IsEnabled()) Autotune_OnUartError();
        else vision_error = 1u;
    }
}

void Serial_StartVision(void)
{
    if (Autotune_IsEnabled()) return;
    HAL_UART_AbortReceive(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    vision_error = 0u;
    start_receive();
}

void Serial_StopVision(void)
{
    if (!Autotune_IsEnabled()) HAL_UART_AbortReceive(&huart1);
}

void Serial_StartAutotune(void)
{
    HAL_UART_AbortReceive(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    start_receive();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;
    if (Autotune_IsEnabled()) {
        Autotune_OnByte(rx_byte);
        start_receive();
    } else if (g_system_state == SYS_STATE_VISION) {
        FSM_VisionRxByte(rx_byte);
        if (g_system_state == SYS_STATE_VISION) start_receive();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;
    if (Autotune_IsEnabled()) Autotune_OnUartError();
    else vision_error = 1u;
}

void Serial_Poll(void)
{
    if (vision_error && !Autotune_IsEnabled()) {
        vision_error = 0u;
        HAL_UART_AbortReceive(&huart1);
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        if (g_system_state == SYS_STATE_VISION) start_receive();
    }
}
