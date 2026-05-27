#include "bsp_uart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

volatile uint8_t Rx_Length = 0;
uint8_t Rx_Buf[128] = {0};
char Tx_Buf[128] = {0};

void Uart_StartReceive(UART_HandleTypeDef *huart)
{
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, Rx_Buf, 128);
}

int Uart_Printf(char *fmt, ...)
{
    int ret;
    va_list ap;
    va_start(ap, fmt);
    ret = vsprintf(Tx_Buf, fmt, ap);
    va_end(ap);
    if (ret > 0)
    {
        HAL_UART_Transmit(&SERIAL_UART, (uint8_t *)Tx_Buf, ret, HAL_MAX_DELAY);
    }
    return ret;
}

//void Uart_Printf(UART_HandleTypeDef huart,char *fmt, ...)
//{
//    char string[256];
//	uint8_t len;
//    va_list ap;
//    va_start(ap, fmt);
//    len = vsprintf(Tx_Buf, fmt, ap);
//    va_end(ap);
//    HAL_UART_Transmit(&huart, (uint8_t *)Tx_Buf, len, HAL_MAX_DELAY);
//}


void Uart_Receive(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_IDLE);
        HAL_UART_DMAStop(huart);

        Rx_Length = 128 - __HAL_DMA_GET_COUNTER(hdma_usart_rx);

        /* Private user code ---------------------------------------------------------*/
        /* USER CODE BEGIN */

        Uart_Function();

        /* USER CODE END */

        Rx_Length = 0;
        memset(Rx_Buf, 0, 128);
        HAL_UART_Receive_DMA(huart, Rx_Buf, 128);
    }
}

__weak void Uart_Function(void)
{
    /* CODE */
    HAL_UART_Transmit(&SERIAL_UART, (uint8_t *)"Rx:", 4, HAL_MAX_DELAY);
    HAL_UART_Transmit(&SERIAL_UART, Rx_Buf, Rx_Length, HAL_MAX_DELAY);
}
