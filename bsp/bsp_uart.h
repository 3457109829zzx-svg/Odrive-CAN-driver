#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include "usart.h"

#define SERIAL_UART huart6

// 启动串口接收
void Uart_StartReceive(UART_HandleTypeDef *huart);
// 串口打印
int Uart_Printf(char *fmt, ...);
//void Uart_Printf(UART_HandleTypeDef huart,char *fmt, ...);
// 串口接收，用于中断函数内 HAL_UART_IRQHandler() 之前
void Uart_Receive(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx);
// 重定义函数
void Uart_Function(void);

#endif
