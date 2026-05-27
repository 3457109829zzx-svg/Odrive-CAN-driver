#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stdbool.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
// extern CAN_HandleTypeDef hcan2;

// 配置can过滤器
extern void CAN_Filter_Init(void);
// 把can的buf中的内容发送出去, 长度为8
extern void CAN_Transmit(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Buf);

#endif
