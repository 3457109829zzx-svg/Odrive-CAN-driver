#include "bsp_can.h"
#include "usart.h"

void CAN_Filter_Init(void)
{

    CAN_FilterTypeDef CAN_FilterInitStructure;
    CAN_FilterInitStructure.FilterActivation = ENABLE;
    CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;
		CAN_FilterInitStructure.FilterIdHigh = 0x0000;//所有ID都能收到 不过滤
    CAN_FilterInitStructure.FilterIdLow = 0x0000;
    CAN_FilterInitStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.FilterBank = 0;
    CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterInitStructure);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    // HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterInitStructure);
    // HAL_CAN_Start(&hcan2);
    // HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

static CAN_TxHeaderTypeDef CAN_TxHeader;
static uint32_t CAN_TxMailbox = 0;

void CAN_Transmit(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Buf)
{

    if ((Buf != NULL))
    {
        CAN_TxHeader.StdId = ID;         /* 指定标准标识符，该值在0x00-0x7FF */
        CAN_TxHeader.IDE = CAN_ID_STD;   /* 指定将要传输消息的标识符类型 */
        CAN_TxHeader.RTR = CAN_RTR_DATA; /* 指定消息传输帧类型 */
        CAN_TxHeader.DLC = 8;            /* 指定将要传输的帧长度 */

        HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, Buf, &CAN_TxMailbox);
    }
}

static CAN_RxHeaderTypeDef CAN_RxHeader;
uint8_t CAN_RxBuf[8];

uint8_t isReceived;

// CAN中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxHeader, CAN_RxBuf);

    if (hcan->Instance == CAN1)
    {
        isReceived = 1;
    }
}
