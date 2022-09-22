#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f4xx.h"
#include "stdlib.h"
#include "stdio.h"

#include "BSP_GPIO.h"

typedef struct
{
	CAN_TypeDef* CANx;			//CAN通道
	BSP_GPIOSource_TypeDef *CANx_RX;	//CANx_RX 引脚
	BSP_GPIOSource_TypeDef *CANx_TX;	//CANx_TX 引脚
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;	//CAN通道配置，包含波特率等等
}BSP_CAN_TypeDef;

typedef struct
{
	CAN_TypeDef* 	CANx;
	CanTxMsg* 		TxMessage;
}BSP_CANSend_TypeDef;

void BSP_CAN_Mode_Init(BSP_CAN_TypeDef *BSP_CANx,uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode,uint8_t PreemptionPriority,uint8_t SubPriority);
 uint8_t	 BSP_CAN_Send_Msg(BSP_CAN_TypeDef *BSP_CANx,CanTxMsg *TxMessage);
 uint8_t	 BSP_CAN_Receive_Msg(BSP_CAN_TypeDef *BSP_CANx,CanRxMsg *RxMessage);

extern const CAN_FilterInitTypeDef  CAN1_FilterInitStructure;
extern const CAN_FilterInitTypeDef  CAN2_FilterInitStructure;

#endif
