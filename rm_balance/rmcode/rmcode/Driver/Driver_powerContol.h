#ifndef __POWER_CONTOL_H
#define __POWER_CONTOL_H

#include "bsp.h"

#define POWER_USARTX							  USART3			  //串口号（E8,E7 对应的是usart7）
#define POWER_USARTX_RX_PIN						  BSP_GPIOE8	//接收引脚
#define POWER_USARTX_TX_PIN						  BSP_GPIOE7	//发送引脚
#define POWER_USART_PreemptionPriority  2						//中断抢占优先级
#define POWER_USART_SubPriority 				0						//中断响应优先级

void Driver_Power_Init(USART_TypeDef *USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,uint8_t PreemptionPriority,uint8_t SubPriority);
#endif
