#ifndef __POWER_CONTOL_H
#define __POWER_CONTOL_H

#include "bsp.h"

#define POWER_USARTX							  USART3			  //���ںţ�E8,E7 ��Ӧ����usart7��
#define POWER_USARTX_RX_PIN						  BSP_GPIOE8	//��������
#define POWER_USARTX_TX_PIN						  BSP_GPIOE7	//��������
#define POWER_USART_PreemptionPriority  2						//�ж���ռ���ȼ�
#define POWER_USART_SubPriority 				0						//�ж���Ӧ���ȼ�

void Driver_Power_Init(USART_TypeDef *USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,uint8_t PreemptionPriority,uint8_t SubPriority);
#endif