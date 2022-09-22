#ifndef __DRIVER_DTU_H
#define __DRIVER_DTU_H

#include "bsp.h"
#include "BSP_GPIO.h"
#include "driver.h"


/**********************************DTU��ʼ������****************************************/
//DTU���ں�
#define WIRELESS_USARTX								USART6
//DTU��������
#define WIRELESS_USARTX_RX_PIN						BSP_GPIOC7	
//DTU��������
#define WIRELESS_USARTX_TX_PIN						BSP_GPIOC6	
//WIRELESS_USART�ж���ռ���ȼ�
#define WIRELESS_USART_PreemptionPriority 			2					
//WIRELESS_USART�ж���Ӧ���ȼ�
#define WIRELESS_USART_SubPriority 					0						

#endif






