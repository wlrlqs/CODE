#ifndef __DRIVER_DTU_H
#define __DRIVER_DTU_H

#include "bsp.h"
#include "BSP_GPIO.h"
#include "driver.h"


/**********************************DTU初始化串口****************************************/
//DTU串口号
#define WIRELESS_USARTX								USART6
//DTU接收引脚
#define WIRELESS_USARTX_RX_PIN						BSP_GPIOC7	
//DTU发送引脚
#define WIRELESS_USARTX_TX_PIN						BSP_GPIOC6	
//WIRELESS_USART中断抢占优先级
#define WIRELESS_USART_PreemptionPriority 			2					
//WIRELESS_USART中断响应优先级
#define WIRELESS_USART_SubPriority 					0						

#endif






