#ifndef __INTERRUPT_SERVICE_FUN_H
#define __INTERRUPT_SERVICE_FUN_H

#include "type_can_isp.h"

typedef struct {
	void (*usart1)	(uint8_t * array,uint16_t arrayLen);
	void (*usart2)	(uint8_t * array,uint16_t arrayLen);
	void (*usart3)	(uint8_t * array,uint16_t arrayLen);
	void (*uart4)	(uint8_t * array,uint16_t arrayLen);
	void (*uart5)	(uint8_t * array,uint16_t arrayLen);
	void (*usart6)	(uint8_t * array,uint16_t arrayLen);
	void (*uart7)	(uint8_t * array,uint16_t arrayLen);
	void (*uart8)	(uint8_t * array,uint16_t arrayLen);
}interruptUsartService_t;

typedef struct {
	void (*can1)	(CanRxMsg *can_rx_msg);
	void (*can2)	(CanRxMsg *can_rx_msg);
}interruptCanService_t;

interruptUsartService_t * getUsartInterruptFunc(void);
interruptCanService_t* getCanInterruptFunc(void);

void interruptServiceFunc_can1(CanRxMsg *can_rx_msg);
void interruptServiceFunc_can2(CanRxMsg *can_rx_msg);
void interruptServiceFunc_usart1(uint8_t * array,uint16_t arrayLen);
void interruptServiceFunc_usart2(uint8_t * array,uint16_t arrayLen);
void interruptServiceFunc_usart3(uint8_t * array,uint16_t arrayLen);
void interruptServiceFunc_uart4(uint8_t * array,uint16_t arrayLen);
void interruptServiceFunc_uart5(uint8_t * array,uint16_t arrayLen);
void interruptServiceFunc_usart6(uint8_t * array,uint16_t arrayLen);
void interruptServiceFunc_uart7(uint8_t * array,uint16_t arrayLen);
void interruptServiceFunc_uart8(uint8_t * array,uint16_t arrayLen);

#endif
