#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "stm32f4xx.h"
#include "stdlib.h"
#include "stdio.h"

//#include "BSP_GPIO.h"
//#include "BSP_DMA.h"
#include "BSP.h"

extern uint8_t *Array_USART1_RX;
extern uint8_t *Array_USART2_RX;
extern uint8_t *Array_USART3_RX;
extern uint8_t *Array_UART4_RX ;
extern uint8_t *Array_UART5_RX ;
extern uint8_t *Array_USART6_RX;
extern uint8_t *Array_UART7_RX ;
extern uint8_t *Array_UART8_RX ;

extern uint8_t *Array_USART1_TX;
extern uint8_t *Array_USART2_TX;
extern uint8_t *Array_USART3_TX;
extern uint8_t *Array_UART4_TX ;
extern uint8_t *Array_UART5_TX ;
extern uint8_t *Array_USART6_TX;
extern uint8_t *Array_UART7_TX ;
extern uint8_t *Array_UART8_TX ;

typedef struct{
	USART_TypeDef* USARTx;									//串口号       
	USART_InitTypeDef USART_InitStructure;	//串口初始化配置，包含波特率，校验等等
	BSP_GPIOSource_TypeDef *USART_TX;				//USART_TX 引脚
	BSP_GPIOSource_TypeDef *USART_RX;				//USART_RX 引脚
}BSP_USART_TypeDef;

typedef struct{
	uint16_t Length;
	uint8_t*	Array_Recv;
}BSP_USART_Recv_TypeDef;

uint32_t USART_TO_ArrayTX(USART_TypeDef* USARTx);
void BSP_USART_Init(BSP_USART_TypeDef *BSP_USARTx, uint8_t PreemptionPriority, uint8_t SubPriority);
void BSP_USART_TX_DMA_Init(BSP_USART_TypeDef *BSP_USARTx);
void BSP_USART_RX_DMA_Init(BSP_USART_TypeDef *BSP_USARTx);
void BSP_USART_SendData(BSP_USART_TypeDef *BSP_USARTx, uint8_t *Array, uint16_t len);
void BSP_USART_SendStr(BSP_USART_TypeDef *BSP_USARTx, uint8_t *Str);
uint8_t BSP_USART1_DMA_SendData(uint8_t *Array, uint16_t len);
uint8_t BSP_USART2_DMA_SendData(uint8_t *Array, uint16_t len);
uint8_t BSP_USART3_DMA_SendData(uint8_t *Array, uint16_t len);
uint8_t BSP_UART4_DMA_SendData(uint8_t *Array, uint16_t len);
uint8_t BSP_UART5_DMA_SendData(uint8_t *Array, uint16_t len);
uint8_t BSP_USART6_DMA_SendData(uint8_t *Array, uint16_t len);
uint8_t BSP_UART7_DMA_SendData(uint8_t *Array, uint16_t len);
uint8_t BSP_UART8_DMA_SendData(uint8_t *Array, uint16_t len);
uint8_t BSP_USART_DMA_SendData(USART_TypeDef *USARTx, uint8_t *Array, uint16_t len);
uint32_t marArrayTX(USART_TypeDef* USARTx,size_t len);
uint32_t marArrayRX(USART_TypeDef* USARTx,size_t len);
#endif
