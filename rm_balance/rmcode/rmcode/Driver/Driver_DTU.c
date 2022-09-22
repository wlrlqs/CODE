#include "Driver_DTU.h"
/***************************************/
static void DTUInit(void);
/***************************************/
deviceInitClass DTUClass = {
	DTUInit,
};
/*
***************************************************
函数名：Driver_DTU_Init
功能：无线数传初始化
入口参数：	I can understand it at one glance.
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
static void Driver_DTU_Init(USART_TypeDef *USARTx,\
							BSP_GPIOSource_TypeDef *USART_RX,\
							BSP_GPIOSource_TypeDef *USART_TX,\
							uint8_t PreemptionPriority,uint8_t SubPriority){
	BSP_USART_TypeDef WIRELESS_USART;
	WIRELESS_USART.USARTx = USARTx;
	WIRELESS_USART.USART_RX = USART_RX;
	WIRELESS_USART.USART_TX = USART_TX;
	//波特率为500000
	WIRELESS_USART.USART_InitStructure.USART_BaudRate = 500000;	
	//字长为8位数据格式
	WIRELESS_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	//一个停止位
	WIRELESS_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;
	//无校验位
	WIRELESS_USART.USART_InitStructure.USART_Parity = USART_Parity_No;
	//接收/发送模式
	WIRELESS_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	//无硬件数据流控制
	WIRELESS_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,		
	
	BSP_USART_Init(&WIRELESS_USART,PreemptionPriority,SubPriority);
	BSP_USART_RX_DMA_Init(&WIRELESS_USART);																	
}
/****************************************************
函数名：DTUInit
功能：数传串口调用初始化
入口参数：无.
返回值：无
应用范围：外部调用
备注：
****************************************************/
static void DTUInit(void){
	Driver_DTU_Init(WIRELESS_USARTX,\
	WIRELESS_USARTX_RX_PIN,\
	WIRELESS_USARTX_TX_PIN,\
	WIRELESS_USART_PreemptionPriority,\
	WIRELESS_USART_SubPriority);
}


