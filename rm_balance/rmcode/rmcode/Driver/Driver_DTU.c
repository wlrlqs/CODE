#include "Driver_DTU.h"
/***************************************/
static void DTUInit(void);
/***************************************/
deviceInitClass DTUClass = {
	DTUInit,
};
/*
***************************************************
��������Driver_DTU_Init
���ܣ�����������ʼ��
��ڲ�����	I can understand it at one glance.
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
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
	//������Ϊ500000
	WIRELESS_USART.USART_InitStructure.USART_BaudRate = 500000;	
	//�ֳ�Ϊ8λ���ݸ�ʽ
	WIRELESS_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	//һ��ֹͣλ
	WIRELESS_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;
	//��У��λ
	WIRELESS_USART.USART_InitStructure.USART_Parity = USART_Parity_No;
	//����/����ģʽ
	WIRELESS_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	//��Ӳ������������
	WIRELESS_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,		
	
	BSP_USART_Init(&WIRELESS_USART,PreemptionPriority,SubPriority);
	BSP_USART_RX_DMA_Init(&WIRELESS_USART);																	
}
/****************************************************
��������DTUInit
���ܣ��������ڵ��ó�ʼ��
��ڲ�������.
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
****************************************************/
static void DTUInit(void){
	Driver_DTU_Init(WIRELESS_USARTX,\
	WIRELESS_USARTX_RX_PIN,\
	WIRELESS_USARTX_TX_PIN,\
	WIRELESS_USART_PreemptionPriority,\
	WIRELESS_USART_SubPriority);
}


