#include "Driver_ClockCount.h"
#include "Driver_USBVCP.h"

static void clockCountInit(void);
deviceInitClass clockClass = {
	clockCountInit,
};

static void clockCountInit(void){	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//ʹ�ܶ�ʱ��TIMxʱ��
	BSP_TIM_RCC_Init(TIM5);		                                            
	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Period 		= CLOCKCOUNT_PERIOD;
	//��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_Prescaler 	= CLOCKCOUNT_PRESCALER;
	//���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_CounterMode 	= TIM_CounterMode_Up;     
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	//����ʱ��TIMx�����ж�
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	//ʹ�ܶ�ʱ��TIMx
	TIM_Cmd(TIM5,ENABLE); 		                                             
	//��ʱ��TIMx�ж�
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CLOCKCOUNT_PRE;
	//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = CLOCKCOUNT_SUB;        
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	usbVCP_Printf("ClockCountInit Successfully \r\n");	
}

