#include "Driver_ClockCount.h"
#include "Driver_USBVCP.h"

static void clockCountInit(void);
deviceInitClass clockClass = {
	clockCountInit,
};

static void clockCountInit(void){	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//使能定时器TIMx时钟
	BSP_TIM_RCC_Init(TIM5);		                                            
	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Period 		= CLOCKCOUNT_PERIOD;
	//定时器分频
	TIM_TimeBaseInitStructure.TIM_Prescaler 	= CLOCKCOUNT_PRESCALER;
	//向上计数模式
	TIM_TimeBaseInitStructure.TIM_CounterMode 	= TIM_CounterMode_Up;     
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	//允许定时器TIMx更新中断
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	//使能定时器TIMx
	TIM_Cmd(TIM5,ENABLE); 		                                             
	//定时器TIMx中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CLOCKCOUNT_PRE;
	//子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = CLOCKCOUNT_SUB;        
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	usbVCP_Printf("ClockCountInit Successfully \r\n");	
}

