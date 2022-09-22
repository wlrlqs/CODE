#include "bsp.h"
#include "interrupt_service_fun.h"

/*
***************************************************
函数名：USART1_IRQHandler
功能：串口1中断服务函数
备注：本串口中断为空闲中断+DMA中断
***************************************************
*/
//DMA2-5
void USART1_IRQHandler(void){																	
	//接收数据的长度
	uint16_t USART1_len;	
	//检测是否是空闲中断
	if(USART_GetITStatus(USART1,USART_IT_IDLE) == SET){
		//清USART_IT_IDLE标志
		USART1_len = USART1->SR;
		USART1_len = USART1->DR; 
		//关闭DMA
		DMA_Cmd(DMA2_Stream5,DISABLE); 
		//读取接收数据长度		
		USART1_len = Length_USART1_RX_Buff - DMA2_Stream5->NDTR;
		//清空标志位		
		DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);	
		while(DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);
		DMA2_Stream5->NDTR = Length_USART1_RX_Buff;
		DMA_Cmd(DMA2_Stream5, ENABLE);
		/*********以下是自定义部分**********/
		getUsartInterruptFunc()->usart1((uint8_t*)DMA2_Stream5->M0AR,USART1_len);
	}
}


/*
***************************************************
函数名：USART2_IRQHandler
功能：串口2中断服务函数
备注：本串口中断为空闲中断+DMA中断
***************************************************
*/	
//DMA1-5
void USART2_IRQHandler(void){
	//接收数据的长度	
	uint16_t USART2_len;	
	if(USART_GetITStatus(USART2,USART_IT_IDLE) == SET){   //USART_IT_IDLE置1产生中断
		//清USART_IT_IDLE标志：读USART_SR
		USART2_len = USART2->SR;  
		USART2_len = USART2->DR; 
		//关闭DMA
		DMA_Cmd(DMA1_Stream5,DISABLE);   
		//清空标志位		
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);	
		while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);
		//读取接收数据长度
		USART2_len = Length_USART2_RX_Buff - DMA1_Stream5->NDTR;	
		DMA1_Stream5->NDTR = Length_USART2_RX_Buff;
		DMA_Cmd(DMA1_Stream5, ENABLE);
		
		/*********以下是自定义部分**********/
		getUsartInterruptFunc()->usart2( (uint8_t*)DMA1_Stream5->M0AR,USART2_len);		
	}
}

/*
***************************************************
函数名：USART3_IRQHandler
功能：串口3中断服务函数
备注：本串口中断为空闲中断+DMA中断
***************************************************
*/
//DMA1-1
void USART3_IRQHandler(void){	
	//接收数据的长度
	uint16_t USART3_len;	
	if(USART_GetITStatus(USART3,USART_IT_IDLE) == SET){
		//清USART_IT_IDLE标志
		USART3_len = USART3->SR;
		USART3_len = USART3->DR; 
		//关闭DMA
		DMA_Cmd(DMA1_Stream1,DISABLE);    
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
		//清空标志位		
		while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
		//读取接收数据长度
		USART3_len = Length_USART3_RX_Buff - DMA1_Stream1->NDTR;		
		DMA1_Stream1->NDTR = Length_USART3_RX_Buff;
		DMA_Cmd(DMA1_Stream1, ENABLE);
		
		/*********以下是自定义部分**********/
		getUsartInterruptFunc()->usart3( (uint8_t*)DMA1_Stream1->M0AR,USART3_len);
	}
}

/*
***************************************************
函数名：UART4_IRQHandler
功能：串口4中断服务函数
备注：本串口中断为空闲中断+DMA中断
***************************************************
*/
//DMA1-2
void UART4_IRQHandler(void){																			
	//接收数据的长度
	uint16_t UART4_len;	
	if(USART_GetITStatus(UART4,USART_IT_IDLE) == SET){
		//清USART_IT_IDLE标志
		UART4_len = UART4->SR;
		UART4_len = UART4->DR; 
		//关闭DMA
		DMA_Cmd(DMA1_Stream2,DISABLE); 
		//清空标志位		
		DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);	
		while(DMA_GetCmdStatus(DMA1_Stream2) != DISABLE);
		//读取接收数据长度
		UART4_len = Length_UART4_RX_Buff - DMA1_Stream2->NDTR;				
		DMA1_Stream2->NDTR = Length_UART4_RX_Buff;
		DMA_Cmd(DMA1_Stream2, ENABLE);	
		/*********以下是自定义部分**********/	
		getUsartInterruptFunc()->uart4((uint8_t*)DMA1_Stream2->M0AR,UART4_len);
	}
}

/*
***************************************************
函数名：UART5_IRQHandler
功能：串口5中断服务函数
备注：本串口中断为空闲中断+DMA中断
***************************************************
*/
//DMA1-0
void UART5_IRQHandler(void){																			
	//接收数据的长度
	uint16_t UART5_len;	
	if(USART_GetITStatus(UART5,USART_IT_IDLE) == SET){
		//清USART_IT_IDLE标志
		UART5_len = UART5->SR;
		UART5_len = UART5->DR; 
		//关闭DMA
		DMA_Cmd(DMA1_Stream0,DISABLE); 
		//清空标志位
		DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);	
		while(DMA_GetCmdStatus(DMA1_Stream0) != DISABLE);
		//读取接收数据长度
		UART5_len = Length_UART5_RX_Buff - DMA1_Stream0->NDTR;	
		DMA1_Stream0->NDTR = Length_UART5_RX_Buff;
		DMA_Cmd(DMA1_Stream0, ENABLE);	
		/*********以下是自定义部分**********/	
		getUsartInterruptFunc()->uart5( (uint8_t*)DMA1_Stream0->M0AR,UART5_len);
	}
}

/*
***************************************************
函数名：USART6_IRQHandler
功能：串口6中断服务函数
备注：本串口中断为空闲中断+DMA中断
现做为数传串口
***************************************************
*/
//DMA2-1
void USART6_IRQHandler(void){																			//DMA2-6
	u16 USART6_len;	//接收数据的长度
	if(USART_GetITStatus(USART6,USART_IT_IDLE) == SET){
		USART6_len = USART6->SR;
		USART6_len = USART6->DR; //清USART_IT_IDLE标志
		
		DMA_Cmd(DMA2_Stream1,DISABLE);    //关闭DMA
		DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);	//清空标志位
		while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);
		USART6_len = Length_USART6_RX_Buff - DMA2_Stream1->NDTR;			//读取接收数据长度
		DMA2_Stream1->NDTR = Length_USART6_RX_Buff;
		DMA_Cmd(DMA2_Stream1, ENABLE);
		/*********以下是自定义部分**********/	
		getUsartInterruptFunc()->usart6((uint8_t*)DMA2_Stream1->M0AR,USART6_len);
	}
}


/*
***************************************************
函数名：UART7_IRQHandler
功能：串口7中断服务函数
备注：本串口中断为空闲中断+DMA中断
***************************************************
*/
//*
//***************************************************
//函数名：UART7_IRQHandler
//功能：串口7中断服务函数
//备注：本串口中断为空闲中断+DMA中断
//***************************************************
//DMA1-3
#ifdef USE_WIRELESS
void UART7_IRQHandler(void){	
	//接收数据的长度
	uint16_t UART7_len;	
	if(USART_GetITStatus(UART7,USART_IT_IDLE) == SET){
		//清USART_IT_IDLE标志
		UART7_len = UART7->SR;
		UART7_len = UART7->DR; 
		//关闭DMA
		DMA_Cmd(DMA1_Stream3,DISABLE);
		//清空标志位
		DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);	
		while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);
		//读取接收数据长度
		UART7_len = Length_UART7_RX_Buff - DMA1_Stream3->NDTR;				
		DMA1_Stream3->NDTR = Length_UART7_RX_Buff;
		DMA_Cmd(DMA1_Stream3, ENABLE);
		
		/*********以下是自定义部分**********/	
		getUsartInterruptFunc()->uart7((uint8_t*)DMA1_Stream3->M0AR,UART7_len);
	}
}
#endif

void UART7_IRQHandler(void){	
	//接收数据的长度
	uint16_t UART7_len;	
	if(USART_GetITStatus(UART7,USART_IT_IDLE) == SET){
		//清USART_IT_IDLE标志
		UART7_len = UART7->SR;
		UART7_len = UART7->DR; 
		//关闭DMA
		DMA_Cmd(DMA1_Stream3,DISABLE);
		//清空标志位
		DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);	
		while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);
		//读取接收数据长度
		UART7_len = Length_UART7_RX_Buff - DMA1_Stream3->NDTR;				
		DMA1_Stream3->NDTR = Length_UART7_RX_Buff;
		DMA_Cmd(DMA1_Stream3, ENABLE);
		
		/*********以下是自定义部分**********/	
		getUsartInterruptFunc()->uart7((uint8_t*)DMA1_Stream3->M0AR,UART7_len);
	}
}

/*
***************************************************
函数名：UART8_IRQHandler
功能：串口8中断服务函数
备注：本串口中断为空闲中断+DMA中断					
***************************************************
*/
//DMA1-6
void UART8_IRQHandler(void){	
	//接收数据的长度
	uint16_t UART8_len;	
	if(USART_GetITStatus(UART8,USART_IT_IDLE) == SET){
		//清USART_IT_IDLE标志
		UART8_len = UART8->SR;
		UART8_len = UART8->DR; 
			
		//关闭DMA
		DMA_Cmd(DMA1_Stream6,DISABLE); 
		//清空标志位
		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6);	
		while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE);
		//读取接收数据长度
		UART8_len = Length_UART8_RX_Buff - DMA1_Stream6->NDTR;				
		DMA1_Stream6->NDTR = Length_UART8_RX_Buff;
		DMA_Cmd(DMA1_Stream6, ENABLE);
		
		/*********以下是自定义部分**********/	
		getUsartInterruptFunc()->uart8((uint8_t*)DMA1_Stream6->M0AR,UART8_len);
	}
}

