#include "bsp.h"
#include "interrupt_service_fun.h"

/*
***************************************************
��������USART1_IRQHandler
���ܣ�����1�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�
***************************************************
*/
//DMA2-5
void USART1_IRQHandler(void){																	
	//�������ݵĳ���
	uint16_t USART1_len;	
	//����Ƿ��ǿ����ж�
	if(USART_GetITStatus(USART1,USART_IT_IDLE) == SET){
		//��USART_IT_IDLE��־
		USART1_len = USART1->SR;
		USART1_len = USART1->DR; 
		//�ر�DMA
		DMA_Cmd(DMA2_Stream5,DISABLE); 
		//��ȡ�������ݳ���		
		USART1_len = Length_USART1_RX_Buff - DMA2_Stream5->NDTR;
		//��ձ�־λ		
		DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);	
		while(DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);
		DMA2_Stream5->NDTR = Length_USART1_RX_Buff;
		DMA_Cmd(DMA2_Stream5, ENABLE);
		/*********�������Զ��岿��**********/
		getUsartInterruptFunc()->usart1((uint8_t*)DMA2_Stream5->M0AR,USART1_len);
	}
}


/*
***************************************************
��������USART2_IRQHandler
���ܣ�����2�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�
***************************************************
*/	
//DMA1-5
void USART2_IRQHandler(void){
	//�������ݵĳ���	
	uint16_t USART2_len;	
	if(USART_GetITStatus(USART2,USART_IT_IDLE) == SET){   //USART_IT_IDLE��1�����ж�
		//��USART_IT_IDLE��־����USART_SR
		USART2_len = USART2->SR;  
		USART2_len = USART2->DR; 
		//�ر�DMA
		DMA_Cmd(DMA1_Stream5,DISABLE);   
		//��ձ�־λ		
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);	
		while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);
		//��ȡ�������ݳ���
		USART2_len = Length_USART2_RX_Buff - DMA1_Stream5->NDTR;	
		DMA1_Stream5->NDTR = Length_USART2_RX_Buff;
		DMA_Cmd(DMA1_Stream5, ENABLE);
		
		/*********�������Զ��岿��**********/
		getUsartInterruptFunc()->usart2( (uint8_t*)DMA1_Stream5->M0AR,USART2_len);		
	}
}

/*
***************************************************
��������USART3_IRQHandler
���ܣ�����3�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�
***************************************************
*/
//DMA1-1
void USART3_IRQHandler(void){	
	//�������ݵĳ���
	uint16_t USART3_len;	
	if(USART_GetITStatus(USART3,USART_IT_IDLE) == SET){
		//��USART_IT_IDLE��־
		USART3_len = USART3->SR;
		USART3_len = USART3->DR; 
		//�ر�DMA
		DMA_Cmd(DMA1_Stream1,DISABLE);    
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
		//��ձ�־λ		
		while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
		//��ȡ�������ݳ���
		USART3_len = Length_USART3_RX_Buff - DMA1_Stream1->NDTR;		
		DMA1_Stream1->NDTR = Length_USART3_RX_Buff;
		DMA_Cmd(DMA1_Stream1, ENABLE);
		
		/*********�������Զ��岿��**********/
		getUsartInterruptFunc()->usart3( (uint8_t*)DMA1_Stream1->M0AR,USART3_len);
	}
}

/*
***************************************************
��������UART4_IRQHandler
���ܣ�����4�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�
***************************************************
*/
//DMA1-2
void UART4_IRQHandler(void){																			
	//�������ݵĳ���
	uint16_t UART4_len;	
	if(USART_GetITStatus(UART4,USART_IT_IDLE) == SET){
		//��USART_IT_IDLE��־
		UART4_len = UART4->SR;
		UART4_len = UART4->DR; 
		//�ر�DMA
		DMA_Cmd(DMA1_Stream2,DISABLE); 
		//��ձ�־λ		
		DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);	
		while(DMA_GetCmdStatus(DMA1_Stream2) != DISABLE);
		//��ȡ�������ݳ���
		UART4_len = Length_UART4_RX_Buff - DMA1_Stream2->NDTR;				
		DMA1_Stream2->NDTR = Length_UART4_RX_Buff;
		DMA_Cmd(DMA1_Stream2, ENABLE);	
		/*********�������Զ��岿��**********/	
		getUsartInterruptFunc()->uart4((uint8_t*)DMA1_Stream2->M0AR,UART4_len);
	}
}

/*
***************************************************
��������UART5_IRQHandler
���ܣ�����5�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�
***************************************************
*/
//DMA1-0
void UART5_IRQHandler(void){																			
	//�������ݵĳ���
	uint16_t UART5_len;	
	if(USART_GetITStatus(UART5,USART_IT_IDLE) == SET){
		//��USART_IT_IDLE��־
		UART5_len = UART5->SR;
		UART5_len = UART5->DR; 
		//�ر�DMA
		DMA_Cmd(DMA1_Stream0,DISABLE); 
		//��ձ�־λ
		DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);	
		while(DMA_GetCmdStatus(DMA1_Stream0) != DISABLE);
		//��ȡ�������ݳ���
		UART5_len = Length_UART5_RX_Buff - DMA1_Stream0->NDTR;	
		DMA1_Stream0->NDTR = Length_UART5_RX_Buff;
		DMA_Cmd(DMA1_Stream0, ENABLE);	
		/*********�������Զ��岿��**********/	
		getUsartInterruptFunc()->uart5( (uint8_t*)DMA1_Stream0->M0AR,UART5_len);
	}
}

/*
***************************************************
��������USART6_IRQHandler
���ܣ�����6�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�
����Ϊ��������
***************************************************
*/
//DMA2-1
void USART6_IRQHandler(void){																			//DMA2-6
	u16 USART6_len;	//�������ݵĳ���
	if(USART_GetITStatus(USART6,USART_IT_IDLE) == SET){
		USART6_len = USART6->SR;
		USART6_len = USART6->DR; //��USART_IT_IDLE��־
		
		DMA_Cmd(DMA2_Stream1,DISABLE);    //�ر�DMA
		DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);	//��ձ�־λ
		while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);
		USART6_len = Length_USART6_RX_Buff - DMA2_Stream1->NDTR;			//��ȡ�������ݳ���
		DMA2_Stream1->NDTR = Length_USART6_RX_Buff;
		DMA_Cmd(DMA2_Stream1, ENABLE);
		/*********�������Զ��岿��**********/	
		getUsartInterruptFunc()->usart6((uint8_t*)DMA2_Stream1->M0AR,USART6_len);
	}
}


/*
***************************************************
��������UART7_IRQHandler
���ܣ�����7�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�
***************************************************
*/
//*
//***************************************************
//��������UART7_IRQHandler
//���ܣ�����7�жϷ�����
//��ע���������ж�Ϊ�����ж�+DMA�ж�
//***************************************************
//DMA1-3
#ifdef USE_WIRELESS
void UART7_IRQHandler(void){	
	//�������ݵĳ���
	uint16_t UART7_len;	
	if(USART_GetITStatus(UART7,USART_IT_IDLE) == SET){
		//��USART_IT_IDLE��־
		UART7_len = UART7->SR;
		UART7_len = UART7->DR; 
		//�ر�DMA
		DMA_Cmd(DMA1_Stream3,DISABLE);
		//��ձ�־λ
		DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);	
		while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);
		//��ȡ�������ݳ���
		UART7_len = Length_UART7_RX_Buff - DMA1_Stream3->NDTR;				
		DMA1_Stream3->NDTR = Length_UART7_RX_Buff;
		DMA_Cmd(DMA1_Stream3, ENABLE);
		
		/*********�������Զ��岿��**********/	
		getUsartInterruptFunc()->uart7((uint8_t*)DMA1_Stream3->M0AR,UART7_len);
	}
}
#endif

void UART7_IRQHandler(void){	
	//�������ݵĳ���
	uint16_t UART7_len;	
	if(USART_GetITStatus(UART7,USART_IT_IDLE) == SET){
		//��USART_IT_IDLE��־
		UART7_len = UART7->SR;
		UART7_len = UART7->DR; 
		//�ر�DMA
		DMA_Cmd(DMA1_Stream3,DISABLE);
		//��ձ�־λ
		DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);	
		while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);
		//��ȡ�������ݳ���
		UART7_len = Length_UART7_RX_Buff - DMA1_Stream3->NDTR;				
		DMA1_Stream3->NDTR = Length_UART7_RX_Buff;
		DMA_Cmd(DMA1_Stream3, ENABLE);
		
		/*********�������Զ��岿��**********/	
		getUsartInterruptFunc()->uart7((uint8_t*)DMA1_Stream3->M0AR,UART7_len);
	}
}

/*
***************************************************
��������UART8_IRQHandler
���ܣ�����8�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�					
***************************************************
*/
//DMA1-6
void UART8_IRQHandler(void){	
	//�������ݵĳ���
	uint16_t UART8_len;	
	if(USART_GetITStatus(UART8,USART_IT_IDLE) == SET){
		//��USART_IT_IDLE��־
		UART8_len = UART8->SR;
		UART8_len = UART8->DR; 
			
		//�ر�DMA
		DMA_Cmd(DMA1_Stream6,DISABLE); 
		//��ձ�־λ
		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6);	
		while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE);
		//��ȡ�������ݳ���
		UART8_len = Length_UART8_RX_Buff - DMA1_Stream6->NDTR;				
		DMA1_Stream6->NDTR = Length_UART8_RX_Buff;
		DMA_Cmd(DMA1_Stream6, ENABLE);
		
		/*********�������Զ��岿��**********/	
		getUsartInterruptFunc()->uart8((uint8_t*)DMA1_Stream6->M0AR,UART8_len);
	}
}

