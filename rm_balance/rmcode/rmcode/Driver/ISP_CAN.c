#include "board.h"
#include "Driver_MotorSever.h"
#include "util.h"
#include "interrupt_service_fun.h"
/*
***************************************************
��������CAN1_RX0_IRQHandler
���ܣ�CAN1�����ж�
��ע��y:700 p:3180
				
***************************************************
*/
CanRxMsg can1_rx_msg;
void CAN1_RX0_IRQHandler(void){
//	CanRxMsg can1_rx_msg;
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
		CAN_Receive(CAN1, CAN_FIFO0, &can1_rx_msg);
		getCanInterruptFunc()->can1(&can1_rx_msg);
	}
}

/*
***************************************************
��������CAN1_TX_IRQHandler
���ܣ�CAN1�����ж�
��ע��
***************************************************
*/
void CAN1_TX_IRQHandler(void){
	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET){
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);

		/*********�������Զ��岿��**********/
	}
}

/*
***************************************************
��������CAN2_RX0_IRQHandler
���ܣ�CAN2�����ж�
��ע��
***************************************************
*/	

void CAN2_RX0_IRQHandler(void){
   CanRxMsg can2_rx_msg;
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &can2_rx_msg);  
		getCanInterruptFunc()->can2(&can2_rx_msg);
	}
}

/*
***************************************************
��������CAN2_TX_IRQHandler
���ܣ�CAN2�����ж�
��ע��
***************************************************
*/
void CAN2_TX_IRQHandler(void){
	if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET){
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
		
		/*********�������Զ��岿��**********/
	}
}
