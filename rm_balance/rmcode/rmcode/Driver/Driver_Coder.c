#include "Driver_Coder.h"

coderStruct_t coderData;

void CoderInit(void){
	BSP_GPIO_EXIT_Init(CODER_AHEAD_PIN,EXTI_Trigger_Rising,CODER_AHEAD_PTY,CODER_AHEAD_STY,GPIO_Mode_IPD);  //ǰ��������ʼ��
	BSP_GPIO_EXIT_Init(CODER_BACK_PIN,EXTI_Trigger_Rising,CODER_BACK_PTY,CODER_BACK_STY,GPIO_Mode_IPD);     //���������ʼ��
}




