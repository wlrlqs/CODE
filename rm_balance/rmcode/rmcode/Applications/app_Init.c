#include "board.h"

taskInit_t taskInitData;

/*--------------------------------------------- 
-----------------------------------------------
//		���棺�����������򣬹��������ش�		 //
//			���ض�����������ɿ��Ը����⣬		 //
//	���ಿ�ָ��ģ����빫����������ȡ�ø���Ȩ��  // 
-----------------------------------------------
---------------------------------------------*/

void appInit(void *Parameters){
	//�¼���־������
	taskInitData.eventGroups = NULL;					                    
	
	taskInitData.eventGroups = xEventGroupCreate();

	/*������ʼ��*/
    //���⴮�ڳ�ʼ��
	usbVCP_Init(USB_USART_PreemptionPriority,USB_USART_SubPriority);  
    //��ȡ���������ͳ�ʼ��
	robotKeyInit();
	//����Ĭ�ϲ���
	configInit();    
	//���µ�·���γ�ʼ��
	BSP_GPIO_Init(BSP_GPIOD13,GPIO_Mode_Out_PP);
	GPIO_ResetBits(GPIOD,GPIO_Pin_13);
	//��ʱ��2(32λ��ʱ��)��ʼ��,���ھ�ȷʱ��ڵ����
	clockClass.Init();
	//IIR��ͨ�˲�����ʼ��
	iir_filter_init();
	//CANFD��ʼ��
	can_fd_spi_Init();
	can_fd_device_Init();
	//ʶ��������Թؼ����ݽ��г�ʼ��
	robotDistinguish();
    //���ݴ�����������ʼ��
	transferInit();
	//IMU��ʼ��
	imuInit();	
	//���ư�ͨ�ų�ʼ��
	slaveSensorConfig();
	//���ݴ����ʼ��
	datatransmissionInit();          		
	//���״̬���ĳ�ʼ��
	supervisorInit();
//    init_data_prepared(BOARD_TYPE);
	/*�����ʼ��*/
	//���̰��ʼ��					
	boardTypeConfirm(BOARD_CHASSIS,chassisSetup);
	//���ذ��ʼ��
	boardTypeConfirm(BOARD_CONTROL,controlSetup);
    //���⴮�ڳ�ʼ����ɱ�־
	usbVCP_Printf("All Init Successfully \r\n");                            
	//ɾ����ǰ����
	vTaskDelete(NULL);									                   
}
/**
static uint16_t uxHighWaterMark = 0;
uxHighWaterMark=uxTaskGetStackHighWaterMark(taskHandle);
printf("ʣ��ռ䣺%d\r\n",uxHighWaterMark);
*/
 
