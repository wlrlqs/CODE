#include "application.h"
#include "driver.h"

/*--------------------------------------------- 
-----------------------------------------------
//		���棺�����������򣬹��������ش�		 //
//			���ض�����������ɿ��Ը����⣬		 //
//	���ಿ�ָ��ģ����빫����������ȡ�ø���Ȩ��  // 
-----------------------------------------------
---------------------------------------------*/
double _time[2]={0};

transferRecStruct_t transferRecData;
transferTask_ptr *transferTask;
transferRecStruct_t *getTransferRecData(){
	return &transferRecData;
}

void static chssisTrans(){
	jointSerial.Receive();
	digitalIncreasing(&jointSerial.sendFlag);
	if(jointSerial.sendFlag == 2){
	  jointSerial.Send(JOINT_L_USARTX,&jointData.send[0]);
		digitalHi(&jointSerial.sendLoop);
	}else if(jointSerial.sendFlag == 3){
		jointSerial.Send(JOINT_R_USARTX,&jointData.send[2]);
		digitalIncreasing(&jointSerial.sendLoop);
	}else if(jointSerial.sendFlag == 4){
		jointSerial.Send(JOINT_L_USARTX,&jointData.send[1]);
		digitalIncreasing(&jointSerial.sendLoop);
	}else if(jointSerial.sendFlag >= 5){
		jointSerial.Send(JOINT_R_USARTX,&jointData.send[3]);
		digitalIncreasing(&jointSerial.sendLoop);
		digitalHi(&jointSerial.sendFlag);
	}
	imuUsartReceive();  //����������

}
void static controlTrans(){
	//TX2����
	visionSerial.Receive();
	//TX2����
	visionSerial.Send(VISION_USARTX,&visionSerial);
	//�����ǽ���
  imuUsartReceive();  //����������
}

//���ݽ���
void transferRecUpdateTask(void * Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();											
	while(true){
		vTaskDelayUntil(&xLastWakeTime,TRANSFER_RECEIVE_PERIOD);
		//�������ݷ���
		transferTask();
		//CANFD���ݽ���
		getCanFdTrans()->Receive(&can_fd_dataReceive);
		getCanFdTrans()->Send(can_fd_dataSend);
		digitalIncreasing(&transferRecData.loops);
	}
}

//���ݽ��������ʼ��
void transferInit(void){
	supervisorData.taskEvent[TRANSFER_REC_TASK] = xTaskCreate(transferRecUpdateTask, \
														"TRANSFER_RECEIVE", \
														TRANSFER_RECEIVE_STACK_SIZE, \
														NULL, \
														TRANSFER_RECEIVE_PRIORITY, \
														&transferRecData.xHandleTask);
	//LCM: ���ݲ�ͬ���ӣ�ѡ���Ӧ���ݽ�������
	if(BOARD_TYPE == BOARD_CONTROL)
		transferTask = controlTrans;
	else if(BOARD_TYPE == BOARD_CHASSIS)
		transferTask = chssisTrans;
    usbVCP_Printf("transferRecInit Successfully \r\n");
}

