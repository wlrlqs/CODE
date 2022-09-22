#include "application.h"
#include "driver.h"

/*--------------------------------------------- 
-----------------------------------------------
//		警告：公共代码区域，故障责任重大		 //
//			除特定区域可以自由可以更改外，		 //
//	其余部分更改，请与公共区域负责人取得更改权限  // 
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
	imuUsartReceive();  //何煜陀螺仪

}
void static controlTrans(){
	//TX2接收
	visionSerial.Receive();
	//TX2发送
	visionSerial.Send(VISION_USARTX,&visionSerial);
	//陀螺仪接收
  imuUsartReceive();  //何煜陀螺仪
}

//数据交换
void transferRecUpdateTask(void * Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();											
	while(true){
		vTaskDelayUntil(&xLastWakeTime,TRANSFER_RECEIVE_PERIOD);
		//串口数据发送
		transferTask();
		//CANFD数据交换
		getCanFdTrans()->Receive(&can_fd_dataReceive);
		getCanFdTrans()->Send(can_fd_dataSend);
		digitalIncreasing(&transferRecData.loops);
	}
}

//数据交换任务初始化
void transferInit(void){
	supervisorData.taskEvent[TRANSFER_REC_TASK] = xTaskCreate(transferRecUpdateTask, \
														"TRANSFER_RECEIVE", \
														TRANSFER_RECEIVE_STACK_SIZE, \
														NULL, \
														TRANSFER_RECEIVE_PRIORITY, \
														&transferRecData.xHandleTask);
	//LCM: 根据不同板子，选择对应数据交换函数
	if(BOARD_TYPE == BOARD_CONTROL)
		transferTask = controlTrans;
	else if(BOARD_TYPE == BOARD_CHASSIS)
		transferTask = chssisTrans;
    usbVCP_Printf("transferRecInit Successfully \r\n");
}

