#include "Driver_Slave_Sensor.h"
#include "vision.h"
#include "Driver_USBVCP.h"
#include "slave_sensor.h"
#include "Driver_judge.h"
#include "processing.h"
#include "shoot.h"
#include "warning.h"
#include "Driver_DMMotor.h"
#include "Driver_WT931.h"
#include "supervisor.h"
#include "config.h"
#include "judge.h"
#include "rc.h"
#include "motorHandle.h"

/*--------------------------------------------- 
-----------------------------------------------
//		���棺�����������򣬹��������ش�		 //
//			���ض�����������ɿ��Ը����⣬		 //
//	���ಿ�ָ��ģ����빫����������ȡ�ø���Ȩ��  // 
-----------------------------------------------
---------------------------------------------*/

slaveSensorStruct_t slaveSensorData;
controlTransStruct_t controlTransData;
controlTransStruct_t slaveTransData;

/**************Vision Data**************/
static void visionDataSend(USART_TypeDef *USARTx,serialStruct_t *serialBuffer);
static void visionDataUpdate(uint8_t *bufferDecode);
static void visionReceive(void);

//LCM: ��������д�����ṹ�� visionSerial ���Ե������µ�����������������˲�������
serialStruct_t visionSerial = {
	visionDataSend,
	visionDataUpdate,
	visionReceive,
};

//��϶�TX2�Ŀ���ָ��
void mixVisionCommand(uint8_t *array) {
	*array = 0x00;
	*array |= getvisionData()->distinguishMode << 0;
	*array |= getvisionData()->enemyType << 3;
	*array |= getvisionData()->buff_mod << 4;
}

//�����Ӿ�����
static void visionDataSend(USART_TypeDef *USARTx,serialStruct_t *serialBuffer) {
	uint8_t *array = (uint8_t *)USART_TO_ArrayTX(USARTx);
	uint8_t index_ptr = 0;
	/*��Ϣ��*/
	//���^
	array[index_ptr++] = VISION_BEGIN;
	//���L��
	array[index_ptr++] = 0x00;
	//�@ȡ��Ű��L��λ��
	serialBuffer->length_position = index_ptr - 1;
	//CRC8У�λ
	array[index_ptr++] = 0x00;
	//�@ȡ��Ϣ���L��
	serialBuffer->header_length = index_ptr;
	
	/*������*/
	//��װ��tx2ָ��
	identifyCamp();
	mixVisionCommand(&array[index_ptr++]);
	//��������
	array[index_ptr++] = getvisionData()->carType;
	array[index_ptr++] = getvisionData()->shootSpeed;
//	//PY: ����ϵͳ����
//	formatTrans32Struct_t shootSpeedData;
//	shootSpeedData.float_temp = getvisionData()->judgeShootSpeed;
//	for(uint8_t index = 0; index < 4; index++)
//		array[index_ptr++] = shootSpeedData.u8_temp[index];
	formatTrans32Struct_t _imuData;
	_imuData.float_temp = SC_PITCH;
	for(uint8_t index = 0; index < 4; index++)
		array[index_ptr++] = _imuData.u8_temp[index];
	_imuData.float_temp = SC_YAW - getGimbalData()->yawAngleSave;	    //LCM: ����TX2 �Ͽ�ʱ����Ϊyaw��0��
	for(uint8_t index = 0; index < 4; index++)
		array[index_ptr++] = _imuData.u8_temp[index];
	_imuData.float_temp = IMU_RATEZ;
		for(uint8_t index = 0; index < 4; index++)
		array[index_ptr++] = _imuData.u8_temp[index];
	for(uint8_t index = 0; index < 2; index++)
		array[index_ptr++] = serialBuffer->CNTR.u8_temp[index];

	visionStoreHistoricalData(SC_PITCH,SC_YAW,serialBuffer->CNTR.u16_temp);	//LCM: 	�洢�˿������� PITCH ��YAW �Ƕ� �� CNTR 
	//CRC16У�λ
	array[serialBuffer->length_position] = index_ptr + 2;
	//CNTR
	digitalIncreasing(&serialBuffer->CNTR.u16_temp);
	//����CRC8	//LCM: CRC8 ������ݰ�ͷ
	Append_CRC8_Check_Sum(array,serialBuffer->header_length);
	//����CRC16	//LCM: CRC16����������ݰ�
	Append_CRC16_Check_Sum(array,array[serialBuffer->length_position]);
	BSP_USART_DMA_SendData(USARTx,array,array[serialBuffer->length_position]);
}

//��ȡ�Ӿ�������Ϣ
static void visionDataDecoding(uint8_t array) {
	getvisionData()->distingushState = array;
	getvisionData()->captureFlag = array & 0x01;
	getvisionData()->sameTargetFlag = array & 0x02;
	getvisionData()->getOrderFlag = array & 0x04;
}

//�Ӿ����ݽ���
static void visionReceive() {
	bufferLoopMod(VISION_BEGIN, visionSerial.header_length, visionSerial.length_position, &visionSerial.bufferLoop, visionSerial.update);
}

static void visionDataUpdate(uint8_t *buffdecode) {
	uint8_t index_ptr = visionSerial.header_length;
	visionDataDecoding(buffdecode[index_ptr++]);
	
//	for(uint8_t axis = 0; axis < 3; axis++){
//		for(uint8_t index = 0; index < 2; index++){
//			getvisionData()->coordinateBase[axis].u8_temp[index] = buffdecode[index_ptr++];
//		}
//	}
//	for(uint8_t axis = 0; axis < 3; axis++){
//		for(uint8_t index = 0; index < 2; index++){
//			getvisionData()->coordinate[axis].u8_temp[index] = buffdecode[index_ptr++];
//		}
//	}
    
    for(uint8_t index = 0; index < 4; index++) {
        getvisionData()->yawData.u8_temp[index] = buffdecode[index_ptr++];
    }
    for(uint8_t index = 0; index < 4; index++) { 
        getvisionData()->pitchData.u8_temp[index] = buffdecode[index_ptr++];
    }
	for(uint8_t index = 0; index < 2; index++){
		getvisionData()->CNTR.u8_temp[index] = buffdecode[index_ptr++];
	}
    
}

/**************smallGimbal Data**********/
static void slaverBoardDataSend(USART_TypeDef *USARTx,serialStruct_t *serialBuffer);
static void slaverBoardDataUpload(uint8_t *bufferdecode);
static void slaverBoardReceive(void);

serialStruct_t slaverBoardSerial = {
	slaverBoardDataSend,
	slaverBoardDataUpload,
	slaverBoardReceive,
};

void mixControlCommand(uint8_t* array , controlTransStruct_t * dataStruct){
	*array = 0x00;
	if(BOARD_TYPE == BOARD_CONTROL){
		*array |= getvisionData()->captureFlag << 0;
		*array |= getvisionData()->enemyType << 1;
		*array |= getshootData()->fricMotorFlag << 2;
		*array |= getvisionData()->miniPTZEnableAim << 3;
	}
	else if(dataStruct != NULL){
		*array |= dataStruct->otherCaptureFlag << 0;
		*array |= dataStruct->otherEnemyType << 1;
		*array |= dataStruct->otherfricMotorFlag << 2;
		*array |= dataStruct->otherAutoMaticFlag << 3;
	}
	digitalClan(&supervisorData.imuCali);
}

void otherControDataDecoding(uint8_t array, controlTransStruct_t * dataStruct){
	dataStruct->otherCaptureFlag = array & 0x01;
	dataStruct->otherEnemyType = array & 0x02;
	dataStruct->otherfricMotorFlag = array & 0x04;
	dataStruct->otherAutoMaticFlag = array & 0x08;
}

//С��̨���ݷ���
static void slaverBoardDataSend(USART_TypeDef *USARTx,serialStruct_t *serialBuffer){
	uint8_t *array = (uint8_t *)USART_TO_ArrayTX(USARTx);
	uint8_t index_ptr = 0;
	/*��Ϣ֡*/
	//֡ͷ
	array[index_ptr++] = SLAVERBOARD_BEGIN;
	//������
	array[index_ptr++] = 0x00;
	//��ȡ��Ű�����λ��
	serialBuffer->length_position = index_ptr - 1;
	//CRC8У��λ
	array[index_ptr++] = 0x00;
	//��ȡ��Ϣ֡����
	serialBuffer->header_length = index_ptr;
	
	/*����֡*/
	mixControlCommand(&array[index_ptr++], NULL);
	
	//��װ��ǰ��̨�����̽Ƕ�
	slaveTransData.pitchGyroAngle.float_temp = getGimbalData()->pitchGyroAngle;
	slaveTransData.yawMotorAngle.float_temp = getGimbalData()->yawMotorAngle;
	
	for(uint8_t index=0;index < 4;index++){
		array[index_ptr++] = slaveTransData.pitchGyroAngle.u8_temp[index];
	}
	for(uint8_t index=0;index < 4;index++){
		array[index_ptr++] = slaveTransData.yawMotorAngle.u8_temp[index];
	}
	
	//��װ��ǰ��̨�ĽǶ�����
	slaveTransData.pitchAngleRef.float_temp = getGimbalData()->pitchAngleRef;
	slaveTransData.yawAngleRef.float_temp = getGimbalData()->yawAngleRef;
	for(uint8_t index=0;index < 4;index++){
		array[index_ptr++] = slaveTransData.pitchAngleRef.u8_temp[index];
	}
	for(uint8_t index=0;index < 4;index++){
		array[index_ptr++] = slaveTransData.yawAngleRef.u8_temp[index];
	}

	//CRC16У��λ
	array[serialBuffer->length_position] = index_ptr + 2;
	//CNTR
	digitalIncreasing(&serialBuffer->CNTR.u16_temp);
	//����CRC8
	Append_CRC8_Check_Sum(array,serialBuffer->header_length);
	//����CRC16
	Append_CRC16_Check_Sum(array,array[serialBuffer->length_position]);
	BSP_USART_DMA_SendData(USARTx,array,array[serialBuffer->length_position]);
	
}

//С��̨���ݽ���
static void slaverBoardDataUpload(uint8_t *bufferdecode){
	uint8_t index_ptr = slaverBoardSerial.header_length;
	formatTrans16Struct_t trans16Data;
	formatTrans32Struct_t trans32Data;
	
	otherControDataDecoding(bufferdecode[index_ptr++], &controlTransData); //3
	
	judgeData.extShootData.bullet_type = bufferdecode[index_ptr++];   //4
	
	for(uint8_t index = 0; index < 4; index++)    			    //8
		trans32Data.u8_temp[index] = bufferdecode[index_ptr++];
	judgeData.extShootData.bullet_speed = trans32Data.float_temp;

	for(uint8_t index = 0; index < 2; index++)					//10
		trans16Data.u8_temp[index] = bufferdecode[index_ptr++];
	judgeData.extPowerHeatData.shooter_id1_17mm_cooling_heat = trans16Data.u16_temp;
	
	for(uint8_t index = 0; index < 2; index++)					//12
		trans16Data.u8_temp[index] = bufferdecode[index_ptr++];
	judgeData.extGameRobotState.max_HP = trans16Data.u16_temp;
	
	for(uint8_t index = 0; index < 2; index++)					//14
		trans16Data.u8_temp[index] = bufferdecode[index_ptr++];
	judgeData.extGameRobotState.remain_HP = trans16Data.u16_temp;
	
	for(uint8_t index = 0; index < 2; index++)
		trans16Data.u8_temp[index] = bufferdecode[index_ptr++];
	judgeData.extGameRobotState.shooter_id1_17mm_cooling_limit = trans16Data.u16_temp;
	
	for(uint8_t index = 0; index < 2; index++)
		trans16Data.u8_temp[index] = bufferdecode[index_ptr++];
	judgeData.extGameRobotState.shooter_id1_17mm_cooling_rate = trans16Data.u16_temp;

	for(uint8_t index = 0; index < 2; index++)
		trans16Data.u8_temp[index] = bufferdecode[index_ptr++];
	judgeData.extGameRobotState.shooter_id1_17mm_speed_limit = trans16Data.u16_temp;
	
	judgeData.extRobotHurt.armor_id = bufferdecode[index_ptr++];
	
	judgeData.extRobotHurt.hurt_type = bufferdecode[index_ptr++];
	
	judgeData.extDartStatus.dart_belong = bufferdecode[index_ptr++];
	
	for(uint8_t index = 0; index < 2; index++)
		trans16Data.u8_temp[index] = bufferdecode[index_ptr++];		
	judgeData.extDartStatus.stage_remaining_time = trans16Data.u16_temp;

	//װ��ң��������
	for(uint8_t index = 0; index < 18; index++){				//32
		controlTransData.otherRcValue[index] = bufferdecode[index_ptr++];
	}
	
	controlTransData.otherRcReadly = bufferdecode[index_ptr++]; //33
		
	for(uint8_t index = 0; index < 4; index++)					//37
		controlTransData.pitchGyroAngle.u8_temp[index] = bufferdecode[index_ptr++];		
	for(uint8_t index = 0; index < 4; index++)					//41
		controlTransData.yawMotorAngle.u8_temp[index] = bufferdecode[index_ptr++];
	for(uint8_t index = 0; index < 4; index++)					//45
		controlTransData.pitchAngleRef.u8_temp[index] = bufferdecode[index_ptr++];	
	for(uint8_t index = 0; index < 4; index++)					//49
		controlTransData.yawAngleRef.u8_temp[index] = bufferdecode[index_ptr++];
	
	getshootData()->shooterHeatRemain_17mm = bufferdecode[index_ptr++];//50
	
	controlTransData.othersenorFlag0 = bufferdecode[index_ptr] & 0x02;
	controlTransData.othersenorFlag1 = bufferdecode[index_ptr++] & 0x01;  //51	
	
	for(uint8_t index = 0; index < 18; index++){
		Array_USART2_RX[index] = controlTransData.otherRcValue[index];
	}	
	
	remoteControlData.rcIspReady = controlTransData.otherRcReadly;
	if(controlTransData.otherRcReadly){
		digitalIncreasing(&remoteControlData.rcError.errorCount.u32_temp);
	}		
}

static void slaverBoardReceive(void){
	bufferLoopMod(MASTERBOARD_BEGIN,slaverBoardSerial.header_length,
			  slaverBoardSerial.length_position,&slaverBoardSerial.bufferLoop,slaverBoardSerial.update);	
}

/**************sentry Data****************/
static void masterBoardDataSend(USART_TypeDef *USARTx,serialStruct_t *serialBuffer);
static void masterBoardDataUpload(uint8_t *bufferdecode);
static void masterBoardReceive(void);
serialStruct_t masterBoardSerial = {
	masterBoardDataSend,
	masterBoardDataUpload,
	masterBoardReceive,
};

//�ڱ����ݷ��ͣ����̰巢�ͣ�
static void masterBoardDataSend(USART_TypeDef *USARTx,serialStruct_t *serialBuffer){
	uint8_t *array = (uint8_t *)USART_TO_ArrayTX(USARTx);
	uint8_t index_ptr = 0;
	formatTrans16Struct_t trans16Data;
	formatTrans32Struct_t trans32Data;
	
	/*��Ϣ֡*/
	//֡ͷ
	array[index_ptr++] = MASTERBOARD_BEGIN;              //0
	//������
	array[index_ptr++] = 0x00; 					   //1
	//��ȡ��Ű�����λ��
	serialBuffer->length_position = index_ptr - 1;
	//CRC8У��λ
	array[index_ptr++] = 0x00;					//2
	//��ȡ��Ϣ֡����
	serialBuffer->header_length = index_ptr;
	
	/*����֡*/
	identifyCamp();
	mixControlCommand(&array[index_ptr++], &controlTransData);		//3
	//��װ����ϵͳ��Ϣ
	
	array[index_ptr++] = judgeData.extShootData.bullet_type;  //4
	
	trans32Data.float_temp = judgeData.extShootData.bullet_speed;
	for(uint8_t index=0;index < 4;index++){					//8
		array[index_ptr++] = trans32Data.u8_temp[index];
	}
	
	trans16Data.u16_temp = judgeData.extPowerHeatData.shooter_id1_17mm_cooling_heat;
	for(uint8_t index=0;index < 2;index++){					//10
		array[index_ptr++] = trans16Data.u8_temp[index];
	}
	
	trans16Data.u16_temp = judgeData.extGameRobotState.max_HP;
	for(uint8_t index=0;index < 2;index++){					//12
		array[index_ptr++] = trans16Data.u8_temp[index];
	}
	
	trans16Data.u16_temp = judgeData.extGameRobotState.remain_HP;
	for(uint8_t index=0;index < 2;index++){    				//14
		array[index_ptr++] = trans16Data.u8_temp[index];
	}
	
	trans16Data.u16_temp = judgeData.extGameRobotState.shooter_id1_17mm_cooling_limit;
	for(uint8_t index = 0; index < 2; index++)
		array[index_ptr++] = trans16Data.u8_temp[index];
	
	trans16Data.u16_temp = judgeData.extGameRobotState.shooter_id1_17mm_cooling_rate;//17mm�����ȴ
	for(uint8_t index = 0; index < 2; index++)
		array[index_ptr++] = trans16Data.u8_temp[index];
	
	trans16Data.u16_temp = judgeData.extGameRobotState.shooter_id1_17mm_speed_limit;
	for(uint8_t index = 0; index < 2; index++)
		array[index_ptr++] = trans16Data.u8_temp[index];
	
	array[index_ptr++] = judgeData.extRobotHurt.armor_id;						 //�ܵ��˺���װ�װ�ID
	
	array[index_ptr++] = judgeData.extRobotHurt.hurt_type;						 //�ܵ��˺�������
	
	array[index_ptr++] = judgeData.extDartStatus.dart_belong;					 //����״̬
	
	trans16Data.u16_temp = judgeData.extDartStatus.stage_remaining_time;         //��������ʣ��ʱ��
	for(uint8_t index = 0; index < 2; index++)
		array[index_ptr++] = trans16Data.u8_temp[index];
	
	//װ��ң��������

	for(uint8_t index=0;index < 18;index++){				//32
		array[index_ptr++] = controlTransData.otherRcValue[index];
	}
	
	array[index_ptr++] = controlTransData.otherRcReadly;	//33
		
	//��װ��ǰ��̨�����̽Ƕ�
	controlTransData.pitchGyroAngle.float_temp = getGimbalData()->pitchGyroAngle;  //�ڱ���
	controlTransData.yawMotorAngle.float_temp = getGimbalData()->yawMotorAngle;
	
	for(uint8_t index=0;index < 4;index++){					//37
		array[index_ptr++] = controlTransData.pitchGyroAngle.u8_temp[index];
	}
	for(uint8_t index=0;index < 4;index++){					//41
		array[index_ptr++] = controlTransData.yawMotorAngle.u8_temp[index];
	}
	
	//��װ��ǰ��̨�ĽǶ�����
	controlTransData.pitchAngleRef.float_temp = getGimbalData()->pitchAngleRef;
	controlTransData.yawAngleRef.float_temp = getGimbalData()->yawAngleRef;
	
	for(uint8_t index=0;index < 4;index++){					//45
		array[index_ptr++] = controlTransData.pitchAngleRef.u8_temp[index];
	}
	for(uint8_t index=0;index < 4;index++){					//49
		array[index_ptr++] = controlTransData.yawAngleRef.u8_temp[index];
	}
	array[index_ptr++] = getshootData()->shooterHeatRemain_17mm;	//50
	
	array[index_ptr] = 0x00;
	array[index_ptr] |= controlTransData.othersenorFlag0 << 1;
 	array[index_ptr++] |= controlTransData.othersenorFlag1 << 0; //51

	//CRC16У��λ
	array[serialBuffer->length_position] = index_ptr + 2;
	//CNTR
	digitalIncreasing(&serialBuffer->CNTR.u16_temp);
	//����CRC8
	Append_CRC8_Check_Sum(array,serialBuffer->header_length);
	//����CRC16
	Append_CRC16_Check_Sum(array,array[serialBuffer->length_position]);
	BSP_USART_DMA_SendData(USARTx,array,array[serialBuffer->length_position]);
	
}

//�ڱ����ݽ���
static void masterBoardDataUpload(uint8_t *bufferdecode){
	uint8_t index_ptr = masterBoardSerial.header_length;
	
	otherControDataDecoding(bufferdecode[index_ptr++], &slaveTransData);
				
	for(uint8_t index = 0; index < 4; index++)
		slaveTransData.pitchGyroAngle.u8_temp[index] = bufferdecode[index_ptr++];		
	for(uint8_t index = 0; index < 4; index++)
		slaveTransData.yawMotorAngle.u8_temp[index] = bufferdecode[index_ptr++];
	for(uint8_t index = 0; index < 4; index++)
		slaveTransData.pitchAngleRef.u8_temp[index] = bufferdecode[index_ptr++];	
	for(uint8_t index = 0; index < 4; index++)
		slaveTransData.yawAngleRef.u8_temp[index] = bufferdecode[index_ptr++];
}

static void masterBoardReceive(void){
	bufferLoopMod(SLAVERBOARD_BEGIN,masterBoardSerial.header_length,
		  masterBoardSerial.length_position,&masterBoardSerial.bufferLoop,masterBoardSerial.update);
}

///**************init Data**************/
//static void init_id_checkSend(uint8_t *array);
//static void init_id_checkUpload(uint8_t *buffDecode);
//CANFDStruct_t initSerial = {
//	init_id_checkSend,
//	init_id_checkUpload,
//};
//static void init_id_checkSend(uint8_t *array){
//	uint8_t index_ptr = 0;
//	//����λcntr
//	array[index_ptr++] = initSerial.CNTR.u8_temp[0];	
//	array[index_ptr++] = initSerial.CNTR.u8_temp[1];
//	switch(BOARD_TYPE){
//		case BOARD_CONTROL:
//			array[index_ptr++] = robotConfigData.typeOfRobot;
//			array[index_ptr++] = YAW_INSTALL_CONFIG;
//			break;
//		case BOARD_CHASSIS:
//			array[index_ptr++] = robotConfigData.id_set;
//			break;
//	}
//	digitalIncreasing(&initSerial.CNTR.u16_temp);
//}

//static void init_id_checkUpload(uint8_t *buffDecode){
//	uint8_t index_ptr = 0;
//	initSerial.serialError.errorCount.u8_temp[0] = buffDecode[index_ptr++];
//	initSerial.serialError.errorCount.u8_temp[1] = buffDecode[index_ptr++];
//	switch(BOARD_TYPE){
//		case BOARD_CHASSIS:
//			robotConfigData.typeOfRobot = buffDecode[index_ptr++];
//			break;
//		case BOARD_CONTROL:
//			robotConfigData.id_set = buffDecode[index_ptr++];
//			YAW_INSTALL_CONFIG = buffDecode[index_ptr++]; 
//			break;
//	}
//}

/**************Main Control Data**************/
static void controlDataSend(uint8_t *array);
static void controlDataUpload(uint8_t *bufferDecode);
CANFDStruct_t controlSerial = {
	controlDataSend,
	controlDataUpload,
};

static void controlDataSend(uint8_t *array){
	uint8_t index_ptr = 0;
	controlSerial.sendpackClass = 0;
	//����λcntr
	array[index_ptr++] = controlSerial.CNTR.u8_temp[0];	
	array[index_ptr++] = controlSerial.CNTR.u8_temp[1];
	//����֡
	array[index_ptr++] = 0x00;
	/*************�Զ�������*****************/
    //���ط�Ƶ���ͱ�������
    index_ptr = controlPackFreDiv(CONTROL_DEVICE_DEFF_REF,array,index_ptr);
    //���ط�Ƶ����������������
    index_ptr = controlPackFreDiv(CONTROL_DEVICE_OTHER,array,index_ptr);
    //���ط�Ƶ����ң��������
    index_ptr = controlPackFreDiv(CONTROL_DEVICE_RC,array,index_ptr);
    //���ط�Ƶ���͵��̸�������
    index_ptr = controlPackFreDiv(CONTROL_DEVICE_CHASE_REF,array,index_ptr);
    //���̷�Ƶ���ͱ���������
    index_ptr = controlPackFreDiv(CONTROL_DEVICE_ENCODER,array,index_ptr);
    
    if(getcontrolData()->chassisBoardDataFinish == NOT_FINISH_STATE){
		index_ptr = controlPackFreDiv(CONTROL_DEVICE_INIT,array,index_ptr);
	}
	digitalIncreasing(&controlSerial.CNTR.u16_temp);
	array[CHASSIS_CONTROL_PACK_CLASS] = controlSerial.sendpackClass;
	
	//usbVCP_Printf("Control Send Length %d \r\n",index_ptr);
}

//���ذ����
static void controlDataUpload(uint8_t *bufferDecode){
	uint8_t index_ptr = 0;
	controlSerial.serialError.errorCount.u8_temp[0] = bufferDecode[index_ptr++];
	controlSerial.serialError.errorCount.u8_temp[1] = bufferDecode[index_ptr++];
	controlSerial.recepackClass = bufferDecode[index_ptr++];//�������յ������ݼ�����
	//���ؽ��յ���������������
	index_ptr = controlPackReceive(CHASSIS_DEVICE_OTHER,controlSerial.recepackClass,bufferDecode,index_ptr);
	//���ؽ��յ��̲���ϵͳ������
	index_ptr = controlPackReceive(CHASSIS_DEVICE_JUDGE_1,controlSerial.recepackClass,bufferDecode,index_ptr);
	//���ؽ��յ��̲���ϵͳ������
	index_ptr = controlPackReceive(CHASSIS_DEVICE_JUDGE_2,controlSerial.recepackClass,bufferDecode,index_ptr);
	//���ؽ��յ��̴�����������
	index_ptr = controlPackReceive(CHASSIS_DEVICE_SENSOR,controlSerial.recepackClass,bufferDecode,index_ptr);

	//usbVCP_Printf("Control Reveice Length %d \r\n",index_ptr);
}

/**************Chassis Data**************/
static void chassisDataSend(uint8_t * array);
static void chassisDataUpload(uint8_t *buffdecode);
CANFDStruct_t chassisSerial = {
	chassisDataSend,
	chassisDataUpload,
};

static void chassisDataSend(uint8_t * array){
	uint8_t index_ptr = 0;
	chassisSerial.sendpackClass = 0;
	//����λcntr
	array[index_ptr++] = chassisSerial.CNTR.u8_temp[0];
	array[index_ptr++] = chassisSerial.CNTR.u8_temp[1];
	//����֡
	array[index_ptr++] = 0x00;
	/*************�Զ�������*****************/
    //���̷�Ƶ������������
    index_ptr = chassisPackFreDiv(CHASSIS_DEVICE_OTHER,array,index_ptr);
    //���̷�Ƶ���Ͳ���ϵͳ����
    index_ptr = chassisPackFreDiv(CHASSIS_DEVICE_JUDGE_1,array,index_ptr);
    //���̷�Ƶ���Ͳ���ϵͳ����
    index_ptr = chassisPackFreDiv(CHASSIS_DEVICE_JUDGE_2,array,index_ptr);
    //���̷�Ƶ���ʹ���������
    index_ptr = chassisPackFreDiv(CHASSIS_DEVICE_SENSOR,array,index_ptr);
    
	digitalIncreasing(&chassisSerial.CNTR.u16_temp);
	array[CHASSIS_CONTROL_PACK_CLASS] = chassisSerial.sendpackClass;
	
	//usbVCP_Printf("Chassis Send Length %d \r\n",index_ptr);
}

//���̰����
static void chassisDataUpload(uint8_t *bufferDecode){
	uint8_t index_ptr = 0;
    static uint8_t runTime = 0;
	chassisSerial.serialError.errorCount.u8_temp[0] = bufferDecode[index_ptr++];
	chassisSerial.serialError.errorCount.u8_temp[1] = bufferDecode[index_ptr++];
	//�����
	chassisSerial.recepackClass = bufferDecode[index_ptr++];
	//���̽������ر��λ�������
	index_ptr = chassisPackReceive(CONTROL_DEVICE_DEFF_REF,chassisSerial.recepackClass,bufferDecode,index_ptr);
	//���̽�������������������
	index_ptr = chassisPackReceive(CONTROL_DEVICE_OTHER,chassisSerial.recepackClass,bufferDecode,index_ptr);
	//���̽�������ң��������
	index_ptr = chassisPackReceive(CONTROL_DEVICE_RC,chassisSerial.recepackClass,bufferDecode,index_ptr);
	//���̽������رջ�����
	index_ptr = chassisPackReceive(CONTROL_DEVICE_CHASE_REF,chassisSerial.recepackClass,bufferDecode,index_ptr);
	//���̽�����������ֵ
	index_ptr = chassisPackReceive(CONTROL_DEVICE_ENCODER,chassisSerial.recepackClass,bufferDecode,index_ptr);
	//���̽��ճ�ʼ������
	index_ptr = chassisPackReceive(CONTROL_DEVICE_INIT,chassisSerial.recepackClass,bufferDecode,index_ptr);
    
    if(getcontrolData()->controlBoardDataFinish == FINISH_STATE && !runTime && getConfigData()->robotType){
		xEventGroupSetBits(taskInitData.eventGroups,CONTROL_RES_OK);   
        digitalHi(&supervisorData.flashSave);
		digitalHi(&runTime);
	}
	//usbVCP_Printf("Chassis Revice Length %d \r\n",index_ptr);
}

void slaveSensorConfig(void){	
	if(BOARD_TYPE == BOARD_CONTROL){
	  //TX2
		driver_slaveSensorInit(VISION_USARTX, VISION_USARTX_RX_PIN, VISION_USARTX_TX_PIN, \
									 VISION_USARTS_BOUND, VISION_USARTX_PRE, VISION_USARTX_SUB);
		//��̨������
		driver_slaveSensorInit(GIMBAL_IMU_USARTX, GIMBAL_IMU_USARTX_RX_PIN, GIMBAL_IMU_USARTX_TX_PIN, \
					 GIMBAL_IMU_USARTS_BOUND, GIMBAL_IMU_USARTX_PRE, GIMBAL_IMU_USARTX_SUB);	
		//��������������
		driver_slaveSensorInit(USER_PRINTF_USARTX, USER_PRINTF_USARTX_RX_PIN, USER_PRINTF_USARTX_TX_PIN, \
					 USER_PRINTF_USARTS_BOUND, USER_PRINTF_USARTX_PRE, USER_PRINTF_USARTX_SUB);	
	}else if(BOARD_TYPE == BOARD_CHASSIS){
		//�ؽڵ��
		// USART1->CR1 |= USART_CR1_OVER8;
		// driver_slaveSensorInit(JOINT_L_USARTX, JOINT_L_USARTX_RX_PIN, JOINT_L_USARTX_TX_PIN, \
		// 		 JOINT_L_USARTS_BOUND, JOINT_L_USARTX_PRE, JOINT_L_USARTX_SUB);	
		// USART6->CR1 |= USART_CR1_OVER8;
		// driver_slaveSensorInit(JOINT_R_USARTX, JOINT_R_USARTX_RX_PIN, JOINT_R_USARTX_TX_PIN, \
		// 		 JOINT_R_USARTS_BOUND, JOINT_R_USARTX_PRE, JOINT_R_USARTX_SUB);	
		//����������
		driver_slaveSensorInit(CHASSIS_IMU_USARTX, CHASSIS_IMU_USARTX_RX_PIN, CHASSIS_IMU_USARTX_TX_PIN, \
					 CHASSIS_IMU_USARTS_BOUND, CHASSIS_IMU_USARTX_PRE, CHASSIS_IMU_USARTX_SUB);	
		//���
		driver_slaveSensorInit(TOF_L_USARTX, TOF_L_USARTX_RX_PIN, TOF_L_USARTX_TX_PIN, \
					 TOF_L_USARTS_BOUND, TOF_L_USARTX_PRE, TOF_L_USARTX_SUB);	
		driver_slaveSensorInit(TOF_R_USARTX, TOF_R_USARTX_RX_PIN, TOF_R_USARTX_TX_PIN, \
					 TOF_R_USARTS_BOUND, TOF_R_USARTX_PRE, TOF_R_USARTX_SUB);			
	}
	slaveSensorData.initFlag = true;  	
}
