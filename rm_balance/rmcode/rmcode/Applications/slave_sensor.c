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
//		警告：公共代码区域，故障责任重大		 //
//			除特定区域可以自由可以更改外，		 //
//	其余部分更改，请与公共区域负责人取得更改权限  // 
-----------------------------------------------
---------------------------------------------*/

slaveSensorStruct_t slaveSensorData;
controlTransStruct_t controlTransData;
controlTransStruct_t slaveTransData;

/**************Vision Data**************/
static void visionDataSend(USART_TypeDef *USARTx,serialStruct_t *serialBuffer);
static void visionDataUpdate(uint8_t *bufferDecode);
static void visionReceive(void);

//LCM: 面向对象的写法，结构体 visionSerial 可以调用如下的三个函数，但造成了查找困难
serialStruct_t visionSerial = {
	visionDataSend,
	visionDataUpdate,
	visionReceive,
};

//混合对TX2的控制指令
void mixVisionCommand(uint8_t *array) {
	*array = 0x00;
	*array |= getvisionData()->distinguishMode << 0;
	*array |= getvisionData()->enemyType << 3;
	*array |= getvisionData()->buff_mod << 4;
}

//发送视觉数据
static void visionDataSend(USART_TypeDef *USARTx,serialStruct_t *serialBuffer) {
	uint8_t *array = (uint8_t *)USART_TO_ArrayTX(USARTx);
	uint8_t index_ptr = 0;
	/*信息幀*/
	//幀頭
	array[index_ptr++] = VISION_BEGIN;
	//包長度
	array[index_ptr++] = 0x00;
	//獲取存放包長度位置
	serialBuffer->length_position = index_ptr - 1;
	//CRC8校驗位
	array[index_ptr++] = 0x00;
	//獲取信息幀長度
	serialBuffer->header_length = index_ptr;
	
	/*數據幀*/
	//填装对tx2指令
	identifyCamp();
	mixVisionCommand(&array[index_ptr++]);
	//车辆类型
	array[index_ptr++] = getvisionData()->carType;
	array[index_ptr++] = getvisionData()->shootSpeed;
//	//PY: 裁判系统射速
//	formatTrans32Struct_t shootSpeedData;
//	shootSpeedData.float_temp = getvisionData()->judgeShootSpeed;
//	for(uint8_t index = 0; index < 4; index++)
//		array[index_ptr++] = shootSpeedData.u8_temp[index];
	formatTrans32Struct_t _imuData;
	_imuData.float_temp = SC_PITCH;
	for(uint8_t index = 0; index < 4; index++)
		array[index_ptr++] = _imuData.u8_temp[index];
	_imuData.float_temp = SC_YAW - getGimbalData()->yawAngleSave;	    //LCM: 传给TX2 上控时回中为yaw轴0点
	for(uint8_t index = 0; index < 4; index++)
		array[index_ptr++] = _imuData.u8_temp[index];
	_imuData.float_temp = IMU_RATEZ;
		for(uint8_t index = 0; index < 4; index++)
		array[index_ptr++] = _imuData.u8_temp[index];
	for(uint8_t index = 0; index < 2; index++)
		array[index_ptr++] = serialBuffer->CNTR.u8_temp[index];

	visionStoreHistoricalData(SC_PITCH,SC_YAW,serialBuffer->CNTR.u16_temp);	//LCM: 	存储此刻陀螺仪 PITCH 、YAW 角度 和 CNTR 
	//CRC16校驗位
	array[serialBuffer->length_position] = index_ptr + 2;
	//CNTR
	digitalIncreasing(&serialBuffer->CNTR.u16_temp);
	//生成CRC8	//LCM: CRC8 针对数据包头
	Append_CRC8_Check_Sum(array,serialBuffer->header_length);
	//生成CRC16	//LCM: CRC16针对整个数据包
	Append_CRC16_Check_Sum(array,array[serialBuffer->length_position]);
	BSP_USART_DMA_SendData(USARTx,array,array[serialBuffer->length_position]);
}

//读取视觉控制信息
static void visionDataDecoding(uint8_t array) {
	getvisionData()->distingushState = array;
	getvisionData()->captureFlag = array & 0x01;
	getvisionData()->sameTargetFlag = array & 0x02;
	getvisionData()->getOrderFlag = array & 0x04;
}

//视觉数据接收
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

//小云台数据发送
static void slaverBoardDataSend(USART_TypeDef *USARTx,serialStruct_t *serialBuffer){
	uint8_t *array = (uint8_t *)USART_TO_ArrayTX(USARTx);
	uint8_t index_ptr = 0;
	/*信息帧*/
	//帧头
	array[index_ptr++] = SLAVERBOARD_BEGIN;
	//包长度
	array[index_ptr++] = 0x00;
	//获取存放包长度位置
	serialBuffer->length_position = index_ptr - 1;
	//CRC8校验位
	array[index_ptr++] = 0x00;
	//获取信息帧长度
	serialBuffer->header_length = index_ptr;
	
	/*数据帧*/
	mixControlCommand(&array[index_ptr++], NULL);
	
	//填装当前云台的码盘角度
	slaveTransData.pitchGyroAngle.float_temp = getGimbalData()->pitchGyroAngle;
	slaveTransData.yawMotorAngle.float_temp = getGimbalData()->yawMotorAngle;
	
	for(uint8_t index=0;index < 4;index++){
		array[index_ptr++] = slaveTransData.pitchGyroAngle.u8_temp[index];
	}
	for(uint8_t index=0;index < 4;index++){
		array[index_ptr++] = slaveTransData.yawMotorAngle.u8_temp[index];
	}
	
	//填装当前云台的角度期望
	slaveTransData.pitchAngleRef.float_temp = getGimbalData()->pitchAngleRef;
	slaveTransData.yawAngleRef.float_temp = getGimbalData()->yawAngleRef;
	for(uint8_t index=0;index < 4;index++){
		array[index_ptr++] = slaveTransData.pitchAngleRef.u8_temp[index];
	}
	for(uint8_t index=0;index < 4;index++){
		array[index_ptr++] = slaveTransData.yawAngleRef.u8_temp[index];
	}

	//CRC16校验位
	array[serialBuffer->length_position] = index_ptr + 2;
	//CNTR
	digitalIncreasing(&serialBuffer->CNTR.u16_temp);
	//生成CRC8
	Append_CRC8_Check_Sum(array,serialBuffer->header_length);
	//生成CRC16
	Append_CRC16_Check_Sum(array,array[serialBuffer->length_position]);
	BSP_USART_DMA_SendData(USARTx,array,array[serialBuffer->length_position]);
	
}

//小云台数据接收
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

	//装填遥控器数据
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

//哨兵数据发送（底盘板发送）
static void masterBoardDataSend(USART_TypeDef *USARTx,serialStruct_t *serialBuffer){
	uint8_t *array = (uint8_t *)USART_TO_ArrayTX(USARTx);
	uint8_t index_ptr = 0;
	formatTrans16Struct_t trans16Data;
	formatTrans32Struct_t trans32Data;
	
	/*信息帧*/
	//帧头
	array[index_ptr++] = MASTERBOARD_BEGIN;              //0
	//包长度
	array[index_ptr++] = 0x00; 					   //1
	//获取存放包长度位置
	serialBuffer->length_position = index_ptr - 1;
	//CRC8校验位
	array[index_ptr++] = 0x00;					//2
	//获取信息帧长度
	serialBuffer->header_length = index_ptr;
	
	/*数据帧*/
	identifyCamp();
	mixControlCommand(&array[index_ptr++], &controlTransData);		//3
	//填装裁判系统信息
	
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
	
	trans16Data.u16_temp = judgeData.extGameRobotState.shooter_id1_17mm_cooling_rate;//17mm最大冷却
	for(uint8_t index = 0; index < 2; index++)
		array[index_ptr++] = trans16Data.u8_temp[index];
	
	trans16Data.u16_temp = judgeData.extGameRobotState.shooter_id1_17mm_speed_limit;
	for(uint8_t index = 0; index < 2; index++)
		array[index_ptr++] = trans16Data.u8_temp[index];
	
	array[index_ptr++] = judgeData.extRobotHurt.armor_id;						 //受到伤害的装甲板ID
	
	array[index_ptr++] = judgeData.extRobotHurt.hurt_type;						 //受到伤害的类型
	
	array[index_ptr++] = judgeData.extDartStatus.dart_belong;					 //导弹状态
	
	trans16Data.u16_temp = judgeData.extDartStatus.stage_remaining_time;         //导弹发射剩余时间
	for(uint8_t index = 0; index < 2; index++)
		array[index_ptr++] = trans16Data.u8_temp[index];
	
	//装填遥控器数据

	for(uint8_t index=0;index < 18;index++){				//32
		array[index_ptr++] = controlTransData.otherRcValue[index];
	}
	
	array[index_ptr++] = controlTransData.otherRcReadly;	//33
		
	//填装当前云台的码盘角度
	controlTransData.pitchGyroAngle.float_temp = getGimbalData()->pitchGyroAngle;  //哨兵改
	controlTransData.yawMotorAngle.float_temp = getGimbalData()->yawMotorAngle;
	
	for(uint8_t index=0;index < 4;index++){					//37
		array[index_ptr++] = controlTransData.pitchGyroAngle.u8_temp[index];
	}
	for(uint8_t index=0;index < 4;index++){					//41
		array[index_ptr++] = controlTransData.yawMotorAngle.u8_temp[index];
	}
	
	//填装当前云台的角度期望
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

	//CRC16校验位
	array[serialBuffer->length_position] = index_ptr + 2;
	//CNTR
	digitalIncreasing(&serialBuffer->CNTR.u16_temp);
	//生成CRC8
	Append_CRC8_Check_Sum(array,serialBuffer->header_length);
	//生成CRC16
	Append_CRC16_Check_Sum(array,array[serialBuffer->length_position]);
	BSP_USART_DMA_SendData(USARTx,array,array[serialBuffer->length_position]);
	
}

//哨兵数据接收
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
//	//计数位cntr
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
	//计数位cntr
	array[index_ptr++] = controlSerial.CNTR.u8_temp[0];	
	array[index_ptr++] = controlSerial.CNTR.u8_temp[1];
	//类型帧
	array[index_ptr++] = 0x00;
	/*************自定义数据*****************/
    //主控分频发送变形期望
    index_ptr = controlPackFreDiv(CONTROL_DEVICE_DEFF_REF,array,index_ptr);
    //主控分频发送其他控制数据
    index_ptr = controlPackFreDiv(CONTROL_DEVICE_OTHER,array,index_ptr);
    //主控分频发送遥控器数据
    index_ptr = controlPackFreDiv(CONTROL_DEVICE_RC,array,index_ptr);
    //主控分频发送底盘跟随期望
    index_ptr = controlPackFreDiv(CONTROL_DEVICE_CHASE_REF,array,index_ptr);
    //底盘分频发送编码器数据
    index_ptr = controlPackFreDiv(CONTROL_DEVICE_ENCODER,array,index_ptr);
    
    if(getcontrolData()->chassisBoardDataFinish == NOT_FINISH_STATE){
		index_ptr = controlPackFreDiv(CONTROL_DEVICE_INIT,array,index_ptr);
	}
	digitalIncreasing(&controlSerial.CNTR.u16_temp);
	array[CHASSIS_CONTROL_PACK_CLASS] = controlSerial.sendpackClass;
	
	//usbVCP_Printf("Control Send Length %d \r\n",index_ptr);
}

//主控板接收
static void controlDataUpload(uint8_t *bufferDecode){
	uint8_t index_ptr = 0;
	controlSerial.serialError.errorCount.u8_temp[0] = bufferDecode[index_ptr++];
	controlSerial.serialError.errorCount.u8_temp[1] = bufferDecode[index_ptr++];
	controlSerial.recepackClass = bufferDecode[index_ptr++];//决定接收到的数据集类型
	//主控接收底盘其他配置数据
	index_ptr = controlPackReceive(CHASSIS_DEVICE_OTHER,controlSerial.recepackClass,bufferDecode,index_ptr);
	//主控接收底盘裁判系统的数据
	index_ptr = controlPackReceive(CHASSIS_DEVICE_JUDGE_1,controlSerial.recepackClass,bufferDecode,index_ptr);
	//主控接收底盘裁判系统的数据
	index_ptr = controlPackReceive(CHASSIS_DEVICE_JUDGE_2,controlSerial.recepackClass,bufferDecode,index_ptr);
	//主控接收底盘传感器的数据
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
	//计数位cntr
	array[index_ptr++] = chassisSerial.CNTR.u8_temp[0];
	array[index_ptr++] = chassisSerial.CNTR.u8_temp[1];
	//类型帧
	array[index_ptr++] = 0x00;
	/*************自定义数据*****************/
    //底盘分频发送其他数据
    index_ptr = chassisPackFreDiv(CHASSIS_DEVICE_OTHER,array,index_ptr);
    //底盘分频发送裁判系统数据
    index_ptr = chassisPackFreDiv(CHASSIS_DEVICE_JUDGE_1,array,index_ptr);
    //底盘分频发送裁判系统数据
    index_ptr = chassisPackFreDiv(CHASSIS_DEVICE_JUDGE_2,array,index_ptr);
    //底盘分频发送传感器数据
    index_ptr = chassisPackFreDiv(CHASSIS_DEVICE_SENSOR,array,index_ptr);
    
	digitalIncreasing(&chassisSerial.CNTR.u16_temp);
	array[CHASSIS_CONTROL_PACK_CLASS] = chassisSerial.sendpackClass;
	
	//usbVCP_Printf("Chassis Send Length %d \r\n",index_ptr);
}

//底盘板接收
static void chassisDataUpload(uint8_t *bufferDecode){
	uint8_t index_ptr = 0;
    static uint8_t runTime = 0;
	chassisSerial.serialError.errorCount.u8_temp[0] = bufferDecode[index_ptr++];
	chassisSerial.serialError.errorCount.u8_temp[1] = bufferDecode[index_ptr++];
	//分类包
	chassisSerial.recepackClass = bufferDecode[index_ptr++];
	//底盘接收主控变形机构期望
	index_ptr = chassisPackReceive(CONTROL_DEVICE_DEFF_REF,chassisSerial.recepackClass,bufferDecode,index_ptr);
	//底盘接收主控其他控制数据
	index_ptr = chassisPackReceive(CONTROL_DEVICE_OTHER,chassisSerial.recepackClass,bufferDecode,index_ptr);
	//底盘接收主控遥控器数据
	index_ptr = chassisPackReceive(CONTROL_DEVICE_RC,chassisSerial.recepackClass,bufferDecode,index_ptr);
	//底盘接收主控闭环期望
	index_ptr = chassisPackReceive(CONTROL_DEVICE_CHASE_REF,chassisSerial.recepackClass,bufferDecode,index_ptr);
	//底盘接收主控码盘值
	index_ptr = chassisPackReceive(CONTROL_DEVICE_ENCODER,chassisSerial.recepackClass,bufferDecode,index_ptr);
	//底盘接收初始化数据
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
		//云台陀螺仪
		driver_slaveSensorInit(GIMBAL_IMU_USARTX, GIMBAL_IMU_USARTX_RX_PIN, GIMBAL_IMU_USARTX_TX_PIN, \
					 GIMBAL_IMU_USARTS_BOUND, GIMBAL_IMU_USARTX_PRE, GIMBAL_IMU_USARTX_SUB);	
		//蓝牙、数传串口
		driver_slaveSensorInit(USER_PRINTF_USARTX, USER_PRINTF_USARTX_RX_PIN, USER_PRINTF_USARTX_TX_PIN, \
					 USER_PRINTF_USARTS_BOUND, USER_PRINTF_USARTX_PRE, USER_PRINTF_USARTX_SUB);	
	}else if(BOARD_TYPE == BOARD_CHASSIS){
		//关节电机
		// USART1->CR1 |= USART_CR1_OVER8;
		// driver_slaveSensorInit(JOINT_L_USARTX, JOINT_L_USARTX_RX_PIN, JOINT_L_USARTX_TX_PIN, \
		// 		 JOINT_L_USARTS_BOUND, JOINT_L_USARTX_PRE, JOINT_L_USARTX_SUB);	
		// USART6->CR1 |= USART_CR1_OVER8;
		// driver_slaveSensorInit(JOINT_R_USARTX, JOINT_R_USARTX_RX_PIN, JOINT_R_USARTX_TX_PIN, \
		// 		 JOINT_R_USARTS_BOUND, JOINT_R_USARTX_PRE, JOINT_R_USARTX_SUB);	
		//底盘陀螺仪
		driver_slaveSensorInit(CHASSIS_IMU_USARTX, CHASSIS_IMU_USARTX_RX_PIN, CHASSIS_IMU_USARTX_TX_PIN, \
					 CHASSIS_IMU_USARTS_BOUND, CHASSIS_IMU_USARTX_PRE, CHASSIS_IMU_USARTX_SUB);	
		//测距
		driver_slaveSensorInit(TOF_L_USARTX, TOF_L_USARTX_RX_PIN, TOF_L_USARTX_TX_PIN, \
					 TOF_L_USARTS_BOUND, TOF_L_USARTX_PRE, TOF_L_USARTX_SUB);	
		driver_slaveSensorInit(TOF_R_USARTX, TOF_R_USARTX_RX_PIN, TOF_R_USARTX_TX_PIN, \
					 TOF_R_USARTS_BOUND, TOF_R_USARTX_PRE, TOF_R_USARTX_SUB);			
	}
	slaveSensorData.initFlag = true;  	
}
