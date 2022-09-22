#include "Driver_MotorSever.h"
#include "Driver_MotorSeverBase.h"
/*RMD电机通信协议*/
void RMD_MotorUpdata(motorConfigStruct_t *motorConfig);
void RMD_MotorReviece(motorConfigStruct_t* motorConfig,CanRxMsg *CanRevData);
/*RM电机通信协议*/
void RM_MotorUpdata(motorConfigStruct_t *motorConfig);
void RM_MotorReviece(motorConfigStruct_t *motorConfig,CanRxMsg *CanRevData);
/*电机指针初始化*/
void initPre(motorConfigStruct_t *commando,motorConfigStruct_t *robot);
void motorSeverPtrInit(void);
bool motorSeverConfirm(bool *device);
/*电机发送&接收*/
void motorSeverSendUpdate(void);
void motorSeverReceive(motorConfigStruct_t* motorUsing,CanRxMsg *CanRevData);
/*初始化类*/
deviceInitClass motorSeverInitClass = {
	//电机配置表初始化
	motorSeverPtrInit,
};
/*任务类*/
motorSeverFuncClass motorSeverClass = {
	//发送更新
	motorSeverSendUpdate,
	//接收更新
	motorSeverReceive,
};

uint8_t getModID(motorConfigStruct_t* motor){
	uint8_t returnID = 0;
	if(motor->motor_lib.motor_type == RM_MOTOR)
		returnID = motor->receive_identifier - 0x200;
	else if(motor->motor_lib.motor_type == GM_MOTOR)
		returnID = motor->receive_identifier - 0x204;
	else if(motor->motor_lib.motor_type == RMD_MOTOR)
		returnID = motor->receive_identifier - 0x140;
	return returnID;
}

void equipMotorCurrent(motorConfigStruct_t* motor,f32_t *current){
	if(motor->motor_lib.motor_type == NO_MOTOR) return;
	motor->currentOut = current;
}

void RMDcurrentDective(motorConfigStruct_t* motor,CAN_SendForm (*motorPack)[CAN_NET_LIST]){
	motorPack[motor->can_net_parameter.board_net][motor->can_net_parameter.can_net].current[0] = &RMDcmd;
	motorPack[motor->can_net_parameter.board_net][motor->can_net_parameter.can_net].current[1] = motor->currentOut;
}

void RMcurrentDective(motorConfigStruct_t* motor,CAN_SendForm (*motorPack)[CAN_NET_LIST]){
	uint8_t modID = getModID(motor);
	switch(modID % 4){
		case 1:
			motorPack[motor->can_net_parameter.board_net][motor->can_net_parameter.can_net].current[0] = motor->currentOut;
			break;
		case 2:
			motorPack[motor->can_net_parameter.board_net][motor->can_net_parameter.can_net].current[1] = motor->currentOut;
			break;
		case 3:
			motorPack[motor->can_net_parameter.board_net][motor->can_net_parameter.can_net].current[2] = motor->currentOut;
			break;
		case 0:
			motorPack[motor->can_net_parameter.board_net][motor->can_net_parameter.can_net].current[3] = motor->currentOut;
			break;
	}
}

void motorCurrentDective(motorConfigStruct_t* motor){
	if(motor->motor_lib.motor_type == RMD_MOTOR)
		RMDcurrentDective(motor,motor_pack);
	else if(motor->motor_lib.motor_type == RM_MOTOR || motor->motor_lib.motor_type == GM_MOTOR)
		RMcurrentDective(motor,motor_pack);
}

void freqDective(motorConfigStruct_t* motor){
	if(motor->motor_lib.motor_frequency == INCREASING_500HZ){
		motor->motor_frequency_parameter.loopIncresing	= INCREASING_500HZ;
		motor->motor_frequency_parameter.loopMod		= MOD_500HZ;
	}
	else if(motor->motor_lib.motor_frequency == INCREASING_250HZ_F){
		motor->motor_frequency_parameter.loopIncresing	= INCREASING_250HZ_F;
		motor->motor_frequency_parameter.loopMod		= MOD_250HZ;
	}
	else if(motor->motor_lib.motor_frequency == INCREASING_250HZ_B){
		motor->motor_frequency_parameter.loopIncresing	= INCREASING_250HZ_B;
		motor->motor_frequency_parameter.loopMod		= MOD_250HZ;
	}
	else return;
}

void encoderDective(motorConfigStruct_t* motor){
	if(motor->motor_lib.motor_type == RMD_MOTOR)
		motor->motor_lib.motor_encoder = ENCODER_14;
	else if(motor->motor_lib.motor_type == RM_MOTOR || motor->motor_lib.motor_type == GM_MOTOR)
		motor->motor_lib.motor_encoder = ENCODER_13;
}
/********************************************************************
函数名：RMD_MotorUpdata
功能：RMD电机发送数据函数
入口参数：	motorConfigStruct_t *motorConfig ：对应的兵种电机参数列表
返回值：无
应用范围：电机CAN发送更新调用
备注：
*********************************************************************/
void RMD_MotorUpdata(motorConfigStruct_t *motorConfig){
	if(motorConfig->motor_lib.motor_type == RMD_MOTOR){
		CanTxMsg *txMessage;
		uint8_t mbox;
		uint16_t i = 0;
		txMessage = (CanTxMsg*)aqCalloc(8,sizeof(CanTxMsg));
		txMessage->StdId = motorConfig->send_identifier;
		txMessage->IDE = CAN_Id_Standard;
		txMessage->RTR = CAN_RTR_Data;
		txMessage->DLC = 0x08;
		//控制命令
		txMessage->Data[0] = (uint8_t)((int16_t)*motor_pack[motorConfig->can_net_parameter.board_net][motorConfig->can_net_parameter.can_net].current[0]); 
		//转矩电流输出
		txMessage->Data[4] = (uint8_t)((int16_t)*motor_pack[motorConfig->can_net_parameter.board_net][motorConfig->can_net_parameter.can_net].current[1] & 0xFF);
		txMessage->Data[5] = (uint8_t)(((int16_t)*motor_pack[motorConfig->can_net_parameter.board_net][motorConfig->can_net_parameter.can_net].current[1] >> 8) & 0xFF);
		mbox = CAN_Transmit(motorConfig->motorType, txMessage);
		while (CAN_TransmitStatus(motorConfig->motorType,mbox) == 0x00) {
			i++;
			if (i >= 0xFFF)break;
		}
		aqFree(txMessage,8,sizeof(CanTxMsg));
	}
	else{
		usbVCP_Printf("Error Driver \r\n");
        return;
	}
}
/********************************************************************
函数名：RMD_MotorReviece
功能：RMD电机接收数据函数
入口参数：	motorConfigStruct_t *motorConfig ：对应的兵种电机参数列表
            CanRxMsg *CanRevData ：CAN总线接收数据
返回值：无
应用范围：CAN接收中断调用
备注：
*********************************************************************/
void RMD_MotorReviece(motorConfigStruct_t *motorConfig,CanRxMsg *CanRevData){
    motorConfig->motor_staus->temperature   = (int8_t)CanRevData->Data[1];
    motorConfig->motor_staus->currunt       = (int16_t)((CanRevData->Data[3] << 8) | CanRevData ->Data[2]);
    motorConfig->motor_staus->motorSpeed    = (int16_t)((CanRevData->Data[5] << 8) | CanRevData->Data[4]);
    motorConfig->motor_staus->motorEncoder  = (uint16_t)((CanRevData->Data[7] << 8) | CanRevData->Data[6]);
}
/********************************************************************
函数名：RM_MotorUpdata
功能：RM电机发送数据函数
入口参数：	motorConfigStruct_t *motorConfig ：对应的兵种电机参数列表
返回值：无
应用范围：电机CAN发送更新调用
备注：
*********************************************************************/
void RM_MotorUpdata(motorConfigStruct_t *motorConfig){
	if(motorConfig->motor_lib.motor_type == RM_MOTOR || motorConfig->motor_lib.motor_type == GM_MOTOR){
		CanTxMsg *txMessage;
		uint8_t mbox;
		uint16_t i = 0;
		txMessage = (CanTxMsg*)aqCalloc(8,sizeof(CanTxMsg));
		txMessage->StdId = motorConfig->send_identifier;
		txMessage->IDE = CAN_Id_Standard;
		txMessage->RTR = CAN_RTR_Data;
		txMessage->DLC = 0x08;
		//current
		txMessage->Data[0] = (uint8_t)(((int16_t)*motor_pack[motorConfig->can_net_parameter.board_net][motorConfig->can_net_parameter.can_net].current[0]) >> 8);
		txMessage->Data[1] = (uint8_t)((int16_t) *motor_pack[motorConfig->can_net_parameter.board_net][motorConfig->can_net_parameter.can_net].current[0]);
		txMessage->Data[2] = (uint8_t)(((int16_t)*motor_pack[motorConfig->can_net_parameter.board_net][motorConfig->can_net_parameter.can_net].current[1]) >> 8);
		txMessage->Data[3] = (uint8_t)((int16_t) *motor_pack[motorConfig->can_net_parameter.board_net][motorConfig->can_net_parameter.can_net].current[1]);
		txMessage->Data[4] = (uint8_t)(((int16_t)*motor_pack[motorConfig->can_net_parameter.board_net][motorConfig->can_net_parameter.can_net].current[2]) >> 8);
		txMessage->Data[5] = (uint8_t)((int16_t) *motor_pack[motorConfig->can_net_parameter.board_net][motorConfig->can_net_parameter.can_net].current[2]);
		txMessage->Data[6] = (uint8_t)(((int16_t)*motor_pack[motorConfig->can_net_parameter.board_net][motorConfig->can_net_parameter.can_net].current[3]) >> 8);
		txMessage->Data[7] = (uint8_t)((int16_t) *motor_pack[motorConfig->can_net_parameter.board_net][motorConfig->can_net_parameter.can_net].current[3]);
		mbox = CAN_Transmit(motorConfig->motorType, txMessage);
		while (CAN_TransmitStatus(motorConfig->motorType,mbox) == 0x00) {
			i++;
			if (i >= 0xFFF)break;
		}
		aqFree(txMessage,8,sizeof(CanTxMsg));
	}
	else{
		usbVCP_Printf("Error Driver \r\n");
        return;
	}
}
/********************************************************************
函数名：RM_MotorReviece
功能：RMD电机接收数据函数
入口参数：	motorConfigStruct_t *motorConfig ：对应的兵种电机参数列表
            CanRxMsg *CanRevData ：CAN总线接收数据
返回值：无
应用范围：CAN接收中断调用
备注：
*********************************************************************/
void RM_MotorReviece(motorConfigStruct_t *motorConfig,CanRxMsg *CanRevData){
    motorConfig->motor_staus->motorEncoder  =   (uint16_t)((CanRevData->Data[0] << 8) | CanRevData->Data[1]);
    motorConfig->motor_staus->motorSpeed    =   (int16_t)((CanRevData->Data[2] << 8) | CanRevData->Data[3]);
    motorConfig->motor_staus->currunt       =   (int16_t)((CanRevData->Data[4] << 8) | CanRevData->Data[5]);
    motorConfig->motor_staus->temperature   =   (int8_t)CanRevData->Data[6];
}
/********************************************************************
函数名：motorSeverPtrInit
功能：电机指针初始化
入口参数：无
返回值：无
应用范围：初始化
备注：
*********************************************************************/
void motorSeverPtrInit(void){
	switch(robotConfigData.typeOfRobot){
		case INFANTRY_ID:{
			initPre(commandoMotorConfig,infantryMotorConfig);
		}break;
		case P_TANK_ID:{
			initPre(commandoMotorConfig,p_tankMotorConfig);
		}break;
		 case AUXILIARY_ID:{
            initPre(commandoMotorConfig,auxiliayMotorConfig);
        }break;
        case SENTRY_ID:{
            initPre(commandoMotorConfig,sentryMotorConfig);
        }break;
        case UAV_ID:{
            initPre(commandoMotorConfig,uavMotorConfig);
        }break;
		case SMALLGIMBAL_ID:{
			initPre(commandoMotorConfig,smgMotorConfig);
		}break;
		case F_TANK_ID:{
			initPre(commandoMotorConfig,f_tankMotorConfig);
		}break;
        default : 
            usbVCP_Printf("NO robot type!! \r\n");   
        break;
	}
}
/********************************************************************
函数名：motorPackSendUpdate
功能：电机发送更新
入口参数：无
返回值：无
应用范围：CAN发送更新调用
备注：
*********************************************************************/
void motorPackSendUpdate(Net_Code_e net_code){
	volatile uint8_t countList;
	uint8_t sameTarget = 0;
	memset(&histSend_mark,0,sizeof(uint8_t) * CAN_NET_LIST);
	for(countList = YAWMOTOR; countList < USE_LIST; countList++){
		//无电机则跳过
		if(commandoMotorConfig[countList].motor_lib.motor_type == NO_MOTOR)
			continue;
		//网络不同则跳过
		else if(net_code != commandoMotorConfig[countList].can_net_parameter.board_net)
			continue;
		else{
			//历史发送位增加
			digitalIncreasing(&histSend_mark[commandoMotorConfig[countList].can_net_parameter.can_net]);
			//防止重复发送
			if(countList != YAWMOTOR){
				if(histSend_mark[commandoMotorConfig[countList].can_net_parameter.can_net] > 0x01)
					sameTarget = 1;
				else
					sameTarget = 0;
			}
			if(!sameTarget)
				if(!((getcanSendData()->loops + commandoMotorConfig[countList].motor_frequency_parameter.loopIncresing) % commandoMotorConfig[countList].motor_frequency_parameter.loopMod))
					commandoMotorConfig[countList].motorUpdate(&commandoMotorConfig[countList]);
		}
	}
}
/********************************************************************
函数名：motorSeverSendUpdate
功能：电机发送更新
入口参数：无
返回值：无
应用范围：CAN发送更新调用
备注：
*********************************************************************/
void motorSeverSendUpdate(void){
	motorPackSendUpdate(board_net);
}
/********************************************************************
函数名：motorSeverReceive
功能：电机接收
入口参数：	motorConfigStruct_t *motorConfig ：对应用途电机参数列表
            CanRxMsg *CanRevData ：CAN总线接收数据
返回值：无
应用范围：CAN接收中断调用
备注：
*********************************************************************/
void motorSeverReceive(motorConfigStruct_t* motorUsing,CanRxMsg *CanRevData){
	if(motorUsing->motorRev)
		motorUsing->motorRev(motorUsing,CanRevData);
	digitalIncreasing(&motorUsing->errorCount);
}

