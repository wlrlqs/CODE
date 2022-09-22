#include "application.h"
#include "motorHandle.h"


//主控发底盘封包

static void mixControlData(uint8_t *array){
	formatTrans8Struct_t mixRecData;
	mixRecData.byte.a_temp = getchassisData()->roDir;
	mixRecData.byte.b_temp = getinfantryAutoData()->rotateFlag;
	mixRecData.byte.c_temp = getpowerData()->rotateFast;
	mixRecData.byte.d_temp = getcontrolData()->controlBoardDataFinish;
	mixRecData.byte.e_temp = getchassisData()->autoMode;
	mixRecData.byte.f_temp = getchassisData()->suspend_chassis;
	*array = mixRecData.u8_temp;
}

//底盘解码主控数据

static void decodingChassisData(uint8_t decode){
	formatTrans8Struct_t decodeRecData;
	decodeRecData.u8_temp = decode;
	getchassisData()->roDir = decodeRecData.byte.a_temp;
	//步兵
	getinfantryAutoData()->rotateFlag =decodeRecData.byte.b_temp;
	getpowerData()->rotateFast = decodeRecData.byte.c_temp;
    getcontrolData()->controlBoardDataFinish = decodeRecData.byte.d_temp;
	getchassisData()->autoMode = decodeRecData.byte.e_temp;
	getchassisData()->suspend_chassis = decodeRecData.byte.f_temp;
}


//底盘发主控封包
static void mixChassisData(uint8_t *array){
	formatTrans8Struct_t mixRecData;
	mixRecData.byte.a_temp = getcontrolData()->chassisBoardDataFinish;
	
	*array = mixRecData.u8_temp;
}

//主控解码底盘数据
static void decodingControlData(uint8_t *decode){
	formatTrans8Struct_t decodeRecData;
	decodeRecData.u8_temp = *decode;
	getcontrolData()->chassisBoardDataFinish = decodeRecData.byte.a_temp;
}

float sendtest = 0;  
//主控发送数据包分频
uint8_t controlPackFreDiv(uint8_t transferdevice,uint8_t *array,uint8_t index_ptr){
	uint8_t freTimes = 0;
	uint8_t freMod = 0;
	switch(transferdevice){
		case CONTROL_DEVICE_DEFF_REF:{
			freTimes = FRE_POSITION_250B;
			freMod = FRE_250;
			if(!((getTransferRecData()->loops + freTimes) % freMod)){
				switch((uint8_t)getConfigData()->robotType){
						case INFANTRY_ID :{
								array[index_ptr++] = getinfantryAutoData()->separate_flag;	
								array[index_ptr++] = getinfantryAutoData()->chassis_fast;	
								array[index_ptr++] = getinfantryAutoData()->aviodFlag;	
								array[index_ptr++] = getchassisData()->direction;
								array[index_ptr++] = getshootData()->shootmode_ui;
								array[index_ptr++] = getshootData()->fricMotorFlag;
								array[index_ptr++] = getvisionData()->captureFlag;
							break;
						}
						 case F_TANK_ID :{//如果是摩擦轮英雄  发送数据给底盘控制电容充放电
								array[index_ptr++] =  getTankAutoData()->chassis_fast;  //无限功率标志位
								array[index_ptr++] = getcapacitorData()->percentage;
								array[index_ptr++] = getTankAutoData()->rotateFlag;
						}   break;
						case P_TANK_ID:{
								array[index_ptr++] = getcapacitorData()->percentage;
								break;
						}
						case AUXILIARY_ID:{
								array[index_ptr++] = getchassisData()->diffSpeedLimit;
								break;
						}     
				}
				controlSerial.sendpackClass |= transferdevice;
			}
			break;
		}
		case CONTROL_DEVICE_OTHER:{
			freTimes = FRE_POSITION_250B;
			freMod = FRE_250;
			if(!((getTransferRecData()->loops + freTimes) % freMod)){
                switch((uint8_t)getConfigData()->robotType){
                    case AUXILIARY_ID:{
                        
                    }break;
					case SENTRY_ID:{
                        mixControlCommand(&array[index_ptr++], NULL);
                    }break;
					
						
                }
				//发送机器人设备号
				formatTrans16Struct_t robotDeviceListData;
				robotDeviceListData.u16_temp = robotConfigData.robotDeviceList;
				array[index_ptr++] = robotDeviceListData.u8_temp[0];
				array[index_ptr++] = robotDeviceListData.u8_temp[1];
				//发送底盘控制模式
				array[index_ptr++] = getchassisData()->ctrlMode;
				//发送混合控制模式
				mixControlData(&array[index_ptr++]);
				//发送底盘控制限速
				
				controlSerial.sendpackClass |= transferdevice;
			}
			break;
		}
		case CONTROL_DEVICE_RC:{
			freTimes = FRE_POSITION_250F;
			freMod = FRE_250;
			if(!((getTransferRecData()->loops + freTimes) % freMod)){
				//底盘期望
				formatTrans32Struct_t speedTarget_X;
				formatTrans32Struct_t speedTarget_Y;
				formatTrans32Struct_t speedTarget_Z;
				formatTrans16Struct_t speedLanding_Y;
				formatTrans16Struct_t rotate;
				
				speedTarget_X.float_temp = remoteControlData.chassisSpeedTarget.x + keyBoardCtrlData.chassisSpeedTarget.x;
				speedTarget_Y.float_temp = remoteControlData.chassisSpeedTarget.y + keyBoardCtrlData.chassisSpeedTarget.y;
				speedTarget_Z.float_temp = remoteControlData.chassisSpeedTarget.z + keyBoardCtrlData.chassisSpeedTarget.z;
				for(uint8_t index = 0; index < 4;index++){
					array[index_ptr++] = speedTarget_X.u8_temp[index];
				}
				for(uint8_t index = 0; index < 4;index++){
					array[index_ptr++] = speedTarget_Y.u8_temp[index];
				}
				for(uint8_t index = 0; index < 4;index++){
					array[index_ptr++] = speedTarget_Z.u8_temp[index];
				}
				
				speedLanding_Y.s16_temp = (s16)getchassisData()->landingSpeedy;
				for(uint8_t index = 0; index < 2;index++){
					array[index_ptr++] = speedLanding_Y.u8_temp[index];
				}
        array[index_ptr++] = getchassisData()->chassisFlash;
				if(getchassisData()->chassisFlash != 0)  getchassisData()->chassisFlash = 0;
				for(uint8_t index = 0; index < 18;index++){
					array[index_ptr++] = Array_USART2_RX[index];
				}
				array[index_ptr++] = controlTransData.otherRcReadly;
				
				array[index_ptr++] = RC_GEAR;
				rotate.s16_temp = RC_ROTATE;
				for(uint8_t index = 0; index < 2;index++){
					array[index_ptr++] = rotate.u8_temp[index];
				}
								
				array[index_ptr++] = (uint8_t)getrobotMode();
				array[index_ptr++] = getchassisData()->turnswitch;
								
				controlSerial.sendpackClass |= transferdevice;				
			}
			break;
		}
		case CONTROL_DEVICE_CHASE_REF:{
			freTimes = FRE_POSITION_250F;
			freMod = FRE_250;
			if(!((getTransferRecData()->loops + freTimes) % freMod)){
				formatTrans32Struct_t chaseRefData;
				chaseRefData.float_temp =getchassisData()->chaseRef;
				for(uint8_t index = 0; index < 4;index++){
					array[index_ptr++] = chaseRefData.u8_temp[index];
				}
				controlSerial.sendpackClass |= transferdevice;
			}
			
			break;
		}
		case CONTROL_DEVICE_ENCODER:{
			freTimes = FRE_POSITION_250B;
			freMod = FRE_250;
			if(!((getTransferRecData()->loops + freTimes) % freMod)){
				formatTrans32Struct_t yawMotorAngleData;
				formatTrans32Struct_t pitchGyrpAngleData;
				
				yawMotorAngleData.float_temp = getGimbalData()->yawMotorAngle;
				for(uint8_t index = 0; index < 4;index++){
					array[index_ptr++] = yawMotorAngleData.u8_temp[index];
				}
				
				pitchGyrpAngleData.float_temp = getGimbalData()->pitchGyroAngle;
				for(uint8_t index = 0; index < 4;index++){
					array[index_ptr++] = pitchGyrpAngleData.u8_temp[index];
				}
				controlSerial.sendpackClass |= transferdevice;
			}
			
			break;
		}
        case  CONTROL_DEVICE_INIT :{
            array[index_ptr++] = YAW_INSTALL_CONFIG;
            array[index_ptr++] = getConfigData()->robotType;
            controlSerial.sendpackClass |= transferdevice;
            break;
        }
	}
	return index_ptr;
}

//主控接收数据包分频
uint8_t controlPackReceive(uint8_t transferdevice,uint8_t packClass,uint8_t *buffdecode, uint8_t index_ptr){
	switch(transferdevice & packClass){
		case CHASSIS_DEVICE_JUDGE_1:{
			/*接收裁判系統數據*/
			formatTrans16Struct_t trans16Data;
			for(uint8_t index = 0; index < 2; index++)
				trans16Data.u8_temp[index] = buffdecode[index_ptr++];
			judgeData.extPowerHeatData.shooter_id1_17mm_cooling_heat = trans16Data.u16_temp;
			//射擊數據
			formatTrans32Struct_t bulletSpeed;
			for(uint8_t index = 0; index < 4; index++)
				bulletSpeed.u8_temp[index] = buffdecode[index_ptr++];
				/**/
			judgeData.extShootData.bullet_speed = bulletSpeed.float_temp;
			/**/
			judgeData.extShootData.bullet_type = buffdecode[index_ptr++];
			break;
		}
		case CHASSIS_DEVICE_JUDGE_2:{
			/*接收裁判系統數據*/
			//狀態數據
			judgeData.extGameRobotState.robot_id = buffdecode[index_ptr++];
			formatTrans16Struct_t trans16Data;
			for(uint8_t index = 0; index < 2; index++)
				trans16Data.u8_temp[index] = buffdecode[index_ptr++];
			judgeData.extGameRobotState.max_HP = trans16Data.u16_temp;
			for(uint8_t index = 0; index < 2; index++)
				trans16Data.u8_temp[index] = buffdecode[index_ptr++];
			judgeData.extGameRobotState.remain_HP = trans16Data.u16_temp;
			for(uint8_t index = 0; index < 2; index++)
				trans16Data.u8_temp[index] = buffdecode[index_ptr++];
			judgeData.extGameRobotState.shooter_id1_17mm_cooling_limit = trans16Data.u16_temp;
			for(uint8_t index = 0; index < 2; index++)
				trans16Data.u8_temp[index] = buffdecode[index_ptr++];
			judgeData.extGameRobotState.shooter_id1_42mm_cooling_limit = trans16Data.u16_temp;
			for(uint8_t index = 0; index < 2; index++)
				trans16Data.u8_temp[index] = buffdecode[index_ptr++];
			judgeData.extGameRobotState.shooter_id1_17mm_cooling_rate = trans16Data.u16_temp;
			for(uint8_t index = 0; index < 2; index++)
				trans16Data.u8_temp[index] = buffdecode[index_ptr++];
			judgeData.extGameRobotState.shooter_id1_42mm_cooling_rate = trans16Data.u16_temp;
			for(uint8_t index = 0; index < 2; index++)
				trans16Data.u8_temp[index] = buffdecode[index_ptr++];
			judgeData.extGameRobotState.chassis_power_limit = trans16Data.u16_temp;
			judgeData.extGameRobotState.shooter_id1_17mm_speed_limit = buffdecode[index_ptr++];
			judgeData.extGameRobotState.shooter_id1_42mm_speed_limit = buffdecode[index_ptr++];
//			judgeData.extRobotHurt.armor_id = buffdecode[index_ptr++];
//			judgeData.extRobotHurt.hurt_type = buffdecode[index_ptr++];
//			judgeData.extDartStatus.dart_belong = buffdecode[index_ptr++];
//			for(uint8_t index = 0; index < 2; index++)
//				trans16Data.u8_temp[index] = buffdecode[index_ptr++];
//			judgeData.extDartStatus.stage_remaining_time = trans16Data.u16_temp;
			break;
		}
		case CHASSIS_DEVICE_SENSOR:{
			switch((uint8_t)getConfigData()->robotType){
						case INFANTRY_ID :{
								getchassisData()->chaseFbd = buffdecode[index_ptr++];		 //电容充放电状态
								getchassisData()->chaseSpeedRef = buffdecode[index_ptr++];	
								formatTrans32Struct_t pitch;
								for(uint8_t index = 0; index < 4; index++)
									pitch.u8_temp[index] = buffdecode[index_ptr++];
								getchassisData()->pitchAngleRef = pitch.float_temp;	
								for(uint8_t index = 0; index < 4; index++)
									pitch.u8_temp[index] = buffdecode[index_ptr++];
								getchassisData()->pitchAngleFbd = pitch.float_temp;	
								formatTrans16Struct_t trans16Data;
								for(uint8_t index = 0; index < 2; index++)
									trans16Data.u8_temp[index] = buffdecode[index_ptr++];
								commandoMotorConfig[8].motor_staus->currunt = trans16Data.s16_temp;
								getchassisLQRData()->fall = buffdecode[index_ptr++];
								break;
						}
						case F_TANK_ID:{
								getcapacitorData()->capacitor_status = buffdecode[index_ptr++];		 //电容充放电状态
								getcapacitorData()->percentage = buffdecode[index_ptr++];					 //电容电量
								break;
						}
						case P_TANK_ID:{
								getchassisData()->chaseFbd = buffdecode[index_ptr++];		 //电容充放电状态
								getchassisData()->chaseSpeedRef = buffdecode[index_ptr++];	
						break;
						}
						case AUXILIARY_ID:{
								//工程气动板数据发送

								break;
						}
						case SENTRY_ID:{
							controlTransData.othersenorFlag0 = buffdecode[index_ptr] & 0x02;
							controlTransData.othersenorFlag1 = buffdecode[index_ptr++] & 0x01;
								break;
					}				
			}
			break;
		}
		case CHASSIS_DEVICE_OTHER:{
            switch((uint8_t)getConfigData()->robotType){
                case AUXILIARY_ID:{
                    //拨弹电机数据反馈
                   
                    break;
                }
                case SENTRY_ID:{
					for(uint8_t index = 0; index < 4;index++){
						slaveTransData.yawMotorAngle.u8_temp[index] = buffdecode[index_ptr++];
					}
					for(uint8_t index = 0; index < 4;index++){
						slaveTransData.pitchGyroAngle.u8_temp[index] = buffdecode[index_ptr++];
					}
					otherControDataDecoding(buffdecode[index_ptr++], &slaveTransData);
                    break;
                }  				
            }
			decodingControlData(&buffdecode[index_ptr++]);
			getchassisData()->chassis_fault = buffdecode[index_ptr++];
			break;
		}
	}
	return index_ptr;
}


//底盘发送数据包分频
uint8_t chassisPackFreDiv(uint8_t transferdevice,uint8_t *array,uint8_t index_ptr){
	uint8_t freTimes = 0;
	uint8_t freMod = 0;
	switch(transferdevice){
		case CHASSIS_DEVICE_JUDGE_1:{
			freTimes = FRE_POSITION_250B;
			freMod = FRE_250;
			if(!((getTransferRecData()->loops + freTimes) % freMod)){
				//發送裁判系統數據
				formatTrans16Struct_t trans16Data;
				/*功率數據*/
				trans16Data.u16_temp = judgeData.extPowerHeatData.shooter_id1_17mm_cooling_heat;//槍口熱量
				for(uint8_t index = 0; index < 2; index++)
					array[index_ptr++] = trans16Data.u8_temp[index];
				/*射擊數據*/
				formatTrans32Struct_t bulletSpeed;
				bulletSpeed.float_temp = judgeData.extShootData.bullet_speed;//子彈速度
				for(uint8_t index = 0; index < 4; index++)
					array[index_ptr++] = bulletSpeed.u8_temp[index];
				array[index_ptr++] = judgeData.extShootData.bullet_type;//子彈類型
				chassisSerial.sendpackClass |= transferdevice;
			}
			break;
		}
		case CHASSIS_DEVICE_JUDGE_2:{
			freTimes = FRE_POSITION_250F;
			freMod = FRE_250;
			if(!((getTransferRecData()->loops + freTimes) % freMod)){
				//發送裁判系統數據
				/*狀態數據*/
				array[index_ptr++] = judgeData.extGameRobotState.robot_id;//機器人ID
				formatTrans16Struct_t trans16Data;
				trans16Data.u16_temp = judgeData.extGameRobotState.max_HP;//最大血量
				for(uint8_t index = 0; index < 2; index++)
					array[index_ptr++] = trans16Data.u8_temp[index];
				trans16Data.u16_temp = judgeData.extGameRobotState.remain_HP;//剩餘血量
				for(uint8_t index = 0; index < 2; index++)
					array[index_ptr++] = trans16Data.u8_temp[index];
				trans16Data.u16_temp = judgeData.extGameRobotState.shooter_id1_17mm_cooling_limit;//17mm最大热量
				for(uint8_t index = 0; index < 2; index++)
					array[index_ptr++] = trans16Data.u8_temp[index];
				trans16Data.u16_temp = judgeData.extGameRobotState.shooter_id1_42mm_cooling_limit;//42mm最大热量
				for(uint8_t index = 0; index < 2; index++)
					array[index_ptr++] = trans16Data.u8_temp[index];
				trans16Data.u16_temp = judgeData.extGameRobotState.shooter_id1_17mm_cooling_rate;//17mm最大冷却
				for(uint8_t index = 0; index < 2; index++)
					array[index_ptr++] = trans16Data.u8_temp[index];
				trans16Data.u16_temp = judgeData.extGameRobotState.shooter_id1_42mm_cooling_rate;//42mm最大冷却
				for(uint8_t index = 0; index < 2; index++)
					array[index_ptr++] = trans16Data.u8_temp[index];
				trans16Data.u16_temp = judgeData.extGameRobotState.chassis_power_limit;//最大功率
				for(uint8_t index = 0; index < 2; index++)
					array[index_ptr++] = trans16Data.u8_temp[index];
				array[index_ptr++] = judgeData.extGameRobotState.shooter_id1_17mm_speed_limit;  //17mm射速上限
				array[index_ptr++] = judgeData.extGameRobotState.shooter_id1_42mm_speed_limit;	 //42mm射速上限
//				array[index_ptr++] = judgeData.extRobotHurt.armor_id;						 //受到伤害的装甲板ID
//				array[index_ptr++] = judgeData.extRobotHurt.hurt_type;						 //受到伤害的类型
//				array[index_ptr++] = judgeData.extDartStatus.dart_belong;					 //导弹状态
//				trans16Data.u16_temp = judgeData.extDartStatus.stage_remaining_time;         //导弹发射剩余时间
//				for(uint8_t index = 0; index < 2; index++)
//					array[index_ptr++] = trans16Data.u8_temp[index];
				chassisSerial.sendpackClass |= transferdevice;
			}
			break;
		}
		case CHASSIS_DEVICE_SENSOR:{
			freTimes = FRE_POSITION_250F;
			freMod = FRE_250;
			if(!((getTransferRecData()->loops + freTimes) % freMod)){
				switch((uint8_t)getConfigData()->robotType){
						case INFANTRY_ID :{
								array[index_ptr++] = fabs(getchassisData()->chaseFbd);
								array[index_ptr++] = fabs(getchassisData()->chaseSpeedRef);
							
								formatTrans32Struct_t pitchData;
								pitchData.float_temp = getchassisData()->pitchAngleRef;
								for(uint8_t index = 0; index < 4; index++)
									array[index_ptr++] = pitchData.u8_temp[index];
							
								pitchData.float_temp = getchassisData()->pitchAngleFbd;
								for(uint8_t index = 0; index < 4; index++)
									array[index_ptr++] = pitchData.u8_temp[index];
							
								formatTrans16Struct_t trans16Data;
								trans16Data.s16_temp = commandoMotorConfig[8].motor_staus->currunt;
								for(uint8_t index = 0; index < 2; index++)
									array[index_ptr++] = trans16Data.u8_temp[index];
								array[index_ptr++] = getchassisLQRData()->fall;
								break;
						}
						case F_TANK_ID:{
								array[index_ptr++] = getcapacitorData()->capacitor_status;		 //电容充放电状态
								array[index_ptr++] = getcapacitorData()->percentage;					 //电容电量
								break;
						}
						case P_TANK_ID:{
								array[index_ptr++] = fabs(getchassisData()->chaseFbd);
								array[index_ptr++] = fabs(getchassisData()->chaseSpeedRef);
								break;
						}
						case AUXILIARY_ID:{
								//工程气动板数据发送

								break;
						}
						case SENTRY_ID:{
								//哨兵底盘数据发送
		array[index_ptr] |= controlTransData.othersenorFlag0 << 1;
		array[index_ptr++] |= controlTransData.othersenorFlag1 << 0;
								break;
						} 					
				}
				chassisSerial.sendpackClass |= transferdevice;
			}
			break;
		}
		case CHASSIS_DEVICE_OTHER:{
			freTimes = FRE_POSITION_250B;
			freMod = FRE_250;
			if(!((getTransferRecData()->loops + freTimes) % freMod)){	
                switch((uint8_t)getConfigData()->robotType){
                    case AUXILIARY_ID:{
                        //拨弹电机数据反馈
                        
                        break;
                    }
                    case SENTRY_ID:{
						for(uint8_t index = 0; index < 4;index++){
							array[index_ptr++] = slaveTransData.yawMotorAngle.u8_temp[index];
						}
						for(uint8_t index = 0; index < 4;index++){
							array[index_ptr++] = slaveTransData.pitchGyroAngle.u8_temp[index];
						}
						mixControlCommand(&array[index_ptr++], &slaveTransData);
                        break;
                    }   					
                }  
				//混合数据发送
				mixChassisData(&array[index_ptr++]);
				array[index_ptr++] = getchassisData()->chassis_fault;
				chassisSerial.sendpackClass |= transferdevice;
			}
			break;
		}
	}
	return index_ptr;
	
}


//底盘接收数据包分频
uint8_t chassisPackReceive(uint8_t transferdevice,uint8_t packClass,uint8_t *buffdecode, uint8_t index_ptr){
	switch(transferdevice & packClass){
		case CONTROL_DEVICE_DEFF_REF:{
			switch((uint8_t)getConfigData()->robotType){
						case INFANTRY_ID :{
								getinfantryAutoData()->separate_flag = buffdecode[index_ptr++];
								getchassisData()->chassis_boost = buffdecode[index_ptr++];
								getinfantryAutoData()->aviodFlag = buffdecode[index_ptr++];
								getchassisData()->direction = buffdecode[index_ptr++];
								getshootData()->shootmode_ui = buffdecode[index_ptr++];
								getshootData()->fricMotorFlag = buffdecode[index_ptr++];
								getvisionData()->captureFlag = buffdecode[index_ptr++];
							break;
						}
						case F_TANK_ID :{//摩擦轮英雄接收云台主控底盘主控控制电容
								getTankAutoData()->chassis_fast = buffdecode[index_ptr++];
								getcapacitorData()->percentage = buffdecode[index_ptr++];
								getTankAutoData()->rotateFlag = buffdecode[index_ptr++];
								break;
						}
						case P_TANK_ID:{
								getcapacitorData()->percentage = buffdecode[index_ptr++];
								break;
						}
						case AUXILIARY_ID:{
								getchassisData()->diffSpeedLimit = buffdecode[index_ptr++];
								break;
						}     
				}
			break;
		}
		case CONTROL_DEVICE_RC:{
			formatTrans32Struct_t speedTarget_X;
			formatTrans32Struct_t speedTarget_Y;
			formatTrans32Struct_t speedTarget_Z;
			formatTrans16Struct_t speedLanding_Y;
			formatTrans16Struct_t rotate;
			for(uint8_t index = 0; index < 4;index++){
				speedTarget_X.u8_temp[index] = buffdecode[index_ptr++];
			}
			for(uint8_t index = 0; index < 4;index++){
				speedTarget_Y.u8_temp[index] = buffdecode[index_ptr++];
			}
			for(uint8_t index = 0; index < 4;index++){
				speedTarget_Z.u8_temp[index] = buffdecode[index_ptr++];
			}
			getchassisData()->sumofChssisTarget.x = speedTarget_X.float_temp;
			getchassisData()->sumofChssisTarget.y = speedTarget_Y.float_temp;
			getchassisData()->sumofChssisTarget.z = speedTarget_Z.float_temp;
			
			for(uint8_t index = 0; index < 2;index++){
				speedLanding_Y.u8_temp[index] = buffdecode[index_ptr++];
			}
			getchassisData()->landingSpeedy = speedLanding_Y.s16_temp;
			getchassisData()->chassisFlash = buffdecode[index_ptr++];
			if(getchassisData()->chassisFlash == 1)
				supervisorData.flashSave = 1;
			for(uint8_t index = 0; index < 18;index++){
				controlTransData.otherRcValue[index] = 	buffdecode[index_ptr++];	
			}
			controlTransData.otherRcReadly = buffdecode[index_ptr++];
			
			RC_GEAR = buffdecode[index_ptr++];
			for(uint8_t index = 0; index < 2;index++){
				rotate.u8_temp[index] = buffdecode[index_ptr++];
			}
			RC_ROTATE = rotate.s16_temp;
			
			setRobotMode((robotModeStruct_t)buffdecode[index_ptr++]);
			getchassisData()->turnswitch = (int8_t)buffdecode[index_ptr++];
			break;
		}
		case CONTROL_DEVICE_CHASE_REF:{
			formatTrans32Struct_t chaseRefData;
			for(uint8_t index = 0; index < 4;index++){
				chaseRefData.u8_temp[index] = buffdecode[index_ptr++];
			}
			getchassisData()->chaseRef = chaseRefData.float_temp;
			break;
		}
		case CONTROL_DEVICE_OTHER:{
       switch((uint8_t)getConfigData()->robotType){
				case AUXILIARY_ID:{
					
				}break;
				case SENTRY_ID:{
					otherControDataDecoding(buffdecode[index_ptr++], &controlTransData);
				}break;
            }
			formatTrans16Struct_t robotDeviceListData;
			robotDeviceListData.u8_temp[0] = buffdecode[index_ptr++];
			robotDeviceListData.u8_temp[1] = buffdecode[index_ptr++];
			robotConfigData.robotDeviceList = robotDeviceListData.u16_temp;
			getchassisData()->ctrlMode = (chassisMode_e)buffdecode[index_ptr++];
			decodingChassisData(buffdecode[index_ptr++]);
			break;
		}
		case CONTROL_DEVICE_ENCODER:{
			formatTrans32Struct_t yawMotorAngleData;
			formatTrans32Struct_t pitchGyroAngleData;
			for(uint8_t index = 0; index < 4;index++){
				yawMotorAngleData.u8_temp[index] = buffdecode[index_ptr++];
			}
			getGimbalData()->yawMotorAngle = yawMotorAngleData.float_temp ;
			
			for(uint8_t index = 0; index < 4;index++){
				pitchGyroAngleData.u8_temp[index] = buffdecode[index_ptr++];
			}
			getGimbalData()->pitchGyroAngle = pitchGyroAngleData.float_temp ;
			break;
		}
        case CONTROL_DEVICE_INIT :{
            YAW_INSTALL_CONFIG = buffdecode[index_ptr++];
            getConfigData()->robotType = buffdecode[index_ptr++];
            break;
        }
	}
	return index_ptr;
}


