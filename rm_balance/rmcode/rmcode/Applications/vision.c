#include "application.h"
#include "Driver_USBVCP.h"
#include "ramp.h"

visionStruct_t visionData;

visionStruct_t* getvisionData() {
    return &visionData;
}

//LCM: 由 controlTrans 调用，按照 transferRecUpdateTask 的频率更新 
void visionStoreHistoricalData(f32_t pitch, f32_t yaw, uint16_t CNTR) {
//	visionData.pitchDataStored[visionData.storedIndex] = pitch;		//LCM: 存储当前陀螺仪角度
//	visionData.yawDataStored[visionData.storedIndex] = yaw;
	visionData.CNTR_DataStored[visionData.storedIndex] = CNTR;		//LCM:	存储当前CNTR值 
	visionData.storedIndex = (visionData.storedIndex + 1) % VISION_STORED_LENGTH;	//存储值下标加一，并保证在存储范围之内	
}

//返回50以内的数证明找到了对应角度或最接近的下标，返回255说明没有适配目标
uint8_t visionMatchAngle(void) {
	uint16_t min = 65535;
	uint8_t r = 0;
	for(uint8_t index = 0; index < VISION_STORED_LENGTH; index++) {
		//如果CNTR相同则直接返回下标
		if(visionData.CNTR_DataStored[index] == visionData.CNTR.u16_temp) {
			return index;
		}
		if(abs(visionData.CNTR_DataStored[index] - visionData.CNTR.u16_temp) < min) {
            min = abs(visionData.CNTR_DataStored[index] - visionData.CNTR.u16_temp);
            r = index;
        }
	}
	//如果没有找到相同的，为防止丢包，在+-1这两个数再找
	if(min < 2) {
		return r;
	}
	else {
		return 255;
	}
}

//模式与子弹类型刷新
void visionSendDataUpdate(uint8_t workMode,uint8_t bullet) {
	visionData.distinguishMode = (visionWorkMode_e)workMode;
	visionData.bullet = (bulletType_e)bullet;
}

//视觉发送数据初始化
void visionSendDataInit(void) {
	visionData.distinguishMode = TX2_STOP;
	visionData.bullet = SMALL_BULLET;
}

//阵营更新
void identifyCamp(void) {
	if(robotConfigData.typeOfRobot == SMALLGIMBAL_ID) {
		visionData.enemyType = controlTransData.otherEnemyType;
	}
	else{
        //如果己方是蓝方
		if(ROBOT_ID > 0x09) {									
			visionData.enemyType = ENEMY_RED;	  
		}							
		else {
			visionData.enemyType = ENEMY_BLUE;
		}	
	}	
}

//视觉射击初始化
void shootVisionInit(void) {
	switch(ROBOT){
		case INFANTRY_ID:
			visionData.carType = INFANTRY_BARREL;
			break;
		case F_TANK_ID:
			visionData.carType = F_TANK_BARREL;
			break;
		case P_TANK_ID:
			visionData.carType = P_TANK_BARREL;
			break;
		case AUXILIARY_ID:
			visionData.carType = AUXILIARY_BARREL;
			break;
		case SENTRY_ID:
			visionData.carType = NEW_SENTRY_BARREL;
			break;
		case SMALLGIMBAL_ID:
			visionData.carType = NEW_SMG_BARREL;
			break;
		case UAV_ID:
			visionData.carType = UAV_BARREL;
			break;
		default:break;
	}
}

void visionDataReset() {
    //重置参数时，将反馈值作为期望（云台不动）
	visionData.pitchCmdBuff = getGimbalData()->pitchAngleFbd;
	visionData.yawCmdBuff = getGimbalData()->yawAngleFbd;
}

//LCM: 不带预判的角度计算函数
void visionGimbalNormal() {
	visionData.yawCmdBuff = visionData.yawData.float_temp;
	visionData.pitchCmdBuff = visionData.pitchData.float_temp;
    
    //若Yaw轴偏转角大于45°，视为无效，云台不动
    if(fabsf(visionData.yawCmdBuff - getGimbalData()->yawAngleFbd) >= 45) {
        visionDataReset();
        visionData.captureFlag = false;     //视为未捕捉到目标 
    }
    //若Pitch轴角度绝对值大于45°，视为无效，云台不动
    if(fabsf(visionData.pitchCmdBuff) >= 45) {
        visionDataReset();
        visionData.captureFlag = false;     //视为未捕捉到目标 
    }
}

//LCM: 对yaw轴pitch轴数据进行最后处理，得到最终Cmd值，Cmd值在gimbal.c中被作为角度期望 
void visionAngleCalculation() {
    
	visionGimbalNormal();

    visionData.yawCmd = visionData.yawCmdBuff;		//LCM: 该值将作为云台角度期望
	visionData.pitchCmd = visionData.pitchCmdBuff;

	//手动预判
	//LCM: 自瞄模式下使用右键，可以带微调
	//LCM: 似乎右键只有这个作用 
	if(visionData.prejudgFlag && (visionData.distinguishMode == TX2_DISTINGUISH_ARMOR)) {
		visionData.yawCmd = visionData.yawCmd + visionData.manualYawBias;
		visionData.pitchCmd = visionData.pitchCmd + visionData.manualPitchBias;	//LCM: 加入削弱后的操作手操作量			
	}
}

//LCM: 讲真这东西该移到英雄自动任务里
//迫击炮云台到达角度计算
void visionMortar() {
	visionData.mortarYawCmdBuff = 30.0f;	
	visionData.mortarPitCmdBuff = 30.0f;	
	visionData.mortarYawCmd = visionData.manualYawBias + visionData.mortarYawCmdBuff; 
	visionData.mortarPitCmd = visionData.manualPitchBias + visionData.mortarPitCmdBuff;
}

//视觉任务更新
void visionUpdateTask(void *Parameters) {
	static bool captureFlag = 0,lastCaptureFlag = 0;	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(true){
		vTaskDelayUntil(&xLastWakeTime,VISION_PERIOD);
		captureFlag = visionData.captureFlag;

		if(!captureFlag){
			lastCaptureFlag = captureFlag;
		}
		else{
			//LCM: 匹配CNTR，匹配失败则跳过此次刷新
			uint8_t dataMatchIndex = visionMatchAngle();	//LCM: 若匹配失败， visionMatchAngle 返回值为255
			if(dataMatchIndex != 255) {
                
			}
			else {
				goto loopIncreasing;
			}
			
			//LCM: 突然捕捉到目标 则重置参数
			if(!lastCaptureFlag) {
				visionDataReset();
			}
			
			//LCM: 若下一帧数据包已传来
			if(visionData.CNTR.u16_temp != visionData.lastCNTR) {
				visionData.cailSuccess = false;
			}
			//LCM: 若下一帧数据包尚未传来
			else {
				visionData.cailSuccess = true;
			}
			visionData.lastCNTR = visionData.CNTR.u16_temp;			
			//角度计算
			visionAngleCalculation();
			lastCaptureFlag = captureFlag;
		}
		loopIncreasing:
			digitalIncreasing(&visionData.loops);
	}
}

//视觉数据初始化
void visionInit(void) {
	memset((void *)&visionData, 0, sizeof(visionData));
	visionData.lastCNTR = visionData.CNTR.u16_temp;
	visionData.initFlag = true;
	shootVisionInit();
	visionData.prejudgFlag = false;	//LCM: 预判打击标志位
	supervisorData.taskEvent[VISION_TASK] = xTaskCreate(visionUpdateTask, \
                                                        "VISION", \
                                                        VISION_STACK_SIZE, \
                                                        NULL, \
														VISION_PRIORITY, \
                                                        &visionData.xHandleTask);
    usbVCP_Printf("visionInit Successfully \r\n");
}
