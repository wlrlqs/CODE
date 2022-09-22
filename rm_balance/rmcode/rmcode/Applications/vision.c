#include "application.h"
#include "Driver_USBVCP.h"
#include "ramp.h"

visionStruct_t visionData;

visionStruct_t* getvisionData() {
    return &visionData;
}

//LCM: �� controlTrans ���ã����� transferRecUpdateTask ��Ƶ�ʸ��� 
void visionStoreHistoricalData(f32_t pitch, f32_t yaw, uint16_t CNTR) {
//	visionData.pitchDataStored[visionData.storedIndex] = pitch;		//LCM: �洢��ǰ�����ǽǶ�
//	visionData.yawDataStored[visionData.storedIndex] = yaw;
	visionData.CNTR_DataStored[visionData.storedIndex] = CNTR;		//LCM:	�洢��ǰCNTRֵ 
	visionData.storedIndex = (visionData.storedIndex + 1) % VISION_STORED_LENGTH;	//�洢ֵ�±��һ������֤�ڴ洢��Χ֮��	
}

//����50���ڵ���֤���ҵ��˶�Ӧ�ǶȻ���ӽ����±꣬����255˵��û������Ŀ��
uint8_t visionMatchAngle(void) {
	uint16_t min = 65535;
	uint8_t r = 0;
	for(uint8_t index = 0; index < VISION_STORED_LENGTH; index++) {
		//���CNTR��ͬ��ֱ�ӷ����±�
		if(visionData.CNTR_DataStored[index] == visionData.CNTR.u16_temp) {
			return index;
		}
		if(abs(visionData.CNTR_DataStored[index] - visionData.CNTR.u16_temp) < min) {
            min = abs(visionData.CNTR_DataStored[index] - visionData.CNTR.u16_temp);
            r = index;
        }
	}
	//���û���ҵ���ͬ�ģ�Ϊ��ֹ��������+-1������������
	if(min < 2) {
		return r;
	}
	else {
		return 255;
	}
}

//ģʽ���ӵ�����ˢ��
void visionSendDataUpdate(uint8_t workMode,uint8_t bullet) {
	visionData.distinguishMode = (visionWorkMode_e)workMode;
	visionData.bullet = (bulletType_e)bullet;
}

//�Ӿ��������ݳ�ʼ��
void visionSendDataInit(void) {
	visionData.distinguishMode = TX2_STOP;
	visionData.bullet = SMALL_BULLET;
}

//��Ӫ����
void identifyCamp(void) {
	if(robotConfigData.typeOfRobot == SMALLGIMBAL_ID) {
		visionData.enemyType = controlTransData.otherEnemyType;
	}
	else{
        //�������������
		if(ROBOT_ID > 0x09) {									
			visionData.enemyType = ENEMY_RED;	  
		}							
		else {
			visionData.enemyType = ENEMY_BLUE;
		}	
	}	
}

//�Ӿ������ʼ��
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
    //���ò���ʱ��������ֵ��Ϊ��������̨������
	visionData.pitchCmdBuff = getGimbalData()->pitchAngleFbd;
	visionData.yawCmdBuff = getGimbalData()->yawAngleFbd;
}

//LCM: ����Ԥ�еĽǶȼ��㺯��
void visionGimbalNormal() {
	visionData.yawCmdBuff = visionData.yawData.float_temp;
	visionData.pitchCmdBuff = visionData.pitchData.float_temp;
    
    //��Yaw��ƫת�Ǵ���45�㣬��Ϊ��Ч����̨����
    if(fabsf(visionData.yawCmdBuff - getGimbalData()->yawAngleFbd) >= 45) {
        visionDataReset();
        visionData.captureFlag = false;     //��Ϊδ��׽��Ŀ�� 
    }
    //��Pitch��ǶȾ���ֵ����45�㣬��Ϊ��Ч����̨����
    if(fabsf(visionData.pitchCmdBuff) >= 45) {
        visionDataReset();
        visionData.captureFlag = false;     //��Ϊδ��׽��Ŀ�� 
    }
}

//LCM: ��yaw��pitch�����ݽ���������õ�����Cmdֵ��Cmdֵ��gimbal.c�б���Ϊ�Ƕ����� 
void visionAngleCalculation() {
    
	visionGimbalNormal();

    visionData.yawCmd = visionData.yawCmdBuff;		//LCM: ��ֵ����Ϊ��̨�Ƕ�����
	visionData.pitchCmd = visionData.pitchCmdBuff;

	//�ֶ�Ԥ��
	//LCM: ����ģʽ��ʹ���Ҽ������Դ�΢��
	//LCM: �ƺ��Ҽ�ֻ��������� 
	if(visionData.prejudgFlag && (visionData.distinguishMode == TX2_DISTINGUISH_ARMOR)) {
		visionData.yawCmd = visionData.yawCmd + visionData.manualYawBias;
		visionData.pitchCmd = visionData.pitchCmd + visionData.manualPitchBias;	//LCM: ����������Ĳ����ֲ�����			
	}
}

//LCM: �����ⶫ�����Ƶ�Ӣ���Զ�������
//�Ȼ�����̨����Ƕȼ���
void visionMortar() {
	visionData.mortarYawCmdBuff = 30.0f;	
	visionData.mortarPitCmdBuff = 30.0f;	
	visionData.mortarYawCmd = visionData.manualYawBias + visionData.mortarYawCmdBuff; 
	visionData.mortarPitCmd = visionData.manualPitchBias + visionData.mortarPitCmdBuff;
}

//�Ӿ��������
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
			//LCM: ƥ��CNTR��ƥ��ʧ���������˴�ˢ��
			uint8_t dataMatchIndex = visionMatchAngle();	//LCM: ��ƥ��ʧ�ܣ� visionMatchAngle ����ֵΪ255
			if(dataMatchIndex != 255) {
                
			}
			else {
				goto loopIncreasing;
			}
			
			//LCM: ͻȻ��׽��Ŀ�� �����ò���
			if(!lastCaptureFlag) {
				visionDataReset();
			}
			
			//LCM: ����һ֡���ݰ��Ѵ���
			if(visionData.CNTR.u16_temp != visionData.lastCNTR) {
				visionData.cailSuccess = false;
			}
			//LCM: ����һ֡���ݰ���δ����
			else {
				visionData.cailSuccess = true;
			}
			visionData.lastCNTR = visionData.CNTR.u16_temp;			
			//�Ƕȼ���
			visionAngleCalculation();
			lastCaptureFlag = captureFlag;
		}
		loopIncreasing:
			digitalIncreasing(&visionData.loops);
	}
}

//�Ӿ����ݳ�ʼ��
void visionInit(void) {
	memset((void *)&visionData, 0, sizeof(visionData));
	visionData.lastCNTR = visionData.CNTR.u16_temp;
	visionData.initFlag = true;
	shootVisionInit();
	visionData.prejudgFlag = false;	//LCM: Ԥ�д����־λ
	supervisorData.taskEvent[VISION_TASK] = xTaskCreate(visionUpdateTask, \
                                                        "VISION", \
                                                        VISION_STACK_SIZE, \
                                                        NULL, \
														VISION_PRIORITY, \
                                                        &visionData.xHandleTask);
    usbVCP_Printf("visionInit Successfully \r\n");
}
