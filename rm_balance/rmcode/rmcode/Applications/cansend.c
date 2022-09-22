#include "config.h"
#include "cansend.h"
#include "control.h"
#include "deforming.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "supervisor.h"
#include "type_robot.h"
#include "DRIVER_VL53L0X.h"
#include "Driver_DMMotor.h"
#include "pneumatic.h"
#include "judge.h"
#include "util.h"
#include "canfd.h"
canSendStruct_t canSendData;
Net_Code_e board_net;
canSendStruct_t* getcanSendData(){
    return &canSendData;
}
//�����ͼ�������
void load_board_net(uint8_t board_type){
	switch(board_type){
		case BOARD_CONTROL:
			board_net = CONTROL_NET;
			break;
		case BOARD_CHASSIS:
			board_net = CHASSIS_NET;
			break;
		default:
			break;
	}
}
//�û��Զ����������ָ��
/********************************************************************
��������outfitPtrInit
���ܣ����O���ָ���ʼ��
��ڲ�����	motorConfigStruct_t *commando:ִ�е���б�
			motorConfigStruct_t *robot:��Ӧ���ֵ���б�
			outfitNum:���O늙C����
����ֵ����
Ӧ�÷�Χ����ʼ��
��ע��
*********************************************************************/
void outfitPtrInit(motorConfigStruct_t *commando,motorConfigStruct_t *robot){
		switch(robotConfigData.typeOfRobot){
			case INFANTRY_ID:{
//				equipMotorCurrent(&robot[ARM_WHEEL],&getchassisData()->armWheelOut);
//				equipMotorCurrent(&robot[SYNCHRONOUS_WHEEL],&getchassisData()->syncWheelOut);
			}break;
			case P_TANK_ID:{
				equipMotorCurrent(&robot[SLAVE_POKEMOTOR],&getshootData()->slave_pokeSpeedOut);
				equipMotorCurrent(&robot[SLAVE_SUPPLY],&getshootData()->slave_supplySpeedOut);
			}break;
			 case AUXILIARY_ID:{
                equipMotorCurrent(&robot[AUX_L_ELE],&auxiliaryDeformingData.elevatorSpeed[0].dataOut); //������				 
                equipMotorCurrent(&robot[AUX_R_ELE],&auxiliaryDeformingData.elevatorSpeed[1].dataOut);//������ 
//                equipMotorCurrent(&robot[AUX_X_SWY],&auxiliaryDeformingData.xSlidewaySpeed.dataOut); //X��				 
//                equipMotorCurrent(&robot[AUX_Y_SWY],&auxiliaryDeformingData.ySlidewaySpeed.dataOut);//Y��
                equipMotorCurrent(&robot[AUX_CLAW],&auxiliaryDeformingData.frontClawSpeed.dataOut);//ǰצ 
				equipMotorCurrent(&robot[AUX_L_REV],&auxiliaryDeformingData.reversingSpeed[0].dataOut); //����				 
                equipMotorCurrent(&robot[AUX_R_REV],&auxiliaryDeformingData.reversingSpeed[1].dataOut);//�һ���
			}break;
			case SENTRY_ID:{
				equipMotorCurrent(&robot[STRY_MOVE_DIRECT],&sentryDeformingData.dataOut);
			}break;
			case UAV_ID:{
			}break;
			case SMALLGIMBAL_ID:{
			}break;
			case F_TANK_ID:{
				equipMotorCurrent(&robot[ARM_WHEEL],&getchassisData()->armWheelOut);
				equipMotorCurrent(&robot[SYNCHRONOUS_WHEEL],&getchassisData()->syncWheelOut);
			}break;
			default : 
			break;
		}
}
/********************************************************************
��������initPre
���ܣ�������ָ���ʼ��
��ڲ�����	motorConfigStruct_t *commando:ִ�е���б�
			motorConfigStruct_t *robot:��Ӧ���ֵ���б�
����ֵ����
Ӧ�÷�Χ��motorSeverPtrInit�����ڲ�����
��ע��
*********************************************************************/
void initPre(motorConfigStruct_t *commando,motorConfigStruct_t *robot){
	volatile uint8_t countList;
	//�����ͼ�������
	load_board_net(BOARD_TYPE);
	//����������ָ��
	//Yaw����
	equipMotorCurrent(&robot[YAWMOTOR],&getGimbalData()->yawSpeedOut);
	//Pitch����
	equipMotorCurrent(&robot[PITCHMOTOR],&getGimbalData()->pitchSpeedOut);
	//Roll����
	equipMotorCurrent(&robot[ROLLMOTOR],&getGimbalData()->rollSpeedOut);
	//�����̵��
	equipMotorCurrent(&robot[POKEMOTOR],&getshootData()->pokeSpeedOut);
	//���ոǵ��
	equipMotorCurrent(&robot[SUPPLYMOTOR],&getshootData()->supplySpeedOut);
	//Ħ����(��)���
	equipMotorCurrent(&robot[FRICLMOTOR],&getshootData()->fricWheelSpeedOut[1]);
	//Ħ����(��)���
	equipMotorCurrent(&robot[FRICRMOTOR],&getshootData()->fricWheelSpeedOut[0]);
	//�󻬿�
	equipMotorCurrent(&robot[11],&getsliding_blocksData()->rateOut[0]);
	//�һ���
	equipMotorCurrent(&robot[12],&getsliding_blocksData()->rateOut[1]);
	//����(��)���
	equipMotorCurrent(&robot[CHASSISMOTOR_L],&getchassisData()->powerCurrent[1]);
	//����(��)���
	equipMotorCurrent(&robot[CHASSISMOTOR_R],&getchassisData()->powerCurrent[0]);
	//������
	outfitPtrInit(commando,robot);
	//��ʼ����
	for(countList = YAWMOTOR; countList < USE_LIST; countList++){
		commando[countList] = robot[countList];
		if(commando[countList].motor_lib.motor_type == NO_MOTOR){
           continue;
        }
		else{
			//����������ֵ
			encoderDective(&commando[countList]);
			//����Ƶ��
			freqDective(&commando[countList]);
			//���ָ��
			motorCurrentDective(&commando[countList]);
		}
	}
}

void canSendUpdate(void){
	motorSeverClass.Send();
	if(ROBOT == INFANTRY_ID || ROBOT == P_TANK_ID || ROBOT == F_TANK_ID)
      //��������
       capacitance_cansend(get_capacitance()->state);
	digitalIncreasing(&canSendData.loops);
}

//CAN���ͳ�ʼ��
void canSendInit(void){								
	driver_can1_init(CAN1,BSP_GPIOD0,BSP_GPIOD1,4,0);
	driver_can2_init(CAN2,BSP_GPIOB5,BSP_GPIOB6,3,0);
}
