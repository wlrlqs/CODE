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
//板类型加载网络
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
//用户自定义配置输出指针
/********************************************************************
函数名：outfitPtrInit
功能：外設电机指针初始化
入口参数：	motorConfigStruct_t *commando:执行电机列表
			motorConfigStruct_t *robot:对应兵种电机列表
			outfitNum:外設電機數量
返回值：无
应用范围：初始化
备注：
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
                equipMotorCurrent(&robot[AUX_L_ELE],&auxiliaryDeformingData.elevatorSpeed[0].dataOut); //左升降				 
                equipMotorCurrent(&robot[AUX_R_ELE],&auxiliaryDeformingData.elevatorSpeed[1].dataOut);//右升降 
//                equipMotorCurrent(&robot[AUX_X_SWY],&auxiliaryDeformingData.xSlidewaySpeed.dataOut); //X轴				 
//                equipMotorCurrent(&robot[AUX_Y_SWY],&auxiliaryDeformingData.ySlidewaySpeed.dataOut);//Y轴
                equipMotorCurrent(&robot[AUX_CLAW],&auxiliaryDeformingData.frontClawSpeed.dataOut);//前爪 
				equipMotorCurrent(&robot[AUX_L_REV],&auxiliaryDeformingData.reversingSpeed[0].dataOut); //左换向				 
                equipMotorCurrent(&robot[AUX_R_REV],&auxiliaryDeformingData.reversingSpeed[1].dataOut);//右换向
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
函数名：initPre
功能：电机输出指针初始化
入口参数：	motorConfigStruct_t *commando:执行电机列表
			motorConfigStruct_t *robot:对应兵种电机列表
返回值：无
应用范围：motorSeverPtrInit函数内部调用
备注：
*********************************************************************/
void initPre(motorConfigStruct_t *commando,motorConfigStruct_t *robot){
	volatile uint8_t countList;
	//板类型加载网络
	load_board_net(BOARD_TYPE);
	//填充输出电流指针
	//Yaw轴电机
	equipMotorCurrent(&robot[YAWMOTOR],&getGimbalData()->yawSpeedOut);
	//Pitch轴电机
	equipMotorCurrent(&robot[PITCHMOTOR],&getGimbalData()->pitchSpeedOut);
	//Roll轴电机
	equipMotorCurrent(&robot[ROLLMOTOR],&getGimbalData()->rollSpeedOut);
	//拨弹盘电机
	equipMotorCurrent(&robot[POKEMOTOR],&getshootData()->pokeSpeedOut);
	//弹舱盖电机
	equipMotorCurrent(&robot[SUPPLYMOTOR],&getshootData()->supplySpeedOut);
	//摩擦轮(左)电机
	equipMotorCurrent(&robot[FRICLMOTOR],&getshootData()->fricWheelSpeedOut[1]);
	//摩擦轮(右)电机
	equipMotorCurrent(&robot[FRICRMOTOR],&getshootData()->fricWheelSpeedOut[0]);
	//左滑块
	equipMotorCurrent(&robot[11],&getsliding_blocksData()->rateOut[0]);
	//右滑块
	equipMotorCurrent(&robot[12],&getsliding_blocksData()->rateOut[1]);
	//底盘(左)电机
	equipMotorCurrent(&robot[CHASSISMOTOR_L],&getchassisData()->powerCurrent[1]);
	//底盘(右)电机
	equipMotorCurrent(&robot[CHASSISMOTOR_R],&getchassisData()->powerCurrent[0]);
	//外设电机
	outfitPtrInit(commando,robot);
	//开始配置
	for(countList = YAWMOTOR; countList < USE_LIST; countList++){
		commando[countList] = robot[countList];
		if(commando[countList].motor_lib.motor_type == NO_MOTOR){
           continue;
        }
		else{
			//编码器码盘值
			encoderDective(&commando[countList]);
			//控制频率
			freqDective(&commando[countList]);
			//输出指针
			motorCurrentDective(&commando[countList]);
		}
	}
}

void canSendUpdate(void){
	motorSeverClass.Send();
	if(ROBOT == INFANTRY_ID || ROBOT == P_TANK_ID || ROBOT == F_TANK_ID)
      //超级电容
       capacitance_cansend(get_capacitance()->state);
	digitalIncreasing(&canSendData.loops);
}

//CAN发送初始化
void canSendInit(void){								
	driver_can1_init(CAN1,BSP_GPIOD0,BSP_GPIOD1,4,0);
	driver_can2_init(CAN2,BSP_GPIOB5,BSP_GPIOB6,3,0);
}
