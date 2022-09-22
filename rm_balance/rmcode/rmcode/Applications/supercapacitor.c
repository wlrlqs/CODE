#include "supercapacitor.h"
#include "rc.h"
#include "keyboard.h"
#include "cansend.h"
#include "judge.h"
#include "auto_infantry.h"


/*--------------------------------------------- 
-----------------------------------------------
//		警告：公共代码区域，故障责任重大		 //
//			除特定区域可以自由可以更改外，		 //
//	其余部分更改，请与公共区域负责人取得更改权限  // 
-----------------------------------------------
---------------------------------------------*/


capacitorData_t capacitorData;
capacitance_Struct_t capacitance;
AutoTaskStruct_t AutoData;
capacitorData_t *getcapacitorData(void){
  return &capacitorData;
}
capacitance_Struct_t *get_capacitance(void){
	return &capacitance;
}

void super_capacitorTask(void)
{
	switch(getinfantryAutoData()->capSchedule){
		case 0:
			if(judgeData.extGameRobotState.remain_HP==0){ //如果死掉  让电容不不能放电   不然底盘电机有点不能拖动  让它进去保护状态
  	     		getinfantryAutoData()->capSchedule = 0;
  	 	     	get_capacitance()->state = 0;  //保护状态   
						getinfantryAutoData()->chassis_fast = 0;
        	}
	      	else if((PRESS_SHIFT && (FORWARD || BACK || LEFT || RIGHT|| getchassisData()->chaseFbd>=1 ||  getchassisData()->chaseSpeedRef>=1 || getinfantryAutoData()->km_rotate == 1)) && (!getcapacitorData()->percentage) || RC_ROTATE>200 ){
		   		get_capacitance()->state = 2;  //放电
		   		getinfantryAutoData()->capSchedule = 3;	 
					getinfantryAutoData()->chassis_fast = 1;
        	}
		    else if(FORWARD || BACK || LEFT || RIGHT|| getchassisData()->chaseFbd>=1 ||  getchassisData()->chaseSpeedRef>=1 || fabs(keyBoardCtrlData.yawGyroTarget)>0.2 || getinfantryAutoData()->rotateFlag || abs(RC_TRANSVERSE)>50 || abs(RC_LONGITUDINAL)>50 || abs(RC_RUDD)>50 || getinfantryAutoData()->km_rotate == 1){ 
				get_capacitance()->state = 0;  //保护状态
		        getinfantryAutoData()->capSchedule = 2;
				getinfantryAutoData()->chassis_fast = 0;
    	    }                 
        	else if(RC_ROTATE < -50 || PRESS_C){//调试用0
				get_capacitance()->state = 1;//充电
		        getinfantryAutoData()->capSchedule = 2;
				getinfantryAutoData()->chassis_fast = 3;
	        }//手动充电 
//			else if(get_judgeData()->extPowerHeatData.chassis_power < 55 && getchassisData()->chaseFbd < 1 && getchassisData()->chaseSpeedRef < 1.0f){
//					getinfantryAutoData()->capSchedule = 4;
//					getinfantryAutoData()->chassis_fast = 3;
//			}
	    break;
	    case 1:
			if(!PRESS_SHIFT)   
				digitalDecline(&getinfantryAutoData()->capSchedule);   //要一直按下才会放电
			break;
      	case 2:
			if(!FORWARD || !BACK || !LEFT || !RIGHT || PRESS_C)  
				getinfantryAutoData()->capSchedule = 0;   //要一直按下才会充电
			break;
      	case 3: 
			getinfantryAutoData()->capSchedule = 0;   
			break;
	}
}
//读电容状态  用于监控机(待优化)
void capacitor_readData(CanRxMsg *can_rx_msg){
	if((can_rx_msg->Data[0]==0XAC)&&(can_rx_msg->Data[1]==0XAD))
	{ 
		    capacitorData.percentage = can_rx_msg->Data[4];//电容电量 
            capacitorData.capacitor_status = can_rx_msg->Data[3];//电容充放电状态 1：充电 2：放电 3：保护
		
				digitalIncreasing(&capacitorData.CapacitorErrorCount);
	}
}


void capacitor_can1_sentData(uint32_t ID_CAN){
	uint8_t mbox;                                   
    volatile uint16_t i=0;                        
	
	CanTxMsg txMessage;
	txMessage.StdId = ID_CAN;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
	
	txMessage.Data[0] = 0xAC;/*******校验*********/
	txMessage.Data[1] = 0xAD;/*******校验*********/
	txMessage.Data[2] = capacitorData.state_ing;//放电
	txMessage.Data[3] = judgeData.extPowerHeatData.chassis_power;
	txMessage.Data[4] = judgeData.extPowerHeatData.chassis_volt>>8;;
	txMessage.Data[5] = get_judgeData()->extGameRobotState.chassis_power_limit;
	txMessage.Data[6] = judgeData.extPowerHeatData.chassis_volt;
	txMessage.Data[7] = 0;
	mbox= CAN_Transmit(CAN1, &txMessage);   
	
	//等待发送结束
    while(CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)
	{
		i++;	
		if(i>=0xFFF)
		break;
	}
}

void capacitor_can2_sentData(uint32_t ID_CAN){
	uint8_t mbox;                                   
    volatile uint16_t i=0;                        
	
	CanTxMsg txMessage;
	txMessage.StdId = ID_CAN;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
	
	txMessage.Data[0] = 0xAC;/*******校验*********/
	txMessage.Data[1] = 0xAD;/*******校验*********/
	
	txMessage.Data[2] = capacitorData.state_ing;//充放电状态
	txMessage.Data[3] = capacitorData.capacitor_1.sendData[1].u8_temp;
	txMessage.Data[4] = capacitorData.capacitor_2.sendData[0].u8_temp;
	txMessage.Data[5] = capacitorData.capacitor_2.sendData[1].u8_temp;
	txMessage.Data[6] = capacitorData.capacitor_3.sendData[0].u8_temp;
	txMessage.Data[7] = capacitorData.capacitor_3.sendData[1].u8_temp;
	mbox= CAN_Transmit(CAN2, &txMessage);   
	
	//等待发送结束
    while(CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)
	{
		i++;	
		if(i>=0xFFF)
		break;
	}
}

/*******超级电容任务*******/
void capacitance_cansend(uint8_t __state){
    
    if(__state==0)
    capacitorData.state_ing=NOTHING;
    if(__state==1)
    capacitorData.state_ing=DISCHARGER;
    if(__state==2)
    capacitorData.state_ing=CHARGER;
	if(ROBOT == INFANTRY_ID||ROBOT == P_TANK_ID||ROBOT == F_TANK_ID)
    capacitor_can1_sentData(0x405);	
//	if()
//    capacitor_can2_sentData(0x405);	
}




