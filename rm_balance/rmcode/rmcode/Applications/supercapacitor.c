#include "supercapacitor.h"
#include "rc.h"
#include "keyboard.h"
#include "cansend.h"
#include "judge.h"
#include "auto_infantry.h"


/*--------------------------------------------- 
-----------------------------------------------
//		���棺�����������򣬹��������ش�		 //
//			���ض�����������ɿ��Ը����⣬		 //
//	���ಿ�ָ��ģ����빫����������ȡ�ø���Ȩ��  // 
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
			if(judgeData.extGameRobotState.remain_HP==0){ //�������  �õ��ݲ����ܷŵ�   ��Ȼ���̵���е㲻���϶�  ������ȥ����״̬
  	     		getinfantryAutoData()->capSchedule = 0;
  	 	     	get_capacitance()->state = 0;  //����״̬   
						getinfantryAutoData()->chassis_fast = 0;
        	}
	      	else if((PRESS_SHIFT && (FORWARD || BACK || LEFT || RIGHT|| getchassisData()->chaseFbd>=1 ||  getchassisData()->chaseSpeedRef>=1 || getinfantryAutoData()->km_rotate == 1)) && (!getcapacitorData()->percentage) || RC_ROTATE>200 ){
		   		get_capacitance()->state = 2;  //�ŵ�
		   		getinfantryAutoData()->capSchedule = 3;	 
					getinfantryAutoData()->chassis_fast = 1;
        	}
		    else if(FORWARD || BACK || LEFT || RIGHT|| getchassisData()->chaseFbd>=1 ||  getchassisData()->chaseSpeedRef>=1 || fabs(keyBoardCtrlData.yawGyroTarget)>0.2 || getinfantryAutoData()->rotateFlag || abs(RC_TRANSVERSE)>50 || abs(RC_LONGITUDINAL)>50 || abs(RC_RUDD)>50 || getinfantryAutoData()->km_rotate == 1){ 
				get_capacitance()->state = 0;  //����״̬
		        getinfantryAutoData()->capSchedule = 2;
				getinfantryAutoData()->chassis_fast = 0;
    	    }                 
        	else if(RC_ROTATE < -50 || PRESS_C){//������0
				get_capacitance()->state = 1;//���
		        getinfantryAutoData()->capSchedule = 2;
				getinfantryAutoData()->chassis_fast = 3;
	        }//�ֶ���� 
//			else if(get_judgeData()->extPowerHeatData.chassis_power < 55 && getchassisData()->chaseFbd < 1 && getchassisData()->chaseSpeedRef < 1.0f){
//					getinfantryAutoData()->capSchedule = 4;
//					getinfantryAutoData()->chassis_fast = 3;
//			}
	    break;
	    case 1:
			if(!PRESS_SHIFT)   
				digitalDecline(&getinfantryAutoData()->capSchedule);   //Ҫһֱ���²Ż�ŵ�
			break;
      	case 2:
			if(!FORWARD || !BACK || !LEFT || !RIGHT || PRESS_C)  
				getinfantryAutoData()->capSchedule = 0;   //Ҫһֱ���²Ż���
			break;
      	case 3: 
			getinfantryAutoData()->capSchedule = 0;   
			break;
	}
}
//������״̬  ���ڼ�ػ�(���Ż�)
void capacitor_readData(CanRxMsg *can_rx_msg){
	if((can_rx_msg->Data[0]==0XAC)&&(can_rx_msg->Data[1]==0XAD))
	{ 
		    capacitorData.percentage = can_rx_msg->Data[4];//���ݵ��� 
            capacitorData.capacitor_status = can_rx_msg->Data[3];//���ݳ�ŵ�״̬ 1����� 2���ŵ� 3������
		
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
	
	txMessage.Data[0] = 0xAC;/*******У��*********/
	txMessage.Data[1] = 0xAD;/*******У��*********/
	txMessage.Data[2] = capacitorData.state_ing;//�ŵ�
	txMessage.Data[3] = judgeData.extPowerHeatData.chassis_power;
	txMessage.Data[4] = judgeData.extPowerHeatData.chassis_volt>>8;;
	txMessage.Data[5] = get_judgeData()->extGameRobotState.chassis_power_limit;
	txMessage.Data[6] = judgeData.extPowerHeatData.chassis_volt;
	txMessage.Data[7] = 0;
	mbox= CAN_Transmit(CAN1, &txMessage);   
	
	//�ȴ����ͽ���
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
	
	txMessage.Data[0] = 0xAC;/*******У��*********/
	txMessage.Data[1] = 0xAD;/*******У��*********/
	
	txMessage.Data[2] = capacitorData.state_ing;//��ŵ�״̬
	txMessage.Data[3] = capacitorData.capacitor_1.sendData[1].u8_temp;
	txMessage.Data[4] = capacitorData.capacitor_2.sendData[0].u8_temp;
	txMessage.Data[5] = capacitorData.capacitor_2.sendData[1].u8_temp;
	txMessage.Data[6] = capacitorData.capacitor_3.sendData[0].u8_temp;
	txMessage.Data[7] = capacitorData.capacitor_3.sendData[1].u8_temp;
	mbox= CAN_Transmit(CAN2, &txMessage);   
	
	//�ȴ����ͽ���
    while(CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)
	{
		i++;	
		if(i>=0xFFF)
		break;
	}
}

/*******������������*******/
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




