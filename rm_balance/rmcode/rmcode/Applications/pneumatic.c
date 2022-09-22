#include "pneumatic.h"
#include "shoot.h"
#include "rc.h"

pneumaticData_t pneumaticData;

//暂无
void pneumaticInit(void){				
	
}

//读传感器状态
void pneumatic_readData(CanRxMsg *can_rx_msg){
	if((can_rx_msg->Data[0]==0XAC)&&(can_rx_msg->Data[1]==0XAD))
	{ 
		switch(can_rx_msg->Data[2]){
			case 0x00:	
				pneumaticData.pneumatic_1.readData[0].u8_temp = can_rx_msg->Data[3];
				pneumaticData.pneumatic_1.readData[1].u8_temp = can_rx_msg->Data[4];
				break;
			case 0x01:	
				pneumaticData.pneumatic_2.readData[0].u8_temp = can_rx_msg->Data[3];
				pneumaticData.pneumatic_2.readData[1].u8_temp = can_rx_msg->Data[4];
				break;
			case 0x02:	
				pneumaticData.pneumatic_3.readData[0].u8_temp = can_rx_msg->Data[3];
				pneumaticData.pneumatic_3.readData[1].u8_temp = can_rx_msg->Data[4];
				break;
		}
	}
}


/* CAN1气动发送 */
void pneumatic_can1_sentData(uint32_t ID_CAN){
	uint8_t mbox;                                   
  volatile uint16_t i=0;
	
	CanTxMsg txMessage;
	txMessage.StdId = ID_CAN;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
	
	txMessage.Data[0] = 0xAC;/*******校验*********/
	txMessage.Data[1] = 0xAD;/*******校验*********/
	
	txMessage.Data[2] = pneumaticData.pneumatic_1.sendData[0].u8_temp;
	txMessage.Data[3] = RC_MODE;
	txMessage.Data[4] = getshootData()->speed_limit;
	txMessage.Data[5] = pneumaticData.pneumatic_2.sendData[1].u8_temp;
	txMessage.Data[6] = pneumaticData.pneumatic_3.sendData[0].u8_temp;
	txMessage.Data[7] = pneumaticData.pneumatic_3.sendData[1].u8_temp;
	
	mbox= CAN_Transmit(CAN1, &txMessage);   
	
	//等待发送结束
    while(CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)
	{
		i++;	
		if(i>=0xFFF)
		break;
	}
}

void p_tank_can1_sentData(uint32_t ID_CAN){
	uint8_t mbox;                                   
  volatile uint16_t i=0;
	
	CanTxMsg txMessage;
	txMessage.StdId = ID_CAN;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
	
	txMessage.Data[0] = 0xAC;/*******校验*********/
	txMessage.Data[1] = 0xAD;/*******校验*********/
	
	txMessage.Data[2] = pneumaticData.pneumatic_1.sendData[0].u8_temp;
	txMessage.Data[3] = pneumaticData.pneumatic_1.sendData[1].u8_temp;
	txMessage.Data[4] = pneumaticData.pneumatic_2.sendData[0].u8_temp;
	txMessage.Data[5] = pneumaticData.pneumatic_2.sendData[1].u8_temp;
	txMessage.Data[6] = pneumaticData.pneumatic_3.sendData[0].u8_temp;
	txMessage.Data[7] = pneumaticData.pneumatic_3.sendData[1].u8_temp;
	
	mbox= CAN_Transmit(CAN1, &txMessage);   
	
	//等待发送结束
    while(CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)
	{
		i++;	
		if(i>=0xFFF)
		break;
	}
}

/* CAN2气动发送 */
void pneumatic_can2_sentData(uint32_t ID_CAN){
	uint8_t mbox;                                   
    volatile uint16_t i=0;                        
	
	CanTxMsg txMessage;
	txMessage.StdId = ID_CAN;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
	
	txMessage.Data[0] = 0xAC;/*******校验*********/
	txMessage.Data[1] = 0xAD;/*******校验*********/
	
	txMessage.Data[2] = pneumaticData.pneumatic_1.sendData[0].u8_temp;
	txMessage.Data[3] = pneumaticData.pneumatic_1.sendData[1].u8_temp;
	txMessage.Data[4] = pneumaticData.pneumatic_2.sendData[0].u8_temp;
	txMessage.Data[5] = pneumaticData.pneumatic_2.sendData[1].u8_temp;
	txMessage.Data[6] = pneumaticData.pneumatic_3.sendData[0].u8_temp;
	txMessage.Data[7] = pneumaticData.pneumatic_3.sendData[1].u8_temp;
	mbox= CAN_Transmit(CAN2, &txMessage);   
	
	//等待发送结束
    while(CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)
	{
		i++;	
		if(i>=0xFFF)
		break;
	}
}

