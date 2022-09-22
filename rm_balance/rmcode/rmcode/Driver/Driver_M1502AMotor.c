#include "Driver_M1502AMotor.h"

uint8_t  _sendData[8];
M1502ACANSendStruct_t M1502ASend;

void M1502A_MotorUpdata(motorConfigStruct_t *motorConfig){
	if(motorConfig->motor_lib.motor_type == M1502A_MOTOR){
		CanTxMsg *txMessage;
		uint8_t mbox;
		uint16_t i = 0;
		txMessage = (CanTxMsg*)aqCalloc(8,sizeof(CanTxMsg));
		txMessage->StdId = motorConfig->send_identifier;
		txMessage->IDE = CAN_Id_Standard;
		txMessage->RTR = CAN_RTR_Data;
		txMessage->DLC = 0x08;
		
		if(motorConfig->send_identifier == 0x1FF){
			txMessage->Data[0] = (uint8_t)(M1502ASend.data[0] >> 8);
			txMessage->Data[1] = (uint8_t)M1502ASend.data[0];
			txMessage->Data[2] = (uint8_t)(M1502ASend.data[1] >> 8);
			txMessage->Data[3] = (uint8_t)M1502ASend.data[1];
			txMessage->Data[4] = (uint8_t)(M1502ASend.data[2] >> 8);
			txMessage->Data[5] = (uint8_t)M1502ASend.data[2];
			txMessage->Data[6] = (uint8_t)(M1502ASend.data[3] >> 8);
			txMessage->Data[7] = (uint8_t)M1502ASend.data[3];
		}else if(motorConfig->send_identifier == 0x2FF){
			txMessage->Data[0] = (uint8_t)(M1502ASend.data[4] >> 8);
			txMessage->Data[1] = (uint8_t)M1502ASend.data[4];
			txMessage->Data[2] = (uint8_t)(M1502ASend.data[5] >> 8);
			txMessage->Data[3] = (uint8_t)M1502ASend.data[5];
			txMessage->Data[4] = (uint8_t)(M1502ASend.data[6] >> 8);
			txMessage->Data[5] = (uint8_t)M1502ASend.data[6];
			txMessage->Data[6] = (uint8_t)(M1502ASend.data[7] >> 8);
			txMessage->Data[7] = (uint8_t)M1502ASend.data[7];
		}
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

void M1502A_MotorReviece(motorConfigStruct_t *motorConfig,CanRxMsg *CanRevData){
    motorConfig->motor_staus->motorSpeed    =   (int16_t)((CanRevData->Data[0] << 8) | CanRevData->Data[1]);
    motorConfig->motor_staus->currunt       =   (int16_t)((CanRevData->Data[2] << 8) | CanRevData->Data[3]);
    motorConfig->motor_staus->motorEncoder  =   (uint16_t)((CanRevData->Data[4] << 8) | CanRevData->Data[5]);
    motorConfig->motor_staus->error         =   (uint8_t)CanRevData->Data[6];
		motorConfig->motor_staus->mode          =   (uint8_t)CanRevData->Data[7];
}

void M1502A_MotorInit(uint8_t mode){
	CanTxMsg *txMessage;
	uint8_t mbox;
	uint16_t i = 0;
	txMessage = (CanTxMsg*)aqCalloc(8,sizeof(CanTxMsg));
	txMessage->StdId = 0x3FF;
	txMessage->IDE = CAN_Id_Standard;
	txMessage->RTR = CAN_RTR_Data;
	txMessage->DLC = 0x08;
	
	txMessage->Data[0] = mode;
	txMessage->Data[1] = mode;
	txMessage->Data[2] = mode;
	txMessage->Data[3] = mode;
	txMessage->Data[4] = mode;
	txMessage->Data[5] = mode;
	txMessage->Data[6] = mode;
	txMessage->Data[7] = mode;

	mbox = CAN_Transmit(CAN2, txMessage);
	while (CAN_TransmitStatus(CAN2,mbox) == 0x00) {
		i++;
		if (i >= 0xFFF)break;
	}
	aqFree(txMessage,8,sizeof(CanTxMsg));
}

void M1502A_MotorStop(void){
	CanTxMsg *txMessage;
	uint8_t mbox;
	uint16_t i = 0;
	txMessage = (CanTxMsg*)aqCalloc(8,sizeof(CanTxMsg));
	txMessage->StdId = 0x1FF;
	txMessage->IDE = CAN_Id_Standard;
	txMessage->RTR = CAN_RTR_Data;
	txMessage->DLC = 0x08;
	
	txMessage->Data[0] = 0;
	txMessage->Data[1] = 0;
	txMessage->Data[2] = 0;
	txMessage->Data[3] = 0;
	txMessage->Data[4] = 0;
	txMessage->Data[5] = 0;
	txMessage->Data[6] = 0;
	txMessage->Data[7] = 0;

	mbox = CAN_Transmit(CAN2, txMessage);
	while (CAN_TransmitStatus(CAN2,mbox) == 0x00) {
		i++;
		if (i >= 0xFFF)break;
	}
	aqFree(txMessage,8,sizeof(CanTxMsg));
}
