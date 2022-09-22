#include "joint.h"
#include "Driver_unitreeMotor.h"

jointDataStruct_t jointData;
jointReceiveStruct_t jointRevStatus;
uint8_t _send[34]={0};

static void jointDataSend(USART_TypeDef *USARTx,jointSendStruct_t *sendDatas);
static void jointDataUpdate(uint8_t *buffdecode,uint8_t usartX);
static void jointReceive(void);

//PY
jointSerialStruct_t jointSerial = {
	jointDataSend,
	jointDataUpdate,
	jointReceive,
};

void jointInit(void){
	jointSerial.sendFlag = 1;
	jointSerial.sendLoop = 100;
	jointSerial.receiveFlag = 1;
	jointSerial.revSchedule = 0;
	jointData.initFlag = 1;
	
	for(uint8_t index=0; index<4; index++){
	  if(index == 0 || index == 2){
		  jointData.send[index].motorID = 1;
		}else{
		  jointData.send[index].motorID = 2;
		}
		jointData.send[index].mode = 10;
	  jointData.send[index].modifyBit = 0xff;
	  jointData.send[index].readBit = 0;
	  jointData.send[index].reserved = 0;
	  jointData.send[index].modify = 0;
  	jointData.send[index].torque = 0;
  	jointData.send[index].speed = 0;
  	jointData.send[index].position = 0;
  	jointData.send[index].speed_Kp = 0;
  	jointData.send[index].position_Kp = 0;
  	jointData.send[index].cmdIndex = 0;
	  jointData.send[index].cmdByte = 0;
	  jointData.send[index].res = 0;
		
		jointData.motor[index].motorID = 0;
  	jointData.motor[index].mode = 0;
  	jointData.motor[index].temperature = 0;
  	jointData.motor[index].error = 0;
    jointData.motor[index].torque = 0;
  	jointData.motor[index].speed = 0;
  	jointData.motor[index].accSpeed = 0;
    jointData.motor[index].position = 0;
	  jointData.motor[index].xGyro = 0;
	  jointData.motor[index].yGyro = 0;
	  jointData.motor[index].zGyro = 0;
	  jointData.motor[index].xAcc = 0;
	  jointData.motor[index].yAcc = 0;
	  jointData.motor[index].zAcc = 0;
	}
}

uint32_t crc32_core(uint32_t* ptr, uint32_t len){
	uint32_t xbit = 0;
	uint32_t data = 0;
	uint32_t CRC32 = 0xFFFFFFFF;
	const uint32_t dwPolynomial = 0x04c11db7;
	for(uint32_t i = 0; i < len; i++){
		xbit = 1 << 31;
		data = ptr[i];
		for(uint32_t bits = 0; bits < 32; bits++){
			if(CRC32 & 0x80000000){
				CRC32 <<= 1;
				CRC32 ^= dwPolynomial;
			}else
				CRC32 <<= 1;
			if(data & xbit)
				CRC32 ^= dwPolynomial;
			xbit >>= 1;
		}
	}
	return CRC32;
}

//PY：发送关节电机数据
static void jointDataSend(USART_TypeDef *USARTx,jointSendStruct_t *sendDatas){
	uint8_t *array = (uint8_t *)USART_TO_ArrayTX(USARTx);
	uint8_t index_ptr = 0;
	
	array[index_ptr++] = 0xFE;
	array[index_ptr++] = 0xEE;
	array[index_ptr++] = sendDatas->motorID;
	array[index_ptr++] = 0;
	array[index_ptr++] = sendDatas->mode;
	array[index_ptr++] = sendDatas->modifyBit;
	array[index_ptr++] = sendDatas->readBit;
	array[index_ptr++] = sendDatas->reserved;
	
	formatTrans32Struct_t _temp32;
	_temp32.u32_temp = sendDatas->modify;
	for(uint8_t index = 0; index < 4; index++)
		array[index_ptr++] = _temp32.u8_temp[index];
	
	formatTrans16Struct_t _temp16;
	_temp16.u16_temp = sendDatas->torque;
	for(uint8_t index = 0; index < 2; index++)
		array[index_ptr++] = _temp16.u8_temp[index];
	
	_temp16.u16_temp = sendDatas->speed;
	for(uint8_t index = 0; index < 2; index++)
		array[index_ptr++] = _temp16.u8_temp[index];
		
	_temp32.u32_temp = sendDatas->position;
	for(uint8_t index = 0; index < 4; index++)
		array[index_ptr++] = _temp32.u8_temp[index];
		
	_temp16.u16_temp = sendDatas->position_Kp;
	for(uint8_t index = 0; index < 2; index++)
		array[index_ptr++] = _temp16.u8_temp[index];
		
	_temp16.u16_temp = sendDatas->speed_Kp;
	for(uint8_t index = 0; index < 2; index++)
		array[index_ptr++] = _temp16.u8_temp[index];
		
	array[index_ptr++] = sendDatas->cmdIndex;
	array[index_ptr++] = sendDatas->cmdByte;
	
	_temp32.u32_temp = sendDatas->res;
	for(uint8_t index=0;index < 4;index++)
		array[index_ptr++] = _temp32.u8_temp[index];
		
	crcStruct_t array32;
	for(uint8_t index = 0; index < 28; index++)
		array32.u8_temp[index] = array[index];
	_temp32.u32_temp = crc32_core(array32.u32_temp, 7);
	for(uint8_t index = 0; index < 4; index++)
		array[index_ptr++] = _temp32.u8_temp[index];
	BSP_USART_DMA_SendData(USARTx,array,34);
}


//PY：关节数据接收
static void jointReceive(void){
		uint8_t *bufferDecode = (uint8_t*)aqCalloc(78, sizeof(uint8_t));
		//拷贝到读取数组中
	
		crcStruct_t array32;
		formatTrans32Struct_t rx32;
		for(uint8_t count = 0; count < 78; count++){
			bufferDecode[count] = jointBroadcastLF.bufferLoop.buffer[count];
		}
		for(uint8_t count = 0; count < 72; count++)
			array32.u8_temp[count] = bufferDecode[count];
		for(uint8_t count = 0; count < 4; count++)
			rx32.u8_temp[count] = bufferDecode[74+count];
		if(crc32_core(array32.u32_temp, 18) == rx32.u32_temp){ 
			jointDataUpdate(bufferDecode,1);
		}
		
		for(uint8_t count = 0; count < 78; count++){
			bufferDecode[count] = jointBroadcastLB.bufferLoop.buffer[count];
		}
		for(uint8_t count = 0; count < 72; count++)
			array32.u8_temp[count] = bufferDecode[count];
		for(uint8_t count = 0; count < 4; count++)
			rx32.u8_temp[count] = bufferDecode[74+count];
		if(crc32_core(array32.u32_temp, 18) == rx32.u32_temp){ 
			jointDataUpdate(bufferDecode,1);
		}
		
		for(uint8_t count = 0; count < 78; count++){
			bufferDecode[count] = jointBroadcastRF.bufferLoop.buffer[count];
		}
		for(uint8_t count = 0; count < 72; count++)
			array32.u8_temp[count] = bufferDecode[count];
		for(uint8_t count = 0; count < 4; count++)
			rx32.u8_temp[count] = bufferDecode[74+count];
		if(crc32_core(array32.u32_temp, 18) == rx32.u32_temp){ 
			jointDataUpdate(bufferDecode,6);
		}
		
		for(uint8_t count = 0; count < 78; count++){
			bufferDecode[count] = jointBroadcastRB.bufferLoop.buffer[count];
		}
		for(uint8_t count = 0; count < 72; count++)
			array32.u8_temp[count] = bufferDecode[count];
		for(uint8_t count = 0; count < 4; count++)
			rx32.u8_temp[count] = bufferDecode[74+count];
		if(crc32_core(array32.u32_temp, 18) == rx32.u32_temp){ 
			jointDataUpdate(bufferDecode,6);
		}
		
		aqFree(bufferDecode,78,sizeof(uint8_t));
}

//PY：关节电机数据更新
static void jointDataUpdate(uint8_t *buffdecode,uint8_t usartX) {
	if(usartX == 1){
		if(buffdecode[2] == 0x01 || buffdecode[2] == 0x02){
			jointData.motor[buffdecode[2]-1].motorID = buffdecode[2];
			jointData.motor[buffdecode[2]-1].mode = buffdecode[4];
			jointData.motor[buffdecode[2]-1].temperature = buffdecode[6];
			jointData.motor[buffdecode[2]-1].error = buffdecode[7];
			jointData.motor[buffdecode[2]-1].torque = (uint16_t)buffdecode[13]<<8 | buffdecode[12];
			jointData.motor[buffdecode[2]-1].speed = (int16_t)(buffdecode[15])<<8 | buffdecode[14];
			jointData.motor[buffdecode[2]-1].accSpeed = (uint16_t)buffdecode[27]<<8 | buffdecode[26];
			jointData.motor[buffdecode[2]-1].position = (int32_t)buffdecode[33]<<24 | (int32_t)buffdecode[32]<<16 | (int32_t)buffdecode[31]<<8 | buffdecode[30];
			jointData.motor[buffdecode[2]-1].xGyro = ((uint16_t)buffdecode[39]<<8 | buffdecode[38]);
			jointData.motor[buffdecode[2]-1].yGyro = ((uint16_t)buffdecode[41]<<8 | buffdecode[40]);
			jointData.motor[buffdecode[2]-1].zGyro = ((uint16_t)buffdecode[43]<<8 | buffdecode[42]);
			jointData.motor[buffdecode[2]-1].xAcc  = ((uint16_t)buffdecode[45]<<8 | buffdecode[44]);
			jointData.motor[buffdecode[2]-1].yAcc  = ((uint16_t)buffdecode[47]<<8 | buffdecode[46]);
			jointData.motor[buffdecode[2]-1].zAcc  = ((uint16_t)buffdecode[49]<<8 | buffdecode[48]);
		}
	}else if(usartX == 6){
		if(buffdecode[2] == 0x01 || buffdecode[2] == 0x02){
			jointData.motor[buffdecode[2]+1].motorID = buffdecode[2];
			jointData.motor[buffdecode[2]+1].mode = buffdecode[4];
			jointData.motor[buffdecode[2]+1].temperature = buffdecode[6];
			jointData.motor[buffdecode[2]+1].error = buffdecode[7];
			jointData.motor[buffdecode[2]+1].torque = (uint16_t)buffdecode[13]<<8 | buffdecode[12];
			jointData.motor[buffdecode[2]+1].speed = (int16_t)(buffdecode[15])<<8 | buffdecode[14];
			jointData.motor[buffdecode[2]+1].accSpeed = (uint16_t)buffdecode[27]<<8 | buffdecode[26];
			jointData.motor[buffdecode[2]+1].position = (int32_t)buffdecode[33]<<24 | (int32_t)buffdecode[32]<<16 | (int32_t)buffdecode[31]<<8 | buffdecode[30];
			jointData.motor[buffdecode[2]+1].xGyro = ((uint16_t)buffdecode[39]<<8 | buffdecode[38]);
			jointData.motor[buffdecode[2]+1].yGyro = ((uint16_t)buffdecode[41]<<8 | buffdecode[40]);
			jointData.motor[buffdecode[2]+1].zGyro = ((uint16_t)buffdecode[43]<<8 | buffdecode[42]);
			jointData.motor[buffdecode[2]+1].xAcc  = ((uint16_t)buffdecode[45]<<8 | buffdecode[44]);
			jointData.motor[buffdecode[2]+1].yAcc  = ((uint16_t)buffdecode[47]<<8 | buffdecode[46]);
			jointData.motor[buffdecode[2]+1].zAcc  = ((uint16_t)buffdecode[49]<<8 | buffdecode[48]);
		}
	}
}


