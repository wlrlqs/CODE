#include "bufferLoop_mod.h"
#include "Driver_Judge.h"

//串口中斷接收函數
//array接收到的數據
//arrayLength接收到的數據長度
//bufferLoop環形數組
void bufferLoopContactISP(uint8_t *array,uint16_t arrayLength,buffefLoopStruct_t *bufferLoop){
	for(uint16_t count = 0; count < arrayLength; count++)
		bufferLoop->buffer[(uint8_t)(bufferLoop->tail + count)] = array[count];
	bufferLoop->tail += arrayLength;
}

//環形數組處理
//bufferBegin幀頭
//headerLength信息幀長度
//lengthPosition存放包長度的元素下標
//bufferLoop接收數組
//decodeTask解包函數
void bufferLoopMod(uint8_t bufferBegin, uint8_t headerLength, uint8_t lengthPosition, buffefLoopStruct_t *bufferLoop, decodeFunction decodeTask) {
	//LCM: 尾下标与头下标之差为目前环形数组中存储的未被读取的数据数量 
	//LCM: 当未被读取数据数量大于 数据包头长度
	while((uint8_t)(bufferLoop->tail - bufferLoop->header) > headerLength) {
		//LCM: 若未读取到头帧，则往后捋
		while(((uint8_t)(bufferLoop->tail - bufferLoop->header) > headerLength) && (bufferLoop->buffer[bufferLoop->header]) != bufferBegin) {
			digitalIncreasing(&bufferLoop->header);
		}
		//LCM: 捋完之后未被读取数据依然大于 数据包头长度	
		if((uint8_t)(bufferLoop->tail - bufferLoop->header) > headerLength) {
			//先驗證CRC8是否通過	//LCM: CRC8针对数据包头进行校验 
			uint8_t *crc8Check = (uint8_t*)aqCalloc(headerLength, sizeof(uint8_t));
			for(uint8_t count = 0; count < headerLength; count++)
				crc8Check[count] = bufferLoop->buffer[(uint8_t)(bufferLoop->header + count)];
			
			//CRC8校驗,不通過則直接跳過
			if(Verify_CRC8_Check_Sum(crc8Check,headerLength)) {
				//LCM: 通过读取携带数据包长度的成员 获取当前数据包长度
				uint8_t bufferLength = bufferLoop->buffer[(uint8_t)(bufferLoop->header + lengthPosition)];
				//剩餘長度必須足夠取完當前包,不滿足則直接結束當前循環
				if((uint8_t)(bufferLoop->tail - bufferLoop->header) < bufferLength) { 
					aqFree(crc8Check, headerLength, sizeof(uint8_t));
					break;
				}
				//申請臨時數組内存
				uint8_t *bufferDecode = (uint8_t*)aqCalloc(bufferLength, sizeof(uint8_t));
				//拷貝到讀取數組中
				for(uint8_t count = 0; count < bufferLength; count++)
					bufferDecode[count] = bufferLoop->buffer[(uint8_t)(bufferLoop->header + count)];
				//16位CRC校驗	//LCM: CRC16针对整个数据包进行检验
				if(Verify_CRC16_Check_Sum(bufferDecode, bufferLength)) { 
					//校驗通過後加上長度
					bufferLoop->header += bufferLength;	//LCM: 头指针往后捋，继续读取剩余数据
					decodeTask(bufferDecode);
				}
				else 
					digitalIncreasing(&bufferLoop->header);
				
				aqFree(bufferDecode,bufferLength,sizeof(uint8_t));
			}
			else 
				digitalIncreasing(&bufferLoop->header);
			
			aqFree(crc8Check,headerLength,sizeof(uint8_t));
		}
	}
}



