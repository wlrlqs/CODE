#include "Driver_UserPrintf.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"

dataTrsfBroadcastStruct_t dataTrsfBroadcast;

static void dataTransferUpdate(uint8_t *buffdecode,uint8_t len);

//���ڴ����ж��еĽ��պ���
void dataTrsfUsartIspFunction(uint8_t *array, uint16_t len)  {
		for(uint8_t i = 0 ; i < len; i++){
			dataTrsfBroadcast.bufferLoop.buffer[i] = array[i];
		}
		dataTrsfBroadcast.length = len;
}

//PY���������ݽ���
void dataTrsfReceive(void){
		uint8_t *bufferDecode = (uint8_t*)aqCalloc(dataTrsfBroadcast.length, sizeof(uint8_t));
		//��������ȡ������
		for(uint8_t count = 0; count < dataTrsfBroadcast.length; count++){
			bufferDecode[count] = dataTrsfBroadcast.bufferLoop.buffer[count];
		}
		dataTransferUpdate(bufferDecode,dataTrsfBroadcast.length);
		aqFree(bufferDecode,dataTrsfBroadcast.length,sizeof(uint8_t));
}

//PY���������ݸ���
static void dataTransferUpdate(uint8_t *buffdecode,uint8_t len){
	
}

/*
***************************************************
��������userPrintf
���ܣ��ض���printf��������
��ڲ�����	format����ʽ
					...����������
����ֵ��len�����͵ĳ���
Ӧ�÷�Χ���ⲿ����
��ע��PY
***************************************************
*/

void userPrintf(const char* format, ...){
	
#define MAX_USER_PRINTF_LEN 256
	
	char Array_USART[MAX_USER_PRINTF_LEN];
	uint8_t *array = (uint8_t *)USART_TO_ArrayTX(USER_PRINTF_USARTX);
	uint32_t len = 0;
	va_list arg_ptr;
	va_start(arg_ptr, format);
	len = vsnprintf(Array_USART, MAX_USER_PRINTF_LEN, format, arg_ptr);
	for(uint8_t index=0; index<len; index++){
		array[index] = Array_USART[index];
	}
	BSP_USART_DMA_SendData(USER_PRINTF_USARTX,array,len);
}
