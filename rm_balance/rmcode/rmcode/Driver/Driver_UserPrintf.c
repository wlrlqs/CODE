#include "Driver_UserPrintf.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"

dataTrsfBroadcastStruct_t dataTrsfBroadcast;

static void dataTransferUpdate(uint8_t *buffdecode,uint8_t len);

//放在串口中断中的接收函数
void dataTrsfUsartIspFunction(uint8_t *array, uint16_t len)  {
		for(uint8_t i = 0 ; i < len; i++){
			dataTrsfBroadcast.bufferLoop.buffer[i] = array[i];
		}
		dataTrsfBroadcast.length = len;
}

//PY：数传数据接收
void dataTrsfReceive(void){
		uint8_t *bufferDecode = (uint8_t*)aqCalloc(dataTrsfBroadcast.length, sizeof(uint8_t));
		//拷贝到读取数组中
		for(uint8_t count = 0; count < dataTrsfBroadcast.length; count++){
			bufferDecode[count] = dataTrsfBroadcast.bufferLoop.buffer[count];
		}
		dataTransferUpdate(bufferDecode,dataTrsfBroadcast.length);
		aqFree(bufferDecode,dataTrsfBroadcast.length,sizeof(uint8_t));
}

//PY：数传数据更新
static void dataTransferUpdate(uint8_t *buffdecode,uint8_t len){
	
}

/*
***************************************************
函数名：userPrintf
功能：重定义printf发送数据
入口参数：	format：格式
					...：不定参数
返回值：len：发送的长度
应用范围：外部调用
备注：PY
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
