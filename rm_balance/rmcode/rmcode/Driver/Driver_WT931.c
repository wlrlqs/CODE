#include "Driver_WT931.h"
#include <string.h>
BSP_USART_TypeDef MPU_WT931;
wt931Data_t wt931Data;

uint8_t setCaliSelf[5] = {0xff,0xaa,0x63,0x00,0x00};     //自动校准
uint8_t delCaliSelf[5] = {0xff,0xaa,0x63,0x01,0x00};

//wt931Data_t *getImuData(){
//    return &wt931Data;
//}
//void Driver_WT931_Init(USART_TypeDef *USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority){
////	static uint8_t caliFlag = DISABLE;
//	BSP_USART_TypeDef MPU_WT931;
//	MPU_WT931.USARTx = USARTx;
//	MPU_WT931.USART_RX = USART_RX;
//	MPU_WT931.USART_TX = USART_TX;
//	MPU_WT931.USART_InitStructure.USART_BaudRate = 921600;									/*波特率为500000*/
//	MPU_WT931.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*字长为8位数据格式*/
//	MPU_WT931.USART_InitStructure.USART_StopBits = USART_StopBits_1;				/*一个停止位*/
//	MPU_WT931.USART_InitStructure.USART_Parity = USART_Parity_No;				/*无校验位*/
//	MPU_WT931.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;							/*接收/发送模式*/	
//	MPU_WT931.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	/*无硬件数据流控制*/	
////	MPU_WT931.DMAFlag = ENABLE;
//	BSP_USART_Init(&MPU_WT931,PreemptionPriority,SubPriority);
//	BSP_USART_RX_DMA_Init(&MPU_WT931);							
////	
////	if(caliFlag == DISABLE){			//设置时使用
////		caliInitWT931(delCaliSelf);
////		caliFlag = ENABLE;
////	}
//}
//void caliInitWT931(uint8_t * sendData){
//	static uint8_t sendNum = 0;
//	while(sendData[sendNum] != '\0'){
//		while(USART_GetFlagStatus(UART8,USART_FLAG_TC )==RESET){
//			USART_SendData(UART8,sendData[sendNum]);
//			sendNum ++;
//		}
//	}
//}



///********************添加读取陀螺仪数据**********************/
//static void dataToCopy(uint8_t array[]){
//	uint8_t dataSum = 0;
//	if(array[1] == 0x51){
//		for(uint8_t i = 0; i<10 ; i++)
//			dataSum += array[i];
//		if(dataSum != array[10]){
//		}
//		else{
//			getImuData()->stcAcc.x = (float)((short)((((int16_t)array[3])<<8)|((int16_t)array[2])))/32768*16;   
//			getImuData()->stcAcc.y = (float)((short)((((int16_t)array[5])<<8)|((int16_t)array[4])))/32768*16;   
//			getImuData()->stcAcc.z = (float)((short)((((int16_t)array[7])<<8)|((int16_t)array[6])))/32768*16;
//			getImuData()->stcAcc.T = (float)((short)((((int16_t)array[9])<<8)|((int16_t)array[8])))/100;
//		}
//		dataSum = 0;
//	}
//	if(array[12] == 0x52){
//		for(uint8_t i = 11; i<21 ; i++)
//			dataSum += array[i];
//		if(dataSum != array[21]){
//		}
//		else{
//			getImuData()->stcGyro.x = (float)((short)((((int16_t)array[14])<<8)|((int16_t)array[13])))/32768*2000;   
//			getImuData()->stcGyro.y = (float)((short)((((int16_t)array[16])<<8)|((int16_t)array[15])))/32768*2000;   
//			getImuData()->stcGyro.z = (float)((short)((((int16_t)array[18])<<8)|((int16_t)array[17])))/32768*2000;
//			getImuData()->stcGyro.T = (float)((short)((((int16_t)array[20])<<8)|((int16_t)array[19])))/100;
//		}
//		dataSum = 0;
//	}
//	if(array[23] == 0x53){
//		for(uint8_t i = 22; i<32 ; i++)
//			dataSum += array[i];
//		if(dataSum != array[32]){
//		}
//		else{
//			getImuData()->stcAngle.x = (float)((short)((((int16_t)array[25])<<8)|((int16_t)array[24])))/32768*180;   
//			getImuData()->stcAngle.y = (float)((short)((((int16_t)array[27])<<8)|((int16_t)array[26])))/32768*180;   
//			getImuData()->stcAngle.z = (float)((short)((((int16_t)array[29])<<8)|((int16_t)array[28])))/32768*180;  	
//			getImuData()->stcAngle.T = (float)((short)((((int16_t)array[31])<<8)|((int16_t)array[30])))/100;
//		}
//		dataSum = 0;
//	}
//}
////读取IMU数据

////void Driver_WT931Read(uint8_t *arrayWT931Receive , uint16_t *RxCount){
////	static short dataSum = 0;
////	if (arrayWT931Receive[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
////	{
////		*RxCount=0;
////		return;
////	}
////	if (*RxCount<11) {return;}//数据不满11个，则返回
////	else{
////		for(uint8_t i = 0; i<10 ; i++)
////			dataSum += arrayWT931Receive[i];
////		if(dataSum == arrayWT931Receive[10]){
////			dataToCopy(arrayWT931Receive);
////		}
////		dataSum = 0;
////		*RxCount=0;
////	}
////}
//void Driver_WT931ReadDMA(uint8_t *arrayWT931Receive){
//	
//	if(arrayWT931Receive[0] != 0x55){
//	}	//若起始和末尾校验失败，则不进行下一步操作
//	else{
//		dataToCopy(arrayWT931Receive);
//	} 
//}
