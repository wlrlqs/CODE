#include "differential_calculation.h"
#include "math.h"
#include "NavAndData.h"
#include "SEGGER_RTT.h"

f32_t frequency[70] = {0};
uint8_t rttUpBuffer[2048];
dataAcquisitionBufStruct_t buf;
f32_t transferData[8] = 							//yaw��̨�ٶȻ���������
{
1.0000,-0.7315,-0.4638,0.1953,
3.5904,-2.9756,-3.5828,2.9832
};

/*
@ param differential:	differential data
@ param currentData:	new data
@ return:							update data
*/
f32_t differentialCal(differentialDataStruct_t *differential,f32_t currentData){
	uint8_t loopNum = differential -> differentialLength;
	differential -> inputData[loopNum - 1] = currentData;
	
	/* H(z)������� */
	differential -> outputData[loopNum - 1] = 0.0f;
	for(uint8_t i = 0;i < loopNum;i++){
		differential -> outputData[loopNum - 1] +=	\
		differential -> coefficient -> xCoefficient[i] * differential -> inputData[loopNum - 1 - i];
		
		if(i != loopNum - 1)
			differential -> outputData[loopNum - 1] -=	\
			differential -> coefficient -> yCoefficient[i + 1] * differential -> outputData[loopNum - 1 - i - 1];
	}

	/* x(n) ���б��� */
	for(uint8_t i = 0;i < loopNum - 1;i++){
			differential->inputData[i] = differential -> inputData[i + 1];
	}
	
	/* y(n) ���б��� */
	for(uint8_t i = 0;i < loopNum - 1;i++){
			differential->outputData[i] = differential -> outputData[i + 1];
	}
	return (differential->outputData[loopNum - 1]);
}

/*
@ param coefficientData:	coefficient data
@ param length:						length of coefficient data
@ return:									structure of differential data
*/
differentialDataStruct_t *differentialInit(f32_t *coefficientData,uint8_t length){
	differentialDataStruct_t *differential;
	differential = (differentialDataStruct_t *)aqDataCalloc(1, sizeof(differentialDataStruct_t));
	differential -> differentialLength = length;
	
	/* differential coefficient initialization */
	differential -> coefficient = (coefficientStruct_t *)aqDataCalloc(1, sizeof(coefficientStruct_t));
	differential -> coefficient -> yCoefficient = coefficientData;
	differential -> coefficient -> xCoefficient = coefficientData + differential -> differentialLength;
	
	/* input and output data initialization */
	differential -> inputData = (f32_t *)aqDataCalloc(differential -> differentialLength,sizeof(f32_t));
	differential -> outputData = (f32_t *)aqDataCalloc(differential -> differentialLength,sizeof(f32_t));
	for(uint8_t i;i < differential -> differentialLength;i++){
		differential -> inputData[i] 	= 0.0f;
		differential -> outputData[i] = 0.0f;
	}
	
	digitalHi(&differential->initFlag);
	return differential;
}

/*
@ param inputData:	input data
@ param outputData:	feedback data
@ param amplitude:	amplitude of input data
@ return:						null
*/
void dataAcquisition(f32_t *inputData,f32_t outputData,f32_t amplitude){
	static uint8_t TCounter = 0,stopFlag = 0;
	static uint16_t counter = 0,index = 0;
	if(!frequency[0])										//���û��ʼ�������ʼ��Ƶ��
		dataAcquisitionInit();
	
	if(!stopFlag){
		if(counter++ < 500 / frequency[index]){
			*inputData = amplitude * (f32_t)sin(2 * M_PI * frequency[index] * counter * 0.002f);
			buf.inputBuffer = *inputData * 1000;
			buf.outputBuffer = outputData * 1000;
			SEGGER_RTT_Write(1, &buf, sizeof(buf));
		}
		else{
			digitalClan(&counter);
			if(++TCounter == 20){						//20�����ڵ���
				digitalIncreasing(&index);		//������һ��Ƶ��
				digitalClan(&TCounter);
			}
			if(!frequency[index])						//ȫ��Ƶ���������
				digitalHi(&stopFlag);					//���ڸ�������
		}
	}
	else{
		digitalClan(inputData);						//���ݲ������
	}
}

void dataAcquisitionInit(void){
	uint8_t i = 0,cnt = 0;
	SEGGER_RTT_ConfigUpBuffer(1, "JScope_i2i2", rttUpBuffer, 2048, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
	for(cnt = 0;frequency[i - 1] < 22;i++,cnt++){
		frequency[i] = 1 + 0.5 * cnt;
	}
	for(cnt = 0;frequency[i - 1] < 40;i++,cnt++){
		frequency[i] = 24 + 2 * cnt;
	}
	for(cnt = 0;frequency[i - 1] < 120;i++,cnt++){
		frequency[i] = 50 + 10 * cnt;
	}
	frequency[i++] = 200;
	frequency[i++] = 250;
	frequency[i++] = 0;
//	frequency[i++] = 333;
//	frequency[i++] = 500;
}
