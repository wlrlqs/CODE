#include "Driver_ADC.h"

__IO uint16_t adc_raw_value[10][4];
static void adcInit(void);
/*��ʼ����*/
deviceInitClass adcClass = {
	adcInit,
};

static void adcInit(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStrcture;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	//ʹ��GPIOCʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_DMA2, ENABLE);
	//ʹ��ADC1ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	//DMA�����ַ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adc_raw_value;
	//�����裩��ַ��DMA������
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 4 * 10;
	//DMA������ģʽ
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//DMA�洢����ģʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//DMA�洢����ֽ�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	//���������ݳ���16λ
	DMA_InitStructure.DMA_MemoryDataSize =DMA_PeripheralDataSize_HalfWord;
	//ѭ��ģʽ��
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	//���ȼ� ���ߣ�
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	//�洢��ͻ�����δ���
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	//����ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	//��ʼ��DMA Stream0
	DMA_Init(DMA2_Stream0,&DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);
	//ģ������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1 |GPIO_Pin_2|GPIO_Pin_3;
	//����������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	//����ģʽ
	ADC_CommonInitStrcture.ADC_Mode = ADC_Mode_Independent;
	//DMAģʽ1
	ADC_CommonInitStrcture.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	//Ԥ��Ƶ4��Ƶ��84/4M��
	ADC_CommonInitStrcture.ADC_Prescaler = ADC_Prescaler_Div4;
	//��������ʱ����ʱ20��ʱ��
	ADC_CommonInitStrcture.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
	
	ADC_CommonInit(&ADC_CommonInitStrcture);
	//�ֱ���12λ
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	//ɨ��ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	//����ת��	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	//��ֹ������⣬ʹ���������
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	//�Ҷ���
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//�ĸ�ͨ��	
	ADC_InitStructure.ADC_NbrOfConversion = 4;

	ADC_Init(ADC1,&ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_144Cycles);
	
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);	
	ADC_DMACmd(ADC1,ENABLE);
	//����ADת����
	ADC_Cmd(ADC1, ENABLE);
	//�������ADC
	ADC_SoftwareStartConv(ADC1);

}













