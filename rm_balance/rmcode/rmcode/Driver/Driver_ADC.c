#include "Driver_ADC.h"

__IO uint16_t adc_raw_value[10][4];
static void adcInit(void);
/*初始化类*/
deviceInitClass adcClass = {
	adcInit,
};

static void adcInit(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStrcture;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	//使能GPIOC时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_DMA2, ENABLE);
	//使能ADC1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	//DMA外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adc_raw_value;
	//（外设）地址到DMA储存器
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 4 * 10;
	//DMA非增量模式
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//DMA存储增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//DMA存储半个字节
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	//储存器数据长度16位
	DMA_InitStructure.DMA_MemoryDataSize =DMA_PeripheralDataSize_HalfWord;
	//循环模式；
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	//优先级 （高）
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	//存储器突发单次传输
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	//外设突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	//初始化DMA Stream0
	DMA_Init(DMA2_Stream0,&DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);
	//模拟输入
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1 |GPIO_Pin_2|GPIO_Pin_3;
	//不带上下拉
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	//独立模式
	ADC_CommonInitStrcture.ADC_Mode = ADC_Mode_Independent;
	//DMA模式1
	ADC_CommonInitStrcture.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	//预分频4（频率84/4M）
	ADC_CommonInitStrcture.ADC_Prescaler = ADC_Prescaler_Div4;
	//两个采样时间延时20个时钟
	ADC_CommonInitStrcture.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
	
	ADC_CommonInit(&ADC_CommonInitStrcture);
	//分辨率12位
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	//扫描模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	//连续转换	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	//禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	//右对齐
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//四个通道	
	ADC_InitStructure.ADC_NbrOfConversion = 4;

	ADC_Init(ADC1,&ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_144Cycles);
	
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);	
	ADC_DMACmd(ADC1,ENABLE);
	//开启AD转换器
	ADC_Cmd(ADC1, ENABLE);
	//软件开启ADC
	ADC_SoftwareStartConv(ADC1);

}













