#include "Driver_SK6812.h"   //����
#include "Driver_Motor_Dshot.h"
#include "BSP.h"

uint16_t BIT_COMPARE_1 = 0;
uint16_t BIT_COMPARE_0 = 0;

SK6812Struct_t sk6812Data;

void getLedHsv(uint16_t index, hsvColor_t *color){
	*color = sk6812Data.ledColorBuffer[index];
}

rgbColor24bpp_t* hsvToRgb24(const hsvColor_t* c){
	static rgbColor24bpp_t r;
	uint16_t val = c->v;
	uint16_t sat = 255 - c->s;
	uint32_t base;
	uint16_t hue = c->h;

	if (sat == 0) { // Acromatic color (gray). Hue doesn't mind.
		r.rgb.r = val;
		r.rgb.g = val;
		r.rgb.b = val;
	} 
	else {
		base = ((255 - sat) * val) >> 8;
		switch (hue / 60) {
			case 0:
			r.rgb.r = val;
			r.rgb.g = (((val - base) * hue) / 60) + base;
			r.rgb.b = base;
			break;
			case 1:
			r.rgb.r = (((val - base) * (60 - (hue % 60))) / 60) + base;
			r.rgb.g = val;
			r.rgb.b = base;
			break;

			case 2:
			r.rgb.r = base;
			r.rgb.g = val;
			r.rgb.b = (((val - base) * (hue % 60)) / 60) + base;
			break;

			case 3:
			r.rgb.r = base;
			r.rgb.g = (((val - base) * (60 - (hue % 60))) / 60) + base;
			r.rgb.b = val;
			break;

			case 4:
			r.rgb.r = (((val - base) * (hue % 60)) / 60) + base;
			r.rgb.g = base;
			r.rgb.b = val;
			break;

			case 5:
			r.rgb.r = val;
			r.rgb.g = base;
			r.rgb.b = (((val - base) * (60 - (hue % 60))) / 60) + base;
			break;
		}
	}
	return &r;
}

//�˴�Ӧ�õ�DMAͨ����Dshot���໥��ͻ��
void SK6812Config(void){
	uint16_t typePrescaler;
	uint16_t typePeriod;
	typePrescaler = (uint16_t)(MHZ_TO_HZ(SK6812_TIMER_MHZ) / MHZ_TO_HZ(SK6812_TIMER_MHZ));
	typePeriod    = (uint16_t)((MHZ_TO_HZ(SK6812_TIMER_MHZ) / typePrescaler) / SK6812_CARRIER_HZ);//105
	BIT_COMPARE_1 = (uint16_t)typePeriod / 11 * 6;
	BIT_COMPARE_0 = (uint16_t)typePeriod / 11 * 3;
	BSP_TIM_PWM_Init(SK6812_TIMER,typePeriod,typePrescaler,NULL,NULL,SK6812_GPIO,NULL);	
	pwmDshotDmaIrqnConfig(DMA1_Stream7_IRQn,1,0);
	BSP_DMA_SK6812_Init(&SK6812_BSP_DMA_TIM_GR,(uint32_t)&SK6812_TIMER->CCR3,(uint32_t)sk6812Data.ledStripDMABuffer,SK6812_DMA_BUFFER_SIZE);
}

static void fastUpdateLEDDMABuffer(rgbColor24bpp_t *color){
	uint32_t grb = (color->rgb.g << 16) | (color->rgb.r << 8) | (color->rgb.b);

	for (int8_t index = 23; index >= 0; index--) {
		sk6812Data.ledStripDMABuffer[sk6812Data.dmaBufferOffset++] = (grb & (1 << index)) ? BIT_COMPARE_1 : BIT_COMPARE_0;
	}
}

void SK6812_SendData(void){		
	DMA_SetCurrDataCounter(SK6812_BSP_DMA_TIM_GR.DMA_Streamx,SK6812_DMA_BUFFER_SIZE);
	TIM_SetCounter(SK6812_TIMER, 0);				
	TIM_DMACmd(SK6812_TIMER,SK682_DMA_TIMCH, ENABLE);
	DMA_Cmd(SK6812_BSP_DMA_TIM_GR.DMA_Streamx, ENABLE);
}

void setOneLedHsv(uint16_t index, const hsvColor_t *color){
	sk6812Data.ledColorBuffer[index] = *color;
}

void setColor(hsvColor_t *color,uint16_t h,uint8_t s,uint8_t v){
	color->h = h;
	color->s = s;
	color->v = v;
}
int16_t R=0,G=0,B=0;
void colorStdInit(){
	setColor(&sk6812Data.colorStd[COLOR_RED],120,0,2);
	setColor(&sk6812Data.colorStd[COLOR_GREEN],0,0,5);
	setColor(&sk6812Data.colorStd[COLOR_YELLOW],50,15,10);
	setColor(&sk6812Data.colorStd[COLOR_DARK],0,0,0);
	setColor(&sk6812Data.colorStd[COLOR_BLUE],220,0,2);
	setColor(&sk6812Data.colorStd[COLOR_PINK],140,30,10);
	setColor(&sk6812Data.colorStd[COLOR_WHITE],0,255,10);
	setColor(&sk6812Data.colorStd[COLOR_PURPLE],200,0,2);
}

void setAllLedColors(hsvColor_t *color){
	for(uint16_t index = 0;index < SK6812_LED_STRIP_LENGTH;index++)
		sk6812Data.ledColorBuffer[index] = *(color);
}

void SK6812UpdateStrip(void){
	static rgbColor24bpp_t *rgb24;
	static int16_t ledIndex;
	sk6812Data.dmaBufferOffset = 0;     // reset buffer memory index				���û���洢������
	ledIndex = 0;                       // reset led index							����LED��ָ��	
	// fill transmit buffer with correct compare values to achieve			����ȷ�ıȽ�ֵ��䴫�仺������ʵ��
	while (ledIndex < SK6812_LED_STRIP_LENGTH){
		rgb24 = hsvToRgb24(&sk6812Data.ledColorBuffer[ledIndex]);						//����ǰ�hsvת����RGB�Ĺؼ���
		fastUpdateLEDDMABuffer(rgb24);
 		ledIndex++;
	}
	SK6812_SendData();
}

void DMA1_Stream7_IRQHandler(void){																	//DMA1-7
	if(DMA_GetITStatus(DMA1_Stream7,DMA_IT_TCIF7)){
		DMA_Cmd(DMA1_Stream7, DISABLE);																	//ʧ��
		TIM_DMACmd(SK6812_TIMER,SK682_DMA_TIMCH, DISABLE);													//ȡ��dma����tim�Ƚ�
		DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);							//���IT_FLAG
	}
}

