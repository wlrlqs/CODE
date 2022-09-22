#ifndef __BSP_DMA_H
#define __BSP_DMA_H

#include "stm32f4xx.h"
#include "stdlib.h"
#include "stdio.h"

#include "BSP_GPIO.h"
#include "BSP_USART.h"
#include "BSP_SPI_HW.h"
typedef const struct
{
	DMA_Stream_TypeDef *DMA_Streamx;
	uint32_t  DMA_channel;
}BSP_DMA_TypeDef;

enum {
	TX_MISSION = 0,
	RX_MISSION,
	MISSION,
};

void BSP_DMA_Init(BSP_DMA_TypeDef *BSP_DMA_Streamx_chx,uint32_t par,uint32_t mar,uint16_t ndtr);
void BSP_DMA_DSHOT_Init(BSP_DMA_TypeDef *BSP_DMA_Streamx_chx,uint32_t par,uint32_t mar,uint16_t ndtr);
void BSP_DMA_SK6812_Init(BSP_DMA_TypeDef *BSP_DMA_Streamx_chx,uint32_t par,uint32_t mar,uint16_t ndtr);
void BSP_DMA_TX_Init(BSP_DMA_TypeDef *BSP_DMA_Streamx_chx,USART_TypeDef *USARTx,uint32_t par,uint16_t ndtr);
void BSP_DMA_RX_Init(BSP_DMA_TypeDef *BSP_DMA_Streamx_chx,USART_TypeDef *USARTx,uint32_t par,uint16_t ndtr);
void BSP_DMA_SPI_TX_Init(BSP_DMA_TypeDef *BSP_DMA_Streamx_chx,uint32_t par,uint32_t mar,uint16_t ndtr);
void BSP_DMA_SPI_RX_Init(BSP_DMA_TypeDef *BSP_DMA_Streamx_chx,uint32_t par,uint32_t mar,uint16_t ndtr);
BSP_DMA_TypeDef *SPI_TO_BSP_DMA_Streamx_chx(BSP_SPI_TypeDef* BSP_SPIx,uint16_t SPI_DMAReq);
uint32_t SPI_DMA_CLEAR_FLAG(DMA_Stream_TypeDef *DMA_Stream_chx);
uint8_t SPI_TO_NVIC_Channel(DMA_Stream_TypeDef *DMA_Stream_chx);
void DMA_SPI_COM(BSP_SPI_TypeDef *BSP_SPIx,uint8_t *data,uint16_t len,uint8_t mode);

/************************ DMA1_Stream0 ************************/
extern BSP_DMA_TypeDef BSP_DMA_SPI3_RX;
extern BSP_DMA_TypeDef BSP_DMA_I2C1_RX;
extern BSP_DMA_TypeDef BSP_DMA_TIM4_CH1;
extern BSP_DMA_TypeDef BSP_DMA_I2S3_EXT_RX;
extern BSP_DMA_TypeDef BSP_DMA_UART5_RX;
extern BSP_DMA_TypeDef BSP_DMA_UART8_TX;
extern BSP_DMA_TypeDef BSP_DMA_TIM5_CH3;
extern BSP_DMA_TypeDef BSP_DMA_TIM5_UP;

/************************ DMA1_Stream1 ************************/
extern BSP_DMA_TypeDef BSP_DMA_TIM2_CH3;
extern BSP_DMA_TypeDef BSP_DMA_TIM2_UP;
extern BSP_DMA_TypeDef BSP_DMA_USART3_RX;
extern BSP_DMA_TypeDef BSP_DMA_UART7_TX;
extern BSP_DMA_TypeDef BSP_DMA_TIM5_CH4;
extern BSP_DMA_TypeDef BSP_DMA_TIM5_TRIG;
extern BSP_DMA_TypeDef BSP_DMA_TIM6_UP;

/************************ DMA1_Stream2 ************************/
extern BSP_DMA_TypeDef BSP_DMA1_SPI3_RX;
extern BSP_DMA_TypeDef BSP_DMA_TIM7_UP;
extern BSP_DMA_TypeDef BSP_DMA1_I2S3_EXT_RX;
extern BSP_DMA_TypeDef BSP_DMA_I2C3_RX;
extern BSP_DMA_TypeDef BSP_DMA_UART4_RX;
extern BSP_DMA_TypeDef BSP_DMA_TIM3_CH4;
extern BSP_DMA_TypeDef BSP_DMA_TIM3_UP;
extern BSP_DMA_TypeDef BSP_DMA_TIM5_CH1;
extern BSP_DMA_TypeDef BSP_DMA_I2C2_RX;

/************************ DMA1_Stream3 ************************/
extern BSP_DMA_TypeDef BSP_DMA_SPI2_RX;
extern BSP_DMA_TypeDef BSP_DMA_TIM4_CH2;
extern BSP_DMA_TypeDef BSP_DMA_I2S2_EXT_RX;
extern BSP_DMA_TypeDef BSP_DMA_USART3_TX;
extern BSP_DMA_TypeDef BSP_DMA_UART7_RX;
extern BSP_DMA_TypeDef BSP_DMA1_TIM5_CH4;
extern BSP_DMA_TypeDef BSP_DMA1_TIM5_TRIG;
extern BSP_DMA_TypeDef BSP_DMA1_I2C2_RX;

/************************ DMA1_Stream4 ************************/
extern BSP_DMA_TypeDef BSP_DMA_SPI2_TX;
extern BSP_DMA_TypeDef BSP_DMA1_TIM7_UP;
extern BSP_DMA_TypeDef BSP_DMA_I2S2_EXT_TX;
extern BSP_DMA_TypeDef BSP_DMA_I2C3_TX;
extern BSP_DMA_TypeDef BSP_DMA_UART4_TX;
extern BSP_DMA_TypeDef BSP_DMA_TIM3_CH1;
extern BSP_DMA_TypeDef BSP_DMA_TIM3_TRIG;
extern BSP_DMA_TypeDef BSP_DMA_TIM5_CH2;
extern BSP_DMA_TypeDef BSP_DMA1_USART3_TX;

/************************ DMA1_Stream5 ************************/
extern BSP_DMA_TypeDef BSP_DMA_SPI3_TX;
extern BSP_DMA_TypeDef BSP_DMA1_I2C1_RX;
extern BSP_DMA_TypeDef BSP_DMA_I2S3_EXT_TX;
extern BSP_DMA_TypeDef BSP_DMA_TIM2_CH1;
extern BSP_DMA_TypeDef BSP_DMA_USART2_RX;
extern BSP_DMA_TypeDef BSP_DMA_TIM3_CH2;
extern BSP_DMA_TypeDef BSP_DMA_DAC1;

/************************ DMA1_Stream6 ************************/
extern BSP_DMA_TypeDef BSP_DMA_I2C1_TX;
extern BSP_DMA_TypeDef BSP_DMA_TIM4_UP;
extern BSP_DMA_TypeDef BSP_DMA_TIM2_CH2;
extern BSP_DMA_TypeDef BSP_DMA_TIM2_CH4;
extern BSP_DMA_TypeDef BSP_DMA_USART2_TX;
extern BSP_DMA_TypeDef BSP_DMA_UART8_RX;
extern BSP_DMA_TypeDef BSP_DMA1_TIM5_UP;
extern BSP_DMA_TypeDef BSP_DMA_DAC2;

/************************ DMA1_Stream7 ************************/
extern BSP_DMA_TypeDef BSP_DMA1_SPI3_TX;
extern BSP_DMA_TypeDef BSP_DMA1_I2C1_TX;
extern BSP_DMA_TypeDef BSP_DMA_TIM4_CH3;
extern BSP_DMA_TypeDef BSP_DMA1_TIM2_CH4;
extern BSP_DMA_TypeDef BSP_DMA1_TIM2_UP;
extern BSP_DMA_TypeDef BSP_DMA_UART5_TX;
extern BSP_DMA_TypeDef BSP_DMA_TIM3_CH3;
extern BSP_DMA_TypeDef BSP_DMA_I2C2_TX;

/***************************************************************************************/
/***************************************************************************************/

/************************ DMA2_Stream0 ************************/
extern BSP_DMA_TypeDef BSP_DMA_ADC1;
extern BSP_DMA_TypeDef BSP_DMA_ADC3;
extern BSP_DMA_TypeDef BSP_DMA_SPI1_RX;
extern BSP_DMA_TypeDef BSP_DMA_SPI4_RX;
extern BSP_DMA_TypeDef BSP_DMA_TIM1_TRIG;

/************************ DMA2_Stream1 ************************/
extern BSP_DMA_TypeDef BSP_DMA_DCMI;
extern BSP_DMA_TypeDef BSP_DMA1_ADC3;
extern BSP_DMA_TypeDef BSP_DMA_SPI4_TX;
extern BSP_DMA_TypeDef BSP_DMA_USART6_RX;
extern BSP_DMA_TypeDef BSP_DMA_TIM1_CH1;
extern BSP_DMA_TypeDef BSP_DMA_TIM8_UP;

/************************ DMA2_Stream2 ************************/
extern BSP_DMA_TypeDef BSP_DMA_TIM8_CH1;
extern BSP_DMA_TypeDef BSP_DMA_TIM8_CH2;
extern BSP_DMA_TypeDef BSP_DMA_TIM8_CH3;
extern BSP_DMA_TypeDef BSP_DMA_ADC2;
extern BSP_DMA_TypeDef BSP_DMA1_SPI1_RX;
extern BSP_DMA_TypeDef BSP_DMA_USART1_RX;
extern BSP_DMA_TypeDef BSP_DMA1_USART6_RX;
extern BSP_DMA_TypeDef BSP_DMA_TIM1_CH2;
extern BSP_DMA_TypeDef BSP_DMA1_TIM8_CH1;

/************************ DMA2_Stream3 ************************/
extern BSP_DMA_TypeDef BSP_DMA1_ADC2;
extern BSP_DMA_TypeDef BSP_DMA_SPI5_RX;
extern BSP_DMA_TypeDef BSP_DMA_SPI1_TX;
extern BSP_DMA_TypeDef BSP_DMA_SDIO;
extern BSP_DMA_TypeDef BSP_DMA1_SPI4_RX;
extern BSP_DMA_TypeDef BSP_DMA1_TIM1_CH1;
extern BSP_DMA_TypeDef BSP_DMA1_TIM8_CH2;

/************************ DMA2_Stream4 ************************/
extern BSP_DMA_TypeDef BSP_DMA1_ADC1;
extern BSP_DMA_TypeDef BSP_DMA_SPI5_TX;
extern BSP_DMA_TypeDef BSP_DMA1_SPI4_TX;
extern BSP_DMA_TypeDef BSP_DMA_TIM1_CH4;
extern BSP_DMA_TypeDef BSP_DMA1_TIM1_TRIG;
extern BSP_DMA_TypeDef BSP_DMA_TIM1_COM;
extern BSP_DMA_TypeDef BSP_DMA1_TIM8_CH3;

/************************ DMA2_Stream5 ************************/
extern BSP_DMA_TypeDef BSP_DMA_SPI6_TX;
extern BSP_DMA_TypeDef BSP_DMA_CRYP_OUT;
extern BSP_DMA_TypeDef BSP_DMA2_SPI1_RX;
extern BSP_DMA_TypeDef BSP_DMA1_USART1_RX;
extern BSP_DMA_TypeDef BSP_DMA_TIM1_UP;
extern BSP_DMA_TypeDef BSP_DMA1_SPI5_RX;

/************************ DMA2_Stream6 ************************/
extern BSP_DMA_TypeDef BSP_DMA2_TIM1_CH1;
extern BSP_DMA_TypeDef BSP_DMA1_TIM1_CH2;
extern BSP_DMA_TypeDef BSP_DMA_TIM1_CH3;
extern BSP_DMA_TypeDef BSP_DMA_SPI6_RX;
extern BSP_DMA_TypeDef BSP_DMA_CRYP_IN;
extern BSP_DMA_TypeDef BSP_DMA1_SDIO;
extern BSP_DMA_TypeDef BSP_DMA_USART6_TX;
extern BSP_DMA_TypeDef BSP_DMA1_TIM1_CH3;
extern BSP_DMA_TypeDef BSP_DMA1_SPI5_TX;

/************************ DMA2_Stream7 ************************/
extern BSP_DMA_TypeDef BSP_DMA1_DCMI;
extern BSP_DMA_TypeDef BSP_DMA_HASH_IN;
extern BSP_DMA_TypeDef BSP_DMA_USART1_TX;
extern BSP_DMA_TypeDef BSP_DMA1_USART6_TX;
extern BSP_DMA_TypeDef BSP_DMA_TIM8_CH4;
extern BSP_DMA_TypeDef BSP_DMA_TIM8_TRIG;
extern BSP_DMA_TypeDef BSP_DMA_TIM8_COM;

#endif
