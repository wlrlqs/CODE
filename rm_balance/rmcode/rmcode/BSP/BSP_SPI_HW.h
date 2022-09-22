#ifndef __BSP_SPI_HW_H
#define __BSP_SPI_HW_H

#include "stm32f4xx.h"
#include "stdlib.h"
#include "stdio.h"

#include "BSP_GPIO.h"


typedef struct
{
	SPI_TypeDef *SPIx;								//SPI号
	BSP_GPIOSource_TypeDef *SPI_NSS;	//SPI_NSS 引脚
	BSP_GPIOSource_TypeDef *SPI_SCK;	//SPI_SCK 引脚
	BSP_GPIOSource_TypeDef *SPI_MISO;	//SPI_MISO引脚
	BSP_GPIOSource_TypeDef *SPI_MOSI;	//SPI_MOSI引脚
}BSP_SPI_TypeDef;	

void BSP_SPI_Init(BSP_SPI_TypeDef* BSP_SPIx);
void BSP_SPIx_SetSpeed(BSP_SPI_TypeDef* BSP_SPIx,uint8_t SPI_BaudRatePrescaler);
uint8_t	 BSP_SPI_ReadWriteByte(BSP_SPI_TypeDef* BSP_SPIx,uint8_t TxData);
void SPI_DMA_TX_Init(BSP_SPI_TypeDef* BSP_SPIx,uint8_t *txBuff,uint16_t dataLen,uint8_t PreemptionPriority,uint8_t SubPriority);
void SPI_DMA_RX_Init(BSP_SPI_TypeDef* BSP_SPIx,uint8_t *rxBuff,uint16_t dataLen,uint8_t PreemptionPriority,uint8_t SubPriority);

/**************常用SPI设置*******************/
extern BSP_SPI_TypeDef BSP_SPI1;
extern BSP_SPI_TypeDef BSP_SPI2;
extern BSP_SPI_TypeDef BSP_SPI3;


#endif
