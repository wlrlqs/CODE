#include <reg52.h>

sbit OLED_SDA_PIN=P0^6;
sbit OLED_SCK_PIN=P0^7;

#ifndef __OLED_H__
#define __OLED_H__

//#include "stm32f1xx_hal.h"

/*
#define   OLED_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()

#define   GPIOx_OLED_PORT               GPIOB
#define   OLED_SCK_PIN                  GPIO_PIN_12
#define   OLED_SCK_ON()                 HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SCK_PIN, GPIO_PIN_SET)
#define   OLED_SCK_OFF()                HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SCK_PIN, GPIO_PIN_RESET)
#define   OLED_SCK_TOGGLE()             HAL_GPIO_TogglePin(GPIOx_OLED_PORT, OLED_SCK_PIN)
#define   OLED_SDA_PIN                  GPIO_PIN_13
#define   OLED_SDA_ON()                 HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define   OLED_SDA_OFF()                HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)
#define   OLED_SDA_TOGGLE()             HAL_GPIO_TogglePin(GPIOx_OLED_PORT, OLED_SDA_PIN)
*///
#define   OLED_SCK_ON()                 OLED_SCK_PIN=1
#define   OLED_SCK_OFF()                OLED_SCK_PIN=0

#define   OLED_SDA_ON()                 OLED_SDA_PIN=1
#define   OLED_SDA_OFF()                OLED_SDA_PIN=0


void OLED_GPIO_Init(void);
void OLED_Init(void);
void OLED_IIC_Start(void);
void OLED_IIC_Stop(void);
void OLED_Send_Cmd(unsigned char);
void OLED_Send_Data(unsigned char);
void OLED_Set_Column(unsigned char);
void OLED_Set_Page(unsigned char);
void OLED_ALL_Clear(void);
void OLED_ALL_Full(void);
void OLED_IIC_Write(unsigned char);
void OLED_Draw_8_6_ASCII(unsigned char,unsigned char,int);
void OLED_Draw_16_8_ASCII(unsigned char,unsigned char,int);
void OLED_Draw_16_16_Hz(unsigned char,unsigned char,int);

#endif