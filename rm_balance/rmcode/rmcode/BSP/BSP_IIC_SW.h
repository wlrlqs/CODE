#ifndef __BSP_IIC_SW_H
#define __BSP_IIC_SW_H

#include "stm32f4xx.h"
#include "stdlib.h"
#include "stdio.h"

#include "BSP_GPIO.h"

#define IIC_WR	0		/* д����bit */
#define IIC_RD	1		/* ������bit */

//IO��������
#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//SDA����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //SDA���ģʽ

//IO��������	 
#define IIC_SCL    PBout(8) //SCL
#define IIC_SDA    PBout(9) //SDA	
#define READ_SCL   PBin(8)  //����SCL
#define READ_SDA   PBin(9)  //����SDA 

#define IIC_Delay()	IIC_SW_Delay(1)

//IIC���в�������
void BSP_IIC_SW_Init(BSP_GPIOSource_TypeDef* BSP_IIC_SCL_GPIO,		//��ʼ��IIC��IO��	
											BSP_GPIOSource_TypeDef* BSP_IIC_SDA_GPIO);	
unsigned char BSP_IIC_WriteData(uint8_t dev_addr,uint8_t regAddr,uint8_t data);
unsigned char BSP_IIC_ReadData(uint8_t dev_addr,uint8_t regAddr,uint8_t *buf,uint8_t length);
void BSP_IIC_Start(void);		//����IIC��ʼ�ź�
void BSP_IIC_Stop(void);		//����IICֹͣ�ź�
void BSP_IIC_Send_Byte(uint8_t txd);		//IIC����һ���ֽ�
uint8_t BSP_IIC_Read_Byte(unsigned char ack);	//IIC��ȡһ���ֽ�
uint8_t BSP_IIC_Wait_Ack(void);	//IIC�ȴ�ACK�ź�
void BSP_IIC_Ack(void);			//IIC����ACK�ź�
void BSP_IIC_NAck(void);		//IIC������ACK�ź� 
uint8_t BSP_IIC_CheckDevice(uint8_t Device_Address);//����豸
void IIC_SW_Delay(unsigned int t);

#endif
