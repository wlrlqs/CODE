#ifndef __DRIVER_SLAVE_SENSOR_H
#define __DRIVER_SLAVE_SENSOR_H

#include "bsp.h"
#include "imu.h"
/**********************************SLAVE_SENSORʼ������****************************************/
#define VISION_USARTX						USART6		//TX2ͨ�Ŵ��ں�
#define VISION_USARTX_RX_PIN		BSP_GPIOC7	//TX2ͨ�Ŵ��ڽ�������
#define VISION_USARTX_TX_PIN		BSP_GPIOC6	//TX2ͨ�Ŵ��ڷ�������
#define VISION_USARTX_PRE			3			//TX2ͨ�Ŵ����ж���ռ���ȼ�
#define VISION_USARTX_SUB			0			//TX2ͨ�Ŵ����ж���Ӧ���ȼ�
#define VISION_USARTS_BOUND			460800
 
#define USER_PRINTF_USARTX						UART7		//������ͨ�Ŵ��ں�
#define USER_PRINTF_USARTX_RX_PIN			BSP_GPIOE7	//������ͨ�Ŵ��ڽ�������
#define USER_PRINTF_USARTX_TX_PIN			BSP_GPIOE8	//������ͨ�Ŵ��ڷ�������
#define USER_PRINTF_USARTX_PRE				3			//������ͨ�Ŵ����ж���ռ���ȼ�
#define USER_PRINTF_USARTX_SUB				0			//������ͨ�Ŵ����ж���Ӧ���ȼ�
#define USER_PRINTF_USARTS_BOUND			460800
 
#define CHASSIS_IMU_USARTX						UART7		//������ͨ�Ŵ��ں�
#define CHASSIS_IMU_USARTX_RX_PIN			BSP_GPIOE7	//������ͨ�Ŵ��ڽ�������
#define CHASSIS_IMU_USARTX_TX_PIN			BSP_GPIOE8	//������ͨ�Ŵ��ڷ�������
#define CHASSIS_IMU_USARTX_PRE				3			//������ͨ�Ŵ����ж���ռ���ȼ�
#define CHASSIS_IMU_USARTX_SUB				0			//������ͨ�Ŵ����ж���Ӧ���ȼ�
#define CHASSIS_IMU_USARTS_BOUND			460800

#define GIMBAL_IMU_USARTX							USART1		//������ͨ�Ŵ��ں�
#define GIMBAL_IMU_USARTX_RX_PIN			BSP_GPIOA10	//������ͨ�Ŵ��ڽ�������
#define GIMBAL_IMU_USARTX_TX_PIN			BSP_GPIOA9	//������ͨ�Ŵ��ڷ�������
#define GIMBAL_IMU_USARTX_PRE				3			//������ͨ�Ŵ����ж���ռ���ȼ�
#define GIMBAL_IMU_USARTX_SUB				0			//������ͨ�Ŵ����ж���Ӧ���ȼ�
#define GIMBAL_IMU_USARTS_BOUND			460800


#define JOINT_R_USARTX				USART6		//�ҹؽڵ��ͨ�Ŵ��ں�
#define JOINT_R_USARTX_RX_PIN		BSP_GPIOC7	//�ҹؽڵ��ͨ�Ŵ��ڽ�������
#define JOINT_R_USARTX_TX_PIN		BSP_GPIOC6	//�ҹؽڵ��ͨ�Ŵ��ڷ�������
#define JOINT_R_USARTX_PRE			3			//�ҹؽڵ��ͨ�Ŵ����ж���ռ���ȼ�
#define JOINT_R_USARTX_SUB			0			//�ҹؽڵ��ͨ�Ŵ����ж���Ӧ���ȼ�
#define JOINT_R_USARTS_BOUND			4800000
 
#define JOINT_L_USARTX					USART1		//��ؽڵ��ͨ�Ŵ��ں�
#define JOINT_L_USARTX_RX_PIN			BSP_GPIOA10	//��ؽڵ��ͨ�Ŵ��ڽ�������
#define JOINT_L_USARTX_TX_PIN			BSP_GPIOA9	//��ؽڵ��ͨ�Ŵ��ڷ�������
#define JOINT_L_USARTX_PRE				3			//��ؽڵ��ͨ�Ŵ����ж���ռ���ȼ�
#define JOINT_L_USARTX_SUB				0			//��ؽڵ��ͨ�Ŵ����ж���Ӧ���ȼ�
#define JOINT_L_USARTS_BOUND			4830000

#define TOF_L_USARTX					USART1		//����ģ��ͨ�Ŵ��ں�
#define TOF_L_USARTX_RX_PIN				BSP_GPIOA10 //����ģ��ͨ�Ŵ��ڽ�������
#define TOF_L_USARTX_TX_PIN				BSP_GPIOA9	//����ģ��ͨ�Ŵ��ڷ�������
#define TOF_L_USARTX_PRE				3			//����ģ��ͨ�Ŵ����ж���ռ���ȼ�
#define TOF_L_USARTX_SUB				0			//����ģ��ͨ�Ŵ����ж���Ӧ���ȼ�
#define TOF_L_USARTS_BOUND				921600		

#define TOF_R_USARTX					USART6		//�Ҳ��ģ��ͨ�Ŵ��ں�
#define TOF_R_USARTX_RX_PIN				BSP_GPIOC7  //�Ҳ��ģ��ͨ�Ŵ��ڽ�������
#define TOF_R_USARTX_TX_PIN				BSP_GPIOC6	//�Ҳ��ģ��ͨ�Ŵ��ڷ�������
#define TOF_R_USARTX_PRE				3			//�Ҳ��ģ��ͨ�Ŵ����ж���ռ���ȼ�
#define TOF_R_USARTX_SUB				0			//�Ҳ��ģ��ͨ�Ŵ����ж���Ӧ���ȼ�
#define TOF_R_USARTS_BOUND				921600		

void driver_slaveSensorInit(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,\
														BSP_GPIOSource_TypeDef *USART_TX,uint32_t baudRate,\
														uint8_t PreemptionPriority,uint8_t SubPriority);

#endif


