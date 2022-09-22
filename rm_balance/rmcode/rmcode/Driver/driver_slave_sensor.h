#ifndef __DRIVER_SLAVE_SENSOR_H
#define __DRIVER_SLAVE_SENSOR_H

#include "bsp.h"
#include "imu.h"
/**********************************SLAVE_SENSOR始化串口****************************************/
#define VISION_USARTX						USART6		//TX2通信串口号
#define VISION_USARTX_RX_PIN		BSP_GPIOC7	//TX2通信串口接收引脚
#define VISION_USARTX_TX_PIN		BSP_GPIOC6	//TX2通信串口发送引脚
#define VISION_USARTX_PRE			3			//TX2通信串口中断抢占优先级
#define VISION_USARTX_SUB			0			//TX2通信串口中断响应优先级
#define VISION_USARTS_BOUND			460800
 
#define USER_PRINTF_USARTX						UART7		//陀螺仪通信串口号
#define USER_PRINTF_USARTX_RX_PIN			BSP_GPIOE7	//陀螺仪通信串口接收引脚
#define USER_PRINTF_USARTX_TX_PIN			BSP_GPIOE8	//陀螺仪通信串口发送引脚
#define USER_PRINTF_USARTX_PRE				3			//陀螺仪通信串口中断抢占优先级
#define USER_PRINTF_USARTX_SUB				0			//陀螺仪通信串口中断响应优先级
#define USER_PRINTF_USARTS_BOUND			460800
 
#define CHASSIS_IMU_USARTX						UART7		//陀螺仪通信串口号
#define CHASSIS_IMU_USARTX_RX_PIN			BSP_GPIOE7	//陀螺仪通信串口接收引脚
#define CHASSIS_IMU_USARTX_TX_PIN			BSP_GPIOE8	//陀螺仪通信串口发送引脚
#define CHASSIS_IMU_USARTX_PRE				3			//陀螺仪通信串口中断抢占优先级
#define CHASSIS_IMU_USARTX_SUB				0			//陀螺仪通信串口中断响应优先级
#define CHASSIS_IMU_USARTS_BOUND			460800

#define GIMBAL_IMU_USARTX							USART1		//陀螺仪通信串口号
#define GIMBAL_IMU_USARTX_RX_PIN			BSP_GPIOA10	//陀螺仪通信串口接收引脚
#define GIMBAL_IMU_USARTX_TX_PIN			BSP_GPIOA9	//陀螺仪通信串口发送引脚
#define GIMBAL_IMU_USARTX_PRE				3			//陀螺仪通信串口中断抢占优先级
#define GIMBAL_IMU_USARTX_SUB				0			//陀螺仪通信串口中断响应优先级
#define GIMBAL_IMU_USARTS_BOUND			460800


#define JOINT_R_USARTX				USART6		//右关节电机通信串口号
#define JOINT_R_USARTX_RX_PIN		BSP_GPIOC7	//右关节电机通信串口接收引脚
#define JOINT_R_USARTX_TX_PIN		BSP_GPIOC6	//右关节电机通信串口发送引脚
#define JOINT_R_USARTX_PRE			3			//右关节电机通信串口中断抢占优先级
#define JOINT_R_USARTX_SUB			0			//右关节电机通信串口中断响应优先级
#define JOINT_R_USARTS_BOUND			4800000
 
#define JOINT_L_USARTX					USART1		//左关节电机通信串口号
#define JOINT_L_USARTX_RX_PIN			BSP_GPIOA10	//左关节电机通信串口接收引脚
#define JOINT_L_USARTX_TX_PIN			BSP_GPIOA9	//左关节电机通信串口发送引脚
#define JOINT_L_USARTX_PRE				3			//左关节电机通信串口中断抢占优先级
#define JOINT_L_USARTX_SUB				0			//左关节电机通信串口中断响应优先级
#define JOINT_L_USARTS_BOUND			4830000

#define TOF_L_USARTX					USART1		//左测距模块通信串口号
#define TOF_L_USARTX_RX_PIN				BSP_GPIOA10 //左测距模块通信串口接收引脚
#define TOF_L_USARTX_TX_PIN				BSP_GPIOA9	//左测距模块通信串口发送引脚
#define TOF_L_USARTX_PRE				3			//左测距模块通信串口中断抢占优先级
#define TOF_L_USARTX_SUB				0			//左测距模块通信串口中断响应优先级
#define TOF_L_USARTS_BOUND				921600		

#define TOF_R_USARTX					USART6		//右测距模块通信串口号
#define TOF_R_USARTX_RX_PIN				BSP_GPIOC7  //右测距模块通信串口接收引脚
#define TOF_R_USARTX_TX_PIN				BSP_GPIOC6	//右测距模块通信串口发送引脚
#define TOF_R_USARTX_PRE				3			//右测距模块通信串口中断抢占优先级
#define TOF_R_USARTX_SUB				0			//右测距模块通信串口中断响应优先级
#define TOF_R_USARTS_BOUND				921600		

void driver_slaveSensorInit(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,\
														BSP_GPIOSource_TypeDef *USART_TX,uint32_t baudRate,\
														uint8_t PreemptionPriority,uint8_t SubPriority);

#endif


