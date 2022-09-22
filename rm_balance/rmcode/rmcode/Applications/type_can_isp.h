#ifndef __TYPE_CAN_ISP_H
#define __TYPE_CAN_ISP_H

#include "Driver_MotorSever.h"
#include "Driver_MotorSeverList.h"

typedef struct{
	uint16_t id;
	uint16_t speed;
	uint16_t current;
	uint16_t position;
	uint8_t  _error;
	uint8_t  mode;
	uint8_t datas[8];
}fankui;

/*INFANTRY*/
void infantry_can1_gimbal(CanRxMsg *can_rx_msg);
void infantry_can2_gimbal(CanRxMsg *can_rx_msg);
void infantry_can1_cahssis(CanRxMsg *can_rx_msg);
void infantry_can2_cahssis(CanRxMsg *can_rx_msg);
/*F_TANK*/
void f_tank_can1_gimbal(CanRxMsg *can_rx_msg);
void f_tank_can2_gimbal(CanRxMsg *can_rx_msg);
void f_tank_can1_chassis(CanRxMsg *can_rx_msg);
void f_tank_can2_chassis(CanRxMsg *can_rx_msg);
/*P_TANK*/
void p_tank_can1_gimbal(CanRxMsg *can_rx_msg);
void p_tank_can2_gimbal(CanRxMsg *can_rx_msg);
void p_tank_can1_cahssis(CanRxMsg *can_rx_msg);
void p_tank_can2_cahssis(CanRxMsg *can_rx_msg);
/*AUXILIARY*/
void auxiliary_can1_gimbal(CanRxMsg *can_rx_msg);
void auxiliary_can2_gimbal(CanRxMsg *can_rx_msg);
void auxiliary_can1_chassis(CanRxMsg *can_rx_msg);
void auxiliary_can2_chassis(CanRxMsg *can_rx_msg);
/*SENTRY*/
void sentry_can1_gimbal(CanRxMsg *can_rx_msg);
void sentry_can2_gimbal(CanRxMsg *can_rx_msg);
void sentry_can1_cahssis(CanRxMsg *can_rx_msg);
void sentry_can2_cahssis(CanRxMsg *can_rx_msg);
/*SMALLGIMBAL*/
void smallGimbal_can1_gimbal(CanRxMsg *can_rx_msg);
void smallGimbal_can2_gimbal(CanRxMsg *can_rx_msg);
void smallGimbal_can1_cahssis(CanRxMsg *can_rx_msg);
void smallGimbal_can2_cahssis(CanRxMsg *can_rx_msg);
/*UAV*/
void uav_can1_gimbal(CanRxMsg *can_rx_msg);
void uav_can2_gimbal(CanRxMsg *can_rx_msg);

#endif
