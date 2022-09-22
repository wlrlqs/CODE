#ifndef __CAN_FD_H
#define __CAN_FD_H

#include "drv_canfd_api.h"
#include "drv_canfd_register.h"

// Transmit Channels
#define APP_TX_FIFO CAN_FIFO_CH2
// Receive Channels    
#define APP_RX_FIFO CAN_FIFO_CH1
//开启RX中断
#define APP_USE_RX_INT
#define MAX_TXQUEUE_ATTEMPTS 50

#define CAN_SPEED CAN_1000K_8M

#define TX_RESPONSE_ID_GIMBAL  0x301
#define TX_RESPONSE_ID_CHASSIS 0x302
#define RX_RESPONSE_ID_GIMBAL  0x302
#define RX_RESPONSE_ID_CHASSIS 0x301

#define ID_CHECK 0x300
//CAN FD配置结构体
typedef struct __CAN_FD_CONFIG{
	struct{
		CAN_TX_FIFO_CONFIG tx_fifo_config;
		CAN_TX_FIFO_EVENT tx_fifo_evt;
	}txConfig;
	struct{
		CAN_RX_FIFO_CONFIG rx_fifo_config;
		CAN_RX_FIFO_EVENT rx_fifo_evt;
		CAN_FILTEROBJ_ID filter_config;
		CAN_MASKOBJ_ID mask_config;
	}rxConfig;
	struct{
		CAN_ERROR_STATE error_status;
		uint8_t tec;
		uint8_t rec;
	}error_master;
	CAN_CONFIG control_config;
	CAN_OPERATION_MODE control_mode;
}can_fd_configStruct_t;
//CAN FD发送结构体
typedef struct __CAN_FD_TX_MSG{
	CAN_TX_MSGOBJ mod;
	uint8_t data[MAX_DATA_BYTES];
	volatile uint16_t cntr;
}can_fd_transmitStruct_t;
//CAN FD接收结构体
typedef struct __CAN_FD_RX_MSG{
	CAN_RX_MSGOBJ mod;
	uint8_t data[MAX_DATA_BYTES];
	volatile uint16_t cntr;
}can_fd_receiveStruct_t;

typedef struct {
	void (*Send)		(can_fd_transmitStruct_t *tx_msg);
	void (*Receive)		(can_fd_receiveStruct_t *rx_msg);
}canFdTrans_t;

extern volatile uint32_t rx_concept;
extern uint8_t txData[MAX_DATA_BYTES];
extern uint8_t rxData[MAX_DATA_BYTES];
extern uint8_t __rxData[MAX_DATA_BYTES];
extern can_fd_transmitStruct_t can_fd_controlData;
extern can_fd_transmitStruct_t can_fd_chassisData;
extern can_fd_transmitStruct_t *can_fd_dataSend;
extern can_fd_receiveStruct_t can_fd_dataReceive;
extern can_fd_transmitStruct_t idData;
canFdTrans_t *getCanFdTrans(void);
void can_fd_spi_Init(void);
void can_fd_device_Init(void);
void can_fd_tx_testing(uint8_t board,uint8_t *txData,uint8_t len);
void can_fd_rx_testing(uint8_t *rxData);
can_fd_transmitStruct_t *arbitration_prepared(can_fd_transmitStruct_t *tx_msg,uint16_t standard_ID,CAN_DLC data_length_control);
#endif
