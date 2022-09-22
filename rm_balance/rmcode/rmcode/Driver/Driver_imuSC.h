#ifndef __DRIVER_IMUSC_H
#define __DRIVER_IMUSC_H

#include "bsp_usart.h"
#include "stm32f4xx.h"
#include "util.h"
#include "stdbool.h"

#define USART_BROADCAST USART2			//串口号
#define USART_BROADCAST_TX_PIN BSP_GPIOA2	//发送引脚
#define USART_BROADCAST_RX_PIN BSP_GPIOA3	//接收引脚
#define USART_BROADCAST_BAUDRATE   921600
#define USART_BROADCAST_PRE_PRIORITY    3 //SBUS_USART中断抢占优先级
#define USART_BROADCAST_SUB_PRIORITY    0 //SBUS_USART中断响应优先级
#define USART_BROADCAST_TX_LENGTH   256 
#define USART_BROADCAST_RX_LENGTH   256

//串口下的地址
#define CONFIG_COM_ADDRESS  0xAD
#define BROADCAST_COM_ADDRESS   0x3F
//黄锡标
#define MAIN_CONTROL_ADDRESS	0x10
//使用canx发送
#define CAN_SEND_TYPE_DEF 		CAN2
typedef enum {
    FIRMWARE_LOG = 0,
    SETTING_LOG,
    CHANGE_GROUP,
    SOFTWARE_RESET,
    RESTROE_FACTORY,
    SAVE_SETTING,
    UPDATE_FIRMWARE,
    GYRO_CALIBRATE,
    ACCEL_PLANE_CALIBRATE,
    ACCEL_HEXAHEDRAL_CALIBRATE,
    MAG_CALIBRATE,
    CANCEL_CALIBRATION,
    FORWARD_DATA0,
    FORWARD_DATA1,
    GET_GYRO_CALIBRATE,
    GET_ACCEL_CALIBRATE,
    GET_MAG_CALIBRATE,
    
    SET_ALL_REG = 0xBF,
    OUTPUT_CONFIG_REG = 0xC0,
    OUTPUT_SWITCH_REG,
    DATA_TYPE_REG,
    IMU_CONFIG_REG,
    IMU_SENSOR_REG,
    MAG_PRESS_REG,
    IMU_FSR_REG,
    MODULAR_CONFIG_REG,
    Y_INSTALL_BIAS_REG,
    X_INSTALL_BIAS_REG = 0xCC,
    Z_INSTALL_BIAS_REG = 0xD0,
    CAN_RECEIVE_ID_REG = 0xD4,
    CAN_BROADCAST_ID_REG = 0xD6
} SPECIAL_COMMAND_E;    

#define SETTING_ID  0x100
#define BRAODCAST_RECEIVE_ID    0x110
#define FORWARD_TO_VCP_ID (0x08 + SETTING_ID)
#define FORWARD_TO_USART_TX_ID (0x0A + SETTING_ID)
#define FORWARD_FROM_USART_ID (0x08 + BRAODCAST_RECEIVE_ID)
#define FORWARD_FROM_VCP_TX_ID (0x0A + BRAODCAST_RECEIVE_ID)

#define IMU_ACCEL_FUSION_RW 0x0001
#define IMU_MAG_FUSION_RW   0x0002
#define IMU_YAW_CONtTINUOUS_RW  0x0004
#define IMU_UASRT_BROADCAST_RW  0x0008
#define IMU_CAN_BROADCAST_RW    0x0010
#define IMU_VCP_BROADCAST_RW    0x0020
#define IMU_USART_BAUDRATE_RW   0x00C0

#define IMU_GYRO_OUTPUT_RW  0x0101
#define IMU_ACCEL_OUTPUT_RW 0x0102
#define IMU_MAG_OUTPUT_RW   0x0104
#define IMU_PRES_OUTPUT_RW  0x0108
#define IMU_ATTITUDE_OUTPUT_RW  0x0110

#define IMU_GYRO_TYPE_RW    0x0201
#define IMU_ACCEL_TYPE_RW   0x0202
#define IMU_ATTITUDE_TYPE_RW    0x0204
#define IMU_HEIGHT_TYPE_RW  0x0208
#define IMU_GYRO_LENGTH_RW 0x0210
#define IMU_ACCEL_LENGTH_RW 0x0220
#define IMU_MAG_LENGTH_RW   0x0240
#define IMU_ATTITUDE_LENGTH_RW  0x0280

#define IMU_IMU_TYPE_RW 0x030F
#define IMU_VCP_FORWARD_CAN_RW  0x0310
#define IMU_VCP_FORWARD_USART_RW    0x0320
#define IMU_USART_FORWARD_CAN_RW    0x0340

#define IMU_GYRO_ODR_RW 0x0407
#define IMU_ACCEL_ODR_RW    0x0438

#define IMU_ATTITUDE_ODR_RW 0x0507
#define IMU_MAG_ODR_RW  0x0518
#define IMU_PRES_ODR_RW 0x0560

#define IMU_GYRO_FSR_RW 0x060F
#define IMU_ACCEL_FSR_RW    0x06F0

enum BROADCAST_CMD_LIST {
    BROADCAST_GYRO_CMD = 0x0001,
    BROADCAST_ACCEL_CMD = 0x0002,
    BROADCAST_MAG_CMD = 0x0004,
    BROADCAST_PRES_CMD = 0x0008,
    BROADCAST_CONFIG_CMD = 0x0010,
    BROADCAST_ATTITUDE_CMD = 0x0020,
    BROADCAST_FORWARD0_CMD = 0x0040,
    BROADCAST_FORWARD1_CMD = 0x0080,
    BROADCAST_GYRO_BIAS_CMD = 0x0100,
    BROADCAST_ACCEL_BIAS_CMD = 0x0200,
    BROADCAST_MAG_BIAS_CMD = 0x0400,
    BROADCAST_RESERVE3_CMD = 0x0800,
    BROADCAST_RESERVE4_CMD = 0x1000,
    BROADCAST_RESERVE5_CMD = 0x2000,
    BROADCAST_RESERVE6_CMD = 0x4000,
    BROADCAST_LOG_CMD = 0x8000,
};

enum {
    GYRO_DPS = 0,
    GYRO_RPS
};

enum {
    ACCEL_BODY = 0,
    ACCEL_WORLD
};

enum {
    OUTPUT_16BIT = 0,
    OUTPUT_32BIT,
};

enum {
    ATTITUDE_EULER = 0,
    ATTITUDE_QUAT
};

enum {
    HEIGHT_RAW = 0,
    HEIGHT_FUSION
};

enum {
    USART_BAUDRATE_460800 = 0,
    USART_BAUDRATE_115200,
    USART_BAUDRATE_230400,
    USART_BAUDRATE_921600
};

enum {
    IMU_TYPE_ADIS16470 = 0,
    IMU_TYPE_ICM20602,
    IMU_TYPE_ICM42605,
    IMU_TYPE_LSM6DSRX,
    IMU_TYPE_BMI088,
};
enum {
	GYRO_X = 0,
    GYRO_Y,
    GYRO_Z,
    ACCEL_X,
    ACCEL_Y,
    ACCEL_Z,
    EULER_X = 10,
    EULER_Y,
    EULER_Z,
    TEMPERATURE = 17, 	
};
//帧头内容
typedef enum {
    //帧地址所在下标
    HEADER_ADDRESS_INDEX = 0x00,
    //帧种类所在下标
    HEADER_TYPE_INDEX,
    //帧长度所在下标
    HEADER_LENGTH_INDEX,
    //帧序号所在下标
    HEADER_CNTRL_INDEX,
    HEADER_CNTRH_INDEX,
    //帧头长度
    HEADER_SIZE = 0x06,
    //帧头CRC8所在下标
    HEADER_HEADER_CRC8_INDEX = HEADER_SIZE - 0x01,
} HEADER_SEQUENCE_E;
//传感器数据包类型次序
typedef enum {
    SENSOR_GYRO_INDEX = 0x00,
    SENSOR_ACCEL_INDEX = 0x03,
    SENSOR_MAG_INDEX = 0x06,
    SENSOR_PRES_INDEX = 0x09,
    SENSOR_EULER_INDEX = 0x0A,
    SENSOR_QUAT_INDEX = 0x0D,
    SENSOR_TEMP_INDEX = 0x11,
    SENSOR_MAX_AMOUNT = 0x20
} SENSOR_BASE_SEQUENCE_E;
//帧类型
typedef enum  {
    FRAME_TYPE_SENSOR = 0x01,
    FRAME_TYPE_FORWARD0,
    FRAME_TYPE_FORWARD1,
    FRAME_TYPE_LOG,
} FRAME_TYPE_E;

typedef enum {
    CAN_TRANS_USART = 0,
    USART_TRANS_CAN,
    VCP_TRANS_CAN,
    VCP_TRANS_USART,
    TRANS_LIST
} FORWARD_ENUM;

//断言失败自动产生一个断点，在debug下生效
#define demoASSERT( x ) if( ( x ) == 0 ) { __breakpoint(0); }	
//传感器数据包的数量所在下标
#define PACKET_AMOUNT_INDEX HEADER_SIZE

typedef struct {
	uint8_t header;
	uint8_t	tail;
	uint8_t buffer[256];
} buffefLoopStruct_t;

typedef struct {
    uint8_t setting[8];
    float installCorrect[3];
    uint16_t receiveID;
    uint16_t broadcastID;
    uint8_t configGroup;
    float accelBias[3];
    float magBias[3];
    float gyroBias[3];
} imuConfigStruct_t;

typedef struct {
    uint8_t *_txBuff;   //串口dma发送指针
    uint8_t *_rxBuff;   //串口dma接收指针
    float gyro[3];
    float accel[3];
    float mag[3];
    float presure;
    float euler[3];
    float q[4];
	float temperature;
    uint16_t CNTR;
	uint16_t errorCount;
	uint16_t lastErrorCount;
	uint16_t intervalNum;
    imuConfigStruct_t config;
    buffefLoopStruct_t bufferLoop;
    buffefLoopStruct_t forwardContent[TRANS_LIST];
    BSP_USART_TypeDef usartConfig;
} imuBroadcastStruct_t; 

imuBroadcastStruct_t *getScimuData(void);
//读取特定位的配置
int8_t imuReadConfigRegister(uint16_t reg);
//写入特定位的配置
int8_t imuWirteConfigRegister(imuConfigStruct_t *config, uint16_t reg, uint8_t data);
//从USART获取一次配置信息
void imuUsartGetOnceParameter(void);
//USART单次写入1个寄存器的配置信息
void imuUsartSetting(uint8_t reg, uint8_t data);
//USART写入全部寄存器的配置信息
void imuUsartAllSetting(uint8_t *data);
//USART特殊命令
void imuUsartSpecialSetting(SPECIAL_COMMAND_E cmd);
//USART转发消息发出
void imuUsartForwardTx(uint8_t forwardTarget, uint8_t *data, uint8_t length);
//放在串口中断中的接收函数
void imuUsartIspFunction(uint8_t *array, uint16_t len);
//USART回环数组解析函数
void imuUsartReceive(void);

//can发送函数
uint8_t canTransferPack(uint32_t canID, uint8_t *src,CAN_TypeDef *CANx) ;
//CAN获取一次配置信息
void imuCanGetOnceParameter(void);
//CAN接收函数
void imuCanReceive(CanRxMsg *can_rx_msg);
//接收报文初始化
void imuBroadcastInit(void);
//获取转发消息的长度
uint8_t getForwardReceiveLength(FORWARD_ENUM group);
//获取转发消息内容
bool getDataFromForward(uint8_t *dst, FORWARD_ENUM group);

void adisInsDecode(void);//黄锡标

#endif


