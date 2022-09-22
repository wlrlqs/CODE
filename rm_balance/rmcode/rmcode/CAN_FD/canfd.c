#include "canfd.h"
#include "board.h"

/***************测试用***************/
CAN_TX_MSGOBJ txObj;
CAN_RX_MSGOBJ rxObj;
volatile uint32_t rx_concept = 0;
uint8_t txData[MAX_DATA_BYTES];
uint8_t rxData[MAX_DATA_BYTES];
uint8_t __rxData[MAX_DATA_BYTES];

/************************************/

can_fd_configStruct_t double_data_setting;
//主控数据
can_fd_transmitStruct_t can_fd_controlData;
//底盘数据
can_fd_transmitStruct_t can_fd_chassisData;
//转发数据
can_fd_transmitStruct_t *can_fd_dataSend;
can_fd_receiveStruct_t can_fd_dataReceive;
can_fd_transmitStruct_t idData;
static void sendTask(can_fd_transmitStruct_t *tx_msg);
static void receiveTask(can_fd_receiveStruct_t *rx_msg);
canFdTrans_t canFdTrans = {
	sendTask,
	receiveTask,
};

canFdTrans_t *getCanFdTrans(){
	return &canFdTrans;
}

void can_fd_spi_Init(void){
	//SPI初始化
	BSP_SPI_Init(&canfd_control);
	//SPI波特率
	BSP_SPIx_SetSpeed(&canfd_control,SPI_BaudRatePrescaler_16);
	BSP_GPIO_Init(CAN_FD_INT1,GPIO_Mode_IPU);
}
//CANFD设备初始化
void can_fd_device_Init(void){
	//重置设备
	drv_can_fd_reset();
	//开启ECC
	drv_can_fd_ecc_enable();
	//重置RAM
	drv_can_fd_ramInit(0xFF);
	
	//配置控制寄存器
	drv_can_fd_controlConfig_objReset(&double_data_setting.control_config);
	//使用ISO标志CRC16
	double_data_setting.control_config.IsoCrcEnable = ISO_CRC;
	double_data_setting.control_config.StoreInTEF = 0;
	drv_can_fd_controlConfig(&double_data_setting.control_config);
	
	//设置TX FIFO
	drv_can_fd_TXfifoConfig_objReset(&double_data_setting.txConfig.tx_fifo_config);
	double_data_setting.txConfig.tx_fifo_config.FifoSize = 7;
	double_data_setting.txConfig.tx_fifo_config.PayLoadSize = CAN_PLSIZE_64;
	drv_can_fd_TXfifoConfig(APP_TX_FIFO,&double_data_setting.txConfig.tx_fifo_config);
	
	//设置RX FIFO
	drv_can_fd_rxChannelConfig_objReset(&double_data_setting.rxConfig.rx_fifo_config);
	double_data_setting.rxConfig.rx_fifo_config.FifoSize = 15;
	double_data_setting.rxConfig.rx_fifo_config.PayLoadSize = CAN_PLSIZE_64;
	drv_can_fd_rxChannelConfig(APP_RX_FIFO,&double_data_setting.rxConfig.rx_fifo_config);
	
	//设置ID过滤器
	double_data_setting.rxConfig.filter_config.SID = 0xDA;
	double_data_setting.rxConfig.filter_config.EXIDE = 0;
	double_data_setting.rxConfig.filter_config.EID = 0x00;
	drv_can_fd_filterObjConfig(CAN_FILTER0,&double_data_setting.rxConfig.filter_config);

	//设置ID屏蔽位
	double_data_setting.rxConfig.mask_config.MSID = 0x0;
	double_data_setting.rxConfig.mask_config.MIDE = 1;
	double_data_setting.rxConfig.mask_config.MEID = 0x0;
	drv_can_fd_filterMaskConfig(CAN_FILTER0,&double_data_setting.rxConfig.mask_config);
	
	//FIFO搭载过滤器
	drv_can_fd_filter_fifoLink(CAN_FILTER0,APP_RX_FIFO,true);
	
	//设置位时间
	drv_can_fd_bitTime_config(CAN_SPEED,CAN_SSP_MODE_AUTO,CAN_SYSCLK_40M);
	
	//设置GPIO0和GPIO1模式
	drv_can_fd_gpio_modeConfig(GPIO_MODE_INT,GPIO_MODE_INT);
	//TX FIFO事件
	drv_can_fd_txChannel_evtEnable(APP_TX_FIFO,CAN_TX_FIFO_NOT_FULL_EVENT);
	//RX FIFO事件
	drv_can_fd_rxChannel_evtEnable(APP_RX_FIFO,CAN_RX_FIFO_NOT_EMPTY_EVENT);
	//开启TX和RX事件
	drv_can_fd_modEvtEnable(CAN_TX_RX_EVENT_ALL_SET);
	
	//CANFD模式
	drv_can_fd_controlConfig_modeSelect(CAN_NORMAL_MODE);
	
	//获取MCP2517FD当前工作模式
	vTaskDelay(1);
	double_data_setting.control_mode = drv_can_fd_controlConfig_modeGet();
}

//发送
void transimitTask(can_fd_transmitStruct_t *tx_msg){
	uint8_t attempts = MAX_TXQUEUE_ATTEMPTS;
	//发送FIFO不为满就可以填充数据到FIFO
	do{
		drv_can_fd_txChannel_evtGet(APP_TX_FIFO,&double_data_setting.txConfig.tx_fifo_evt);
		if(attempts == 0){
            __nop();
            __nop();
            drv_can_fd_errorCount_stateGet(&double_data_setting.error_master.tec,
											&double_data_setting.error_master.rec,
											&double_data_setting.error_master.error_status);
            return;
        }
        attempts--;
	}while(!(double_data_setting.txConfig.tx_fifo_evt & CAN_TX_FIFO_NOT_FULL_EVENT));
	
	uint8_t num_byte = drv_can_fd_dlcToDataBytes((CAN_DLC)tx_msg->mod.bitSet.ctrl.DLC);
	//装填数据到TX FIFO
	drv_can_fd_txChannelLoad(APP_TX_FIFO,&tx_msg->mod,tx_msg->data,num_byte,true);
	digitalIncreasing(&tx_msg->cntr);
}

//仲裁段准备
can_fd_transmitStruct_t *arbitration_prepared(can_fd_transmitStruct_t *tx_msg,uint16_t standard_ID,CAN_DLC data_length_control){
	tx_msg->mod.word[0] = 0;
	tx_msg->mod.word[1] = 0;
	
	tx_msg->mod.bitSet.id.SID = standard_ID;
	tx_msg->mod.bitSet.id.EID = 0;
	
	tx_msg->mod.bitSet.ctrl.BRS = 1;
	tx_msg->mod.bitSet.ctrl.DLC = data_length_control;
	tx_msg->mod.bitSet.ctrl.FDF = 1;
	tx_msg->mod.bitSet.ctrl.IDE = 0;
	
	return tx_msg;
}

//数据段填充
static void sendTask(can_fd_transmitStruct_t *tx_msg){
	memset(tx_msg->data,0,sizeof(uint8_t) * 64);
	switch(tx_msg->mod.bitSet.id.SID){
		 case TX_RESPONSE_ID_GIMBAL :{
			 controlSerial.Send(tx_msg->data);
			break;
		}
		case TX_RESPONSE_ID_CHASSIS:{
			chassisSerial.Send(tx_msg->data);
			break;
		}
	}
	//填充完成后开始发送
	transimitTask(tx_msg);
}

static void receiveTask(can_fd_receiveStruct_t *rx_msg){
	uint8_t midMode;
#ifdef APP_USE_RX_INT
	if(!CAN_FD_RX_INT){
#else
	drv_can_fd_rxChannel_evtGet(APP_RX_FIFO,&double_data_setting.rxConfig.rx_fifo_evt);
	if(double_data_setting.rxConfig.rx_fifo_evt & CAN_RX_FIFO_NOT_EMPTY_EVENT){
#endif
		drv_can_fd_rxMsg(APP_RX_FIFO,&rx_msg->mod,rx_msg->data,MAX_DATA_BYTES);
		drv_can_fd_rxChannel_evtOverflowClear(APP_RX_FIFO);
		switch(rx_msg->mod.bitSet.id.SID){
			case RX_RESPONSE_ID_GIMBAL :{
				controlSerial.Upload(rx_msg->data);
				break;
			}
			case RX_RESPONSE_ID_CHASSIS:{
				if(robotMode == MODE_INIT)	midMode = MODE_RELAX;
				else midMode = robotMode;
				chassisSerial.Upload(rx_msg->data);	
				if(midMode != robotMode) lastRobotMode = midMode;
				break;
			}
		}
		digitalIncreasing(&rx_msg->cntr);
	}
}

