#include "canfd.h"
#include "board.h"

/***************������***************/
CAN_TX_MSGOBJ txObj;
CAN_RX_MSGOBJ rxObj;
volatile uint32_t rx_concept = 0;
uint8_t txData[MAX_DATA_BYTES];
uint8_t rxData[MAX_DATA_BYTES];
uint8_t __rxData[MAX_DATA_BYTES];

/************************************/

can_fd_configStruct_t double_data_setting;
//��������
can_fd_transmitStruct_t can_fd_controlData;
//��������
can_fd_transmitStruct_t can_fd_chassisData;
//ת������
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
	//SPI��ʼ��
	BSP_SPI_Init(&canfd_control);
	//SPI������
	BSP_SPIx_SetSpeed(&canfd_control,SPI_BaudRatePrescaler_16);
	BSP_GPIO_Init(CAN_FD_INT1,GPIO_Mode_IPU);
}
//CANFD�豸��ʼ��
void can_fd_device_Init(void){
	//�����豸
	drv_can_fd_reset();
	//����ECC
	drv_can_fd_ecc_enable();
	//����RAM
	drv_can_fd_ramInit(0xFF);
	
	//���ÿ��ƼĴ���
	drv_can_fd_controlConfig_objReset(&double_data_setting.control_config);
	//ʹ��ISO��־CRC16
	double_data_setting.control_config.IsoCrcEnable = ISO_CRC;
	double_data_setting.control_config.StoreInTEF = 0;
	drv_can_fd_controlConfig(&double_data_setting.control_config);
	
	//����TX FIFO
	drv_can_fd_TXfifoConfig_objReset(&double_data_setting.txConfig.tx_fifo_config);
	double_data_setting.txConfig.tx_fifo_config.FifoSize = 7;
	double_data_setting.txConfig.tx_fifo_config.PayLoadSize = CAN_PLSIZE_64;
	drv_can_fd_TXfifoConfig(APP_TX_FIFO,&double_data_setting.txConfig.tx_fifo_config);
	
	//����RX FIFO
	drv_can_fd_rxChannelConfig_objReset(&double_data_setting.rxConfig.rx_fifo_config);
	double_data_setting.rxConfig.rx_fifo_config.FifoSize = 15;
	double_data_setting.rxConfig.rx_fifo_config.PayLoadSize = CAN_PLSIZE_64;
	drv_can_fd_rxChannelConfig(APP_RX_FIFO,&double_data_setting.rxConfig.rx_fifo_config);
	
	//����ID������
	double_data_setting.rxConfig.filter_config.SID = 0xDA;
	double_data_setting.rxConfig.filter_config.EXIDE = 0;
	double_data_setting.rxConfig.filter_config.EID = 0x00;
	drv_can_fd_filterObjConfig(CAN_FILTER0,&double_data_setting.rxConfig.filter_config);

	//����ID����λ
	double_data_setting.rxConfig.mask_config.MSID = 0x0;
	double_data_setting.rxConfig.mask_config.MIDE = 1;
	double_data_setting.rxConfig.mask_config.MEID = 0x0;
	drv_can_fd_filterMaskConfig(CAN_FILTER0,&double_data_setting.rxConfig.mask_config);
	
	//FIFO���ع�����
	drv_can_fd_filter_fifoLink(CAN_FILTER0,APP_RX_FIFO,true);
	
	//����λʱ��
	drv_can_fd_bitTime_config(CAN_SPEED,CAN_SSP_MODE_AUTO,CAN_SYSCLK_40M);
	
	//����GPIO0��GPIO1ģʽ
	drv_can_fd_gpio_modeConfig(GPIO_MODE_INT,GPIO_MODE_INT);
	//TX FIFO�¼�
	drv_can_fd_txChannel_evtEnable(APP_TX_FIFO,CAN_TX_FIFO_NOT_FULL_EVENT);
	//RX FIFO�¼�
	drv_can_fd_rxChannel_evtEnable(APP_RX_FIFO,CAN_RX_FIFO_NOT_EMPTY_EVENT);
	//����TX��RX�¼�
	drv_can_fd_modEvtEnable(CAN_TX_RX_EVENT_ALL_SET);
	
	//CANFDģʽ
	drv_can_fd_controlConfig_modeSelect(CAN_NORMAL_MODE);
	
	//��ȡMCP2517FD��ǰ����ģʽ
	vTaskDelay(1);
	double_data_setting.control_mode = drv_can_fd_controlConfig_modeGet();
}

//����
void transimitTask(can_fd_transmitStruct_t *tx_msg){
	uint8_t attempts = MAX_TXQUEUE_ATTEMPTS;
	//����FIFO��Ϊ���Ϳ���������ݵ�FIFO
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
	//װ�����ݵ�TX FIFO
	drv_can_fd_txChannelLoad(APP_TX_FIFO,&tx_msg->mod,tx_msg->data,num_byte,true);
	digitalIncreasing(&tx_msg->cntr);
}

//�ٲö�׼��
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

//���ݶ����
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
	//�����ɺ�ʼ����
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

