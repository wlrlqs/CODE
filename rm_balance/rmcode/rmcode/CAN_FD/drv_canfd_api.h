#ifndef __DRV_CANFD_API_H
#define __DRV_CANFD_API_H

#include "bsp.h"
#include <stdbool.h>

#define CAN_FD_SPI_DEFAULT \
{\
	.SPIx = SPI4,\
	.SPI_NSS = BSP_GPIOE4,\
	.SPI_SCK = BSP_GPIOE2,\
	.SPI_MISO = BSP_GPIOE5,\
	.SPI_MOSI = BSP_GPIOE6\
}
//MCP2517FD�ж�
#define CAN_FD_INT  BSP_GPIOB4
//MCP2517FD TX�ж�
#define CAN_FD_INT0 BSP_GPIOD6
//MCP2517FD RX�ж�
#define CAN_FD_INT1 BSP_GPIOD7
#define TX_INT CAN_FD_INT0
#define RX_INT CAN_FD_INT1
//MCP2517FDƬѡ
#define CAN_FD_CS   PEout(4)
#define CAN_FD_RX_INT PDin(7)
#define LSB_SENDING
// Revision
//#define REV_A
#define REV_B

// Select ISO/non-ISO CRC
#define ISO_CRC 1

// Before B0 address of filter registers was fixed
#ifdef REV_A
#define FIXED_FILTER_ADDRESS
#endif

// Number of implemented FIFOs
#ifndef FPGA
#define CAN_FIFO_08TO15_IMPLEMENTED
#define CAN_FIFO_16TO31_IMPLEMENTED
#endif

// Number of implemented Filters
#ifndef FPGA
#define CAN_FILTER_08TO15_IMPLEMENTED
#define CAN_FILTER_16TO31_IMPLEMENTED
#endif

// Internal oscillator implemented
#ifdef MCP2520FD
#define CAN_INTERNAL_OSC_PRESENT
#endif

// Restricted Operation Mode implemented
#ifdef REV_B
#define CAN_RESTRICTED_MODE_PRESENT
#endif

// Transmit Queue
#ifdef REV_B
#define CAN_TXQUEUE_IMPLEMENTED
#endif

// Up to A1 silicon we had to multiply user address by 4.
#ifdef REV_A
#define USERADDRESS_TIMES_FOUR
#endif

// Maximum Size of TX/RX Object
#define MAX_MSG_SIZE 76

// Maximum number of data bytes in message
#define MAX_DATA_BYTES 64

#define DEFAULT_SPI_BUFF_SIZE 96
//MCP2517FD����ģʽ
typedef enum {
    CAN_NORMAL_MODE = 0x00,//��׼ģʽ,֧��CANFD��CAN2.0
    CAN_SLEEP_MODE = 0x01,//����ģʽ
    CAN_INTERNAL_LOOPBACK_MODE = 0x02,//�ڲ�����ģʽ
    CAN_LISTEN_ONLY_MODE = 0x03,//ֻ��ģʽ
    CAN_CONFIGURATION_MODE = 0x04,//����ģʽ
    CAN_EXTERNAL_LOOPBACK_MODE = 0x05,//�ⲿ����ģʽ
    CAN_CLASSIC_MODE = 0x06,//����ģʽ,֧��CAN2.0
    CAN_RESTRICTED_MODE = 0x07,//���޹���ģʽ
    CAN_INVALID_MODE = 0xFF//����ģʽ
}CAN_OPERATION_MODE;

//MCP2517FD FIFOͨ��
typedef enum {
    CAN_FIFO_CH0,
    CAN_FIFO_CH1,
    CAN_FIFO_CH2,
    CAN_FIFO_CH3,
    CAN_FIFO_CH4,
    CAN_FIFO_CH5,
    CAN_FIFO_CH6,
    CAN_FIFO_CH7,
#ifdef CAN_FIFO_08TO15_IMPLEMENTED
    CAN_FIFO_CH8,
    CAN_FIFO_CH9,
    CAN_FIFO_CH10,
    CAN_FIFO_CH11,
    CAN_FIFO_CH12,
    CAN_FIFO_CH13,
    CAN_FIFO_CH14,
    CAN_FIFO_CH15,
#endif
#ifdef CAN_FIFO_16TO31_IMPLEMENTED
    CAN_FIFO_CH16,
    CAN_FIFO_CH17,
    CAN_FIFO_CH18,
    CAN_FIFO_CH19,
    CAN_FIFO_CH20,
    CAN_FIFO_CH21,
    CAN_FIFO_CH22,
    CAN_FIFO_CH23,
    CAN_FIFO_CH24,
    CAN_FIFO_CH25,
    CAN_FIFO_CH26,
    CAN_FIFO_CH27,
    CAN_FIFO_CH28,
    CAN_FIFO_CH29,
    CAN_FIFO_CH30,
    CAN_FIFO_CH31,
#endif
    CAN_FIFO_TOTAL_CHANNELS
}CAN_FIFO_CHANNEL;

#ifdef CAN_TXQUEUE_IMPLEMENTED
#define CAN_FIFO_FIRST_CHANNEL  CAN_FIFO_CH1
#define CAN_TXQUEUE_CH0         CAN_FIFO_CH0
#else
#define CAN_FIFO_FIRST_CHANNEL  CAN_FIFO_CH0
#endif

#define N_MCP2517_CTRL_REGS 5
static uint32_t mcp2517ControlResetValues[] = {
    0x00000460, 0x00000003, 0x00000000, 0x00000000, 0x00000000
};

//MCP2517FD ����֡����
typedef enum {
    CAN_DLC_0,
    CAN_DLC_1,
    CAN_DLC_2,
    CAN_DLC_3,
    CAN_DLC_4,
    CAN_DLC_5,
    CAN_DLC_6,
    CAN_DLC_7,
    CAN_DLC_8,
    CAN_DLC_12,
    CAN_DLC_16,
    CAN_DLC_20,
    CAN_DLC_24,
    CAN_DLC_32,
    CAN_DLC_48,
    CAN_DLC_64
}CAN_DLC;

//MCP2517FD TX FIFO״̬
typedef enum {
    CAN_TX_FIFO_FULL = 0,
    CAN_TX_FIFO_STATUS_MASK = 0x1F7,
    CAN_TX_FIFO_NOT_FULL = 0x01,
    CAN_TX_FIFO_HALF_FULL = 0x02,
    CAN_TX_FIFO_EMPTY = 0x04,
    CAN_TX_FIFO_ATTEMPTS_EXHAUSTED = 0x10,
    CAN_TX_FIFO_ERROR = 0x20,
    CAN_TX_FIFO_ARBITRATION_LOST = 0x40,
    CAN_TX_FIFO_ABORTED = 0x80,
    CAN_TX_FIFO_TRANSMITTING = 0x100
}CAN_TX_FIFO_STATUS;

//CAN��������ͨ��
#define CAN_TXREQ_CH0  0x00000001
#define CAN_TXREQ_CH1  0x00000002
#define CAN_TXREQ_CH2  0x00000004
#define CAN_TXREQ_CH3  0x00000008
#define CAN_TXREQ_CH4  0x00000010
#define CAN_TXREQ_CH5  0x00000020
#define CAN_TXREQ_CH6  0x00000040
#define CAN_TXREQ_CH7  0x00000080
#define CAN_TXREQ_CH8  0x00000100
#define CAN_TXREQ_CH9  0x00000200
#define CAN_TXREQ_CH10 0x00000400
#define CAN_TXREQ_CH11 0x00000800
#define CAN_TXREQ_CH12 0x00001000
#define CAN_TXREQ_CH13 0x00002000
#define CAN_TXREQ_CH14 0x00004000
#define CAN_TXREQ_CH15 0x00008000
#define CAN_TXREQ_CH16 0x00010000
#define CAN_TXREQ_CH17 0x00020000
#define CAN_TXREQ_CH18 0x00040000
#define CAN_TXREQ_CH19 0x00080000
#define CAN_TXREQ_CH20 0x00100000
#define CAN_TXREQ_CH21 0x00200000
#define CAN_TXREQ_CH22 0x00400000
#define CAN_TXREQ_CH23 0x00800000
#define CAN_TXREQ_CH24 0x01000000
#define CAN_TXREQ_CH25 0x02000000
#define CAN_TXREQ_CH26 0x04000000
#define CAN_TXREQ_CH27 0x08000000
#define CAN_TXREQ_CH28 0x10000000
#define CAN_TXREQ_CH29 0x20000000
#define CAN_TXREQ_CH30 0x40000000
#define CAN_TXREQ_CH31 0x80000000

//CAN���Ĺ�����
typedef enum {
    CAN_FILTER0,
    CAN_FILTER1,
    CAN_FILTER2,
    CAN_FILTER3,
    CAN_FILTER4,
    CAN_FILTER5,
    CAN_FILTER6,
    CAN_FILTER7,
#ifdef CAN_FILTER_08TO15_IMPLEMENTED
    CAN_FILTER8,
    CAN_FILTER9,
    CAN_FILTER10,
    CAN_FILTER11,
    CAN_FILTER12,
    CAN_FILTER13,
    CAN_FILTER14,
    CAN_FILTER15,
#endif
#ifdef CAN_FILTER_16TO31_IMPLEMENTED
    CAN_FILTER16,
    CAN_FILTER17,
    CAN_FILTER18,
    CAN_FILTER19,
    CAN_FILTER20,
    CAN_FILTER21,
    CAN_FILTER22,
    CAN_FILTER23,
    CAN_FILTER24,
    CAN_FILTER25,
    CAN_FILTER26,
    CAN_FILTER27,
    CAN_FILTER28,
    CAN_FILTER29,
    CAN_FILTER30,
    CAN_FILTER31,
#endif
    CAN_FILTER_TOTAL,
}CAN_FILTER;

//CAN���ô���
typedef enum {
    CAN_TXBWS_NO_DELAY,
    CAN_TXBWS_2,
    CAN_TXBWS_4,
    CAN_TXBWS_8,
    CAN_TXBWS_16,
    CAN_TXBWS_32,
    CAN_TXBWS_64,
    CAN_TXBWS_128,
    CAN_TXBWS_256,
    CAN_TXBWS_512,
    CAN_TXBWS_1024,
    CAN_TXBWS_2048,
    CAN_TXBWS_4096
}CAN_TX_BANDWITH_SHARING;

//CAN�豸��������С
typedef enum {
    CAN_DNET_FILTER_DISABLE = 0,
    CAN_DNET_FILTER_SIZE_1_BIT,
    CAN_DNET_FILTER_SIZE_2_BIT,
    CAN_DNET_FILTER_SIZE_3_BIT,
    CAN_DNET_FILTER_SIZE_4_BIT,
    CAN_DNET_FILTER_SIZE_5_BIT,
    CAN_DNET_FILTER_SIZE_6_BIT,
    CAN_DNET_FILTER_SIZE_7_BIT,
    CAN_DNET_FILTER_SIZE_8_BIT,
    CAN_DNET_FILTER_SIZE_9_BIT,
    CAN_DNET_FILTER_SIZE_10_BIT,
    CAN_DNET_FILTER_SIZE_11_BIT,
    CAN_DNET_FILTER_SIZE_12_BIT,
    CAN_DNET_FILTER_SIZE_13_BIT,
    CAN_DNET_FILTER_SIZE_14_BIT,
    CAN_DNET_FILTER_SIZE_15_BIT,
    CAN_DNET_FILTER_SIZE_16_BIT,
    CAN_DNET_FILTER_SIZE_17_BIT,
    CAN_DNET_FILTER_SIZE_18_BIT
}CAN_DNET_FILTER_SIZE;

//CAN����ͨ��״̬
typedef enum {
    CAN_RX_FIFO_EMPTY = 0,
    CAN_RX_FIFO_STATUS_MASK = 0x0F,
    CAN_RX_FIFO_NOT_EMPTY = 0x01,
    CAN_RX_FIFO_HALF_FULL = 0x02,
    CAN_RX_FIFO_FULL = 0x04,
    CAN_RX_FIFO_OVERFLOW = 0x08
}CAN_RX_FIFO_STATUS;

//CAN�����¼�FIFO״̬
typedef enum {
    CAN_TEF_FIFO_EMPTY = 0,
    CAN_TEF_FIFO_STATUS_MASK = 0x0F,
    CAN_TEF_FIFO_NOT_EMPTY = 0x01,
    CAN_TEF_FIFO_HALF_FULL = 0x02,
    CAN_TEF_FIFO_FULL = 0x04,
    CAN_TEF_FIFO_OVERFLOW = 0x08
}CAN_TEF_FIFO_STATUS;

//CAN RX CODE
typedef enum {
    CAN_RXCODE_FIFO_CH0,
    CAN_RXCODE_FIFO_CH1,
    CAN_RXCODE_FIFO_CH2,
    CAN_RXCODE_FIFO_CH3,
    CAN_RXCODE_FIFO_CH4,
    CAN_RXCODE_FIFO_CH5,
    CAN_RXCODE_FIFO_CH6,
    CAN_RXCODE_FIFO_CH7,
#ifdef CAN_FIFO_08TO15_IMPLEMENTED
    CAN_RXCODE_FIFO_CH8,
    CAN_RXCODE_FIFO_CH9,
    CAN_RXCODE_FIFO_CH10,
    CAN_RXCODE_FIFO_CH11,
    CAN_RXCODE_FIFO_CH12,
    CAN_RXCODE_FIFO_CH13,
    CAN_RXCODE_FIFO_CH14,
    CAN_RXCODE_FIFO_CH15,
#endif
#ifdef CAN_FIFO_16TO31_IMPLEMENTED
    CAN_RXCODE_FIFO_CH16,
    CAN_RXCODE_FIFO_CH17,
    CAN_RXCODE_FIFO_CH18,
    CAN_RXCODE_FIFO_CH19,
    CAN_RXCODE_FIFO_CH20,
    CAN_RXCODE_FIFO_CH21,
    CAN_RXCODE_FIFO_CH22,
    CAN_RXCODE_FIFO_CH23,
    CAN_RXCODE_FIFO_CH24,
    CAN_RXCODE_FIFO_CH25,
    CAN_RXCODE_FIFO_CH26,
    CAN_RXCODE_FIFO_CH27,
    CAN_RXCODE_FIFO_CH28,
    CAN_RXCODE_FIFO_CH29,
    CAN_RXCODE_FIFO_CH30,
    CAN_RXCODE_FIFO_CH31,
#endif
    CAN_RXCODE_TOTAL_CHANNELS,
    CAN_RXCODE_NO_INT = 0x40,
    CAN_RXCODE_RESERVED
}CAN_RXCODE;

//CAN TX CODE
typedef enum {
    CAN_TXCODE_FIFO_CH0,
    CAN_TXCODE_FIFO_CH1,
    CAN_TXCODE_FIFO_CH2,
    CAN_TXCODE_FIFO_CH3,
    CAN_TXCODE_FIFO_CH4,
    CAN_TXCODE_FIFO_CH5,
    CAN_TXCODE_FIFO_CH6,
    CAN_TXCODE_FIFO_CH7,
#ifdef CAN_FIFO_08TO15_IMPLEMENTED
    CAN_TXCODE_FIFO_CH8,
    CAN_TXCODE_FIFO_CH9,
    CAN_TXCODE_FIFO_CH10,
    CAN_TXCODE_FIFO_CH11,
    CAN_TXCODE_FIFO_CH12,
    CAN_TXCODE_FIFO_CH13,
    CAN_TXCODE_FIFO_CH14,
    CAN_TXCODE_FIFO_CH15,
#endif
#ifdef CAN_FIFO_16TO31_IMPLEMENTED
    CAN_TXCODE_FIFO_CH16,
    CAN_TXCODE_FIFO_CH17,
    CAN_TXCODE_FIFO_CH18,
    CAN_TXCODE_FIFO_CH19,
    CAN_TXCODE_FIFO_CH20,
    CAN_TXCODE_FIFO_CH21,
    CAN_TXCODE_FIFO_CH22,
    CAN_TXCODE_FIFO_CH23,
    CAN_TXCODE_FIFO_CH24,
    CAN_TXCODE_FIFO_CH25,
    CAN_TXCODE_FIFO_CH26,
    CAN_TXCODE_FIFO_CH27,
    CAN_TXCODE_FIFO_CH28,
    CAN_TXCODE_FIFO_CH29,
    CAN_TXCODE_FIFO_CH30,
    CAN_TXCODE_FIFO_CH31,
#endif
    CAN_TXCODE_TOTAL_CHANNELS,
    CAN_TXCODE_NO_INT = 0x40,
    CAN_TXCODE_RESERVED
}CAN_TXCODE;

//CAN I CODE
typedef enum {
    CAN_ICODE_FIFO_CH0,
    CAN_ICODE_FIFO_CH1,
    CAN_ICODE_FIFO_CH2,
    CAN_ICODE_FIFO_CH3,
    CAN_ICODE_FIFO_CH4,
    CAN_ICODE_FIFO_CH5,
    CAN_ICODE_FIFO_CH6,
    CAN_ICODE_FIFO_CH7,
#ifdef CAN_FIFO_08TO15_IMPLEMENTED
    CAN_ICODE_FIFO_CH8,
    CAN_ICODE_FIFO_CH9,
    CAN_ICODE_FIFO_CH10,
    CAN_ICODE_FIFO_CH11,
    CAN_ICODE_FIFO_CH12,
    CAN_ICODE_FIFO_CH13,
    CAN_ICODE_FIFO_CH14,
    CAN_ICODE_FIFO_CH15,
#endif
#ifdef CAN_FIFO_16TO31_IMPLEMENTED
    CAN_ICODE_FIFO_CH16,
    CAN_ICODE_FIFO_CH17,
    CAN_ICODE_FIFO_CH18,
    CAN_ICODE_FIFO_CH19,
    CAN_ICODE_FIFO_CH20,
    CAN_ICODE_FIFO_CH21,
    CAN_ICODE_FIFO_CH22,
    CAN_ICODE_FIFO_CH23,
    CAN_ICODE_FIFO_CH24,
    CAN_ICODE_FIFO_CH25,
    CAN_ICODE_FIFO_CH26,
    CAN_ICODE_FIFO_CH27,
    CAN_ICODE_FIFO_CH28,
    CAN_ICODE_FIFO_CH29,
    CAN_ICODE_FIFO_CH30,
    CAN_ICODE_FIFO_CH31,
#endif
    CAN_ICODE_TOTAL_CHANNELS,
    CAN_ICODE_NO_INT = 0x40,
    CAN_ICODE_CERRIF,
    CAN_ICODE_WAKIF,
    CAN_ICODE_RXOVIF,
    CAN_ICODE_ADDRERR_SERRIF,
    CAN_ICODE_MABOV_SERRIF,
    CAN_ICODE_TBCIF,
    CAN_ICODE_MODIF,
    CAN_ICODE_IVMIF,
    CAN_ICODE_TEFIF,
    CAN_ICODE_TXATIF,
    CAN_ICODE_RESERVED
}CAN_ICODE;

//CAN����FIFO�¼�
typedef enum {
    CAN_TX_FIFO_NO_EVENT = 0,
    CAN_TX_FIFO_ALL_EVENTS = 0x17,
    CAN_TX_FIFO_NOT_FULL_EVENT = 0x01,
    CAN_TX_FIFO_HALF_FULL_EVENT = 0x02,
    CAN_TX_FIFO_EMPTY_EVENT = 0x04,
    CAN_TX_FIFO_ATTEMPTS_EXHAUSTED_EVENT = 0x10
}CAN_TX_FIFO_EVENT;

//CAN����FIFO�¼�
typedef enum {
    CAN_RX_FIFO_NO_EVENT = 0,
    CAN_RX_FIFO_ALL_EVENTS = 0x0F,
    CAN_RX_FIFO_NOT_EMPTY_EVENT = 0x01,
    CAN_RX_FIFO_HALF_FULL_EVENT = 0x02,
    CAN_RX_FIFO_FULL_EVENT = 0x04,
    CAN_RX_FIFO_OVERFLOW_EVENT = 0x08
}CAN_RX_FIFO_EVENT;

//CAN�����¼�FIFO�¼�
typedef enum {
    CAN_TEF_FIFO_NO_EVENT = 0,
    CAN_TEF_FIFO_ALL_EVENTS = 0x0F,
    CAN_TEF_FIFO_NOT_EMPTY_EVENT = 0x01,
    CAN_TEF_FIFO_HALF_FULL_EVENT = 0x02,
    CAN_TEF_FIFO_FULL_EVENT = 0x04,
    CAN_TEF_FIFO_OVERFLOW_EVENT = 0x08
}CAN_TEF_FIFO_EVENT;

//CANģ���¼�
typedef enum {
    CAN_NO_EVENT = 0,
    CAN_ALL_EVENTS = 0xFF1F,
    CAN_TX_EVENT = 0x0001,
    CAN_RX_EVENT = 0x0002,
	CAN_TX_RX_EVENT_ALL_SET = 0x0003,
    CAN_TIME_BASE_COUNTER_EVENT = 0x0004,
    CAN_OPERATION_MODE_CHANGE_EVENT = 0x0008,
    CAN_TEF_EVENT = 0x0010,

    CAN_RAM_ECC_EVENT = 0x0100,
    CAN_SPI_CRC_EVENT = 0x0200,
    CAN_TX_ATTEMPTS_EVENT = 0x0400,
    CAN_RX_OVERFLOW_EVENT = 0x0800,
    CAN_SYSTEM_ERROR_EVENT = 0x1000,
    CAN_BUS_ERROR_EVENT = 0x2000,
    CAN_BUS_WAKEUP_EVENT = 0x4000,
    CAN_RX_INVALID_MESSAGE_EVENT = 0x8000
}CAN_MODULE_EVENT;

//CAN����״̬
typedef enum {
    CAN_ERROR_FREE_STATE = 0,
    CAN_ERROR_ALL = 0x3F,
    CAN_TX_RX_WARNING_STATE = 0x01,
    CAN_RX_WARNING_STATE = 0x02,
    CAN_TX_WARNING_STATE = 0x04,
    CAN_RX_BUS_PASSIVE_STATE = 0x08,
    CAN_TX_BUS_PASSIVE_STATE = 0x10,
    CAN_TX_BUS_OFF_STATE = 0x20
}CAN_ERROR_STATE;

//CAN���ݸ���
typedef enum {
    CAN_PLSIZE_8,
    CAN_PLSIZE_12,
    CAN_PLSIZE_16,
    CAN_PLSIZE_20,
    CAN_PLSIZE_24,
    CAN_PLSIZE_32,
    CAN_PLSIZE_48,
    CAN_PLSIZE_64
}CAN_FIFO_PLSIZE;

//CANϵͳʱ��
typedef enum {
    CAN_SYSCLK_40M,
    CAN_SYSCLK_20M,
    CAN_SYSCLK_10M
}CAN_SYSCLK_SPEED;

//CAN���β�����ģʽ
typedef enum {
    CAN_SSP_MODE_OFF,
    CAN_SSP_MODE_MANUAL,
    CAN_SSP_MODE_AUTO
}CAN_SSP_MODE;

//CANλʱ������(������)
typedef enum {
    CAN_500K_1M,    // 0x00
    CAN_500K_2M,    // 0x01
    CAN_500K_3M,
    CAN_500K_4M,
    CAN_500K_5M,    // 0x04
    CAN_500K_6M7,
    CAN_500K_8M,    // 0x06
    CAN_500K_10M,
    CAN_250K_500K,  // 0x08
    CAN_250K_833K,
    CAN_250K_1M,
    CAN_250K_1M5,
    CAN_250K_2M,
    CAN_250K_3M,
    CAN_250K_4M,
    CAN_1000K_4M,   // 0x0f
    CAN_1000K_8M,
    CAN_125K_500K   // 0x11
}CAN_BITTIME_SETUP;

//GPIOģʽѡ��
typedef enum {
    GPIO_MODE_INT,
    GPIO_MODE_GPIO
}GPIO_PIN_MODE;

//CAN���ߴ������
typedef struct _CAN_BUS_ERROR_COUNT {
    uint8_t NREC;
    uint8_t NTEC;
    uint8_t DREC;
    uint8_t DTEC;
}CAN_BUS_ERROR_COUNT;
//CAN���ߴ����־��
typedef struct _CAN_BUS_DIAG_FLAGS {
    uint32_t NBIT0_ERR : 1;
    uint32_t NBIT1_ERR : 1;
    uint32_t NACK_ERR : 1;
    uint32_t NFORM_ERR : 1;
    uint32_t NSTUFF_ERR : 1;
    uint32_t NCRC_ERR : 1;
    uint32_t unimplemented1 : 1;
    uint32_t TXBO_ERR : 1;
    uint32_t DBIT0_ERR : 1;
    uint32_t DBIT1_ERR : 1;
    uint32_t unimplemented2 : 1;
    uint32_t DFORM_ERR : 1;
    uint32_t DSTUFF_ERR : 1;
    uint32_t DCRC_ERR : 1;
    uint32_t ESI : 1;
    uint32_t DLC_MISMATCH : 1;
}CAN_BUS_DIAG_FLAGS;
//CAN���ߴ������
typedef union _CAN_BUS_DIAGNOSTIC {
    struct {
        CAN_BUS_ERROR_COUNT errorCount;
        uint16_t errorFreeMsgCount;
        CAN_BUS_DIAG_FLAGS flag;
    }bitSet;
    uint32_t word[2];
    uint8_t byte[8];
}CAN_BUS_DIAGNOSTIC;
//CAN����
typedef struct _CAN_CONFIG {
    uint32_t DNetFilterCount : 5;//������ѡ��
    uint32_t IsoCrcEnable : 1;//CRC��׼ѡ��(ISO11898--1:2015�淶)
    uint32_t ProtocolExpectionEventDisable : 1;//Э���쳣�¼�����ֹλ
    uint32_t WakeUpFilterEnable : 1;//CAN������·�����˲���ʹ��λ
    uint32_t WakeUpFilterTime : 2;//�����˲���ʱ��
    uint32_t BitRateSwitchDisable : 1;//�������л���ֹλ
    uint32_t RestrictReTxAttempts : 1;//�����ط�����λ
    uint32_t EsiInGatewayMode : 1;//����ģʽ��ESIλ
    uint32_t SystemErrorToListenOnly : 1;//ϵͳ����ʱ�л�����ģʽ
    uint32_t StoreInTEF : 1;//�洢�����¼���FIFO
    uint32_t TXQEnable : 1;//ʹ�ܷ��Ͷ���
    uint32_t TxBandWidthSharing : 4;//���ʹ�����λ(���η�����ʱ)
}CAN_CONFIG;

//CAN����FIFO����
typedef struct _CAN_TX_FIFO_CONFIG {
    uint32_t RTREnable : 1;
    uint32_t TxPriority : 5;
    uint32_t TxAttempts : 2;
    uint32_t FifoSize : 5;
    uint32_t PayLoadSize : 3;
}CAN_TX_FIFO_CONFIG;

//CAN���Ͷ�������
typedef struct _CAN_TX_QUEUE_CONFIG {
    uint32_t TxPriority : 5;
    uint32_t TxAttempts : 2;
    uint32_t FifoSize : 5;
    uint32_t PayLoadSize : 3;
}CAN_TX_QUEUE_CONFIG;

//CAN��Ϣ����ID
typedef struct _CAN_MSGOBJ_ID {
    uint32_t SID : 11; //��׼ID
    uint32_t EID : 18; //��չID
    uint32_t SID11 : 1; 
    uint32_t unimplemented1 : 2;
}CAN_MSGOBJ_ID;

//CAN���ͱ��Ŀ���
typedef struct _CAN_TX_MSGOBJ_CTRL {
    uint32_t DLC : 4;
    uint32_t IDE : 1;
    uint32_t RTR : 1;
    uint32_t BRS : 1;
    uint32_t FDF : 1;
    uint32_t ESI : 1;
    uint32_t SEQ : 7;
    uint32_t unimplemented1 : 16;
}CAN_TX_MSGOBJ_CTRL;

//CAN��Ϣʱ���
typedef uint32_t CAN_MSG_TIMESTAMP;

//CAN������Ϣ����
typedef union _CAN_TX_MSGOBJ {
    struct {
        CAN_MSGOBJ_ID id;
        CAN_TX_MSGOBJ_CTRL ctrl;
        CAN_MSG_TIMESTAMP timeStamp;
    }bitSet;
    uint32_t word[3];
    uint8_t byte[12];
}CAN_TX_MSGOBJ;

//CAN����ID������
typedef struct _CAN_FILTEROBJ_ID {
    uint32_t SID : 11;
    uint32_t EID : 18;
    uint32_t SID11 : 1;
    uint32_t EXIDE : 1;
    uint32_t unimplemented1 : 1;
}CAN_FILTEROBJ_ID;

//CAN����ID������
typedef struct _CAN_MASKOBJ_ID {
    uint32_t MSID : 11;
    uint32_t MEID : 18;
    uint32_t MSID11 : 1;
    uint32_t MIDE : 1;
    uint32_t unimplemented1 : 1;
}CAN_MASKOBJ_ID;

//CAN����FIFO����
typedef struct _CAN_RX_FIFO_CONFIG {
    uint32_t RxTimeStampEnable : 1;
    uint32_t FifoSize : 5;
    uint32_t PayLoadSize : 3;
}CAN_RX_FIFO_CONFIG;

//CAN���ձ��Ŀ���
typedef struct _CAN_RX_MSGOBJ_CTRL {
    uint32_t DLC : 4;
    uint32_t IDE : 1;
    uint32_t RTR : 1;
    uint32_t BRS : 1;
    uint32_t FDF : 1;
    uint32_t ESI : 1;
    uint32_t unimplemented1 : 2;
    uint32_t FilterHit : 5;
    uint32_t unimplemented2 : 16;
}CAN_RX_MSGOBJ_CTRL;

//CAN������Ϣ����
typedef union _CAN_RX_MSGOBJ {
    struct {
        CAN_MSGOBJ_ID id;
        CAN_RX_MSGOBJ_CTRL ctrl;
        CAN_MSG_TIMESTAMP timeStamp;
    }bitSet;
    uint32_t word[3];
    uint8_t byte[12];
}CAN_RX_MSGOBJ;

//CAN�����¼�FIFO��Ϣ����
typedef union _CAN_TEF_MSGOBJ {
    struct {
        CAN_MSGOBJ_ID id;
        CAN_TX_MSGOBJ_CTRL ctrl;
        CAN_MSG_TIMESTAMP timeStamp;
    }bitSet;
    uint32_t word[3];
    uint8_t byte[12];
}CAN_TEF_MSGOBJ;

//CAN�����¼�FIFO����
typedef struct _CAN_TEF_CONFIG {
    uint32_t TimeStampEnable : 1;
    uint32_t FifoSize : 5;
}CAN_TEF_CONFIG;

//CAN��������
typedef struct _CAN_OSC_CTRL {
    uint32_t PllEnable : 1;
    uint32_t OscDisable : 1;
    uint32_t SclkDivide : 1;
    uint32_t ClkOutDivide : 2;
}CAN_OSC_CTRL;

//CAN����״̬
typedef struct _CAN_OSC_STATUS {
    uint32_t PllReady : 1;
    uint32_t OscReady : 1;
    uint32_t SclkReady : 1;
}CAN_OSC_STATUS;

/*CRC16������*/
#define CRC16_BASE 	0xFFFF
#define CRC16_UPPER 1
/*******CAN-FD CONTROL*******/
//MCP2517FD��λ
void drv_can_fd_reset(void);
/*******CAN-FD BYTE*******/
//MCP2517FDд��1�ֽ�
void drv_can_fd_spi_writeByte(uint16_t address,uint8_t data);
//MCP2517FD��ȡ1�ֽ�
uint8_t drv_can_fd_spi_readByte(uint16_t address);
//MCP2517FD��ȫд��1�ֽ�
void drv_can_fd_spi_writeByte_safe(uint16_t address,uint8_t data);
/*******CAN-FD WORD*******/
//MCP2517FDд��1����
void drv_can_fd_spi_writeWord(uint16_t address,uint32_t data);
//MCP2517FD��ȡ1����
uint32_t drv_can_fd_spi_readWord(uint16_t address);
//MCP2517FD��ȫд��1����
void drv_can_fd_spi_writeWord_safe(uint16_t address,uint32_t data);
/*******CAN-FD HALF WORD*******/
//MCP2517FDд������
void drv_can_fd_spi_writeHalfWord(uint16_t address,uint32_t data);
//MCP2517FD��ȡ�����
uint16_t drv_can_fd_spi_readHalfWord(uint16_t address);
/*******CAN-FD BYTE ARRAY*******/
//MCP2517FDд�벻�����ֽ�
void drv_can_fd_spi_writeByteArray(uint16_t address,uint8_t *transimit,uint16_t len);
//MCP2517FD��CRCУ���д�벻�����ֽ�
void drv_can_fd_spi_writeByteArrayCRC(uint16_t address,uint8_t *transimit,uint16_t len,bool fromRAM);
//MCP2517FD��ȡ�������ֽ�
void drv_can_fd_spi_readByteArray(uint16_t address,uint8_t *receive,uint16_t len);
//MCP2517FD��CRCУ���ȡ�������ֽ�
void drv_can_fd_spi_readByteArrayCRC(uint16_t address,uint8_t *receive,uint16_t len,bool fromRAM,bool *crc_error);
/*******CAN-FD WORD ARRAY*******/
//MCP2517FDд�벻������
void drv_can_fd_spi_writeWordArray(uint16_t address,uint32_t *transimit,uint16_t len);
//MCP2517FD��ȡ��������
void drv_can_fd_spi_readWordArray(uint16_t address,uint32_t *receive,uint16_t len);
/*******CAN-FD CONTROL CONFIG*******/
//����MCP2517FD���ƼĴ���
void drv_can_fd_controlConfig(CAN_CONFIG *configSetting);
//��ȡMCP2517FD���ƼĴ�����ʼ������
void drv_can_fd_controlConfig_objReset(CAN_CONFIG *configSetting);
//ѡ��MCP2517FD����ģʽ
void drv_can_fd_controlConfig_modeSelect(CAN_OPERATION_MODE opMode);
//��ȡMCP2517FD����ģʽ
CAN_OPERATION_MODE drv_can_fd_controlConfig_modeGet(void);
/*******CAN-FD TX CONFIG*******/
//����MCP2517FD����FIFO
void drv_can_fd_TXfifoConfig(CAN_FIFO_CHANNEL channel,CAN_TX_FIFO_CONFIG *configSetting);
//��ȡMCP2517FD����FIFO��ʼ������
void drv_can_fd_TXfifoConfig_objReset(CAN_TX_FIFO_CONFIG *configSetting);
//����MCP2517FD���Ͷ���
void drv_can_fd_TXqueueConfig(CAN_TX_QUEUE_CONFIG *configSetting);
//��ȡMCP2517FD���Ͷ��г�ʼ������
void drv_can_fd_TXqueueConfig_objReset(CAN_TX_QUEUE_CONFIG *configSetting);
//MCP2517FD����ͨ��װ������֡
void drv_can_fd_txChannelLoad(CAN_FIFO_CHANNEL channel,CAN_TX_MSGOBJ *txObj,uint8_t *txData,uint32_t txLen,bool flush);
//MCP2517FD����ͨ������
void drv_can_fd_txChannelUpdate(CAN_FIFO_CHANNEL channel,bool flush);
//��MCP2517FD����FIFO����
void drv_can_fd_txChannelFlush(CAN_FIFO_CHANNEL channel);
//��ȡMCP2517FD�ķ���FIFO״̬
void drv_can_fd_txChannelStatusGet(CAN_FIFO_CHANNEL channel,CAN_TX_FIFO_STATUS *status);
//MCP2517FD����ͨ������
void drv_can_fd_txChannelReset(CAN_FIFO_CHANNEL channel);
//����MCP2517FD��������
void drv_can_fd_txReqSet(uint32_t txreq);
//��ȡ��ǰMCP2517FD��������
void drv_can_fd_txReqGet(uint32_t *txreq);
//��ֹMCP2517FD����ͨ��
void drv_can_fd_txChannelAbort(CAN_FIFO_CHANNEL channel);
//��ֹ����MCP2517FD����ͨ��
void drv_can_fd_txChannelAbortAll(CAN_FIFO_CHANNEL channel);
//����MCP2517FD����
void drv_can_fd_txBandWidthSharingSet(CAN_TX_BANDWITH_SHARING txbws);
/*******CAN-FD RX CONFIG*******/
//����MCP2517FD ID���Ĺ�����
void drv_can_fd_filterObjConfig(CAN_FILTER filter,CAN_FILTEROBJ_ID *id);
//����MCP2517FD ����ID������
void drv_can_fd_filterMaskConfig(CAN_FILTER filter,CAN_MASKOBJ_ID *mask);
//MCP2517FD������ָ��FIFO
void drv_can_fd_filter_fifoLink(CAN_FILTER filter,CAN_FIFO_CHANNEL channel,bool enable);
//����MCP2517FD���Ĺ�����
void drv_can_fd_filterEnable(CAN_FILTER filter);
//�ر�MCP2517FD���Ĺ�����
void drv_can_fd_filterDisable(CAN_FILTER filter);
//����MCP2517FD�豸������
void drv_can_fd_DeviceNetFilterCountSet(CAN_DNET_FILTER_SIZE dnfc);
//��ȡ��ǰMCP2517FD����ͨ��״̬
void drv_can_fd_rxChannelStatusGet(CAN_FIFO_CHANNEL channel,CAN_RX_FIFO_STATUS *status);
//����MCP2517FD����ͨ��
void drv_can_fd_rxChannelConfig(CAN_FIFO_CHANNEL channel,CAN_RX_FIFO_CONFIG *configSetting);
//��ȡMCP2517FD����ͨ����ʼ������
void drv_can_fd_rxChannelConfig_objReset(CAN_RX_FIFO_CONFIG *configSetting);
//MCP2517FD������Ϣ
void drv_can_fd_rxMsg(CAN_FIFO_CHANNEL channel,CAN_RX_MSGOBJ *rxObj,uint8_t *rxData,uint8_t rxLen);
//MCP2517FD����ͨ������
void drv_can_fd_rxChannelUpdate(CAN_FIFO_CHANNEL channel);
//MCP2517FD����FIFO��λ
void drv_can_fd_rxChannelReset(CAN_FIFO_CHANNEL channel);
/*******CAN-FD EVENT FIFO*******/
//��ȡ��ǰMCP2517FD�����¼�FIFO״̬
void drv_can_fd_txEvtfifoStatusGet(CAN_TEF_FIFO_STATUS *status);
//MCP2517FD�����¼�FIFO��Ϣ
void drv_can_fd_txEvtfifoMsg(CAN_TEF_MSGOBJ *tefObj);
//MCP2517FD�����¼�FIFO��λ
void drv_can_fd_txEvtfifoReset(void);
//MCP2517FD�����¼�FIFO����
void drv_can_fd_txEvtfifoUpdate(void);
//����MCP2517FD�����¼�FIFO
void drv_can_fd_txEvtfifoConfig(CAN_TEF_CONFIG *configSetting);
//��ȡMCP2517FD�����¼�FIFO����
void drv_can_fd_txEvtConfig_objReset(CAN_TEF_CONFIG *configSetting);
/*******CAN-FD MODULE EVENT*******/
//��ȡMCP2517FD��ǰ�ж�����
void drv_can_fd_modEvtGet(CAN_MODULE_EVENT *flags);
//����MCP2517FD�ж�
void drv_can_fd_modEvtEnable(CAN_MODULE_EVENT flags);
//�ر�MCP2517FD�ж�
void drv_can_fd_modEvtDisable(CAN_MODULE_EVENT flags);
//���MCP2517FD�¼���־
void drv_can_fd_modEvtClear(CAN_MODULE_EVENT flags);
//��ȡMCP2517FD RX CODE
void drv_can_fd_modEvt_rxCodeGet(CAN_RXCODE *rxCode);
//��ȡMCP2517FD TX CODE
void drv_can_fd_modEvt_txCodeGet(CAN_TXCODE *txCode);
//��ȡMCP2517FDʹ�õĹ�����
void drv_can_fd_modEvt_filterHitGet(CAN_FILTER *filterHit);
//��ȡMCP2517FD I CODE
void drv_can_fd_modEvt_iCodeGet(CAN_ICODE *icode);
/*******CAN-FD TX FIFO EVENT*******/
//��ȡMCP2517FD����FIFO�ж�����
void drv_can_fd_txChannel_evtGet(CAN_FIFO_CHANNEL channel,CAN_TX_FIFO_EVENT *flags);
//��ȡ��ǰMCP2517FD�����ж�
void drv_can_fd_txEvtGet(uint32_t *txif);
//��ȡ��ǰMCP2517FD���ͳ����ж�
void drv_can_fd_txEvtAttemptGet(uint32_t *txatif);
//��ȡMCP2517FD����FIFOͨ����
void drv_can_fd_txChannel_indexGet(CAN_FIFO_CHANNEL channel,uint8_t *idx);
//ʹ��MCP2517FD����FIFO�ж�
void drv_can_fd_txChannel_evtEnable(CAN_FIFO_CHANNEL channel,CAN_TX_FIFO_EVENT flags);
//ʧ��MCP2517FD����FIFO�ж�
void drv_can_fd_txChannel_evtDisable(CAN_FIFO_CHANNEL channel,CAN_TX_FIFO_EVENT flags);
//���MCP2517FD����FIFO�����ж�
void drv_can_fd_txChannel_evtAttemptClear(CAN_FIFO_CHANNEL channel);
/*******CAN-FD RX FIFO EVENT*******/
//��ȡMCP2517FD����FIFO�ж�����
void drv_can_fd_rxChannel_evtGet(CAN_FIFO_CHANNEL channel,CAN_RX_FIFO_EVENT *flags);
//��ȡ��ǰMCP2517FD�����ж�
void drv_can_fd_rxEvtGet(uint32_t *rxif);
//��ȡ��ǰMCP2517FD��������ж�
void drv_can_fd_rxEvtOverflowGet(uint32_t *rxovif);
//��ȡMCP2517FD����FIFOͨ����
void drv_can_fd_rxChannel_indexGet(CAN_FIFO_CHANNEL channel,uint8_t* idx);
//ʹ��MCP2517FD����FIFO�ж�
void drv_can_fd_rxChannel_evtEnable(CAN_FIFO_CHANNEL channel,CAN_RX_FIFO_EVENT flags);
//ʧ��MCP2517FD����FIFO�ж�
void drv_can_fd_rxChannel_evtDisable(CAN_FIFO_CHANNEL channel,CAN_RX_FIFO_EVENT flags);
//���MCP2517FD����FIFO����ж�
void drv_can_fd_rxChannel_evtOverflowClear(CAN_FIFO_CHANNEL channel);
/*******CAN-FD TX EVENT FIFO EVENT*******/
//��ȡMCP2517FD�����¼�FIFO�ж�����
void drv_can_fd_txEvtfifo_evtGet(CAN_TEF_FIFO_EVENT *flags);
//ʹ��MCP2517FD�����¼�FIFO�ж�
void drv_can_fd_txEvtfifo_evtEnable(CAN_TEF_FIFO_EVENT flags);
//ʧ��MCP2517FD�����¼�FIFO�ж�
void drv_can_fd_txEvtfifo_evtDisable(CAN_TEF_FIFO_EVENT flags);
//���MCP2517FD�����¼�FIFO����ж�
void drv_can_fd_txEvtfifo_evtOverflowClear(void);
/*******CAN-FD ERROR MASTER*******/
//��ȡMCP2517FD���ʹ������
void drv_can_fd_errorCount_txGet(uint8_t *tec);
//��ȡMCP2517FD���մ������
void drv_can_fd_errorCount_rxGet(uint8_t *rec);
//��ȡMCP2517FD����״̬
void drv_can_fd_errorStateGet(CAN_ERROR_STATE *flags);
//��ȡMCP2517FD����״̬����
void drv_can_fd_errorCount_stateGet(uint8_t *tec,uint8_t *rec,CAN_ERROR_STATE *flags);
//��ȡMCP2517FD�������
void drv_can_fd_busDiagnosticsGet(CAN_BUS_DIAGNOSTIC *bd);
//���MCP2517FD�������
void drv_can_fd_busDiagnosticsClear(void);
/*******CAN-FD ECC*******/
//ʹ��MCP2517FD ECC
void drv_can_fd_ecc_enable(void);
/*******CAN-FD OSC/BIT TIME*******/
//MCP2517FD��������
void drv_can_fd_osc_enable(void);
//MCP2517FD��������
void drv_can_fd_osc_config(CAN_OSC_CTRL ctrl);
//��ȡMCP2517FD������ʼ������
void drv_can_fd_osc_config_objReset(CAN_OSC_CTRL *ctrl);
//��ȡMCP2517FD����״̬
void drv_can_fd_osc_status(CAN_OSC_STATUS *status);
//����MCP2517FDλʱ��
void drv_can_fd_bitTime_config(CAN_BITTIME_SETUP bitTime,CAN_SSP_MODE sspMode,CAN_SYSCLK_SPEED sysclk);
//����MCP2517FD�ٲö�λʱ��(40MHz)
void drv_can_fd_bitTime_config_nominal40MHz(CAN_BITTIME_SETUP bitTime);
//����MCP2517FD���ݶ�λʱ��(40MHz)
void drv_can_fd_bitTime_config_data40MHz(CAN_BITTIME_SETUP bitTime,CAN_SSP_MODE sspMode);
//����MCP2517FD�ٲö�λʱ��(20MHz)
void drv_can_fd_bitTime_config_nominal20MHz(CAN_BITTIME_SETUP bitTime);
//����MCP2517FD���ݶ�λʱ��(20MHz)
void drv_can_fd_bitTime_config_data20MHz(CAN_BITTIME_SETUP bitTime,CAN_SSP_MODE sspMode);
//����MCP2517FD�ٲö�λʱ��(10MHz)
void drv_can_fd_bitTime_config_nominal10MHz(CAN_BITTIME_SETUP bitTime);
//����MCP2517FD���ݶ�λʱ��(10MHz)
void drv_can_fd_bitTime_config_data10MHz(CAN_BITTIME_SETUP bitTime,CAN_SSP_MODE sspMode);
/*******CAN-FD GPIO*******/
//MCP2517FD����GPIOģʽ
void drv_can_fd_gpio_modeConfig(GPIO_PIN_MODE gpio0,GPIO_PIN_MODE gpio1);
//MCP2517FD����GPIO���ģʽ
void drv_can_fd_gpio_dirConfig(GPIO_PIN_MODE gpio0,GPIO_PIN_MODE gpio1);
/*******CAN-FD TOOL*******/
//MCP2517FD CRC16У��λ������
uint16_t drv_can_fd_crc16_checksum(uint8_t* data,uint16_t size);
//MCP2517FD ����֡����
uint32_t drv_can_fd_dlcToDataBytes(CAN_DLC dlc);
//��ʼ��MCP2517FD RAM
void drv_can_fd_ramInit(uint8_t dCon);

extern BSP_SPI_TypeDef canfd_control;
extern const uint16_t crc16_table[256];

#endif
