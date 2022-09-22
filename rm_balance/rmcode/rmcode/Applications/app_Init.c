#include "board.h"

taskInit_t taskInitData;

/*--------------------------------------------- 
-----------------------------------------------
//		警告：公共代码区域，故障责任重大		 //
//			除特定区域可以自由可以更改外，		 //
//	其余部分更改，请与公共区域负责人取得更改权限  // 
-----------------------------------------------
---------------------------------------------*/

void appInit(void *Parameters){
	//事件标志组清零
	taskInitData.eventGroups = NULL;					                    
	
	taskInitData.eventGroups = xEventGroupCreate();

	/*公共初始化*/
    //虚拟串口初始化
	usbVCP_Init(USB_USART_PreemptionPriority,USB_USART_SubPriority);  
    //读取机器人类型初始化
	robotKeyInit();
	//加载默认参数
	configInit();    
	//恒温电路屏蔽初始华
	BSP_GPIO_Init(BSP_GPIOD13,GPIO_Mode_Out_PP);
	GPIO_ResetBits(GPIOD,GPIO_Pin_13);
	//定时器2(32位定时器)初始化,用于精确时间节点计算
	clockClass.Init();
	//IIR低通滤波器初始化
	iir_filter_init();
	//CANFD初始化
	can_fd_spi_Init();
	can_fd_device_Init();
	//识别机器并对关键数据进行初始化
	robotDistinguish();
    //数据传输接收任务初始化
	transferInit();
	//IMU初始化
	imuInit();	
	//控制板通信初始化
	slaveSensorConfig();
	//数据传输初始化
	datatransmissionInit();          		
	//监控状态机的初始化
	supervisorInit();
//    init_data_prepared(BOARD_TYPE);
	/*特殊初始化*/
	//底盘板初始化					
	boardTypeConfirm(BOARD_CHASSIS,chassisSetup);
	//主控板初始化
	boardTypeConfirm(BOARD_CONTROL,controlSetup);
    //虚拟串口初始化完成标志
	usbVCP_Printf("All Init Successfully \r\n");                            
	//删除当前任务
	vTaskDelete(NULL);									                   
}
/**
static uint16_t uxHighWaterMark = 0;
uxHighWaterMark=uxTaskGetStackHighWaterMark(taskHandle);
printf("剩余空间：%d\r\n",uxHighWaterMark);
*/
 
