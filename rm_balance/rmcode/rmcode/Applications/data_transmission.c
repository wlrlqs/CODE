#include "data_transmission.h"
#include "Driver_powerContol.h"
#include "power.h"
#include "Driver_USBVCP.h"

void datatransmissionInit(void){
#ifdef USE_WIRELESS                     //无线数传的初始化
	wirelessInit();                   
#endif
	
#ifdef USE_UPPER                        //上位机接收发送初始化
	upperMonitorInit();                
#endif
	
#ifdef USE_POWER_LIMIT
	sendPowerDataInit();
#endif
    usbVCP_Printf("datatransmissionInit Successfully \r\n");
}





