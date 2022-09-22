#include "data_transmission.h"
#include "Driver_powerContol.h"
#include "power.h"
#include "Driver_USBVCP.h"

void datatransmissionInit(void){
#ifdef USE_WIRELESS                     //���������ĳ�ʼ��
	wirelessInit();                   
#endif
	
#ifdef USE_UPPER                        //��λ�����շ��ͳ�ʼ��
	upperMonitorInit();                
#endif
	
#ifdef USE_POWER_LIMIT
	sendPowerDataInit();
#endif
    usbVCP_Printf("datatransmissionInit Successfully \r\n");
}





