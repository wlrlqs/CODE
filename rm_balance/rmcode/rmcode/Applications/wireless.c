#include "wireless.h"
#include "keyboard.h"
#include "board.h"
#include "judge.h"
#include "shoot.h"


/********************************************************************************/
/*********                                                             *********/
/*********    谁写的代码，注释都没几行，写的注释还贼SB,一点作用都没有    **********/
/*********              这个文件我看了一个赛季都没看懂是干啥的           *********/
/*********                                                             *********/
/*******************************************************************************/

BSP_USART_TypeDef WIRELESS;
wirelessStruct_t wirelessData;
/* 两个宏定义只能定义一个 */
#define ANO_DT_USE_UART7			//使用串口
/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、f32_t等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

dt_flag_t f;					//需要发送数据的标志

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length){
#ifdef ANO_DT_USE_USB_HID
	usbVCP_SendBuffer(dataToSend,length);
#endif
#ifdef ANO_DT_USE_UART7
	WIRELESS.USARTx = WIRELESS_USARTX;
	BSP_USART_SendData( &WIRELESS, dataToSend, length);
#endif
}
static void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum){
	wirelessData.dataNeedSend[0]=0xAA;
	wirelessData.dataNeedSend[1]=0xAA;
	wirelessData.dataNeedSend[2]=0xEF;
	wirelessData.dataNeedSend[3]=2;
	wirelessData.dataNeedSend[4]=head;
	wirelessData.dataNeedSend[5]=check_sum;
	
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<6;i++)
		sum += wirelessData.dataNeedSend[i];
	wirelessData.dataNeedSend[6]=sum;

	ANO_DT_Send_Data(wirelessData.dataNeedSend, 7);
}


void ANO_DT_Data_Exchange(void){
	static uint8_t cnt = 0;
	static uint8_t senser_cnt 	= 10;
	static uint8_t senser2_cnt = 50;
	static uint8_t user_cnt 	  = 10;
	static uint8_t status_cnt 	= 15;
	static uint8_t rcdata_cnt 	= 20;
	static uint8_t motopwm_cnt	= 20;
	static uint8_t power_cnt		=	50;
	static uint8_t speed_cnt   = 50;
	static uint8_t location_cnt   = 200;
	static uint8_t fly_ready = 0;
	
	if((cnt % senser_cnt) == (senser_cnt-1))
		f.sendSenser = 1;

	if((cnt % senser2_cnt) == (senser2_cnt-1))
		f.sendSenser2 = 1;	

	if((cnt % user_cnt) == (user_cnt-2))
		f.sendUser = 1;
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.sendStatus = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.sendRcData = 1;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-2))
		f.sendMotoPwm = 1;	
	
	if((cnt % power_cnt) == (power_cnt-2))
		f.sendPower = 1;		
	
	if((cnt % speed_cnt) == (speed_cnt-3))
		f.sendSpeed = 1;		
	
	if((cnt % location_cnt) == (location_cnt-3))
	{
		f.sendLocation += 1;		
	}
	
	if(++cnt>200) cnt = 0;
/////////////////////////////////////////////////////////////////////////////////////
	if(f.sendCheck)
	{
		f.sendCheck = 0;
		ANO_DT_Send_Check(wirelessData.checkDataNeedSend,wirelessData.checkSumNeedSend);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendVersion)
	{
		f.sendVersion = 0;
		ANO_DT_Send_Version(4,300,100,400,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendStatus)
	{
		f.sendStatus = 0;
		fly_ready = supervisorData.state & STATE_ARMED;
		ANO_DT_Send_Status(SC_ROLL,SC_PITCH,SC_YAW,0,0,fly_ready);
	}	
///////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendSpeed)
	{
		f.sendSpeed = 0;
		ANO_DT_Send_Speed(0,0,0);
	}
///////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendUser)
	{
		f.sendUser = 0;
		ANO_DT_Send_User();
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendSenser)
	{
		f.sendSenser = 0; 
		ANO_DT_Send_Senser(getGimbalData()->pitchAngleFbd*100,getGimbalData()->pitchAngleRef*100,getGimbalData()->yawAngleFbd*100,
											 getGimbalData()->yawAngleRef*100,getchassisData()->chaseFbd*100,0,
											 getGimbalData()->yawSpeedFbd*100,0,0); 

	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendSenser2)
	{
		f.sendSenser2 = 0;
		ANO_DT_Send_Senser2(0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendRcData)
	{
		f.sendRcData = 0;
		ANO_DT_Send_RCData(remoteControlData.dt7Value.rcRawData.CH2+1500,remoteControlData.dt7Value.rcRawData.CH3+1500,\
											 remoteControlData.dt7Value.rcRawData.CH0+1500,remoteControlData.dt7Value.rcRawData.CH1+1500,\
											 0,0,0,0,0,0);
											 
	}	
///////////////////////////////////////////////////////////////////////////////////////	
	else if(f.sendMotoPwm)
	{
		f.sendMotoPwm = 0;
		ANO_DT_Send_MotoPWM(0,0,0,0,0,0,0,0);
	}	
///////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendPid1)
	{
		f.sendPid1 = 0;
		ANO_DT_Send_PID(1,
                        0.001f*getConfigData()->pitchRatePID.PID_P,0.001f*getConfigData()->pitchRatePID.PID_I,0.001f*getConfigData()->pitchRatePID.PID_D,
						0.001f*getConfigData()->yawRatePID.PID_P,0.001f*getConfigData()->yawRatePID.PID_I,0.001f*getConfigData()->yawRatePID.PID_D,
						0.001f*getConfigData()->rollRatePID.PID_P,0.001f*getConfigData()->rollRatePID.PID_I,0.001f*getConfigData()->rollRatePID.PID_D);
	}	
///////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendPid2)
	{
		ANO_DT_Send_PID(1,
                        0.001f*getConfigData()->pitchAnglePID.PID_P,0.001f*getConfigData()->pitchAnglePID.PID_I,0.001f*getConfigData()->pitchAnglePID.PID_D,
						0.001f*getConfigData()->yawAnglePID.PID_P,0.001f*getConfigData()->yawAnglePID.PID_I,0.001f*getConfigData()->yawAnglePID.PID_D,
						0.001f*getConfigData()->rollAnglePID.PID_P,0.001f*getConfigData()->rollAnglePID.PID_I,0.001f*getConfigData()->rollAnglePID.PID_D);
	}
///////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendPid3)
	{
		f.sendPid3 = 0;
		ANO_DT_Send_PID(3,
                        0.1f*getConfigData()->chassisSpeedPID.PID_P,0.1f*getConfigData()->chassisSpeedPID.PID_I,0.1f*getConfigData()->chassisSpeedPID.PID_D,
						0.01f*getConfigData()->chassisChasePID.PID_P,0.01f*getConfigData()->chassisChasePID.PID_I,0.01f*getConfigData()->chassisChasePID.PID_D,
						0,0,0);
	}
	else if(f.sendPid4)
	{
		f.sendPid4 = 0;
		ANO_DT_Send_PID(4,0,0,0,getConfigData()->shootSmallDialPID.PID_P,getConfigData()->shootSmallDialPID.PID_I,getConfigData()->shootSmallDialPID.PID_D,
											0,0,0);
	}
}

void ANO_DT_Send_Status(f32_t angle_rol, f32_t angle_pit, f32_t angle_yaw, s32 alt, uint8_t fly_model, uint8_t armed){
	uint8_t _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x01;
	wirelessData.dataNeedSend[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	
	wirelessData.dataNeedSend[_cnt++]=BYTE3(_temp2);
	wirelessData.dataNeedSend[_cnt++]=BYTE2(_temp2);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp2);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp2);
	
	wirelessData.dataNeedSend[_cnt++] = fly_model;
	
	wirelessData.dataNeedSend[_cnt++] = armed;
	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	wirelessData.dataNeedSend[_cnt++]=sum;
	
	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}
void ANO_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z){
	uint8_t _cnt=0;
	vs16 _temp;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x02;
	wirelessData.dataNeedSend[_cnt++]=0;
	
	_temp = a_x;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = 0;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);	
	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	wirelessData.dataNeedSend[_cnt++] = sum;
	
	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}

void ANO_DT_Send_Senser2(s32 bar_alt,uint16_t csb_alt){
	uint8_t _cnt=0;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x07;
	wirelessData.dataNeedSend[_cnt++]=0;
	
	wirelessData.dataNeedSend[_cnt++]=BYTE3(bar_alt);
	wirelessData.dataNeedSend[_cnt++]=BYTE2(bar_alt);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(bar_alt);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(bar_alt);

	wirelessData.dataNeedSend[_cnt++]=BYTE1(csb_alt);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(csb_alt);
	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	wirelessData.dataNeedSend[_cnt++] = sum;
	
	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}
void ANO_DT_Send_RCData(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6){
	uint8_t _cnt=0;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x03;
	wirelessData.dataNeedSend[_cnt++]=0;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(thr);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(thr);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(yaw);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(yaw);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(rol);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(rol);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(pit);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(pit);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(aux1);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(aux1);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(aux2);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(aux2);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(aux3);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(aux3);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(aux4);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(aux4);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(aux5);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(aux5);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(aux6);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(aux6);

	wirelessData.dataNeedSend[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	
	wirelessData.dataNeedSend[_cnt++]=sum;
	
	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}
void ANO_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8){
	uint8_t _cnt=0;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x06;
	wirelessData.dataNeedSend[_cnt++]=0;
	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_1);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_1);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_2);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_2);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_3);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_3);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_4);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_4);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_5);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_5);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_6);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_6);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_7);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_7);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_8);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_8);
	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	
	wirelessData.dataNeedSend[_cnt++]=sum;
	
	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}
void ANO_DT_Send_PID(uint8_t group,f32_t p1_p,f32_t p1_i,f32_t p1_d,f32_t p2_p,f32_t p2_i,f32_t p2_d,f32_t p3_p,f32_t p3_i,f32_t p3_d){
	uint8_t _cnt=0;
	vs16 _temp;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x10+group-1;
	wirelessData.dataNeedSend[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	
	wirelessData.dataNeedSend[_cnt++]=sum;

	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}
void ANO_DT_Send_User(void){
	uint8_t _cnt=0;
	vs16 _temp;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA; 
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xf1; //用户数据
	wirelessData.dataNeedSend[_cnt++]=0;
	
	
	_temp = 0;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);

	_temp = (int16_t)0;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	
	_temp = (int16_t)0;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);	
	
	_temp = (int16_t)0;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	
  _temp = (int16_t)0;              //5
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);

	

	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	
	wirelessData.dataNeedSend[_cnt++]=sum;

	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}
void ANO_DT_Send_Speed(f32_t x_s,f32_t y_s,f32_t z_s){
	uint8_t _cnt=0;
	vs16 _temp;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x0B;
	wirelessData.dataNeedSend[_cnt++]=0;
	
	_temp = (int)(0.1f *x_s);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *y_s);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *z_s);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	
	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	wirelessData.dataNeedSend[_cnt++]=sum;
	
	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);

}
void ANO_DT_Send_Version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver){
	uint8_t _cnt=0;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x00;
	wirelessData.dataNeedSend[_cnt++]=0;
	
	wirelessData.dataNeedSend[_cnt++]=hardware_type;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(hardware_ver);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(hardware_ver);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(software_ver);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(software_ver);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(protocol_ver);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(protocol_ver);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(bootloader_ver);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(bootloader_ver);
	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	wirelessData.dataNeedSend[_cnt++]=sum;
	
	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(uint8_t data){
	static uint8_t RxBuffer[50];
	static uint8_t _data_len = 0,_data_cnt = 0;
	static uint8_t state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
uint16_t RX_CH[9];
void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num){
	uint8_t sum = 0;
	for(uint8_t i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
		{
			digitalHi(&wirelessData.imuCalibrate);
		}
		else if(*(data_buf+4)==0X02)
			digitalHi(&wirelessData.imuCalibrate);
		else if(*(data_buf+4)==0X03)
		{
			digitalHi(&wirelessData.imuCalibrate);			
		}
		else if(*(data_buf+4)==0X04)
		{
			digitalHi(&wirelessData.magCalibrate);
		}
		else if((*(data_buf+4)>=0X021)&&(*(data_buf+4)<=0X26))
		{
			//acc_3d_calibrate_f = 1;
		}
	}
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.sendPid1 = 1;
			f.sendPid2 = 1;
			f.sendPid3 = 1;
			f.sendPid4 = 1;
			f.sendPid5 = 1;
			f.sendPid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//读取版本信息
		{
			f.sendVersion = 1;
		}
	}
	if(*(data_buf+2)==0X10)								//PID1
	{
		getConfigData()->pitchRatePID.PID_P  = 1*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		getConfigData()->pitchRatePID.PID_I  = 1*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		getConfigData()->pitchRatePID.PID_D  = 1*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		getConfigData()->yawRatePID.PID_P = 1*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		getConfigData()->yawRatePID.PID_I = 1*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		getConfigData()->yawRatePID.PID_D = 1*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
		getConfigData()->rollRatePID.PID_P 	= 1*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
		getConfigData()->rollRatePID.PID_I 	= 1*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
		getConfigData()->rollRatePID.PID_D	= 1*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		if(f.sendCheck == 0)
		{
			f.sendCheck = 1;
			wirelessData.checkDataNeedSend = *(data_buf+2);
			wirelessData.checkSumNeedSend = sum;
		}
		digitalHi(&supervisorData.flashSave);
	}
	if(*(data_buf+2)==0X11)								//PID2
	{
		getConfigData()->pitchAnglePID.PID_P = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		getConfigData()->pitchAnglePID.PID_I = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		getConfigData()->pitchAnglePID.PID_D = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		getConfigData()->yawAnglePID.PID_P = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		getConfigData()->yawAnglePID.PID_I = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		getConfigData()->yawAnglePID.PID_D = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
		getConfigData()->rollAnglePID.PID_P	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
		getConfigData()->rollAnglePID.PID_I = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
		getConfigData()->rollAnglePID.PID_D = 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		if(f.sendCheck == 0)
		{
			f.sendCheck = 1;
			wirelessData.checkDataNeedSend = *(data_buf+2);
			wirelessData.checkSumNeedSend = sum;                                                                 
		}
		digitalHi(&supervisorData.flashSave);
	}
	if(*(data_buf+2)==0X12)								//PID3
	{	
		getConfigData()->chassisSpeedPID.PID_P  = 0.01*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		getConfigData()->chassisSpeedPID.PID_I  = 0.01*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		getConfigData()->chassisSpeedPID.PID_D  = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
	
		getConfigData()->chassisChasePID.PID_P = 0.1*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		getConfigData()->chassisChasePID.PID_I = 0.1*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		getConfigData()->chassisChasePID.PID_D = 0.1*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
	
		if(f.sendCheck == 0)
		{
			f.sendCheck = 1;
			wirelessData.checkDataNeedSend = *(data_buf+2);
			wirelessData.checkSumNeedSend = sum;
		}
		pidZeroState(getchassisData()->chasePID);
		digitalHi(&supervisorData.flashSave);
	}
	if(*(data_buf+2)==0X13)								//PID4
	{

		getConfigData()->shootSmallDialPID.PID_P  = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		getConfigData()->shootSmallDialPID.PID_I  = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		getConfigData()->shootSmallDialPID.PID_D  = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );	
			
		if(f.sendCheck == 0)
		{
			f.sendCheck = 1;
			wirelessData.checkDataNeedSend = *(data_buf+2);
			wirelessData.checkSumNeedSend = sum;
		}
		digitalHi(&supervisorData.flashSave);
	}
}

void dataSendToGroundStation(void){

#if (CONTROLLER == DAGGER)
	ANO_DT_Data_Exchange();
#else
	ANO_DT_Data_Exchange();
#endif
}


void dataTransportTask(void *Parameters){
	while(1)
	{
		vTaskDelay(WIRELESS_PERIOD);
		dataSendToGroundStation();
		digitalIncreasing(&wirelessData.loops);             
	}
	
}
void wirelessInit(void){
#ifdef ANO_DT_USE_USB_HID
	usbVCP_Init(WIRELESS_VCP_PreemptionPriority,WIRELESS_VCP_SubPriority);
#endif
#ifdef ANO_DT_USE_UART7
	DTUClass.Init();
#endif
	supervisorData.taskEvent[WIRELESS_TASK] = xTaskCreate(dataTransportTask,"DATATRANS",WIRELESS_STACK_SIZE,NULL,WIRELESS_PRIORITY,&wirelessData.xHandleTask);
}
