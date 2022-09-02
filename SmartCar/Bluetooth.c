#include <reg52.h>
//红外
extern unsigned char ircode[4];
sbit beep=P2^3;
//标志
extern bit IFFlag;
//oled
extern void	oled_speed();
extern void	oled_manual();
extern void	oled_find_track();
extern void	oled_auto_avoid();
extern void	oled_auto_follow();
//红外决策
extern void DECISION(void);

void ConfigUART(unsigned int baud)
{
    SCON  = 0x50;  //配置串口为模式1
    TMOD &= 0x0F;  //清零T1的控制位
    TMOD |= 0x20;  //配置T1为模式2
    TH1 = 256 - (11059200/12/32)/baud;  //计算T1重载值
    TL1 = TH1;     //初值等于重载值
    ET1 = 0;       //禁止T1中断
    ES  = 1;       //使能串口中断
    TR1 = 1;       //启动T1
}

void InterruptUART() interrupt 4
{

	if(RI)
	{
		IFFlag=0;
		RI = 0;
		ircode[2]=SBUF;
		DECISION();
	}
	if(TI)
	{
		TI = 0;
	}
}
