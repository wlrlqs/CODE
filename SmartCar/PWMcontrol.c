#include <reg52.h>

unsigned char PWM_LEVEL_1=8;//2	 4 6 8 10	五挡
unsigned char PWM_LEVEL_2=8;
unsigned char COUNT_INTERRUPT=0;

sbit EM1_P=P1^5;
sbit EM2_P=P1^6;


void InitPWM()
{
//        T2MOD=0;  //配置定时器2控制寄存器，这里其实不用配置，T2CON上电默认就是0
//        T2CON=0;  //配置定时器2工作模式寄存器，这里其实不用配置，T2MOD上电默认就是0
        TH2=(65536-1000)/256;                        //定时器2赋初值
        TL2=(65536-1000)%256;                        //1ms                                       
        ET2=1;                        //开定时器2中断
        EA=1;
        TR2=1;                        //启动定时器2
}


void PWM() interrupt 5
{
    TF2=0;        //!!!注意!!! 定时器2必须由软件对溢出标志位清零，硬件不能清零
	TH2=(65536-1000)/256;                        //定时器2赋初值
    TL2=(65536-1000)%256;
    COUNT_INTERRUPT++;        

	if(COUNT_INTERRUPT==10)COUNT_INTERRUPT=0,EM2_P=1,EM1_P=1;

	if(COUNT_INTERRUPT==0)EM1_P=1,EM2_P=1;
	if(COUNT_INTERRUPT==(PWM_LEVEL_1-1))EM1_P=0;
	if(COUNT_INTERRUPT==PWM_LEVEL_2)EM2_P=0;
}