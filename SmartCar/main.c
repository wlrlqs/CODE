#include <reg52.h>
#include <intrins.h>
//红外使用T0
void delay(unsigned int n,unsigned char mode);
int Delay1ms(int n);
void DECISION_MAIN();

//红外
extern unsigned char ircode[4];
extern void InitInfrared(void);
extern bit IFFlag;
//电机
extern void EMinit();
extern void go_stop();
extern void go_right();
extern void find_track();
extern void auto_follow();
extern void auto_avoid();
extern void measure_distance();
//PWM
extern void InitPWM();
extern void R_follow();			
extern void L_follow();	
//蓝牙
extern void ConfigUART(unsigned int baud);
//蜂鸣器
sbit beep=P2^3;
//Oled
extern void OledInit();

void main()
{
	EMinit();
	PT1 = 1;
	InitPWM();
  InitInfrared();
	ConfigUART(9600);
	OledInit();
	  EA = 1;      //总中
	
    while(1)
    {
			//find_track();
			//R_follow();
			//auto_avoid();
		//auto_foll
		DECISION_MAIN();		
    }
}
void delay(unsigned int n,unsigned char mode)
{	 unsigned char x;unsigned int x1;unsigned char x2;
	 switch(mode)
	 {
	 	case 0: while(n--){x=100;while(x--);};break;//ms
		case 1: while(n--){x1=10000;while(x1--){x2=100;while(x2--);}};break;//s
	 }
}
/*int Delay1ms(int n)		//@11.0592MHz
{
	unsigned char i, j;
	for(n;n>0;n--){
	_nop_();
	_nop_();
	_nop_();
	i = 11;
	j = 190;
	do
	{
		while (--j);
	} while (--i);
}
}*/

void DECISION_MAIN()
{
			switch(ircode[2])
			{
				case 0x1c:
				case 0x11:beep = 1;go_stop();break;
				case 0x08:
				case 0x5a:break;
				case 0x0c:find_track();break;
				case 0x5e:auto_avoid();break;
				case 0x42:auto_follow();break;
				default:break;
			}	
}
