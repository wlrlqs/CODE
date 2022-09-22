#include <reg52.h>

sbit IR_INPUT = P0^1;  //红外接收引脚
sbit beep = P2^3;
unsigned char ircode[4]={0,0,0x11,0};  //红外代码接收缓冲区
//标志
bit IFFlag;
//电机
extern void EMinit();
extern void go_stop();
extern void go_still();
extern void go_forward();
extern void go_back();
extern void go_left();
extern void go_right();
extern void rotate_l();
extern void rotate_r();
extern void accelerate();
extern void decelerate();
extern void find_track();
extern void auto_avoid();
extern void auto_follow();
//oled
extern void	oled_speed();
extern void	oled_manual();
extern void	oled_find_track();
extern void	oled_auto_avoid();
extern void	oled_auto_follow();
//OLED
extern bit Display;

void InitInfrared()
{	
	IR_INPUT = 1;
	//PT0 = 1;//优先
	TMOD = 0x01;
	TR0 = 0;
	ET0 = 0;//停止溢出
	IT0 = 1;//下沿
	EX0 = 1;//外中
}

unsigned int GetHighTime()
{
	TH0 = 0;
	TL0 = 0;
	TR0 = 1;
	while(IR_INPUT)
	{
		if(TH0 > 0x40)
		{
			break;
		}
	}
	TR0 = 0;

	return(TH0 * 256 + TL0);
}

unsigned int GetLowTime()
{
	TH0 = 0;
	TL0 = 0;
	TR0 = 1;
	while(!IR_INPUT)
	{
		if(TH0 > 0x40)
		{
			break;
		}
	}
	TR0 = 0;

	return(TH0 * 256 + TL0);
}
unsigned char NOSIGN()
{
	unsigned long int times=0;
	while(1)
	{
		if(IR_INPUT)times++;
		if(!IR_INPUT)times=0;
		if(times>1900)
		{
			return 1;
		}
	}	  
}
void DECISION(void)
{
	switch(ircode[2])//对应
	{
		case 0x11:go_still();	oled_manual();break;
		case 0x1c:beep=	0;		oled_manual();break;//停 5
		case 0x18:go_forward();	oled_manual();break;//上	2
		case 0x52:go_back();	oled_manual();break;//下	8
		case 0x08:go_left();	oled_manual();break;//左	4
		case 0x5a:go_right();	oled_manual();break;//右	6
		case 0x07:decelerate();	oled_speed();break;//-
		case 0x15:accelerate();	oled_speed();break;//+
		case 0x0c:oled_find_track();	find_track();	break;//	 1
		case 0x5e:oled_auto_avoid();	auto_avoid();	break;//	 3
		case 0x42:oled_auto_follow();	auto_follow();	break;//	 7
		case 0x31:rotate_l();	oled_manual();break;//左	4
		case 0x32:rotate_r();	oled_manual();break;//右	6	
		case 0x77:Display=~Display;break;//右	6
		//case 0x33:oled_auto_follow();  R_follow();break;//右跟随	6
		//case 0x34:oled_auto_follow();  L_follow();break;//左跟随	6
	}
}
void EXINT1_ISR() interrupt 0
{
	unsigned char i, j;
	unsigned int time;
	unsigned char byt;
	time = GetLowTime();
	if((time <7833) || (time > 8755))	   
	{
		return;	
	}

	time = GetHighTime();
	if((time<3686) || (time > 4608))
	{
		return;
	}

	for(i=0; i<4; i++)
	{
		for(j=0; j<8; j++)
		{
			time = GetLowTime();
			if((time<313) ||(time >718))
			{
				return;
			}
			time = GetHighTime();
			if((time>313) && (time <718))
			{
				byt >>= 1;	
			}
			else if((time>1345) && (time<1751))
			{
				byt >>= 1;
				byt |= 0x80;
			}
			else
			{
				return;
			}
		}
		ircode[i] = byt;
	}
	IFFlag=1;
	DECISION();
	while(1)
	{
		if(NOSIGN())
		{
			beep = 1;
			IR_INPUT = 1;
			break;
		}
	}
}