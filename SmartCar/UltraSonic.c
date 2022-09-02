#include <reg52.h>

sbit RX = P2^0;//ECHO³¬Éù²¨Ä£¿é»ØÏì¶Ë
sbit TX = P2^1;//TRIG³¬Éù²¨Ä£¿é´¥·¢¶Ë
extern unsigned char ircode[4];
unsigned int Distance=0;
unsigned int time;
bit Display=0;
extern void oled_Distance();

void Open_US()
{
	 unsigned char count=1;
	 TH0=0;
	 TL0=0;
	 TX=1;			 
     while(count--);
	 TX=0;
}
void Receive_US()
{		

	while(!RX)
	{
		if(ircode[2]!=0x5e && ircode[2]!=0x42  && ircode[2]!=0x22)return;
	}
	TR0=1;
	while(RX)
	{
		if(ircode[2]!=0x5e && ircode[2]!=0x42)
		{
			TR0=0;
			return;
		}
	}
	time=TH0*256+TL0;
	TR0=0;
	Distance=(unsigned int)(time*1.085)*0.17;//MM

}
void measure_distance()
{
	Open_US();
	Receive_US();
	oled_Distance();
}