#include <reg52.h>

#define LCD1602_DATAPINS P0
sbit LCD1602_E =P3^4;
sbit LCD1602_RW=P3^6;
sbit LCD1602_RS=P3^5;
//延时
extern void delay(unsigned int n,unsigned char mode);
//PWM
extern unsigned char PWM_LEVEL_1;
extern unsigned char PWM_LEVEL_2;
//US
extern unsigned long int Distance;

unsigned char i;
unsigned char MN[] ="Manual    ";
unsigned char FT[] ="FindTrack ";
unsigned char AA[] ="AutoAvoid ";
unsigned char AF[] ="AutoFollow";

void LcdWriteCom(unsigned char com)	  //写入命令
{
	LCD1602_E = 0;     //使能
	LCD1602_RS = 0;	   //选择发送命令
	LCD1602_RW = 0;	   //选择写入
	
	LCD1602_DATAPINS = com;     //放入命令
	delay(1,0);		//等待数据稳定

	LCD1602_E = 1;	          //写入时序
	delay(5,0);	  //保持时间
	LCD1602_E = 0;
}		   
 
void LcdWriteData(unsigned char dat)			//写入数据
{
	LCD1602_E = 0;	//使能清零
	LCD1602_RS = 1;	//选择输入数据
	LCD1602_RW = 0;	//选择写入

	LCD1602_DATAPINS = dat; //写入数据
	delay(1,0);

	LCD1602_E = 1;   //写入时序
	delay(5,0);   //保持时间
	LCD1602_E = 0;
}

void lcd_manual()
{
	LcdWriteCom(0x80+0x06);
	for(i=0;i<10;i++)
	{
		LcdWriteData(MN[i]);
	}
}
void lcd_speed()
{
	LcdWriteCom(0x80+0x40+0x07);
	switch(PWM_LEVEL_1)
	{
		case 2	:LcdWriteData('1');break;
		case 4	:LcdWriteData('2');break;
		case 6	:LcdWriteData('3');break;
		case 8	:LcdWriteData('4');break;
		case 10	:LcdWriteData('5');break;
	}
}
void lcd_find_track()
{
	LcdWriteCom(0x80+0x06);
	for(i=0;i<10;i++)
	{
		LcdWriteData(FT[i]);
	}
}
void lcd_auto_avoid()
{
	LcdWriteCom(0x80+0x06);
	for(i=0;i<10;i++)
	{
		LcdWriteData(AA[i]);	      
	}
}
void lcd_auto_follow()
{
	LcdWriteCom(0x80+0x06);
	for(i=0;i<10;i++)
	{
		LcdWriteData(AF[i]);
	}
}
void lcd_Distance()
{
	LcdWriteCom(0x80+0x40+0x09);
	LcdWriteData((Distance/1000)%10+48);
	LcdWriteData((Distance/100)%10+48);
	LcdWriteData((Distance/10)%10+48);
	LcdWriteData(Distance%10+48);
}		   
void LcdInit()	
{
	unsigned char words_1[]="Mode:";	
	unsigned char words_2[]="Speed:";	
 	LcdWriteCom(0x38);  //开显示
	LcdWriteCom(0x0c);  //开显示不显示光标
	LcdWriteCom(0x06);  //写一个指针加1
	LcdWriteCom(0x01);  //清屏
	LcdWriteCom(0x80);  //设置数据指针起点

	for(i=0;i<5;i++)
	{
		LcdWriteData(words_1[i]);
	}
	lcd_manual();
	LcdWriteCom(0x80+0x40);
	for(i=0;i<6;i++)
	{
		LcdWriteData(words_2[i]);
	}
	LcdWriteCom(0x80+0x40+0x07);
	LcdWriteData('3');

	lcd_Distance();
}
