#include "reg52.h"
#include "stdio.h"

#define uchar unsigned char
#define uint unsigned int 

void uart_cfg();
void send_byte(uchar by);
void send_string(uchar *p);
void delay(uchar i);
/*简单延时*/
void delay(unsigned int xms)		//@11.0592MHz
{
	unsigned char i, j;
	while(xms){

	i = 2;
	j = 199;
	do
	{
		while (--j);
	} while (--i);
	xms--;
}
}
void main()
{	
	unsigned int a=10;
	uart_cfg(); ////9600bps@11.0592MHz   ,0xf9
	delay(5000);
	send_string("DIR(2); \r\n");
	send_string("DIR(0);\r\n");
	delay(1000);
	send_string("DC16(30,30,'串口模块',5);\r\n");
	delay(3000);
       while(1)
		{
		send_string("DC16(30,30,'串口模块',5);\r\n");
		delay(500);
                              
		}
}

 void uart_cfg(){
 

	PCON &= 0x7F;		//波特率不倍速
	SCON = 0x50;		//8位数据,可变波特率
	AUXR &= 0xBF;		//定时器1时钟为Fosc/12,即12T
	AUXR &= 0xFE;		//串口1选择定时器1为波特率发生器
	TMOD &= 0x0F;		//清除定时器1模式位
	TMOD |= 0x20;		//设定定时器1为8位自动重装方式
	TL1 = 0xFD;		//设定定时初值
	TH1 = 0xFD;		//设定定时器重装值
	ET1 = 0;		//禁止定时器1中断
	TR1 = 1;		//启动定时器1
	 
 }

 /*中断处理函数*/
// void uart_interrupt() interrupt 4{
//
//        if(RI==1)RI = 0;
//
//        if(TI==1);
//               //TI = 0;
//    }

 
/*发送一个 字符*/
void send_byte(uchar by){

    SBUF = by;
    while(!TI);//当写下这句的时候，就不要在中断函数里面在写TI = 0;这句了，不然进入中断函数将TI清零之后，程序就会一直卡在这里
    TI = 0;       //在这里将TI清零
}
/*发送一个字符串*/
void send_string(uchar *p){

    while(*p!= '\0'){
    
        send_byte(*p);
        p++;

    }
}
