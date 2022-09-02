#include "oled.h"
#include "oledfont.h"
//#include "tim.h"

//PWM
extern unsigned char PWM_LEVEL_1;
extern unsigned char PWM_LEVEL_2;
//US
extern unsigned int Distance;
void Delay_us(unsigned char n)
{
	while(n--);
}

//OLED初始化
void OLED_Init(void)
{
 unsigned char i;

 OLED_SDA_OFF();
 OLED_SCK_ON();

 for(i=0;i<25;i++)
 {
  OLED_Send_Cmd(OLED_Init_cmd[i]);
 }

}

//
void OLED_IIC_Start(void)
{
 OLED_SDA_ON();
 Delay_us(1);
 OLED_SCK_ON();
 Delay_us(1);
 OLED_SDA_OFF();
 Delay_us(1);  ///
 OLED_SCK_OFF();
 OLED_IIC_Write(0x78);
}
 
//
void OLED_IIC_Stop(void)
{
 OLED_SDA_OFF();
 Delay_us(1);
 OLED_SCK_ON();
 Delay_us(1); ///
 OLED_SDA_ON();
}

//OLED发送指令函数
void OLED_Send_Cmd(unsigned char command)
{
 OLED_IIC_Start();
 OLED_IIC_Write(0x00); 
 OLED_IIC_Write(command);
 OLED_IIC_Stop();
}

//OLED发送显示8点数据函数
void OLED_Send_Data(unsigned char dat)
{
 OLED_IIC_Start();
 OLED_IIC_Write(0x40);
 OLED_IIC_Write(dat);
 OLED_IIC_Stop();
}

//OLED设置列数 128列 0~127
void OLED_Set_Column(unsigned char column)
{
 OLED_Send_Cmd(0x10|(column>>4));//设置列地址高位
 OLED_Send_Cmd(0x00|(column&0x0f));//设置列地址低位
}

//OLED设置大行数 8大行 0~7 每大行由8小行组成
void OLED_Set_Page(unsigned char line)
{
 OLED_Send_Cmd(0xb0+line);
}

//OLED清屏函数 黑屏
void OLED_ALL_Clear(void)
{
 unsigned char line,column;
 for(line=0;line<8;line++)
 {  
  OLED_Set_Page(line);
  OLED_Set_Column(0);
  for(column=0;column<128;column++)
  {
   OLED_Send_Data(0x00);
  }
 }
}

//OLED清屏函数 第一大行黄色 其他7大行蓝色
void OLED_ALL_Full(void)
{
 unsigned char line,column;
 for(line=0;line<8;line++)
 { 
  OLED_Set_Page(line);
  OLED_Set_Column(0);
  for(column=0;column<128;column++)
  {
   OLED_Send_Data(0xff);
  }
 }
}

//OLED写入1Byte数据
void OLED_IIC_Write(unsigned char date)
{
 unsigned char i, temp;
 temp = date;
 for(i=0; i<8; i++)
 { 
  OLED_SCK_OFF();
  if((temp&0x80) == 0)
  {
   OLED_SDA_OFF();
  }
  else
  {
   OLED_SDA_ON();
  }
  temp = temp << 1;
  Delay_us(1);
  OLED_SCK_ON();
  Delay_us(1);
 }
 OLED_SCK_OFF();
 Delay_us(1);
 OLED_SDA_ON();
 Delay_us(1);
 OLED_SCK_ON();
 Delay_us(1);
 OLED_SCK_OFF();
 Delay_us(1);
}

//OLED显示一个8x6的ASCII码符号
/*
void OLED_Draw_8_6_ASCII(unsigned char line,unsigned char column,int ASCII_num)
{
 int i;
 OLED_Set_Page(line);
 OLED_Set_Column(column);
 for(i=0;i<6;i++)
 {
  OLED_Send_Data(ASCII8x6[ASCII_num][i]);
 }
}
*/

//OLED显示一个16x8的ASCII码符号
void OLED_Draw_16_8_ASCII(unsigned char line,unsigned char column,int ASCII_num)
{
 int i; 
 OLED_Set_Page(line);
 OLED_Set_Column(column);
 for(i=0;i<8;i++)
 {
  OLED_Send_Data(ASCII16x8[ASCII_num][i]);
 }
 OLED_Set_Page(line+1);
 OLED_Set_Column(column);
 for(i=8;i<16;i++)
 {
  OLED_Send_Data(ASCII16x8[ASCII_num][i]);
 }
}  

//OLED显示一个16x16的汉字符号
void OLED_Draw_16_16_Hz(unsigned char line,unsigned char column,int Hz_num)
{
 int i;
 OLED_Set_Page(line);
 OLED_Set_Column(column);
 for(i=0;i<16;i++)
 {
  OLED_Send_Data(Hz16x16[Hz_num][i]);
 }
 OLED_Set_Page(line+1);
 OLED_Set_Column(column);
 for(i=16;i<32;i++)
 {
  OLED_Send_Data(Hz16x16[Hz_num][i]);
 }
}

void oled_manual()
{
	OLED_Draw_16_16_Hz(0,48,6);
	OLED_Draw_16_16_Hz(0,64,7);
}
void oled_speed()
{
	switch(PWM_LEVEL_1)
	{
		case 2	:OLED_Draw_16_8_ASCII(3,48,1);break;
		case 4	:OLED_Draw_16_8_ASCII(3,48,2);;break;
		case 6	:OLED_Draw_16_8_ASCII(3,48,3);;break;
		case 8	:OLED_Draw_16_8_ASCII(3,48,4);;break;
		case 10	:OLED_Draw_16_8_ASCII(3,48,5);;break;
	}
}
void oled_find_track()
{
	OLED_Draw_16_16_Hz(0,48,8);
	OLED_Draw_16_16_Hz(0,64,9);
}
void oled_auto_avoid()
{
	OLED_Draw_16_16_Hz(0,48,10);
	OLED_Draw_16_16_Hz(0,64,11);
}
void oled_auto_follow()
{
	OLED_Draw_16_16_Hz(0,48,12);
	OLED_Draw_16_16_Hz(0,64,13);
}
void oled_Distance()
{
	OLED_Draw_16_8_ASCII(6,48,(Distance/10000)%10);
	OLED_Draw_16_8_ASCII(6,56,(Distance/1000)%10);
	OLED_Draw_16_8_ASCII(6,64,(Distance/100)%10);
	OLED_Draw_16_8_ASCII(6,72,(Distance/10)%10);
	OLED_Draw_16_8_ASCII(6,80,Distance%10);
}		   
void OledInit()	
{
	 OLED_Init();
	 OLED_ALL_Clear();
	 OLED_Draw_16_16_Hz(0,0,0);
	 OLED_Draw_16_16_Hz(0,16,1);
	 OLED_Draw_16_8_ASCII(0,32,10);

	 OLED_Draw_16_16_Hz(3,0,2);
	 OLED_Draw_16_16_Hz(3,16,3);
	 OLED_Draw_16_8_ASCII(3,32,10);

	 OLED_Draw_16_16_Hz(6,0,4);
	 OLED_Draw_16_16_Hz(6,16,5);
	 OLED_Draw_16_8_ASCII(6,32,10);

	 oled_manual();
	 oled_speed();
}




