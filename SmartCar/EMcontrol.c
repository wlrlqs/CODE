#include <reg52.h>
//电机驱动
sbit EM1_P=P1^5;
sbit EM1_1=P1^2;
sbit EM1_2=P1^1;

sbit EM2_P=P1^6;
sbit EM2_1=P1^4;
sbit EM2_2=P1^3;

sbit R_LED=P3^5;
sbit L_LED=P3^4;
sbit R_LED_DOWN=P3^3;
sbit L_LED_DOWN=P3^2;



//循迹避障模块接线
sbit left_led1 = P3^7;	//左寻迹信号为0 没有识别到黑线 为1识别到黑线
sbit right_led1= P3^6;	//右寻迹信号为0 没有识别到黑线 为1识别到黑线

//蜂鸣
sbit beep = P2^3;
//超声
extern unsigned int Distance;
extern void measure_distance();
//延时
extern void delay(unsigned int n,unsigned char mode);
//红外
extern unsigned char ircode[4];
//PWM
extern unsigned char PWM_LEVEL_1;
extern unsigned char PWM_LEVEL_2;
void R_follow();
void L_follow();

void EMinit()
{
	EM1_1=1,EM1_2=1;
	EM2_1=1,EM2_2=1;
	EM1_P=1;
	EM2_P=1;
}
void go_stop()
{

	EM1_1=0,EM1_2=0;
	EM2_1=0,EM2_2=0;

}
void go_still()
{
	EM1_1=1,EM1_2=1;
	EM2_1=1,EM2_2=1;
}
void go_forward()
{
	EM1_1=1,EM1_2=0;
	EM2_1=1,EM2_2=0;

}
void go_back()
{
	EM1_1=0,EM1_2=1;
	EM2_1=0,EM2_2=1;

}
void go_left()
{
	EM1_1=1,EM1_2=1;
	EM2_1=1,EM2_2=0;

}
void go_right()
{
	EM1_1=1,EM1_2=0;
	EM2_1=1,EM2_2=1;

}
void rotate_l()
{
	EM1_1=0,EM1_2=1;
	EM2_1=1,EM2_2=0;
	
}
void rotate_r()
{
	EM1_1=1,EM1_2=0;
	EM2_1=0,EM2_2=1;

}
void accelerate()
{
	switch(PWM_LEVEL_1)
	{
		case 2:PWM_LEVEL_1=4;break;
		case 4:PWM_LEVEL_1=6;break;		
		case 6:PWM_LEVEL_1=8;break;
		case 8:PWM_LEVEL_1=10;break;
	}
	switch(PWM_LEVEL_2)
	{
		case 2:PWM_LEVEL_2=4;break;
		case 4:PWM_LEVEL_2=6;break;		
		case 6:PWM_LEVEL_2=8;break;
		case 8:PWM_LEVEL_2=10;break;		
	}
}
void decelerate()
{
	switch(PWM_LEVEL_1)
	{
		case 10:PWM_LEVEL_1=8;break;		
		case 8:PWM_LEVEL_1=6;break;
		case 6:PWM_LEVEL_1=4;break;
		case 4:PWM_LEVEL_1=2;break;
	}
	switch(PWM_LEVEL_2)
	{
		case 10:PWM_LEVEL_2=8;break;		
		case 8:PWM_LEVEL_2=6;break;
		case 6:PWM_LEVEL_2=4;break;
		case 4:PWM_LEVEL_2=2;break;		
	}
}
void find_track()
{
	if(left_led1 == 0 && right_led1 == 0)//左右没识别到黑线
	{
		//accelerate();
		go_forward();//前进
		//decelerate();
		delay(4,0);//18650:40
			go_stop();
	}
	else
	{
		if(left_led1 == 1 && right_led1 == 0)//小车右边出线
		{
			
			go_left();//左转
			delay(10,0); //18650:310
			go_stop();
		}
		if(left_led1 == 0 && right_led1 == 1)//小车左边出线
		{
			
			go_right();//右转
			delay(10,0);
			go_stop();
		}
		/*if(left_led1 == 0 && right_led1 == 0)//左右都没识别到黑线
		{
			go_back();//后退
			delay(20,0);
			go_stop();
		}		*/
	}
}
void auto_avoid()
{
	//PT0 = 1;//防止PWM影响US
	measure_distance();
	go_forward();
	if(Distance>250)
	{
	if(R_LED==0&&L_LED==1)
	{
		go_left();
	}
	if(R_LED==1&&L_LED==0)
	{
	 go_right();
	}
}
	if(Distance<=250)
	{
		if(R_LED_DOWN==0&&L_LED_DOWN==1)
		{
			go_left();
		}
		if(R_LED_DOWN==1&&L_LED_DOWN==0)
		{
			go_right();
		}
		if(R_LED_DOWN==0&&L_LED_DOWN==0)
		{
			rotate_r();
		}
		if(R_LED_DOWN==1&&L_LED_DOWN==1)
		{			
			if(R_LED==0&&L_LED==1)
			{
				go_left();
			}
			if(R_LED==1&&L_LED==0)
			{
				go_right();
			}
			if(R_LED==0&&L_LED==0)
			{
				rotate_r();
			}
			if(R_LED==1&&L_LED==1)
			{
				rotate_r();
			}
		}
	}
}

void auto_follow()//物体跟随
{

	//PT0 = 1;//防止PWM影响US
	measure_distance();
	//为0 识别障碍物 为1没有识别到障碍物
	if((300<Distance && Distance < 700))
	{	/*
		if(left_led2 == 0 && right_led2 == 0)//左右识别到障碍物，前进跟随
		{
			go_forward();//前进
			delay(10,0);
			go_stop();
		}
		if(left_led2 == 1 && right_led2 == 0)//小车右侧识别到障碍物，右转跟随
		{
			go_right();//右转
		}
		if(left_led2 == 0 && right_led2 == 1)//小车左侧识别到障碍物，左转跟随
		{
			go_left();//左转
		}*/
			go_forward();//前进
			delay(10,0);
	}
	else if(R_LED_DOWN==0||L_LED_DOWN==0)
	{
		if(R_LED_DOWN==0)
		{
			R_follow();
		}
		else
		{
			L_follow();	
		}
	}
	else
	{
		go_stop();
	}
	/*else if(Distance<500)
	{
		go_back();//
		delay(10,0);

	}
	else  
	{
		go_stop();
		delay(5,0);
	}	*/
}

void R_follow()
{
	
	if(R_LED_DOWN==0&&R_LED==0)
	{	
		go_forward();
	}
	if(R_LED_DOWN==0&&R_LED==1)
	{
		go_fight();
	}
	if(R_LED_DOWN==1&&R_LED==1)
	{
		stop();
	}

}

void L_follow()
{
	
	if(L_LED_DOWN==0&&L_LED==0)
	{	
		go_forward();
	}
	if(L_LED_DOWN==0&&L_LED==1)
	{
		go_left();
	}
	if(L_LED_DOWN==1&&L_LED==1)
	{
		stop();
	}

}
