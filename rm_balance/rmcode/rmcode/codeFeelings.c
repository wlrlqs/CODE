/**************************************************************
			彭源：原代码时钟配置如下，描述为1s中断一
			次，但我计算出的结果为1.07s，而且我认为ARR
			寄存器为16位寄存器，设置值1000000已经超
			过了其最大范围，是有问题的。              
**************************************************************/
#define CLOCKCOUNT_PRESCALER 90-1
/*1s 中断一次*/
#define CLOCKCOUNT_PERIOD    1000000-1     
/**************************************************************
			彭源：所以我将配置改为如下配置，定时器1s
			中断一次，根据中心代码计算方式，计时精度
			可到0.0001s。            
**************************************************************/
#define CLOCKCOUNT_PRESCALER 8400-1
/*1s 中断一次*/
#define CLOCKCOUNT_PERIOD    10000-1     




/***************************************************************
			彭源：此代码定时器1s中断一次，但计时精度
			可以达到0.0001s。
***************************************************************/
double getClockCount(void){
	clockCountData.clockTick  = (double)(clockCountData.saveTimer*10000 + TIM5->CNT)*1e-4f; 
	return clockCountData.clockTick;	
}
/****************************************************************/