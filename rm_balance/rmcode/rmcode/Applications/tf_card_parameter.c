#include "tf_card_parameter.h"
#include "config.h"

uint8_t tfFATFS_Init(void);
uint8_t tfWriteOneParameter(uint8_t id, const uint8_t* strPara, float parameter);
uint8_t tfReadOneParameter(uint8_t id, const uint8_t* strPara, float* parameter);

FATFS FATFS_TF; //文件系统目标

/*
***************************************************
函数名：tfFATFS_Init
功能：TF卡与FATFS初始化
入口参数：无
返回值：初始化结果
				0x00：正常
				0x10：(STA_NOINIT<<4)，TF卡初始化失败
				其余：请看FRESULT枚举定义
应用范围：外部调用
备注：
***************************************************
*/
uint8_t tfFATFS_Init(void){
	uint8_t res;
    //初始化TF卡磁盘，获取状态
	res = disk_initialize(0);	
	if(res){
        //为防止与下面挂载函数反馈冲突将其左移4位
		return res<<4;					
	}
    //立即挂载SD卡
	res = f_mount(&FATFS_TF,"0:",1);	
	return res;
}

uint8_t tFCardConfig(void){
	BSP_GPIO_Init(TFCARD_INT,GPIO_Mode_IPU);
    //开机如果检测到就直接初始化一次
	if(TFCARD_INSERT_IO == TFCARD_INSERT){						
		parameterRunData.TFError = tfFATFS_Init();	
	}
	return parameterRunData.TFError;
}

/*
***************************************************
函数名：f_readParaLine
功能：读取存放参数的一行
入口参数：fp：文件指针
				strPara：参数字符串
				strLine：输出参数行字符串
返回值：读取结果，0：成功，1：失败
应用范围：内部调用
备注：
***************************************************
*/
uint8_t f_readParaLine(FIL* fp, const uint8_t* strPara, uint8_t* strLine){
    //暂存每一行字符串
	uint8_t arrayTemp[64];	
	uint8_t res = 1;
	TCHAR* pr_res = NULL;
	//移动文件对象指针到最开始
	f_lseek(fp,0);	
	
	do{
        //读取一行字符串(以"\r\n"结尾)
		pr_res = f_gets ((TCHAR *)arrayTemp, 64, fp);	
		if(strstr((const char *)arrayTemp,(const char *)strPara) != NULL)
		{
			//判断参数字符串后面是否接着'='，防止出现行字符串包含参数字符串
			if(arrayTemp[strlen((const char *)strPara)] == '=')
			{
				res = 0;
				break;
			}
		}
        //判断是否读完文件
	}while(pr_res != NULL);	
	
	if(res == 0)
	{
		strcpy((char *)strLine,(const char *)arrayTemp);
	}
	return res;
}

/*
***************************************************
函数名：tfOpenFileID
功能：打开ID对应文件
入口参数：fp：文件指针
					id：ID号
				mode：文件打开模式
返回值：请看枚举FRESULT
应用范围：外部调用
备注：
***************************************************
*/
FRESULT tfOpenFileID(FIL* fp, uint8_t id, BYTE mode){
	FRESULT res;
    //暂存ID对应文件的路径
	char pathFileID[16];	
	//获取ID对应的文件路径
	sprintf(pathFileID, "0:/RM/ID%d.txt", (int)id);
	res = f_open(fp, pathFileID, mode);
	if(res) 	
        //关闭文件
		f_close(fp);
	return res;
}

/*
***************************************************
函数名：tfOverwrite
功能：覆写参数
入口参数：id：ID号
				strPara：参数字符串指针
				parameter：输出参数
				amount：数量
返回值：初始化结果
				0x00：正常
				0x10：(STA_NOINIT<<4)，TF卡初始化失败
				其余：请看FRESULT枚举定义
应用范围：外部调用
备注：如果不存在对应的ID号的文件则新建一个文件
***************************************************
*/

uint8_t tfOverwrite(uint8_t id, systemConfig_t* systemConfigData()){
	FIL fpID;		//ID文件
	char strLine[32];			//暂存每一行参数字符串
	uint16_t index = 0;
    uint16_t amount = 0;
	uint8_t res;
	//创建ID文件以及以读写的方式打开，如存在则覆盖
	res = tfOpenFileID(&fpID, id, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
	//如果打开失败则退出
	if(res)		return res;
    amount = sizeof(systemConfig_t)/sizeof(float);
	f_lseek(&fpID,0);
	while(index < amount){
        //向暂存文件写入其余函数
		sprintf(strLine, "%s=%g\r\n",configTFStrings[index],*(&(systemConfigData()->configVersion)+index));
        f_puts(strLine, &fpID);
		index ++;
	}
	f_close(&fpID);
	return 0;
}

/*
***************************************************
函数名：tfOverread
功能：通读参数
入口参数：id：ID号
				strPara：参数字符串指针
				parameter：输出参数
				amount：数量
返回值：初始化结果
				0x00：正常
				0x10：(STA_NOINIT<<4)，TF卡初始化失败
				其余：请看FRESULT枚举定义
应用范围：外部调用
备注：
***************************************************
*/
uint8_t tfOverread(uint8_t id, systemConfig_t* systemConfigData()){
	FIL fpID;		        //ID文件
	char strLine[64];		//暂存每一行参数字符串
	uint16_t index = 0;
    uint16_t amount = 0;
	uint8_t res;
	//打开已存在的ID文件以及以读取的方式打开
	res = tfOpenFileID(&fpID, id, FA_OPEN_EXISTING | FA_READ);
	//如果打开失败则退出
	if(res)		return res;
    amount = sizeof(systemConfig_t)/sizeof(float);
	f_lseek(&fpID,0);
	while(index < amount){
		f_gets(strLine,64,&fpID);
		sscanf(strLine,"%*s%g",(&(systemConfigData()->configVersion)+index));
		index ++;
	}
	f_close(&fpID);
	return 0;
}



