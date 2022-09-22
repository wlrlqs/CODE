#include "tf_card_parameter.h"
#include "config.h"

uint8_t tfFATFS_Init(void);
uint8_t tfWriteOneParameter(uint8_t id, const uint8_t* strPara, float parameter);
uint8_t tfReadOneParameter(uint8_t id, const uint8_t* strPara, float* parameter);

FATFS FATFS_TF; //�ļ�ϵͳĿ��

/*
***************************************************
��������tfFATFS_Init
���ܣ�TF����FATFS��ʼ��
��ڲ�������
����ֵ����ʼ�����
				0x00������
				0x10��(STA_NOINIT<<4)��TF����ʼ��ʧ��
				���ࣺ�뿴FRESULTö�ٶ���
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
uint8_t tfFATFS_Init(void){
	uint8_t res;
    //��ʼ��TF�����̣���ȡ״̬
	res = disk_initialize(0);	
	if(res){
        //Ϊ��ֹ��������غ���������ͻ��������4λ
		return res<<4;					
	}
    //��������SD��
	res = f_mount(&FATFS_TF,"0:",1);	
	return res;
}

uint8_t tFCardConfig(void){
	BSP_GPIO_Init(TFCARD_INT,GPIO_Mode_IPU);
    //���������⵽��ֱ�ӳ�ʼ��һ��
	if(TFCARD_INSERT_IO == TFCARD_INSERT){						
		parameterRunData.TFError = tfFATFS_Init();	
	}
	return parameterRunData.TFError;
}

/*
***************************************************
��������f_readParaLine
���ܣ���ȡ��Ų�����һ��
��ڲ�����fp���ļ�ָ��
				strPara�������ַ���
				strLine������������ַ���
����ֵ����ȡ�����0���ɹ���1��ʧ��
Ӧ�÷�Χ���ڲ�����
��ע��
***************************************************
*/
uint8_t f_readParaLine(FIL* fp, const uint8_t* strPara, uint8_t* strLine){
    //�ݴ�ÿһ���ַ���
	uint8_t arrayTemp[64];	
	uint8_t res = 1;
	TCHAR* pr_res = NULL;
	//�ƶ��ļ�����ָ�뵽�ʼ
	f_lseek(fp,0);	
	
	do{
        //��ȡһ���ַ���(��"\r\n"��β)
		pr_res = f_gets ((TCHAR *)arrayTemp, 64, fp);	
		if(strstr((const char *)arrayTemp,(const char *)strPara) != NULL)
		{
			//�жϲ����ַ��������Ƿ����'='����ֹ�������ַ������������ַ���
			if(arrayTemp[strlen((const char *)strPara)] == '=')
			{
				res = 0;
				break;
			}
		}
        //�ж��Ƿ�����ļ�
	}while(pr_res != NULL);	
	
	if(res == 0)
	{
		strcpy((char *)strLine,(const char *)arrayTemp);
	}
	return res;
}

/*
***************************************************
��������tfOpenFileID
���ܣ���ID��Ӧ�ļ�
��ڲ�����fp���ļ�ָ��
					id��ID��
				mode���ļ���ģʽ
����ֵ���뿴ö��FRESULT
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
FRESULT tfOpenFileID(FIL* fp, uint8_t id, BYTE mode){
	FRESULT res;
    //�ݴ�ID��Ӧ�ļ���·��
	char pathFileID[16];	
	//��ȡID��Ӧ���ļ�·��
	sprintf(pathFileID, "0:/RM/ID%d.txt", (int)id);
	res = f_open(fp, pathFileID, mode);
	if(res) 	
        //�ر��ļ�
		f_close(fp);
	return res;
}

/*
***************************************************
��������tfOverwrite
���ܣ���д����
��ڲ�����id��ID��
				strPara�������ַ���ָ��
				parameter���������
				amount������
����ֵ����ʼ�����
				0x00������
				0x10��(STA_NOINIT<<4)��TF����ʼ��ʧ��
				���ࣺ�뿴FRESULTö�ٶ���
Ӧ�÷�Χ���ⲿ����
��ע����������ڶ�Ӧ��ID�ŵ��ļ����½�һ���ļ�
***************************************************
*/

uint8_t tfOverwrite(uint8_t id, systemConfig_t* systemConfigData()){
	FIL fpID;		//ID�ļ�
	char strLine[32];			//�ݴ�ÿһ�в����ַ���
	uint16_t index = 0;
    uint16_t amount = 0;
	uint8_t res;
	//����ID�ļ��Լ��Զ�д�ķ�ʽ�򿪣�������򸲸�
	res = tfOpenFileID(&fpID, id, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
	//�����ʧ�����˳�
	if(res)		return res;
    amount = sizeof(systemConfig_t)/sizeof(float);
	f_lseek(&fpID,0);
	while(index < amount){
        //���ݴ��ļ�д�����ຯ��
		sprintf(strLine, "%s=%g\r\n",configTFStrings[index],*(&(systemConfigData()->configVersion)+index));
        f_puts(strLine, &fpID);
		index ++;
	}
	f_close(&fpID);
	return 0;
}

/*
***************************************************
��������tfOverread
���ܣ�ͨ������
��ڲ�����id��ID��
				strPara�������ַ���ָ��
				parameter���������
				amount������
����ֵ����ʼ�����
				0x00������
				0x10��(STA_NOINIT<<4)��TF����ʼ��ʧ��
				���ࣺ�뿴FRESULTö�ٶ���
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
uint8_t tfOverread(uint8_t id, systemConfig_t* systemConfigData()){
	FIL fpID;		        //ID�ļ�
	char strLine[64];		//�ݴ�ÿһ�в����ַ���
	uint16_t index = 0;
    uint16_t amount = 0;
	uint8_t res;
	//���Ѵ��ڵ�ID�ļ��Լ��Զ�ȡ�ķ�ʽ��
	res = tfOpenFileID(&fpID, id, FA_OPEN_EXISTING | FA_READ);
	//�����ʧ�����˳�
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



