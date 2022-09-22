#include "warning.h"
#include "config.h"
#include "supervisor.h"
#include "shoot.h"
#include "auto_task.h"
#include "type_robot.h"
#include "local_id.h"
#include "imu.h"
#include "rc.h"
#include "judge.h"
#include "vision.h"
#include "power.h"

warningStruct_t warningData;

//������˸
void light_blink_switch(uint32_t loops,uint8_t freq,uint8_t __switch){
	uint8_t invertFreq = 0;
	invertFreq = (1000 / WARNING_STACK_PERIOD) / freq;      //10/Ƶ��
	if(!(loops % invertFreq)){
		__switch = !__switch;
		if(invertFreq == 10)            //Ƶ�ʣ�freq��Ϊ1����
			__switch = ENABLE;
	}
	if(!__switch)
		warningData.displayColor = SK6812_DARK;
}

//����������ʾ
void light_alone_control(uint8_t light_pos,hsvColor_t *op_display_color){
	setOneLedHsv(light_pos,op_display_color);
}

//��������������ʾ
void light_sequence_control(uint8_t light_index,uint8_t display_num,hsvColor_t *op_display_color,hsvColor_t *cl_display_color){
	while(light_index < SK6812_LED_STRIP_LENGTH){
		if(light_index < display_num)
			setOneLedHsv(light_index,op_display_color);
		else
			setOneLedHsv(light_index,cl_display_color);
		light_index ++;
	}
}

//�����л�����״̬��״̬��		
void lightBarsStateSwitch(uint16_t state,uint8_t valve){																																			
	if (valve)
        //�Ѷ�Ӧλ��1
		warningData.lightBarsState.u16_temp |= state;											
	else
        //�Ѷ�Ӧλ��0
		warningData.lightBarsState.u16_temp &= ~state;											
}

//���ڼ�鵱ǰ����״̬
void lightBarsErrorCheck(void){																		
    //(1)���ң�����Ƿ�ʧ  0x0001		
	if(supervisorData.state & STATE_RADIO_LOSS){												
		lightBarsStateSwitch(RC_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(RC_FAULT,DISABLE);
	}
    //(2)������ϵͳ�Ƿ����  0x0002
	if(supervisorData.state & STATE_JUDGE_ERROR){												
		lightBarsStateSwitch(JUDGE_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(JUDGE_FAULT,DISABLE);
	}
	//(3)����������Ƿ����  0x0004
	if(supervisorData.state & STATE_SENSOR_IMU_ERROR){											
		lightBarsStateSwitch(SENSOR_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(SENSOR_FAULT,DISABLE);
	}
    //(4)��⶯��ϵͳ����̨����Ƿ����  0x0008
	if(supervisorData.state & STATE_MOTOR_ERROR){												
		lightBarsStateSwitch(MOTOR_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(MOTOR_FAULT,DISABLE);
	}
    //(5)���TX2�Ƿ�ͨ��  0x0010
	if(supervisorData.state & STATE_VISION_ERROR){											
		lightBarsStateSwitch(VISION_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(VISION_FAULT,DISABLE);
	}
	//(6)˫��ͨ�ż��	0x0020
	if(supervisorData.state & STATE_TRANS_ERROR){											
		lightBarsStateSwitch(DOUBLE_TRANS_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(DOUBLE_TRANS_FAULT,DISABLE);
	}
    //(7)�����ݰ��Ƿ����  0x0040
	if(supervisorData.state & STATE_CAPACITANCE_ERROR){																						
		lightBarsStateSwitch(POWER_LIMIT_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(POWER_LIMIT_FAULT,DISABLE);
	}
}
//�����������״̬
uint8_t check_shooter_working(void){
	static uint8_t result = 0x00;
	uint32_t lastErr_fricL   = 0;
	uint32_t lastErr_fricR   = 0;
	uint32_t lastErr_trigger = 0;
	uint32_t lastErr_supply  = 0;
	//�����ų�����
	//��Ħ���ֹ���
	if(commandoMotorConfig[FRICLMOTOR].errorCount - lastErr_fricL == 0)
		result = 1 << 0;
	//��Ħ���ֹ���
	else if(commandoMotorConfig[FRICRMOTOR].errorCount - lastErr_fricR == 0)
		result = 1 << 1;
	//�����̹���
	else if(commandoMotorConfig[POKEMOTOR].errorCount - lastErr_trigger == 0)
		result = 1 << 2;
	//���ո�
	else if(commandoMotorConfig[SUPPLYMOTOR].errorCount - lastErr_supply == 0)
		result = 1 << 3;
	else
		result = 0x00;
	
	//���¼���
	lastErr_fricL   = commandoMotorConfig[FRICLMOTOR].errorCount;
	lastErr_fricR   = commandoMotorConfig[FRICRMOTOR].errorCount;
	lastErr_trigger = commandoMotorConfig[POKEMOTOR].errorCount;
	lastErr_supply  = commandoMotorConfig[SUPPLYMOTOR].errorCount;
	
	return result;
}
//��̨��������״̬
uint8_t check_gimbal_working(void){
	static uint8_t result = 0x00;
	uint32_t lastErr_pitch = 0;
	uint32_t lastErr_yaw = 0;
	//�����ų�����
	//yaw�����
	if(commandoMotorConfig[YAWMOTOR].errorCount - lastErr_yaw == 0)
		result = 1 << 0;
	//pitch�����
	else if(commandoMotorConfig[PITCHMOTOR].errorCount - lastErr_pitch == 0)
		result = 1 << 1;
	else
		result = 0x00;
	
	//���¼���
	lastErr_yaw   = commandoMotorConfig[YAWMOTOR].errorCount;
	lastErr_pitch = commandoMotorConfig[PITCHMOTOR].errorCount;
	
	return result;
}
//�ƴ�״̬������
void lightBarsStateUpdata(void){
    static uint8_t lightBarsSwitch = ENABLE;
	uint8_t shooter_process = 0x00;
	uint8_t gimbal_process = 0x00;
	/*****************************************1*****************************************/
	//Ħ����ʹ��
	if(getshootData()->fricMotorFlag)
		warningData.displayColor = SK6812_GREEN;    //ʹ���̵�
	else
		warningData.displayColor = SK6812_RED;      //��ʹ���̵�
	light_alone_control(0,&warningData.displayColor);  //0�����һ�ŵ�
	/*****************************************2*****************************************/
	//�����ȫ�ȼ�
	if(getshootData()->shootStatusMode_17mm == SAFE){
		warningData.displayColor = SK6812_GREEN;
		warningData.blinkFrequency = 1;             //�����ȫ�̵�
	}
	else{
		warningData.displayColor = SK6812_RED;
		warningData.blinkFrequency = 5;             //����ȫ�����
	}
	light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);  //������˸����
	light_alone_control(1,&warningData.displayColor); //1����ڶ��ŵ�
	/*****************************************3*****************************************/
	//����ȼ�
	switch(getshootData()->speed_limit){
		case SPEED_17MM_LOW:
			warningData.displayColor = SK6812_GREEN;     //17mm 15m/s  �̵�
			break;
		case SPEED_17MM_MID:
			warningData.displayColor = SK6812_YELLOW;    //17mm 18m/s  �Ƶ�
			break;
		case SPEED_17MM_HIGH:
			warningData.displayColor = SK6812_BLUE;      //17mm 22m/s  ����
			break;
		case SPEED_17MM_EXTREME:
			warningData.displayColor = SK6812_WHITE;     //17mm 30m/s  �׵�
			break;  
		case SPEED_42MM_LOW:
			warningData.displayColor = SK6812_GREEN;     //42mm 10m/s  �̵�
			break;
		case SPEED_42MM_MID:
			warningData.displayColor = SK6812_YELLOW;    //42mm 12m/s  �Ƶ�
			break;
		case SPEED_42MM_HIGH:
			warningData.displayColor = SK6812_BLUE;      //42mm 14m/s  ����
			break;
		case SPEED_42MM_EXTREME:
			warningData.displayColor = SK6812_WHITE;     //42mm 16m/s  �׵�
			break;
		default:
			warningData.displayColor = SK6812_PINK;      //�޲���ϵͳʱ������ �۵�
			break;
	}
	light_alone_control(2,&warningData.displayColor);//2��������ŵ�
	/*****************************************4*****************************************/
	//���ʵȼ�
	switch(get_judgeData( )->extGameRobotState.chassis_power_limit){
		case 40:
			warningData.displayColor = SK6812_GREEN;    //����40 ���̵���
		  warningData.blinkFrequency = 5;  
      break;		
		case 45:
			warningData.displayColor = SK6812_GREEN;    //����45 ���̵Ƴ���
		  warningData.blinkFrequency = 1;
			break;
		case 50:
			warningData.displayColor = SK6812_YELLOW;   //����50 ���Ƶ���
		  warningData.blinkFrequency = 5; 
			break;
		case 55:
			warningData.displayColor = SK6812_YELLOW;   //����55 ���ƵƳ���
		  warningData.blinkFrequency = 1; 
			break;
		case 60: 
			warningData.displayColor = SK6812_BLUE;     //����60 ��������
	  	warningData.blinkFrequency = 5;     
			break;
		case 65: 
			warningData.displayColor = SK6812_BLUE;     //����65 �����Ƴ���
	  	warningData.blinkFrequency = 1;     
			break;		
		case 70:
			warningData.displayColor = SK6812_PURPLE;   //����70 ���ϵ���
		  warningData.blinkFrequency = 5;      
			break;
		case 80:
			warningData.displayColor = SK6812_PURPLE;   //����80 ���ϵƳ���
			warningData.blinkFrequency = 1; 
		break;
		case 90:
			warningData.displayColor = SK6812_WHITE;    //����90 ���׵���
		  warningData.blinkFrequency = 5;  
			break;
		case 100:
			warningData.displayColor = SK6812_WHITE;    //����100 ���׵Ƴ���
		  warningData.blinkFrequency = 5;  
			break;		
		case 120:
			warningData.displayColor = SK6812_PINK;     //����120 ���۵���
		  warningData.blinkFrequency = 5;  
			break;		
		default:
			warningData.displayColor = SK6812_PINK;     //����ϵͳ�����ع��� ���۵Ƴ���
		  warningData.blinkFrequency = 1;
			break;
	}
	light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);  //������˸����
	light_alone_control(3,&warningData.displayColor);//3������Ŀŵ�
	/*****************************************5*****************************************/
	//�����������״̬
	shooter_process = check_shooter_working();
	switch(shooter_process){
		//�������̵�
		case 0x00:
			warningData.displayColor = SK6812_GREEN;
			warningData.blinkFrequency = 1;
			break;
		//��Ħ���ֹ��ϣ��Ƶ���
		case 0x01:
			warningData.displayColor = SK6812_YELLOW;
			warningData.blinkFrequency = 2;
			break;
		//��Ħ���ֹ��ϣ�������
		case 0x02:
			warningData.displayColor = SK6812_BLUE;
			warningData.blinkFrequency = 2;
			break;
		//�����̹��ϣ�  �����
		case 0x04:
			warningData.displayColor = SK6812_RED;
			warningData.blinkFrequency = 2;
			break;
		//���ոǹ��ϣ�  �۵���
		case 0x08:
			warningData.displayColor = SK6812_PINK;
			warningData.blinkFrequency = 2;
			break;
		//�������ϻ������ֹ��ϣ���Ƴ���
		default:
			warningData.displayColor = SK6812_RED;
		  warningData.blinkFrequency = 1;
		  break;
	}
	light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);//������˸����
	light_alone_control(4,&warningData.displayColor);//4�������ŵ�
	/*****************************************6*****************************************/
	//��̨��������״̬
	gimbal_process = check_gimbal_working();
	switch(gimbal_process){
		//����
		case 0x00:
			warningData.displayColor = SK6812_GREEN;
			warningData.blinkFrequency = 1;
			break;
		//yaw�����
		case 0x01:
			warningData.displayColor = SK6812_BLUE;
			warningData.blinkFrequency = 3;
			break;
		//pitch�����
		case 0x02:
			warningData.displayColor = SK6812_WHITE;
			warningData.blinkFrequency = 3;
			break;
	  //�����������ֹ��ϣ���Ƴ���
		default:
			warningData.displayColor = SK6812_RED;
		  warningData.blinkFrequency = 1;
		  break;
	}
	light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);//������˸����
	light_alone_control(5,&warningData.displayColor);//5��������ŵ�
	/*****************************************7*****************************************/
	//���̻�������״̬
	switch(getchassisData()->chassis_fault){
		//�������̵�
		case 0x00:
			warningData.displayColor = SK6812_GREEN;
			warningData.blinkFrequency = 1;
			break;
		//��ǰ�ֹ��ϣ��Ƶ���
		case 0x01:
			warningData.displayColor = SK6812_YELLOW;
			warningData.blinkFrequency = 5;
			break;
		//��ǰ�ֹ��ϣ�������
		case 0x02:
			warningData.displayColor = SK6812_BLUE;
			warningData.blinkFrequency = 5;
			break;
		//����ֹ��ϣ��۵���
		case 0x04:
			warningData.displayColor = SK6812_PINK;
			warningData.blinkFrequency = 5;
			break;
		//�Һ��ֹ��ϣ������
		case 0x08:
			warningData.displayColor = SK6812_RED;
			warningData.blinkFrequency = 5;
			break;
	}
	light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);//������˸����
	light_alone_control(6,&warningData.displayColor);//6������߿ŵ�
}


//���ڱ���
/*
һ������λ��ʾһ�ֻ����˵Ĵ���״̬
��Ʊ�ʾ���ڴ����̵Ʊ�ʾ����
��ߴ���λ���ڽ�����Ч��,���������չ���������ȼ���ߵĵ���
������Ч��:��ɫ->��ɫ(��ʾ�޴���)�����������̵�,֪���д���ĵ�ʱȫ����Ϊ��ɫ,�ٴν���ѭ��,�д���ĵ�Ϊ��ɫ
�������ȼ�����ߴ���״̬��:1�ƴ���>2�ƴ���>3�ƴ���>4�ƴ���>5�ƴ���>6�ƴ���>7�ƴ���......>16�ƴ���(���֧��16������)
*/
void lightBarsReportErrors(void){
	uint8_t index = 0;//����
	uint8_t nowFaultBit = 0; 
	uint16_t remainFaultSate;
	//��������Ҫ�����ĵ���λ(�д���״̬)
	if(warningData.lightBarsState.u16_temp != 0){
		//��ǰ����״̬
		remainFaultSate = warningData.lightBarsState.u16_temp;
        /**�����λ���鿪ʼ���������һ�ŵ���ȥ�ҵ���ߴ���״̬**/
	 	while(index < SK6812_LED_STRIP_LENGTH && !warningData.highestFaultFlag){    
			//��ǰ��index�ŵ���״̬
			nowFaultBit = remainFaultSate % 2;
			//���״̬����
			if(nowFaultBit){
				//��ߴ���״̬������ŵ���,��ȡ������
				warningData.highestFault = index;
				//��ߴ���״̬�ѻ�ȡ����������־����ѭ��
				digitalHi(&warningData.highestFaultFlag);
			}
			//��ǰ����״̬���ƣ���ȡ��һ�ŵ���״̬
			remainFaultSate = remainFaultSate >> 1;
			//����+1
			index++;
		}
		digitalLo(&warningData.highestFaultFlag);
	}
	//�������
	index = 0;
    /**�ڵ�һλ����ߴ���λ����һ������Ч��,���ƽ�����**/
	//������ߴ���״̬
	if(warningData.highestFault != 0){
		//����<=��ߴ���λ�ĵ���
		while(index < warningData.highestFault){
			//����>=���״̬ѭ��λ(���״̬ѭ��λ<=���״̬λ,ÿ��ѭ�����+1)
			if(index >= warningData.highestFaultLoop)
				//���øõ���Ϊ����
				setOneLedHsv(index,&SK6812_DARK);
			else
				//���øõ���Ϊ�̵�
				setOneLedHsv(index,&SK6812_GREEN);
			//����+1
			index ++;
		}
	}
	//����ߴ���λ������
	index = warningData.highestFault;
	//��ǰʣ��Ĵ���״̬
	remainFaultSate = warningData.lightBarsState.s16_temp >> warningData.highestFault;
	/**����С��������������ӵ�ǰ��ߴ���λ��ʼ���������һ������**/
	//��ʾ����״̬�ĵ��飨��ɫ�����޴���״̬�ĵ��飨��ɫ��
	while(index < SK6812_LED_STRIP_LENGTH){
		//��ǰ����λ��״̬
		nowFaultBit = remainFaultSate % 2;
		//�����ǰλ���ڴ���״̬
		if(nowFaultBit)
			//���øõ���Ϊ���
			setOneLedHsv(index,&SK6812_RED);
		//�����ڴ���״̬
		else
			//���øõ���Ϊ�̵�
			setOneLedHsv(index,&SK6812_GREEN);
		//��ǰ����״̬����1λ����ȡ��һ�ŵ���λ��״̬
		remainFaultSate = remainFaultSate >> 1;
		//����+1
		index ++;
	}
    //500msˢ��
	if(!(warningData.loops % 5)){
		//���״̬ѭ��λ����
		digitalIncreasing(&warningData.highestFaultLoop);
		//���״̬ѭ��λ�������״̬λ���ѭ��λ
		if(warningData.highestFaultLoop > warningData.highestFault)
			digitalClan(&warningData.highestFaultLoop);
	}
	else
		//�����ߴ���״̬
		digitalClan(&warningData.highestFault);
}

//����ϵͳ�������ݸ���
void lightBarsJudgeUpdate(void){
//	if(getvisionData()->captureFlag&&getvisionData()->cailSuccess)
//		judgeData.extShowData.data1 = UKF_POS_Z;
//	else
//		judgeData.extShowData.data1 = 0.0;
//	
//	//���ݵ����ٷֱȣ�17vΪ������ѹ
//	judgeData.extShowData.data2 = (float)(100 * (getpowerData() ->capVol - 17) / (getpowerData() ->capVolMax - 17));
//	//û����Ħ���ֵƲ���
//	if(!getshootData()->fricMotorFlag)	
//		//���ȫ��		
//		judgeData.extShowData.mask = 0x00;									
//	else{
//		if(getshootData()->shootMode_17mm == MANUAL_SINGLE){	
//			//����1�ŵ�
//			judgeData.extShowData.mask |= 0x01;
//			judgeData.extShowData.mask &= 0xF9;
//		}
//		else if(getshootData()->shootMode_17mm == MANUAL_CONTINUOUS){
//			//������2�ŵ�
//			judgeData.extShowData.mask |= 0x03;
//			judgeData.extShowData.mask &= 0xFB;
//		}
//		else if(getshootData()->shootMode_17mm == AUTO_CONTINUOUS){
//			//�������ŵ�
//			judgeData.extShowData.mask |= 0x07;
//		}
//		if(getinfantryAutoData()->rotateFlag)
//            //С����ģʽ			
//			judgeData.extShowData.mask |= 0x08;
//		else
//			judgeData.extShowData.mask &= 0xF7;
//		
//		if(getinfantryAutoData()->aviodFlag)
//			//Ť��ģʽ			
//			judgeData.extShowData.mask |= 0x10;
//		else
//			judgeData.extShowData.mask &= 0xEF;
//			//�����ݷŵ�
//		if(openCap)							   													
//			judgeData.extShowData.mask |= 0x20;
//		else
//			judgeData.extShowData.mask &= 0xDF;
//	}
}

/****************����˵��******************
    "SK6812_LED_STRIP_LENGTH"       ----> �ƴ��ĳ���  7
	"warningData.displayNumber"     ----> �����Ƶ���Ŀ����������
	"warningData.displayColor"      ----> �ƴ�����ɫ 
    "setOneLedHsv(index,&warningData.displayColor);" ----> �˺���Ϊ��ƺ��� index�������Ƶ���Ŀ����warningData.displayColor �ƴ�����ɫ
    "safeMode" ----> �����ȼ�   SAFE����ȫ�����ƴ�ȫ���̵�  DANGEROUS��Σ�գ����ƴ�ȫ�����
    "contrlMode" ----> ����ģʽ  MANUAL_SINGLE��������������һ�ŵ�  MANUAL_CONTINUOUS�������������������ŵ�  AUTO_CONTINUOUS�����������������ŵ�  
    �ƴ������Ƶ����ã������ң���
    ����1�ŵ�    �ƴ���ɫ�������ȫ״̬����
    ������2�ŵ�  �ƴ���ɫ�������ȫ״̬����
    ����3�ŵ�    �ƴ���ɫ�������ȫ״̬����
    ���������4�ŵ���ʾŤ��,С����    Ť������ƣ�  С���ݣ��̵ƣ�
    ���������5�ŵ���ʾ�Զ�����״̬	 X_TASK(�۵�)  R_TASK����ƣ�  V_TASK���̵ƣ� Z_TASK���Ƶƣ� G_TASK(����)  C_TASK(�׵�)������ʱ����
	���������6�ŵ���ʾ������̨�׻����Ŀ���״̬	 ��ֹ(������˸)	 ����(�׵Ƴ���)
    ���������7�ŵ���ʾ����״̬  �������Ϊ��ȫ�����������̵ƣ�   ���ݷŵ磨�̵ƣ���˸��  ���ݼ���û�磬���棨��ƣ� ������ʱ�Ʋ���
    �ڼ���ģʽ��û����Ħ���ֵ�ȫΪ��ɫ    
*******************************************/


//���ڿ���״̬
void lightBarsOfContrl(uint8_t shootContrlMode,uint8_t shootSafeMode){					
	uint8_t index = 0;//����
	static uint8_t lightBarsSwitch = ENABLE;
	/*****************************************1~3*****************************************/
    //���ģʽ
	switch(shootContrlMode){
		//�������
		case MANUAL_SINGLE:
			warningData.displayNumber = 1;//��1��
			break;
		//���������
		case MANUAL_CONTINUOUS:
			warningData.displayNumber = 2;//��2��
			break;
		//�������
		case AUTO_CONTINUOUS:
			warningData.displayNumber = 3;//��3��
			break;
	}
    //ʶ�������ȫ״̬
	switch(shootSafeMode){
		//��ȫ
		case SAFE:  
			warningData.blinkFrequency = 1;              //��ɫ����
			warningData.displayColor = SK6812_GREEN;
			break;
		//Σ��
		case DANGEROUS:
			warningData.blinkFrequency = 5;              //��ɫ��˸
			warningData.displayColor = SK6812_RED;
			break;
	}
	//������˸����Ч��
	light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);
	//��������ʾ��ʾ���ģʽ
	light_sequence_control(index,warningData.displayNumber,&warningData.displayColor,&SK6812_DARK);
	/*****************************************4*****************************************/
	//�Ҳ��4������ʾŤ��,С����
	//Ť�������
	if(autoTaskData->aviodFlag)
		warningData.displayColor = SK6812_RED;
	//С���ݣ��̵�
	else if(autoTaskData->rotateFlag)
		warningData.displayColor = SK6812_GREEN;
	else
		warningData.displayColor = SK6812_DARK;

	light_alone_control(SK6812_LED_STRIP_LENGTH - 4,&warningData.displayColor);
	/*****************************************5*****************************************/
	//�Ҳ��3������������ʾ����״̬
    //���������Ҳ�ĵ���������ʾ����״̬
	switch(autoTaskData->currentTask){
		case X_TASK:     //�۵�  ������������
			warningData.displayColor = SK6812_PINK;													
			break;
		case R_TASK:     //���  �������ӵ�����
			warningData.displayColor = SK6812_RED;													
			break;	
		case V_TASK:     //�̵�  ������������׼
			warningData.displayColor = SK6812_GREEN;
			break;
		case Z_TASK:     //����  �������������
			warningData.displayColor = SK6812_BLUE;
			break;
		case G_TASK:     //�Ƶ�  ������������
			warningData.displayColor = SK6812_YELLOW;
			break;
		case C_TASK:     //�׵�  ��������̨��
			warningData.displayColor = SK6812_WHITE;
			break;
		default:         //������ ����
			warningData.displayColor = SK6812_DARK;
			break;
	}
	
	light_alone_control(SK6812_LED_STRIP_LENGTH - 3,&warningData.displayColor);
	/*****************************************6*****************************************/
	//�Ҳ��2��������ʾ��������̨�׻���
	if(ROBOT == INFANTRY_ID){
		if(getrobotMode() == MODE_KM){                   
			if(get_infDeforming()->up_step_switch){
				warningData.displayColor = SK6812_WHITE;     //�׵� ����������ͬ�������ҿ��Կ��ƻ�е��
				warningData.blinkFrequency = 1;
			}
			else{
				warningData.displayColor = SK6812_BLUE;     //����  ������ͬ����ֹͣ�˶���ʱ���ɿ��ƻ�е��
				warningData.blinkFrequency = 3;
			}
		}
		else
			warningData.displayColor = SK6812_DARK;
		light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);
		light_alone_control(SK6812_LED_STRIP_LENGTH - 2,&warningData.displayColor);
	}
	/*****************************************7*****************************************/
	//���ұߵĵ���ʾ����״̬
  if(ROBOT == INFANTRY_ID || ROBOT == F_TANK_ID){
		 if(getcapacitorData()->percentage < 40)
			  //�ٷ�֮��ʮ����ɫ
				warningData.displayColor = SK6812_RED;
         else if(getcapacitorData()->percentage < 60)
			 //�ٷ�֮��ʮ����ɫ
				warningData.displayColor = SK6812_YELLOW;
         else if(getcapacitorData()->percentage < 80)
			 //�ٷ�֮��ʮ����ɫ 
				warningData.displayColor = SK6812_PINK;
		 else if(getcapacitorData()->percentage < 85)
			 //�ٷ�֮��ʮ�壺��ɫ 
				warningData.displayColor = SK6812_WHITE;
		 else if(getcapacitorData()->percentage < 90)
			 //�ٷ�֮��ʮ����ɫ
				warningData.displayColor = SK6812_BLUE;
         else if(getcapacitorData()->percentage <= 100)
	         //�ٷְ٣�    ��ɫ
				warningData.displayColor = SK6812_GREEN;
		 
	switch(getcapacitorData()->capacitor_status){
			case Charging:      //��磺��˸
				warningData.blinkFrequency = 2;
			break;
			case Discharging:   //�ŵ磺����
				warningData.blinkFrequency = 1;
			break;
			case Protection:   //����������
				warningData.displayColor = SK6812_DARK;
			break;
		}
		light_blink_switch(warningData.loops,warningData.blinkFrequency,lightBarsSwitch);
		light_alone_control(SK6812_LED_STRIP_LENGTH - 1,&warningData.displayColor);
	}
	/*****************************************ALL*****************************************/
	if(getrobotMode() == MODE_KM && !getshootData()->fricMotorFlag){   //KM����ģʽ��û�п�Ħ���֣��ƴ�ȫ��
		setAllLedColors(&SK6812_RED);
	}
}


//�ƴ�״̬����
void lightBarsUpdate(void){																						
    //��ȡ����
	lightBarsErrorCheck();																		
    //���������ң�ص�״̬	û�ж���						
	if(!(supervisorData.state & STATE_RADIO_LOSS)){											
		if(supervisorData.state & STATE_DISARMED || remoteControlData.dt7Value.keyBoard.bit.CTRL)  
            //û�н���(�¿�)���ڼ���ģʽ�°���CTRL��ʾ��ǰ�����˹���״̬
			lightBarsReportErrors();
        //�����ӵ�п���Ȩ������²���û�а���CTRL
		else{
			//ң����ģʽ
			if(getrobotMode() == MODE_RC)
				//�ƴ�״̬������            
				lightBarsStateUpdata();
			//����ģʽ
			else if(getrobotMode() == MODE_KM){
				switch(robotConfigData.typeOfRobot){
					case INFANTRY_ID:
						lightBarsOfContrl(getshootData()->shootMode_17mm,getshootData()->shootStatusMode_17mm); 
						break;
					case P_TANK_ID:
						lightBarsOfContrl(getshootData()->shootMode_17mm,getshootData()->shootStatusMode_17mm);
						break;
					//���̳����˻����滹�����ǵ�����	
					case AUXILIARY_ID:																							
						break;
					//�ڱ����õƴ�
					case SENTRY_ID:																								
						break;
					case UAV_ID:
						lightBarsOfContrl(getshootData()->shootMode_17mm,getshootData()->shootStatusMode_17mm);
						break;
					case F_TANK_ID:
						lightBarsOfContrl(getshootData()->shootMode_17mm,getshootData()->shootStatusMode_17mm);
				}
			}
		}
	}
	else
	{
		lightBarsReportErrors();    //����
	}
	SK6812UpdateStrip();
    
} 


void warningUpdate(void){
    //�ƴ���׼ɫ��ʼ��  ��дɫ��
	colorStdInit();
    //sk6812 ����
    lightBarsUpdate();																									
    //����ϵͳ�������ݸ���
//	lightBarsJudgeUpdate();														
    //LED����		���ذ���3ɫLED����ָʾ״̬										
	appSightClass.led(supervisorData.ledState);                           
    //��������������
	appSightClass.beep(supervisorData.beepState);												 		
    //״̬����
	digitalClan(&supervisorData.beepState);													  	
	digitalIncreasing(&warningData.loops);
}
