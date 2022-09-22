#include "keyboard.h"
#include "ramp.h"
#include "control.h"
#include "gimbal.h"
#include "chassis.h"
#include "vision.h"
#include "shoot.h"
#include "deforming.h"
#include "rc.h"
#include "math.h"
#include "config.h"

/*--------------------------------------------- 
-----------------------------------------------
//		���棺�����������򣬹��������ش�		 //
//			���ض�����������ɿ��Ը����⣬		 //
//	���ಿ�ָ��ģ����빫����������ȡ�ø���Ȩ��  // 
-----------------------------------------------
---------------------------------------------*/

keyBoardCtrlStruct_t keyBoardCtrlData;
keyBoardState_e leftKeyDriver(void){      
	static uint8_t keyState = 0;         // ����״̬����
	static uint16_t keyTime = 0;         // ������ʱ����
	uint8_t keyPress;
	keyBoardState_e keyReturn; 
    // ��� ���ذ���ֵ
	keyReturn = KEY_RELEASE;                         			
    // ��ǰ��ֵ 
	keyPress = remoteControlData.dt7Value.mouse.Press_L;   
	switch (keyState){        
        // ����״̬0���ж����ް�������
		case KEY_STATE_0:{                 
            // �а�������
			if(keyPress){                
                // ����ʱ��������
				keyTime = 0;                  
                // Ȼ����� ����״̬1
				keyState = KEY_STATE_1;        
			}        
		}break;
        // ����״̬1�����������ȷ�������Ƿ���Ч���������󴥣���������Ч�Ķ��壺�����������³����趨������ʱ�䡣
		case KEY_STATE_1:{                  
			if(keyPress){                     
                // һ��5ms
				keyTime++;                     
                // ����
                // �������ʱ�䳬�� ����ʱ�䣬���ж�Ϊ���µİ�����Ч��������Ч�������֣��������߳��������� ����״̬2�� �����ж�������������Ч����
				if(keyTime>=SINGLE_PRESS_TIME){   
					keyState = KEY_STATE_2;      
				}
			}         
            // �������ʱ��û�г������ж�Ϊ�󴥣�������Ч������ ����״̬0�������ȴ�����
			else keyState = KEY_STATE_0;      
		}break; 
        // ����״̬2���ж�������Ч�����ࣺ�ǵ��������ǳ���
		case KEY_STATE_2:{                  
            // ��������� �趨�ĳ���ʱ�� ���ͷţ����ж�Ϊ���� 
			if(!keyPress){                
                // ���� ��Ч����ֵ������
				keyReturn = KEY_PRESS_ONCE;      
                // ���� ����״̬0�������ȴ�����
				keyState = KEY_STATE_0;         
			} 
			else{
				keyTime++;                     
                // �������ʱ�䳬�� �趨�ĳ���ʱ��
				if(keyTime >= LONG_PRESS_TIME){   
                    // ���� ��Ч��ֵֵ������
					keyReturn = KEY_PRESS_LONG;            
                    // ȥ״̬3���ȴ������ͷ�
					keyState = KEY_STATE_3;       
				}
			}
		}break;
        // �ȴ������ͷ�
		case KEY_STATE_3:{                   
			if (!keyPress){ 
                // �����ͷź󣬽��� ����״̬0 ��������һ�ΰ������ж�
				keyState = KEY_STATE_0;        
			}
			else	keyReturn = KEY_PRESS_LONG;				
		}break; 
	}
    // ���� ����ֵ
	return keyReturn;                     
}
keyBoardState_e rightKeyDriver(void){     
	volatile static uint8_t keyState = 0;    		        // ����״̬����	
	volatile static uint16_t keyTime = 0;                   // ������ʱ����
	uint8_t keyPress;
	keyBoardState_e keyReturn; 
    // ��� ���ذ���ֵ
	keyReturn = KEY_RELEASE;                         
    // ��ǰ��ֵ
	keyPress = remoteControlData.dt7Value.mouse.Press_R;   
	switch (keyState){       
        // ����״̬0���ж����ް�������
		case KEY_STATE_0:{                  
            // �а�������
			if(keyPress){               
                // ����ʱ��������
				keyTime = 0;                   
                // Ȼ����� ����״̬1
				keyState = KEY_STATE_1;        
			}        
		}break;
        // ����״̬1�����������ȷ�������Ƿ���Ч���������󴥣���������Ч�Ķ��壺�����������³����趨������ʱ�䡣
		case KEY_STATE_1:{                  
			if(keyPress){                     
                // ����
				keyTime++;                     
				if(keyTime>=SINGLE_PRESS_TIME){   
                    // �������ʱ�䳬�� ����ʱ�䣬���ж�Ϊ���µİ�����Ч��������Ч�������֣��������߳��������� ����״̬2�� �����ж�������������Ч����
					keyState = KEY_STATE_2;      
				}
			}         
            // �������ʱ��û�г������ж�Ϊ�󴥣�������Ч������ ����״̬0�������ȴ�����
			else keyState = KEY_STATE_0;      
		}break; 
        // ����״̬2���ж�������Ч�����ࣺ�ǵ��������ǳ���
		case KEY_STATE_2:{                   
            // ��������� �趨�ĳ���ʱ�� ���ͷţ����ж�Ϊ����
			if(keyPress == 0){                
                // ���� ��Ч����ֵ������
				keyReturn = KEY_PRESS_ONCE;      
                // ���� ����״̬0�������ȴ�����
				keyState = KEY_STATE_0;         
			} 
			else{
				keyTime++;                     
                // �������ʱ�䳬�� �趨�ĳ���ʱ��
				if(keyTime >= LONG_PRESS_TIME){   
                    // ���� ��Ч��ֵֵ������
					keyReturn = KEY_PRESS_LONG;            
                    // ȥ״̬3���ȴ������ͷ�
					keyState = KEY_STATE_3;      
				}
			}
		}break;
        // �ȴ������ͷ�
		case KEY_STATE_3:{                   
			if (keyPress == 0){ 
                // �����ͷź󣬽��� ����״̬0 ��������һ�ΰ������ж�
				keyState = KEY_STATE_0;         
			}
			else	keyReturn = KEY_PRESS_LONG;				
		}break; 
	}
    // ���� ����ֵ
	return keyReturn;                     
}

void keyFsm(keyBoardState_e *sta, uint8_t key){
  switch (*sta){
    case KEY_RELEASE:{
      if (key)
        //�������� 
        *sta = KEY_WAIT_EFFECTIVE;          
      else
        *sta = KEY_RELEASE;
    }break;
    //�ȴ�����������Ч��������
    case KEY_WAIT_EFFECTIVE:{					
      if (key){	
          //״̬ת��
        *sta = KEY_PRESS_ONCE;        
			}
      else
        *sta = KEY_RELEASE;
    }break;
    
    //����������Ч
    case KEY_PRESS_ONCE:{							
      if (key){  
          //״̬ת��
        *sta = KEY_PRESS_DOWN;       
        if (sta == &keyBoardCtrlData.lkSta)
          //�󳤰���������
          keyBoardCtrlData.lk_cnt = 0;  
        else
          //�ҳ�����������
          keyBoardCtrlData.rk_cnt = 0; 	
      }
      else
        *sta = KEY_RELEASE;
    }break;
    //����һֱ����
    case KEY_PRESS_DOWN:{							
      if (key){  
        if (sta == &keyBoardCtrlData.lkSta){
          //�����㹻ʱ��
          if (keyBoardCtrlData.lk_cnt++ > LONG_PRESS_TIME)
            //����״̬
            *sta = KEY_PRESS_LONG;                                  
        }
        else{
          if (keyBoardCtrlData.rk_cnt++ > LONG_PRESS_TIME)
            *sta = KEY_PRESS_LONG;
        }
      }
      else
        *sta = KEY_RELEASE;
    }break;
    //����״̬��
    case KEY_PRESS_LONG:{							
      //û�а�������
      if (!key){											
        *sta = KEY_RELEASE;     
      }
    }break;
   
    default:
			break;
  }
}

static void moveDirectionCtrl(uint8_t forward, uint8_t back,uint8_t left, uint8_t right,uint8_t fast,uint8_t slow,int16_t rotate) { 
    //������ٵ��ٶ�
	static f32_t forwardAccele = 0, backAccele = 0;     
	static f32_t rightAccele = 0, leftAccele = 0;
	if(getrobotMode() == MODE_KM){
	    keyBoardCtrlData.chassisSpeedTarget.z = rotate * REAL_MOTOR_SPEED_SCALE * 65;	   //С����Ԥ��ֵ��
		if(forward){
			//�ٶ��𽥵���
			forwardAccele += getConfigData()->chassisKBAcc * REAL_MOTOR_SPEED_SCALE * 0.75f;
			backAccele = 0;
			forwardAccele = constrainFloat(forwardAccele, 0.0f, getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE);
			//���°����ٶȴ�500��ʼ�𽥵���������shiftֱ������
			keyBoardCtrlData.chassisSpeedTarget.y = MIN_CHASSIS_SPEED * getchassisData()->speedLimit \
													+ forwardAccele * getchassisData()->speedLimit \
													+ fast * getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, 0.0f, \
																  getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE);                                
			//������ǰǰ���ٶȣ��ɿ�����ʱ������ٶȿ�ʼ���ٵݼ�
			backAccele = keyBoardCtrlData.chassisSpeedTarget.y; 						
		}
		else if(back){
			//������ڵ����ı���
			forwardAccele = 0;
			backAccele -= getConfigData()->chassisKBAcc * REAL_MOTOR_SPEED_SCALE * 0.75f;
			//�����ٶ�Ϊǰ���ٶ�һ��
			backAccele = constrainFloat(backAccele, -getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE, 0.0f);
			//���°����ٶȴ�500��ʼ�𽥵���������shiftֱ������
			keyBoardCtrlData.chassisSpeedTarget.y = -MIN_CHASSIS_SPEED * getchassisData()->speedLimit \
													+ backAccele * getchassisData()->speedLimit \
													- fast * getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, \
																-getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE, 0.0f);     
		}
		else{
			//�ɿ�����ʱ������Ҽ������ı���
			forwardAccele = 0;
			backAccele = 0;
			//֮ǰ������ǰ��
			if(keyBoardCtrlData.chassisSpeedTarget.y >= 0) { 		 					
				//�ɿ�����ʱ�ٶȿ��ٵݼ� 
				keyBoardCtrlData.chassisSpeedTarget.y = (keyBoardCtrlData.chassisSpeedTarget.y \
														 - 4 * getConfigData()->chassisKBAcc * REAL_MOTOR_SPEED_SCALE) * (!fast);
				keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, 0.0, \
																	   getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE);  
			}
			//֮ǰ�����Ǻ��
			if(keyBoardCtrlData.chassisSpeedTarget.y < 0) {
				//�ɿ�����ʱ�ٶȿ��ٵݼ� 
				keyBoardCtrlData.chassisSpeedTarget.y = (keyBoardCtrlData.chassisSpeedTarget.y \
														 + 4 * getConfigData()->chassisKBAcc * REAL_MOTOR_SPEED_SCALE) * (!fast);
				keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, \
																-0.5f* getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE, 0.0f);  
			}
		}
		//�����Ҽ�ʱ�����������ı���
		if(right){
			leftAccele = 0;                                  
			//�ٶ��𽥵���			
			rightAccele += getConfigData()->chassisKBAcc * REAL_MOTOR_SPEED_SCALE * 0.5f;          
			rightAccele = constrainFloat(rightAccele, 0.0f,  getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE); 
			//���°����ٶȴ�500��ʼ�𽥵���������shiftֱ������
			keyBoardCtrlData.chassisSpeedTarget.x = MIN_CHASSIS_SPEED * getchassisData()->speedLimit \
													+ rightAccele * getchassisData()->speedLimit \
													+ fast *  getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.x = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.x, 0.0f, \
																   getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE);
		}
		//�������ʱ����Ҽ������ı���
		else if(left){
			rightAccele = 0;
			//�ٶ��𽥵���
			leftAccele -= getConfigData()->chassisKBAcc * REAL_MOTOR_SPEED_SCALE * 0.5f;
			leftAccele = constrainFloat(leftAccele, -getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE, 0.0f); 
			//���°����ٶȴ�500��ʼ�𽥵���������shiftֱ������
			keyBoardCtrlData.chassisSpeedTarget.x = -MIN_CHASSIS_SPEED * getchassisData()->speedLimit \
													+ leftAccele * getchassisData()->speedLimit \
													- fast * getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.x = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.x, \
																		-getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE,0.0f);                                
		}
		else{
			//�ɿ�����ʱ������Ҽ������ı���
			rightAccele = 0;                                							
			leftAccele = 0;
			//֮ǰ�������Ҽ�
			if(keyBoardCtrlData.chassisSpeedTarget.x >= 0) { 							
				//�ɿ�����ʱ�ٶȿ��ٵݼ� 
				keyBoardCtrlData.chassisSpeedTarget.x = (keyBoardCtrlData.chassisSpeedTarget.x \
														 - 4 * getConfigData()->chassisKBAcc * REAL_MOTOR_SPEED_SCALE) * (!fast);  
				keyBoardCtrlData.chassisSpeedTarget.x = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.x, 0.0, \
																	  getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE);  
			}
			//֮ǰ���������
			if(keyBoardCtrlData.chassisSpeedTarget.x < 0) {  
				//�ɿ�����ʱ�ٶȿ��ٵݼ� 
				keyBoardCtrlData.chassisSpeedTarget.x = (keyBoardCtrlData.chassisSpeedTarget.x \
														 + 4 * getConfigData()->chassisKBAcc * REAL_MOTOR_SPEED_SCALE) * (!fast); 
				keyBoardCtrlData.chassisSpeedTarget.x = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.x, \
																	   - getConfigData()->chassisKBSpeed  * REAL_MOTOR_SPEED_SCALE, 0.0f);  
			}
		}
	}
	else{
		keyBoardCtrlData.chassisSpeedTarget.x = keyBoardCtrlData.chassisSpeedTarget.y = 0;
		forwardAccele = backAccele = rightAccele = leftAccele = 0;
	}
}

static void gimbalOperationFunc(int16_t pitRefSpd, int16_t yawRefSpd){
	if(getrobotMode() == MODE_KM){
#if YAW_SPEED_SINGLE
		if(getrobotMode() == MODE_KM && !(getGimbalData()->autoMode && getvisionData()->captureFlag) \
			&& !getinfantryAutoData()->rotateFlag && ROBOT == INFANTRY_ID){
			//����⻷����
			digitalClan(&keyBoardCtrlData.yawGyroTarget);
			//������̨���ٶ�������Yaw��ս��ٶȵ���
			keyBoardCtrlData.yawSpeedTarget = -yawRefSpd * getConfigData()->gimbalKBScale* 32;
			//���ٶ������޷�
			keyBoardCtrlData.yawSpeedTarget = keyBoardCtrlData.yawSpeedTarget > 7.3f?  7.3f : \
			(keyBoardCtrlData.yawSpeedTarget < -7.3f? -7.3f : keyBoardCtrlData.yawSpeedTarget);
		}
		else{
			//�սǶ��⻷
			keyBoardCtrlData.yawGyroTarget = yawRefSpd * getConfigData()->gimbalKBScale * 2;
		}
#else
		keyBoardCtrlData.yawGyroTarget = yawRefSpd * getConfigData()->gimbalKBScale * 2;
#endif
		keyBoardCtrlData.pitchGyroTarget = -pitRefSpd * getConfigData()->gimbalKBScale * 2;
	}
	else{
		keyBoardCtrlData.pitchSpeedTarget = 0;
		keyBoardCtrlData.yawSpeedTarget   = 0;
		keyBoardCtrlData.pitchGyroTarget  = 0;
		keyBoardCtrlData.yawGyroTarget	  = 0;
	}
}

void getKeyboardMouseState(void){
   keyFsm(&keyBoardCtrlData.lkSta,remoteControlData.dt7Value.mouse.Press_L);
   keyFsm(&keyBoardCtrlData.rkSta,remoteControlData.dt7Value.mouse.Press_R);
}

void keyboardChassisHook(void){
	if((ROBOT == F_TANK_ID)&&(getTankAutoData()->get_pill_Flag))
	    moveDirectionCtrl(BACK,FORWARD, RIGHT, LEFT, FAST_SPD, SLOW_SPD, remoteControlData.dt7Value.mouse.X);
	else
	    moveDirectionCtrl(FORWARD, BACK, LEFT, RIGHT, FAST_SPD, SLOW_SPD, remoteControlData.dt7Value.mouse.X);
}

void keyboardGimbalHook(void){
	gimbalOperationFunc(remoteControlData.dt7Value.mouse.Y, remoteControlData.dt7Value.mouse.X);
}



