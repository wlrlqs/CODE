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
//		警告：公共代码区域，故障责任重大		 //
//			除特定区域可以自由可以更改外，		 //
//	其余部分更改，请与公共区域负责人取得更改权限  // 
-----------------------------------------------
---------------------------------------------*/

keyBoardCtrlStruct_t keyBoardCtrlData;
keyBoardState_e leftKeyDriver(void){      
	static uint8_t keyState = 0;         // 按键状态变量
	static uint16_t keyTime = 0;         // 按键计时变量
	uint8_t keyPress;
	keyBoardState_e keyReturn; 
    // 清除 返回按键值
	keyReturn = KEY_RELEASE;                         			
    // 当前键值 
	keyPress = remoteControlData.dt7Value.mouse.Press_L;   
	switch (keyState){        
        // 按键状态0：判断有无按键按下
		case KEY_STATE_0:{                 
            // 有按键按下
			if(keyPress){                
                // 清零时间间隔计数
				keyTime = 0;                  
                // 然后进入 按键状态1
				keyState = KEY_STATE_1;        
			}        
		}break;
        // 按键状态1：软件消抖（确定按键是否有效，而不是误触）。按键有效的定义：按键持续按下超过设定的消抖时间。
		case KEY_STATE_1:{                  
			if(keyPress){                     
                // 一次5ms
				keyTime++;                     
                // 消抖
                // 如果按键时间超过 消抖时间，即判定为按下的按键有效。按键有效包括两种：单击或者长按，进入 按键状态2， 继续判定到底是那种有效按键
				if(keyTime>=SINGLE_PRESS_TIME){   
					keyState = KEY_STATE_2;      
				}
			}         
            // 如果按键时间没有超过，判定为误触，按键无效，返回 按键状态0，继续等待按键
			else keyState = KEY_STATE_0;      
		}break; 
        // 按键状态2：判定按键有效的种类：是单击，还是长按
		case KEY_STATE_2:{                  
            // 如果按键在 设定的长按时间 内释放，则判定为单击 
			if(!keyPress){                
                // 返回 有效按键值：单击
				keyReturn = KEY_PRESS_ONCE;      
                // 返回 按键状态0，继续等待按键
				keyState = KEY_STATE_0;         
			} 
			else{
				keyTime++;                     
                // 如果按键时间超过 设定的长按时间
				if(keyTime >= LONG_PRESS_TIME){   
                    // 返回 有效键值值：长按
					keyReturn = KEY_PRESS_LONG;            
                    // 去状态3，等待按键释放
					keyState = KEY_STATE_3;       
				}
			}
		}break;
        // 等待按键释放
		case KEY_STATE_3:{                   
			if (!keyPress){ 
                // 按键释放后，进入 按键状态0 ，进行下一次按键的判定
				keyState = KEY_STATE_0;        
			}
			else	keyReturn = KEY_PRESS_LONG;				
		}break; 
	}
    // 返回 按键值
	return keyReturn;                     
}
keyBoardState_e rightKeyDriver(void){     
	volatile static uint8_t keyState = 0;    		        // 按键状态变量	
	volatile static uint16_t keyTime = 0;                   // 按键计时变量
	uint8_t keyPress;
	keyBoardState_e keyReturn; 
    // 清除 返回按键值
	keyReturn = KEY_RELEASE;                         
    // 当前键值
	keyPress = remoteControlData.dt7Value.mouse.Press_R;   
	switch (keyState){       
        // 按键状态0：判断有无按键按下
		case KEY_STATE_0:{                  
            // 有按键按下
			if(keyPress){               
                // 清零时间间隔计数
				keyTime = 0;                   
                // 然后进入 按键状态1
				keyState = KEY_STATE_1;        
			}        
		}break;
        // 按键状态1：软件消抖（确定按键是否有效，而不是误触）。按键有效的定义：按键持续按下超过设定的消抖时间。
		case KEY_STATE_1:{                  
			if(keyPress){                     
                // 消抖
				keyTime++;                     
				if(keyTime>=SINGLE_PRESS_TIME){   
                    // 如果按键时间超过 消抖时间，即判定为按下的按键有效。按键有效包括两种：单击或者长按，进入 按键状态2， 继续判定到底是那种有效按键
					keyState = KEY_STATE_2;      
				}
			}         
            // 如果按键时间没有超过，判定为误触，按键无效，返回 按键状态0，继续等待按键
			else keyState = KEY_STATE_0;      
		}break; 
        // 按键状态2：判定按键有效的种类：是单击，还是长按
		case KEY_STATE_2:{                   
            // 如果按键在 设定的长按时间 内释放，则判定为单击
			if(keyPress == 0){                
                // 返回 有效按键值：单击
				keyReturn = KEY_PRESS_ONCE;      
                // 返回 按键状态0，继续等待按键
				keyState = KEY_STATE_0;         
			} 
			else{
				keyTime++;                     
                // 如果按键时间超过 设定的长按时间
				if(keyTime >= LONG_PRESS_TIME){   
                    // 返回 有效键值值：长按
					keyReturn = KEY_PRESS_LONG;            
                    // 去状态3，等待按键释放
					keyState = KEY_STATE_3;      
				}
			}
		}break;
        // 等待按键释放
		case KEY_STATE_3:{                   
			if (keyPress == 0){ 
                // 按键释放后，进入 按键状态0 ，进行下一次按键的判定
				keyState = KEY_STATE_0;         
			}
			else	keyReturn = KEY_PRESS_LONG;				
		}break; 
	}
    // 返回 按键值
	return keyReturn;                     
}

void keyFsm(keyBoardState_e *sta, uint8_t key){
  switch (*sta){
    case KEY_RELEASE:{
      if (key)
        //按键按下 
        *sta = KEY_WAIT_EFFECTIVE;          
      else
        *sta = KEY_RELEASE;
    }break;
    //等待按键按下有效（消抖）
    case KEY_WAIT_EFFECTIVE:{					
      if (key){	
          //状态转移
        *sta = KEY_PRESS_ONCE;        
			}
      else
        *sta = KEY_RELEASE;
    }break;
    
    //按键按下有效
    case KEY_PRESS_ONCE:{							
      if (key){  
          //状态转移
        *sta = KEY_PRESS_DOWN;       
        if (sta == &keyBoardCtrlData.lkSta)
          //左长按计数清零
          keyBoardCtrlData.lk_cnt = 0;  
        else
          //右长按计数清零
          keyBoardCtrlData.rk_cnt = 0; 	
      }
      else
        *sta = KEY_RELEASE;
    }break;
    //按键一直按着
    case KEY_PRESS_DOWN:{							
      if (key){  
        if (sta == &keyBoardCtrlData.lkSta){
          //按下足够时间
          if (keyBoardCtrlData.lk_cnt++ > LONG_PRESS_TIME)
            //长按状态
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
    //长按状态下
    case KEY_PRESS_LONG:{							
      //没有按键按下
      if (!key){											
        *sta = KEY_RELEASE;     
      }
    }break;
   
    default:
			break;
  }
}

static void moveDirectionCtrl(uint8_t forward, uint8_t back,uint8_t left, uint8_t right,uint8_t fast,uint8_t slow,int16_t rotate) { 
    //保存减速的速度
	static f32_t forwardAccele = 0, backAccele = 0;     
	static f32_t rightAccele = 0, leftAccele = 0;
	if(getrobotMode() == MODE_KM){
	    keyBoardCtrlData.chassisSpeedTarget.z = rotate * REAL_MOTOR_SPEED_SCALE * 65;	   //小陀螺预期值：
		if(forward){
			//速度逐渐递增
			forwardAccele += getConfigData()->chassisKBAcc * REAL_MOTOR_SPEED_SCALE * 0.75f;
			backAccele = 0;
			forwardAccele = constrainFloat(forwardAccele, 0.0f, getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE);
			//按下按键速度从500开始逐渐递增，按下shift直接满速
			keyBoardCtrlData.chassisSpeedTarget.y = MIN_CHASSIS_SPEED * getchassisData()->speedLimit \
													+ forwardAccele * getchassisData()->speedLimit \
													+ fast * getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, 0.0f, \
																  getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE);                                
			//保留当前前进速度，松开按键时以这个速度开始快速递减
			backAccele = keyBoardCtrlData.chassisSpeedTarget.y; 						
		}
		else if(back){
			//清除用于递增的变量
			forwardAccele = 0;
			backAccele -= getConfigData()->chassisKBAcc * REAL_MOTOR_SPEED_SCALE * 0.75f;
			//后退速度为前进速度一半
			backAccele = constrainFloat(backAccele, -getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE, 0.0f);
			//按下按键速度从500开始逐渐递增，按下shift直接满速
			keyBoardCtrlData.chassisSpeedTarget.y = -MIN_CHASSIS_SPEED * getchassisData()->speedLimit \
													+ backAccele * getchassisData()->speedLimit \
													- fast * getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, \
																-getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE, 0.0f);     
		}
		else{
			//松开按键时清除左右键递增的变量
			forwardAccele = 0;
			backAccele = 0;
			//之前按的是前键
			if(keyBoardCtrlData.chassisSpeedTarget.y >= 0) { 		 					
				//松开按键时速度快速递减 
				keyBoardCtrlData.chassisSpeedTarget.y = (keyBoardCtrlData.chassisSpeedTarget.y \
														 - 4 * getConfigData()->chassisKBAcc * REAL_MOTOR_SPEED_SCALE) * (!fast);
				keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, 0.0, \
																	   getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE);  
			}
			//之前按的是后键
			if(keyBoardCtrlData.chassisSpeedTarget.y < 0) {
				//松开按键时速度快速递减 
				keyBoardCtrlData.chassisSpeedTarget.y = (keyBoardCtrlData.chassisSpeedTarget.y \
														 + 4 * getConfigData()->chassisKBAcc * REAL_MOTOR_SPEED_SCALE) * (!fast);
				keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, \
																-0.5f* getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE, 0.0f);  
			}
		}
		//按下右键时清除左键递增的变量
		if(right){
			leftAccele = 0;                                  
			//速度逐渐递增			
			rightAccele += getConfigData()->chassisKBAcc * REAL_MOTOR_SPEED_SCALE * 0.5f;          
			rightAccele = constrainFloat(rightAccele, 0.0f,  getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE); 
			//按下按键速度从500开始逐渐递增，按下shift直接满速
			keyBoardCtrlData.chassisSpeedTarget.x = MIN_CHASSIS_SPEED * getchassisData()->speedLimit \
													+ rightAccele * getchassisData()->speedLimit \
													+ fast *  getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.x = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.x, 0.0f, \
																   getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE);
		}
		//按下左键时清除右键递增的变量
		else if(left){
			rightAccele = 0;
			//速度逐渐递增
			leftAccele -= getConfigData()->chassisKBAcc * REAL_MOTOR_SPEED_SCALE * 0.5f;
			leftAccele = constrainFloat(leftAccele, -getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE, 0.0f); 
			//按下按键速度从500开始逐渐递增，按下shift直接满速
			keyBoardCtrlData.chassisSpeedTarget.x = -MIN_CHASSIS_SPEED * getchassisData()->speedLimit \
													+ leftAccele * getchassisData()->speedLimit \
													- fast * getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.x = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.x, \
																		-getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE,0.0f);                                
		}
		else{
			//松开按键时清除左右键递增的变量
			rightAccele = 0;                                							
			leftAccele = 0;
			//之前按的是右键
			if(keyBoardCtrlData.chassisSpeedTarget.x >= 0) { 							
				//松开按键时速度快速递减 
				keyBoardCtrlData.chassisSpeedTarget.x = (keyBoardCtrlData.chassisSpeedTarget.x \
														 - 4 * getConfigData()->chassisKBAcc * REAL_MOTOR_SPEED_SCALE) * (!fast);  
				keyBoardCtrlData.chassisSpeedTarget.x = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.x, 0.0, \
																	  getConfigData()->chassisKBSpeed * REAL_MOTOR_SPEED_SCALE);  
			}
			//之前按的是左键
			if(keyBoardCtrlData.chassisSpeedTarget.x < 0) {  
				//松开按键时速度快速递减 
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
			//清除外环期望
			digitalClan(&keyBoardCtrlData.yawGyroTarget);
			//鼠标给云台角速度期望，Yaw轴闭角速度单环
			keyBoardCtrlData.yawSpeedTarget = -yawRefSpd * getConfigData()->gimbalKBScale* 32;
			//角速度期望限幅
			keyBoardCtrlData.yawSpeedTarget = keyBoardCtrlData.yawSpeedTarget > 7.3f?  7.3f : \
			(keyBoardCtrlData.yawSpeedTarget < -7.3f? -7.3f : keyBoardCtrlData.yawSpeedTarget);
		}
		else{
			//闭角度外环
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



