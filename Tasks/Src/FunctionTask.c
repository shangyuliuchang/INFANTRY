/**
  ******************************************************************************
  * File Name          : FunctionTask.c
  * Description        : 用于记录机器人独有的功能
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

#define GATE_CLOSE	0
#define GATE_OPEN 	1
#define STIR_STEP_ANGLE 45
#define	LONG_CD		250
#define	SHORT_CD	80

#define FRIC_ON() \
{\
	if(block_flag == 0) ShootState = 1;\
	else ShootState = 0;\
	FRICL.TargetAngle = -FricSpeed;\
	FRICR.TargetAngle = FricSpeed;\
}
#define FRIC_OFF() \
{\
	ShootState = 0;\
	FRICL.TargetAngle = 0;\
	FRICR.TargetAngle = 0;\
}

KeyboardMode_e KeyboardMode = NO_CHANGE;
KeyboardMode_e LastKeyboardMode = NO_CHANGE;
MouseMode_e MouseLMode = NO_CLICK;
MouseMode_e MouseRMode = NO_CLICK;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//斜坡函数
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ChassisSpeed_Ref_t ChassisSpeedRef;
uint16_t LastKey=0;

int ChassisTwistGapAngle = 0;

int32_t auto_counter=0;		//用于准确延时的完成某事件
int32_t auto_counter_stir=0;
int16_t auto_counter_heat0=0;
int16_t auto_counter_heat1=0;
int16_t shoot_cd=0;

int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;
uint8_t TestMode = 0;
uint8_t ShootState = 0;
uint8_t ChassisTwistState = 0;
uint8_t cdflag0 = 0;
uint8_t cdflag1 = 0;
uint8_t burst = 0;
int16_t cur_cd = LONG_CD; //发射间隔（ms），修改上方LONG_CD,SHORT_CD的宏定义

#ifdef INFANTRY_3
#define FRIC_SPEED_SLOW 	4000
#define FRIC_SPEED_FAST 	5000
#define SHOOT_SPEED_SLOW	12.0f
#define SHOOT_SPEED_FAST	18.2f
#endif
int16_t FricSpeed = FRIC_SPEED_FAST;


uint8_t chassis_lock = 0;//底盘锁定
int16_t chassis_follow_center = GM_YAW_ZERO;//底盘跟随前方角度
uint8_t block_flag = 0;//卡弹标记
uint8_t gate_state = GATE_CLOSE;//弹仓盖状态

//INFANTRY
/***防卡弹相关变量***/


//初始化
void FunctionTaskInit()
{
	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
	FBSpeedRamp.ResetCounter(&FBSpeedRamp);
	
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
	
	KeyboardMode=NO_CHANGE;
}

void OptionalFunction()
{
	#ifdef USE_POWERLIMITATION
//	if(Cap_Get_Cap_State() == CAP_STATE_STOP)
//	{
//		PowerLimitation(); //基于自测功率的功率限制，适用于充电和停止状态
//	}
//	else
//	{
//		if (Cap_Get_Cap_State() == CAP_STATE_RECHARGE || Cap_Get_Cap_State() == CAP_STATE_TEMP_RECHARGE)
//			CurBased_PowerLimitation();//基于自测功率的功率限制，适用于充电和停止状态
//		else
//		{
//			if (Cap_Get_Cap_State() == CAP_STATE_RELEASE)
//				CapBased_PowerLimitation();//超级电容工作模式下的功率限制
//		}
//	}
	PowerLimitation(); //基于自测功率的功率限制，适用于充电和停止状态
	#endif
}

//******************
//遥控器模式功能编写
//******************
void RemoteControlProcess(Remote *rc)
{
	static WorkState_e LastState = NORMAL_STATE;
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	
	if(WorkState == NORMAL_STATE)
	{
		if(LastState!= WorkState && Cap_Get_Cap_State() != CAP_STATE_STOP)
		{
			#ifndef SUB_BOARD
			Cap_State_Switch(CAP_STATE_STOP);
			#endif
		}
		
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
		#ifdef USE_CHASSIS_FOLLOW
		#ifndef INFANTRY
			GMY.TargetAngle += channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#else
			GMY.TargetAngle -= channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle += channellcol * RC_GIMBAL_SPEED_REF;
		#endif
		#else
		ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
		GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#endif
		
		chassis_lock = 0;
		ChassisTwistState = 0;
		FRIC_OFF();
		aim_mode = 0;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
		if(LastState!= WorkState && Cap_Get_Cap_State() != CAP_STATE_RECHARGE)
		{
			#ifndef SUB_BOARD
			Cap_State_Switch(CAP_STATE_RECHARGE);
			#endif
		}
		
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
		#ifdef USE_CHASSIS_FOLLOW
		#ifndef INFANTRY
			GMY.TargetAngle += channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#else
			GMY.TargetAngle -= channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle += channellcol * RC_GIMBAL_SPEED_REF;
		#endif
		#else
		ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
		GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#endif
		
		chassis_lock = 0;
		ChassisTwistState = 0;
		FRIC_ON();
		aim_mode = 0;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
		if(LastState!= WorkState && Cap_Get_Cap_State() != CAP_STATE_RELEASE && Cap_Get_Power_Voltage() > 11.5)
		{
			#ifndef SUB_BOARD
			Cap_State_Switch(CAP_STATE_RELEASE);
			#endif
		}
		
		if (Cap_Get_Power_Voltage() > 12 && Cap_Get_Cap_State() == CAP_STATE_RELEASE)
		{
		  ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF*2;
		  ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF*1.5f;
    }
		else
		{
//			ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
//		  ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF*3/2;
			ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF*2;
		  ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF*1.5f;
		}
		#ifdef USE_CHASSIS_FOLLOW
		#ifndef INFANTRY
			GMY.TargetAngle += channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#else
			GMY.TargetAngle -= channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle += channellcol * RC_GIMBAL_SPEED_REF;
		#endif
		#else
		if (Cap_Get_Power_Voltage() > 12 && Cap_Get_Cap_State() == CAP_STATE_RELEASE)
		{
		  ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF*1.5f;
		}
		else
		{
			ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
		}
		GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#endif
		
		chassis_lock = 0;
		ChassisTwistState = 0;
		FRIC_ON();
		aim_mode = 0;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
		if(LastState != WorkState)
		{
			ShootOneBullet();
		}
	}
	
	if(rc->ch3 == 0x16C)//遥控器左侧拨到最下
		gate_state = GATE_OPEN;
	else
		gate_state = GATE_CLOSE;
	
	if(ChassisTwistState)
	{
		ChassisTwist();
	}
	else ChassisDeTwist();

	Bullet_Block_Handler();
	Gate_Handler(gate_state);
	
	LastState = WorkState;
}


uint16_t KM_FORWORD_BACK_SPEED 	= NORMAL_FORWARD_BACK_SPEED;
uint16_t KM_LEFT_RIGHT_SPEED  	= NORMAL_LEFT_RIGHT_SPEED;
void KeyboardModeFSM(Key *key);
void MouseModeFSM(Mouse *mouse);


//------------
//原键鼠模式重分配
//@尹云鹏   controlTask changed
//左拨杆下才为真正的键鼠模式，上、中两档位可编辑
//------------
extern uint8_t sendfinish;  extern int32_t cps[4][4000];//用于串口发送功率数据@唐欣阳

void MouseKeyControlProcess(Mouse *mouse, Key *key,Remote *rc)
{	
	static WorkState_e LastState = NORMAL_STATE;
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	if(WorkState == NORMAL_STATE)
	{
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
		#ifdef USE_CHASSIS_FOLLOW
		#ifndef INFANTRY
			GMY.TargetAngle += channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#else
			GMY.TargetAngle -= channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle += channellcol * RC_GIMBAL_SPEED_REF;
		#endif
		#else
		ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
		#endif
		
		chassis_lock = 0;
		ChassisTwistState = 0;
		FRIC_OFF();
		aim_mode = PREDICT_MODE;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
		#ifdef USE_CHASSIS_FOLLOW
		#ifndef INFANTRY
			GMY.TargetAngle += channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#else
			GMY.TargetAngle -= channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle += channellcol * RC_GIMBAL_SPEED_REF;
		#endif
		#else
		ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
		#endif
		
		chassis_lock = 0;
		ChassisTwistState = 1;
		FRIC_OFF();
		aim_mode = ANTI_GYRO_MODE;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	//键鼠模式
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
		MINMAX(mouse->x, -150, 150); 
		MINMAX(mouse->y, -150, 150); 
		
		#ifdef USE_CHASSIS_FOLLOW
		#ifndef INFANTRY
			GMY.TargetAngle += mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT;
			GMP.TargetAngle += mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT;
		#else
			GMY.TargetAngle -= mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT;
			GMP.TargetAngle -= mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT;
		#endif
		#else
		ChassisSpeedRef.rotate_ref = -mouse->x * RC_ROTATE_SPEED_REF;
		#endif
		
		FRIC_ON();
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);

		MouseModeFSM(mouse);
		
		switch(MouseRMode)
		{
			case SHORT_CLICK:
			{
				if(ShootState)
				{

				}
			}break;
			case LONG_CLICK:
			{
				if(ShootState)
				{
					
				}
			}break;
			default: break;
		}
		
		switch (MouseLMode)
		{
			case SHORT_CLICK:
			{
				
			}break;
			case LONG_CLICK:
			{
				if(!mouse->press_r)
				{
					if(shoot_cd == 0)
						ShootOneBullet();
				}
			}
			default: break;
		}

		KeyboardModeFSM(key);
		
		//组合键键位
		switch (KeyboardMode)
		{
			case SHIFT_CTRL:
			{
				burst = 1;
				break;
			}
			case CTRL:
			{
				burst = 0;
				if(key->v & KEY_R) gate_state = GATE_OPEN;
				if(key->v & KEY_G)
				{
					if(chassis_lock) GMP.TargetAngle = -20;
				}
				break;
			}
			case SHIFT:
			{
				burst = 0;
				chassis_lock = 0;
				break;
			}
			case NO_CHANGE:
			{
				burst = 0;
				if(key->v & KEY_G) chassis_lock = 1;
				break;
			}
		}
		
		//吊射模式云台微调
		if(chassis_lock)
		{
			if(key->v & KEY_W)  			//key: w
				GMP.TargetAngle -= SHOOTMODE_GM_ADJUST_ANGLE;
			else if(key->v & KEY_S) 	//key: s
				GMP.TargetAngle += SHOOTMODE_GM_ADJUST_ANGLE;
			if(key->v & KEY_D)  			//key: d
				GMY.TargetAngle += SHOOTMODE_GM_ADJUST_ANGLE;
			else if(key->v & KEY_A) 	//key: a
				GMY.TargetAngle -= SHOOTMODE_GM_ADJUST_ANGLE;
		}
		
		//底盘运动控制
		if(key->v & KEY_W)  			//key: w
			ChassisSpeedRef.forward_back_ref =  KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
		else if(key->v & KEY_S) 	//key: s
			ChassisSpeedRef.forward_back_ref = -KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
		else
		{
			ChassisSpeedRef.forward_back_ref = 0;
			FBSpeedRamp.ResetCounter(&FBSpeedRamp);
		}
		if(key->v & KEY_D)  			//key: d
			ChassisSpeedRef.left_right_ref =  KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
		else if(key->v & KEY_A) 	//key: a
			ChassisSpeedRef.left_right_ref = -KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
		else
		{
			ChassisSpeedRef.left_right_ref = 0;
			LRSpeedRamp.ResetCounter(&LRSpeedRamp);
		}
		
		//扭腰、陀螺模式选择
		if(key->v & KEY_Q) {ChassisTwistState = 1; chassis_lock = 0;}
		if(key->v & KEY_E) {ChassisTwistState = 2; chassis_lock = 0;}
		if(key->v & KEY_F) Reset();
		
		//射频选择
		if(KeyboardMode == SHIFT_CTRL || KeyboardMode == CTRL || (mouse->press_r && mouse->press_l && auto_shoot_flag == 1))
		{
			cur_cd = SHORT_CD;
			if(shoot_cd > cur_cd)
				shoot_cd = cur_cd;
		}
		else
		{
			cur_cd = LONG_CD;
			if(shoot_cd > cur_cd)
				shoot_cd = cur_cd;
		}
		
		//射速选择
		if(key->v & KEY_Z)
		{
			FricSpeed = FRIC_SPEED_SLOW;
			realBulletSpeed0 = SHOOT_SPEED_SLOW;
		}
		else if(key->v & KEY_X)
		{
			FricSpeed = FRIC_SPEED_FAST;
			realBulletSpeed0 = SHOOT_SPEED_FAST;
		}
		
		//自瞄模式选择
		if(key->v & KEY_C)
		{
			aim_mode = PREDICT_MODE;
			if(mouse->press_r) AutoShoot(aim_mode);
		}
		else if(key->v & KEY_V)
		{
			aim_mode = ANTI_GYRO_MODE;
			if(mouse->press_r) AutoShoot(aim_mode);
		}
		else if(mouse->press_r)
		{
			if(GMP.RealAngle > 7)
			{
				aim_mode = PREDICT_MODE;
				if(MouseRMode == LONG_CLICK) AutoShoot(aim_mode);
			}
			else
			{
				aim_mode = ANTI_GYRO_MODE;
				AutoShoot(aim_mode);
			}
		}
		else
		{
			aim_mode = 0;
		}
			
		if(LastState != WorkState)
		{
			ChassisTwistState = 0;
		}
	}

	//扭腰、陀螺处理
	if(ChassisTwistState) ChassisTwist();
	else ChassisDeTwist();
	//防卡弹处理
	Bullet_Block_Handler();
	
	LED_Show_SuperCap_Voltage(1);
	
	LastKey=key->v;
	LastState = WorkState;
}

KeyBoard_MoveMode KeyBoardMoveMode = MoveMode_CAP_RECHARGE_MODE;

void KeyboardModeFSM(Key *key)
{
	if((key->v & 0x30) == 0x30)//Shift_Ctrl
	{
		KeyboardMode=SHIFT_CTRL;
	}
	else if(key->v & KEY_SHIFT)//Shift
	{
		KeyboardMode=SHIFT;
	}
	else if(key->v & KEY_CTRL)//Ctrl
	{
		KeyboardMode=CTRL;
	}
	else
	{
		KeyboardMode=NO_CHANGE;
	}	
	
/*
	SHIFT_CTRL
	*/
	/*
	CTRL
	*/
	if (KeyboardMode != LastKeyboardMode && KeyboardMode == SHIFT_CTRL)
	{ 
		if (KeyBoardMoveMode == MoveMode_CAP_STOP_MODE )
		{
			KeyBoardMoveMode = MoveMode_CAP_RECHARGE_MODE;
		}
		else if (KeyBoardMoveMode == MoveMode_CAP_RECHARGE_MODE || KeyBoardMoveMode == MoveMode_CAP_RELEASE_HIGH_MODE)
		{
			KeyBoardMoveMode = MoveMode_CAP_STOP_MODE;
		}
	}
	/*
	SHIFT
	*/
	if (KeyboardMode == SHIFT){
		if (KeyBoardMoveMode != MoveMode_CAP_RELEASE_HIGH_MODE /*&& KeyBoardMoveMode != MoveMode_CAP_STOP_MODE */){
			KeyBoardMoveMode = MoveMode_CAP_RELEASE_HIGH_MODE;
		}			
	}else{
		if (KeyBoardMoveMode == MoveMode_CAP_RELEASE_HIGH_MODE){
			KeyBoardMoveMode = MoveMode_CAP_RECHARGE_MODE;
		}
	}
	if (KeyBoardMoveMode == MoveMode_CAP_STOP_MODE){
		KeyBoardMoveMode = MoveMode_CAP_RECHARGE_MODE;
	}
	
	/*
	
	*/
	switch (KeyBoardMoveMode){
		case MoveMode_CAP_RELEASE_LOW_MODE://
			if(Cap_Get_Power_Voltage() > 11 && Cap_Get_Cap_State() != CAP_STATE_TEMP_RECHARGE && Cap_Get_Cap_State() != CAP_STATE_RELEASE)
		  {
				#ifndef SUB_BOARD
			  Cap_State_Switch(CAP_STATE_RELEASE);
				#endif
		  }
			KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
      KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
			break;
		case MoveMode_CAP_RELEASE_HIGH_MODE: //
			if(Cap_Get_Power_Voltage() > 11.5 && Cap_Get_Cap_State() != CAP_STATE_TEMP_RECHARGE && Cap_Get_Cap_State() != CAP_STATE_RELEASE)
		  {
			  Cap_State_Switch(CAP_STATE_RELEASE);
		  }
		  if(Cap_Get_Cap_Voltage() > 13) 
		  {
			  KM_FORWORD_BACK_SPEED=  HIGH_FORWARD_BACK_SPEED;
			  KM_LEFT_RIGHT_SPEED = HIGH_LEFT_RIGHT_SPEED;
		  }
		  else
		  {
			  KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
			  KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
		  }
			break;
		case MoveMode_CAP_STOP_MODE://
//			if(Cap_Get_Cap_State() != CAP_STATE_STOP)
//		  {
//			  Cap_State_Switch(CAP_STATE_STOP);
//		  }
			KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
      KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
			break;
		case MoveMode_CAP_RECHARGE_MODE://
			if(Cap_Get_Cap_State() != CAP_STATE_RECHARGE)
		  {
				#ifndef SUB_BOARD
			  Cap_State_Switch(CAP_STATE_RECHARGE);
				#endif
		  }
			KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
      KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
			break;
	}
	/*
	CTRL
	*/
	if (KeyboardMode == CTRL){
     KM_FORWORD_BACK_SPEED=  LOW_FORWARD_BACK_SPEED; 
     KM_LEFT_RIGHT_SPEED = LOW_LEFT_RIGHT_SPEED;
	}
	LastKeyboardMode=KeyboardMode;
}


void MouseModeFSM(Mouse *mouse)
{
	static uint8_t counterl = 0;
	static uint8_t counterr = 0;
	switch (MouseLMode)
	{
		case SHORT_CLICK:
		{
			counterl++;
			if(mouse->press_l == 0)
			{
				MouseLMode = NO_CLICK;
				counterl = 0;
			}
			else if(counterl>=50)
			{
				MouseLMode = LONG_CLICK;
				counterl = 0;
			}
			else
			{
				MouseLMode = SHORT_CLICK;
			}
		}break;
		case LONG_CLICK:
		{
			if(mouse->press_l==0)
			{
				MouseLMode = NO_CLICK;
			}
			else
			{
				MouseLMode = LONG_CLICK;
			}
		}break;
		case NO_CLICK:
		{
			if(mouse->press_l)
			{
				ShootOneBullet();
				MouseLMode = SHORT_CLICK;
			}
		}break;
	}
	
	switch (MouseRMode)
	{
		case SHORT_CLICK:
		{
			counterr++;
			if(mouse->press_r == 0)
			{
				MouseRMode = NO_CLICK;
				counterr = 0;
			}
			else if(counterr>=50)
			{
				MouseRMode = LONG_CLICK;
				counterr = 0;
			}
			else
			{
				MouseRMode = SHORT_CLICK;
			}
		}break;
		case LONG_CLICK:
		{
			if(mouse->press_r==0)
			{
				MouseRMode = NO_CLICK;
			}
			else
			{
				MouseRMode = LONG_CLICK;
			}
		}break;
		case NO_CLICK:
		{
			if(mouse->press_r)
			{
				MouseRMode = SHORT_CLICK;
			}
		}break;
	}
}

//用于遥控器模式下超级电容测试模式的控制
void FreshSuperCState(void)
{
	static uint8_t counter = 0;
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2))
	{
		counter++;
		if(counter==40)
		{
			TestMode = (TestMode==1)?0:1;
		}
	}
	else{
		counter = 0;
	}
	if(TestMode==1)
	{
		HAL_GPIO_WritePin(GPIOF, LED_GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF, LED_GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_SET);
	}
}

void ChassisTwist(void)
{
	
}

void ChassisDeTwist(void)
{
	ChassisTwistGapAngle = 0;
}

void ShootOneBullet(void)
{
	#ifndef USE_HEAT_LIMIT
	STIR.TargetAngle -= STIR_STEP_ANGLE;
	#else
	cdflag0 = (/*JUDGE_State == ONLINE && */fakeHeat0 > (maxHeat0 - realBulletSpeed0) && !burst) ? 1 : 0;
	if(!cdflag0 && ShootState)
	{
		STIR.TargetAngle -= 45;
		fakeHeat0 += realBulletSpeed0;
		auto_counter_heat0 = 200;
		shoot_cd = cur_cd;
	}
	#endif
}

void Bullet_Block_Handler(void)
{
	OnePush(STIR.RxMsgC6x0.moment < -6000,
	{
		STIR.TargetAngle += 90;
		auto_counter_stir = 100;
		block_flag = 1;
		ShootState = 0;
	});
	if(block_flag == 1)
	{
		OnePush(auto_counter_stir==0,
		{
			if(STIR.RxMsgC6x0.moment > -500)
			{	
				STIR.TargetAngle -= 45;
				ShootState = 1;
				block_flag = 0;
			}
			else
			{
				STIR.TargetAngle += 45;
				auto_counter_stir = 100;
			}
		});
	}
}

void Reset(void)
{
	ChassisTwistState = 0;
	chassis_lock = 0;
	gate_state = GATE_CLOSE;
}

uint8_t gate_first_enter = 1;
void Gate_Handler(uint8_t gate_state)
{
	
}
