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

#ifdef INFANTRY_3
#define FRIC_SPEED_1 	4000
#define FRIC_SPEED_2 	5000
#define FRIC_SPEED_3 	6000
#define SHOOT_SPEED_1	12.0f
#define SHOOT_SPEED_2	18.2f
#define SHOOT_SPEED_3	20.0f
#endif

//遥控常量
#define RC_CHASSIS_SPEED_REF    		0.85f
#define RC_ROTATE_SPEED_REF 				0.07f
#define RC_GIMBAL_SPEED_REF					0.006f
//遥控器死区
#define IGNORE_RANGE 								0
//速度常量
#define NORMAL_FORWARD_BACK_SPEED 	600
#define NORMAL_LEFT_RIGHT_SPEED  		600/1.5f
#define HIGH_FORWARD_BACK_SPEED 		800
#define HIGH_LEFT_RIGHT_SPEED   		800/1.5f
#define LOW_FORWARD_BACK_SPEED 			300
#define LOW_LEFT_RIGHT_SPEED   			300/1.5f
//扭腰幅度
#define CHASSIS_TWIST_ANGLE_LIMIT		60
//鼠标长按时间阈值
#define MOUSE_LR_RAMP_TICK_COUNT		50
#define MOUSR_FB_RAMP_TICK_COUNT		60
//鼠标灵敏度
#define MOUSE_TO_YAW_ANGLE_INC_FACT		((aim_mode != 0 && find_enemy) ? 0.03f : 0.06f)
#define MOUSE_TO_PITCH_ANGLE_INC_FACT	((aim_mode != 0	&& find_enemy) ? 0.03f : 0.06f)
//云台微调幅度
#define SHOOTMODE_GM_ADJUST_ANGLE		0.05f
//射击变量
#define STIR_STEP_ANGLE 45
#define	LONG_CD		200
#define	SHORT_CD	50

#define GATE_CLOSE	0
#define GATE_OPEN 	1

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

void Test_Mode_Handler(void);
void Bullet_Block_Handler(void);
void Gate_Handler(uint8_t gate_state);
void Chassis_forward_back_Handler(void);
void Reset(uint8_t);

KeyboardMode_e KeyboardMode = NO_CHANGE;
KeyboardMode_e LastKeyboardMode = NO_CHANGE;
MouseMode_e MouseLMode = NO_CLICK;
MouseMode_e MouseRMode = NO_CLICK;
//斜坡函数
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
//速度变量
ChassisSpeed_Ref_t ChassisSpeedRef;
//计时变量
int32_t auto_counter=0;
int32_t auto_counter_stir=0;
int16_t auto_counter_heat0=0;
int16_t auto_counter_heat1=0;
int16_t shoot_cd=0;
//遥控器拨杆数据
int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;

uint8_t TestMode = 0;
uint8_t ShootState = 0;
uint8_t ChassisTwistState = 0;
int8_t ChassisTwistGapAngle = 0;
uint8_t chassis_lock = 0;//底盘锁定
int16_t chassis_follow_center = GM_YAW_ZERO;//底盘跟随前方角度
uint8_t chassis_change_forward_back = 0;
uint8_t change_forward_back_rcd = 0;
uint8_t change_forward_back_step = 0;

int16_t FricSpeed = FRIC_SPEED_1;//摩擦轮转速
float BulletSpeed = 18.0f;
uint8_t burst = 0;//无视热量限制
uint8_t block_flag = 0;//卡弹标记
int16_t cur_cd = LONG_CD; //发射间隔

uint8_t gate_state = GATE_CLOSE;//弹仓盖状态
uint8_t gate_first_enter = 1;


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
	
	FRIC_OFF();
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
		#ifndef USE_CAP3
		if(LastState!= WorkState && Cap_Get_Cap_State() != CAP_STATE_STOP)
		{
			#ifndef BOARD_SLAVE
			Cap_State_Switch(CAP_STATE_STOP);
			#endif
		}
		#endif //USE_CAP3
		
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
		#ifdef USE_CHASSIS_FOLLOW
			GMY.TargetAngle += YAW_DIR * channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF;
		#else
			ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
			GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF;
		#endif
		
		chassis_lock = 0;
		ChassisTwistState = 0;
		FRIC_OFF();
		aim_mode = 0;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
		#ifndef USE_CAP3
		if(LastState!= WorkState && Cap_Get_Cap_State() != CAP_STATE_RECHARGE)
		{
			#ifndef BOARD_SLAVE
			Cap_State_Switch(CAP_STATE_RECHARGE);
			#endif
		}
		#endif //USE_CAP3
		
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
		#ifdef USE_CHASSIS_FOLLOW
			GMY.TargetAngle += YAW_DIR * channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF;
		#else
			ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
			GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF;
		#endif
		
		chassis_lock = 0;
		ChassisTwistState = 0;
		FRIC_ON();
		aim_mode = 0;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
		#ifndef USE_CAP3
		if(LastState!= WorkState && Cap_Get_Cap_State() != CAP_STATE_RELEASE && Cap_Get_Power_Voltage() > 11.5)
		{
			#ifndef BOARD_SLAVE
			Cap_State_Switch(CAP_STATE_RELEASE);
			#endif
		}
		#endif //USE_CAP3
		
		if (Cap_Get_Cap_Voltage() > 12 && Cap_Get_Cap_State() == CAP_STATE_RELEASE)
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
			GMY.TargetAngle += YAW_DIR * channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF;
		#else
			if (Cap_Get_Cap_Voltage() > 12 && Cap_Get_Cap_State() == CAP_STATE_RELEASE)
			{
				ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF*1.5f;
			}
			else
			{
				ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
			}
			GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF;
		#endif
		
		chassis_lock = 0;
		ChassisTwistState = 0;
		FRIC_ON();
		aim_mode = 0;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
		if(LastState != WorkState)
		{
			ShootOneBullet(BULLET_TYPE);
		}
	}
	
	if(rc->ch3 == 0x16C)//遥控器左侧拨到最下
		gate_state = GATE_OPEN;
	else
		gate_state = GATE_CLOSE;

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
			GMY.TargetAngle += YAW_DIR * channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF;
		#else
			ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
			GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF;
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
			GMY.TargetAngle += YAW_DIR * channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF;
		#else
			ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
			GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF;
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
			GMY.TargetAngle += YAW_DIR * mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT;
			GMP.TargetAngle -= PIT_DIR * mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT;
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
						ShootOneBullet(BULLET_TYPE);
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
				GMP.TargetAngle += PIT_DIR * SHOOTMODE_GM_ADJUST_ANGLE;
			else if(key->v & KEY_S) 	//key: s
				GMP.TargetAngle -= PIT_DIR * SHOOTMODE_GM_ADJUST_ANGLE;
			if(key->v & KEY_D)  			//key: d
				GMY.TargetAngle += YAW_DIR * SHOOTMODE_GM_ADJUST_ANGLE;
			else if(key->v & KEY_A) 	//key: a
				GMY.TargetAngle -= YAW_DIR * SHOOTMODE_GM_ADJUST_ANGLE;
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
		if(key->v & KEY_E) {ChassisTwistState = 3; chassis_lock = 0;}
		if(key->v & KEY_F) Reset(0);
		
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
			FricSpeed = FRIC_SPEED_1;
			BulletSpeed = SHOOT_SPEED_1;
		}
		else if(key->v & KEY_X)
		{
			FricSpeed = FRIC_SPEED_2;
			BulletSpeed = SHOOT_SPEED_2;
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
		
		//UI test
		if(KeyboardMode == CTRL && key->v & KEY_B)
		{
			Client_Graph_Start();
		}
		else if(key->v & KEY_B)
		{
			Client_Graph_Clear();
		}
		
		if(LastState != WorkState)
		{
			ChassisTwistState = 0;
		}
	}

	//防卡弹处理
	Bullet_Block_Handler();
	
	LED_Show_SuperCap_Voltage(1);
	
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
	#ifndef USE_CAP3
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
	#else
	if(KeyboardMode == SHIFT){
		if(KeyBoardMoveMode != MoveMode_CAP_RELEASE_HIGH_MODE){
			KeyBoardMoveMode = MoveMode_CAP_RELEASE_HIGH_MODE;
		}
	}else{
		if(KeyBoardMoveMode != MoveMode_CAP_RELEASE_LOW_MODE){
			KeyBoardMoveMode = MoveMode_CAP_RELEASE_LOW_MODE;
		}
	}
	#endif
	
	/*
	
	*/
	switch (KeyBoardMoveMode){
		case MoveMode_CAP_RELEASE_LOW_MODE://
			#ifndef USE_CAP3
			if(Cap_Get_Cap_Voltage() > 11 && Cap_Get_Cap_State() != CAP_STATE_TEMP_RECHARGE && Cap_Get_Cap_State() != CAP_STATE_RELEASE)
		  {
				#ifndef BOARD_SLAVE
			  Cap_State_Switch(CAP_STATE_RELEASE);
				#endif
		  }
			#endif
			KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
      KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
			break;
		case MoveMode_CAP_RELEASE_HIGH_MODE: //
			#ifndef USE_CAP3
			if(Cap_Get_Cap_Voltage() > 11.5 && Cap_Get_Cap_State() != CAP_STATE_TEMP_RECHARGE && Cap_Get_Cap_State() != CAP_STATE_RELEASE)
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
			#endif
			KM_FORWORD_BACK_SPEED=  HIGH_FORWARD_BACK_SPEED;
			KM_LEFT_RIGHT_SPEED = HIGH_LEFT_RIGHT_SPEED;
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
			#ifndef USE_CAP3
			if(Cap_Get_Cap_State() != CAP_STATE_RECHARGE)
		  {
				#ifndef BOARD_SLAVE
			  Cap_State_Switch(CAP_STATE_RECHARGE);
				#endif
		  }
			#endif //USE_CAP3
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
				ShootOneBullet(BULLET_TYPE);
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
void Test_Mode_Handler(void)
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
	else
	{
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
	switch (ChassisTwistState)
	{
		case 1:
		{
			ChassisSpeedRef.rotate_ref = 50;
			break;
		}
		case 2:
		{
			ChassisSpeedRef.rotate_ref = 50;
			break;
		}
		case 3:
		{
			switch (ChassisTwistGapAngle)
			{
				case 0:
				{
					ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT;
					break;
				}
				case CHASSIS_TWIST_ANGLE_LIMIT:
				{
					if(fabs(YAW_DIR * (chassis_follow_center - GMY.RxMsgC6x0.angle) * 360 / 8192.0f - ChassisTwistGapAngle) < 10)
						ChassisTwistGapAngle = -CHASSIS_TWIST_ANGLE_LIMIT;
					break;
				}
				case -CHASSIS_TWIST_ANGLE_LIMIT:
				{
					if(fabs(YAW_DIR * (chassis_follow_center - GMY.RxMsgC6x0.angle) * 360 / 8192.0f - ChassisTwistGapAngle) < 10)
						ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT;
					break;
				}
			}
			ChassisSpeedRef.rotate_ref = (GMY.RxMsgC6x0.angle - chassis_follow_center) * 360 / 8192.0f - ChassisTwistGapAngle;
			break;
		}
		default:
		{
			ChassisTwistGapAngle = 0;
			ChassisSpeedRef.rotate_ref = (GMY.RxMsgC6x0.angle - chassis_follow_center) * 360 / 8192.0f - ChassisTwistGapAngle;
			break;
		}
	}
}

void Chassis_forward_back_Handler(void)
{
	if(!chassis_lock)
	{
		if(change_forward_back_rcd != chassis_change_forward_back)
		{
			change_forward_back_step = 1;
		}

		switch(change_forward_back_step)
		{
			case 2:
			{
				chassis_follow_center = GM_YAW_ZERO - 2048;
				if(fabs((GMY.RxMsgC6x0.angle - chassis_follow_center) * 360 / 8192.0f) < 45)
				{
					change_forward_back_step = 1;
				}
				break;
			}
			case 1:
			{
				chassis_follow_center = GM_YAW_ZERO - (chassis_change_forward_back ? 4096 : 0);
				if(fabs((GMY.RxMsgC6x0.angle - chassis_follow_center) * 360 / 8192.0f) < 45)
				{
					change_forward_back_step = 0;
				}
				break;
			}
			default: change_forward_back_step = 0; break;
		}
		
		change_forward_back_rcd = chassis_change_forward_back;
	}
}

void ShootOneBullet(uint8_t state)
{
	#ifndef USE_HEAT_LIMIT
	STIR.TargetAngle -= STIR_STEP_ANGLE;
	#else
	uint8_t cdflag0 = 0;
	uint8_t cdflag1 = 0;
	if(state == 0)
	{
		cdflag0 = (maxHeat0 - fakeHeat0 < 20 && !burst) ? 1 : 0;
		if(!cdflag0 && ShootState)
		{
			STIR.TargetAngle -= STIR_STEP_ANGLE;
			fakeHeat0 += 10;
			auto_counter_heat0 = 200;
			shoot_cd = cur_cd;
		}
	}
	if(state == 1)
	{
		cdflag1 = (maxHeat1 - fakeHeat1 < 105 && !burst) ? 1 : 0;
		if(!cdflag1 && ShootState)
		{
			STIR.TargetAngle -= STIR_STEP_ANGLE;
			fakeHeat0 += 100;
			auto_counter_heat0 = 200;
			shoot_cd = cur_cd;
		}
	}
	#endif
}

void Bullet_Block_Handler(void)
{
	OnePush(STIR.RxMsgC6x0.moment < -6000,
	{
		STIR.TargetAngle += 1.5 * STIR_STEP_ANGLE;
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
				STIR.TargetAngle -= 0.5 * STIR_STEP_ANGLE;
				ShootState = 1;
				block_flag = 0;
			}
			else
			{
				STIR.TargetAngle += STIR_STEP_ANGLE;
				auto_counter_stir = 100;
			}
		});
	}
}

void Gate_Handler(uint8_t gate_state)
{
	
}

void Reset(uint8_t chassis_forward_back)
{
	ChassisTwistState = 0;
	chassis_lock = 0;
	chassis_change_forward_back =  chassis_forward_back;
	gate_state = GATE_CLOSE;
}
