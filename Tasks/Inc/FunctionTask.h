/**
  ******************************************************************************
  * File Name          : FunctionTask.h
  * Description        : 用于记录机器人独有的功能
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __FUNCTIONTASK_H
#define __FUNCTIONTASK_H

#include "includes.h"

//遥控常量区
#define RC_CHASSIS_SPEED_REF    	0.85f
#define RC_ROTATE_SPEED_REF 			0.07f
#define RC_GIMBAL_SPEED_REF				0.006f

#define IGNORE_RANGE 					200

//键鼠常量区
#define KEY_W				0x1
#define KEY_S				0x2
#define KEY_A				0x4
#define KEY_D				0x8
#define KEY_SHIFT		0x10
#define KEY_CTRL		0x20
#define KEY_Q				0x40
#define KEY_E				0x80
#define KEY_R				0x100
#define KEY_F				0x200
#define KEY_G				0x400
#define KEY_Z				0x800
#define KEY_X				0x1000
#define KEY_C				0x2000
#define KEY_V				0x4000
#define KEY_B				0x8000

#define NORMAL_FORWARD_BACK_SPEED 	600
#define NORMAL_LEFT_RIGHT_SPEED  		600/1.5f
#define HIGH_FORWARD_BACK_SPEED 		800
#define HIGH_LEFT_RIGHT_SPEED   		800/1.5f
#define LOW_FORWARD_BACK_SPEED 			300
#define LOW_LEFT_RIGHT_SPEED   			300/1.5f

#define CHASSIS_TWIST_ANGLE_LIMIT				60

#define MOUSE_LR_RAMP_TICK_COUNT		50
#define MOUSR_FB_RAMP_TICK_COUNT		60

#define MOUSE_TO_YAW_ANGLE_INC_FACT		((aim_mode != 0 && find_enemy) ? 0.03f : 0.06f)
#define MOUSE_TO_PITCH_ANGLE_INC_FACT	((aim_mode != 0	&& find_enemy) ? 0.03f : 0.06f)

#define MK_ROTATE_SPEED_REF 			0.90f

#define SHOOTMODE_GM_ADJUST_ANGLE			0.05f

#define MAXHP1 300
#define MAXHP2 500
#define MAXHP3 700

#define COOLDOWN01 40.0f
#define COOLDOWN02 60.0f
#define COOLDOWN03 80.0f

#define MAXHEAT01 240
#define MAXHEAT02 360
#define MAXHEAT03 480

#define COOLDOWN11 20.0f
#define COOLDOWN12 40.0f
#define COOLDOWN13 60.0f

#define MAXHEAT11 200
#define MAXHEAT12 300
#define MAXHEAT13 400

#define OnePush(button,execution)\
{\
	static uint8_t cache;\
	static uint8_t cnt=0;\
	if(cache != (button)){\
		cache = (button);\
		cnt = 0;\
	}\
	else if(cnt == 5){\
		if(cache) execution;\
		cnt=11;\
	}\
	else if(cnt < 5) cnt++;\
}

#define Delay(TIM,execution)\
{\
	static uint16_t time=TIM;\
	if(!time--)\
	{\
		time = TIM;\
		execution;\
	}\
}

typedef enum
{
	SHIFT,
	CTRL,
	SHIFT_CTRL,
	NO_CHANGE,
}KeyboardMode_e;

typedef enum
{
	SHORT_CLICK,
	LONG_CLICK,
	NO_CLICK,
}MouseMode_e;

typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

typedef enum 
{
	  MoveMode_CAP_STOP_MODE,
	  MoveMode_CAP_RECHARGE_MODE,
	  MoveMode_CAP_RELEASE_LOW_MODE,
	  MoveMode_CAP_RELEASE_HIGH_MODE,
}KeyBoard_MoveMode;

extern ChassisSpeed_Ref_t ChassisSpeedRef; 
extern int ChassisTwistGapAngle;
extern uint8_t ChassisTwistState;
extern int32_t auto_counter;
extern int32_t auto_counter_stir;
extern int16_t auto_counter_heat0;
extern int16_t auto_counter_heat1;
extern uint8_t chassis_lock;
extern int16_t chassis_follow_center;
extern uint8_t gate_first_enter;
extern int16_t shoot_cd;

void FunctionTaskInit(void);
void Limit_Position(void);
void OptionalFunction(void);
void FreshSuperCState(void);
void ChassisTwist(void);
void ChassisDeTwist(void);
void ShootOneBullet(void);
void Bullet_Block_Handler(void);
void Reset(void);
void Gate_Handler(uint8_t gate_state);

#endif /*__FUNCTIONTASK_H*/
