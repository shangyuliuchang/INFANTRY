/**
  ******************************************************************************
  * File Name          : JudgeTask.h
  * Description        : 裁判系统处理任务，得到裁判系统信息
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __JUDGETASK_H
#define __JUDGETASK_H

#include "includes.h"

typedef enum
{
	ONLINE = 0,
	OFFLINE = 1,
}JudgeState_e;

//比赛状态数据：0x0001。发送频率：1Hz
typedef __packed struct
{
uint8_t game_type : 4;
uint8_t game_progress : 4;
uint16_t stage_remain_time;
} ext_game_state_t;

//比赛机器人状态：0x0201。发送频率：10Hz
typedef __packed struct
{
uint8_t robot_id;
uint8_t robot_level;
uint16_t remain_HP;
uint16_t max_HP;
uint16_t shooter_heat0_cooling_rate;
uint16_t shooter_heat0_cooling_limit;
uint16_t shooter_heat1_cooling_rate;
uint16_t shooter_heat1_cooling_limit;
uint8_t mains_power_gimbal_output : 1;
uint8_t mains_power_chassis_output : 1;
uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

//实时功率热量数据：0x0202。发送频率：50Hz
typedef __packed struct
{
uint16_t chassis_volt;
uint16_t chassis_current;
float chassis_power;
uint16_t chassis_power_buffer;
uint16_t shooter_heat0;
uint16_t shooter_heat1;
} ext_power_heat_data_t;

//机器人位置：0x0203。发送频率：10Hz
typedef __packed struct
{
float x;
float y;
float z;
float yaw;
} ext_game_robot_pos_t;

//机器人增益：0x0204。发送频率：状态改变后发送
typedef __packed struct
{
uint8_t power_rune_buff;
}ext_buff_musk_t;

//空中机器人能量状态：0x0205。发送频率：10Hz
typedef __packed struct
{
uint8_t energy_point;
uint8_t attack_time;
} aerial_robot_energy_t;

//伤害状态：0x0206。发送频率：伤害发生后发送
typedef __packed struct
{
uint8_t armor_id : 4;
uint8_t hurt_type : 4;
} ext_robot_hurt_t;

//实时射击信息：0x0207。发送频率：射击后发送
typedef __packed struct
{
uint8_t bullet_type;
uint8_t bullet_freq;
float bullet_speed;
} ext_shoot_data_t;


//交互数据接收信息：0x0301。发送频率：上限 10Hz
typedef __packed struct
{
uint16_t data_cmd_id;
uint16_t send_ID;
uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

//客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180。发送频率：上限 10Hz
typedef __packed struct
{
float data1;
float data2;
float data3;
uint8_t masks;
} client_custom_data_t;

//交互数据 机器人间通信：0x0301。发送频率：上限 10Hz
//typedef __packed struct
//{
//uint8_t data[];
//} robot_interactive_data_t;

extern ext_game_robot_state_t GameRobotState;
extern ext_power_heat_data_t PowerHeat;
extern ext_buff_musk_t BuffMask;
extern ext_shoot_data_t ShootData;
extern client_custom_data_t custom_data;

void judgeUartRxCpltCallback(void);
void InitJudgeUart(void);
extern void getJudgeState(void);
extern JudgeState_e JUDGE_State;
extern float fakeHeat0;
extern float fakeHeat1;
extern float realBulletSpeed0;
extern uint16_t realHeat0;
extern int16_t remainHeat0;
extern int16_t remainHeat1;
extern uint8_t syncCnt0;
extern float cooldown0;
extern float cooldown1;
extern uint16_t maxHeat0;
extern uint16_t maxHeat1;

void Referee_Update_RobotState(void);
void Referee_Update_PowerHeatData(void);
void Referee_Update_BuffMask(void);
void Referee_Update_ShootData(void);
void Referee_Transmit_UserData(void);
#endif /*__ JUDGETASK_H */
