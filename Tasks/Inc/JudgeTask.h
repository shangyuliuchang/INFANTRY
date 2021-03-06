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
} ext_game_status_t;

//比赛结果数据：0x0002。发送频率：比赛结束后发送（不需要使用）
typedef __packed struct
{
	uint8_t winner;
} ext_game_result_t;

//机器人存活数据：0x0003。发送频率：1Hz
typedef __packed struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_6_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_6_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;

//场地事件数据：0x0101。发送频率：事件改变后发送
typedef __packed struct
{
	uint32_t event_type;
} ext_event_data_t;

//补给站动作标识：0x0102。发送频率：动作改变后发送（不需要使用）
typedef __packed struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
} ext_supply_projectile_action_t;

//补给站预约子弹：cmd_id (0x0103)。发送频率：上限10Hz。RM对抗赛尚未开放
typedef __packed struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_num;
} ext_supply_projectile_booking_t;

//裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送
typedef __packed struct
{
	uint8_t level;
	uint8_t foul_robot_id;
} ext_referee_warning_t;

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
} ext_game_robot_status_t;

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
} ext_buff_t;

//空中机器人能量状态：0x0205。发送频率：10Hz
typedef __packed struct
{
	uint8_t energy_point;
	uint8_t attack_time;
} ext_aerial_robot_energy_t;

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

//弹丸剩余发射数：0x0208。发送频率：1Hz周期发送，空中机器人以及哨兵机器人主控发送
typedef __packed struct
{
	uint16_t bullet_remaining_num;
} ext_bullet_remaining_t;

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

//学生机器人间通信 cmd_id 0x0301，内容ID:0x0200~0x02FF
typedef __packed struct
{
	uint8_t* data;
} robot_interactive_data_t;

//客户端自定义图形 机器人间通信：0x0301。内容 ID:0x0100。发送频率：上限10Hz
typedef __packed struct
{
	uint8_t operate_type;
	uint8_t graphic_type;
	uint8_t graphic_name[5];
	uint8_t layer;
	uint8_t color;
	uint8_t width;
	uint16_t start_x;
	uint16_t start_y;
	uint16_t radius;
	uint16_t end_x;
	uint16_t end_y;
	int16_t start_angle;
	int16_t end_angle;
	uint8_t text_length;
	uint8_t text[30];
} ext_client_graphic_draw_t;

typedef __packed struct
{
	float power_limit;
	float speed_limit0;
	float speed_limit1;
} robot_status_t;

typedef __packed struct
{
	ext_game_status_t GameStatus;
	ext_game_robot_HP_t RobotHP;
	ext_event_data_t EventData;
	ext_referee_warning_t refereeWarning;
	ext_game_robot_status_t GameRobotState;
	ext_power_heat_data_t PowerHeat;
	ext_game_robot_pos_t RobotPos;
	ext_buff_t Buff;
	ext_aerial_robot_energy_t AreialRobotEnergy;
	ext_robot_hurt_t RobotHurt;
	ext_shoot_data_t ShootData;
	ext_bullet_remaining_t bulletRemaining;
} referee_data_t;

extern referee_data_t RefereeData;
extern JudgeState_e JUDGE_State;
extern uint16_t maxHeat0;
extern uint16_t maxHeat1;
extern float fakeHeat0;
extern float fakeHeat1;
extern float cooldown0;
extern float cooldown1;
extern robot_status_t cur_robot_status;

void judgeUartRxCpltCallback(void);
void InitJudgeUart(void);
void getJudgeState(void);
void Referee_Transmit(void);
void fakeHeatCalc(void);
void Client_Graph_Start(void);
void Client_Graph_Clear(void);
#endif /*__ JUDGETASK_H */
