/**
  ******************************************************************************
  * File Name          : JudgeTask.c
  * Description        : 裁判系统处理任务，得到裁判系统信息
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#include "offical_Judge_Handler.h"
#include <stdlib.h>
#include <string.h>

static void Referee_Update_GameStatus(void);
static void Referee_Update_RobotHP(void);
static void Referee_Update_EventData(void);
static void Referee_Update_Warning(void);
static void Referee_Update_RobotState(void);
static void Referee_Update_PowerHeatData(void);
static void Referee_Update_RobotPos(void);
static void Referee_Update_Buff(void);
static void Referee_Update_AerialEnergy(void);
static void Referee_Update_RobotHurt(void);
static void Referee_Update_ShootData(void);
static void Referee_Update_BulletRemaining(void);

uint8_t JUDGE_Received = 0;
JudgeState_e JUDGE_State = OFFLINE;
referee_data_t RefereeData;
client_custom_data_t custom_data;
ext_client_graphic_draw_t client_graph;

uint16_t maxHeat0 = MAXHEAT03;
uint16_t maxHeat1 = MAXHEAT03;
float fakeHeat0 = 0;
float fakeHeat1 = 0;
float cooldown0 = COOLDOWN03;
float cooldown1 = COOLDOWN13;
robot_status_t cur_robot_status;

uint8_t tmp_judge;
void InitJudgeUart(void){
	Referee_Transmit_UserData();
	if(HAL_UART_Receive_DMA(&JUDGE_UART, &tmp_judge, 1) != HAL_OK){
			Error_Handler();
	}
	tx_free = 0;
}

uint8_t receiving = 0;
uint8_t received = 0;
uint8_t buffer[80] = {0}; 
uint8_t buffercnt = 0;
uint16_t cmdID;
void judgeUartRxCpltCallback(void)
{
	if(receiving) 
	{
		if(buffercnt > 40)buffercnt = 4;
		buffer[buffercnt] = tmp_judge;
		buffercnt++;
		
		if(buffercnt == 5)
		{
			if (myVerify_CRC8_Check_Sum(buffer, 5)==0) 
			{
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 7) cmdID = (0x0000 | buffer[5]) | (buffer[6] << 8);
	
		if(buffercnt == 12 && cmdID == 0x0001)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 12))
			{
				Referee_Update_GameStatus();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 10 && cmdID == 0x0002)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 10))
			{
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 37 && cmdID == 0x0003)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 37))
			{
				Referee_Update_RobotHP();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 13 && cmdID == 0x0101)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 13))
			{
				Referee_Update_EventData();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 12 && cmdID == 0x0102)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 12))
			{
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 11 && cmdID == 0x0103)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 11))
			{
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 11 && cmdID == 0x0103)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 11))
			{
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 11 && cmdID == 0x0104)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 11))
			{
				Referee_Update_Warning();
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 24 && cmdID == 0x0201)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 24))
			{
				Referee_Update_RobotState();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 23 && cmdID == 0x0202)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 23))
			{
				Referee_Update_PowerHeatData();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}

		if(buffercnt == 25 && cmdID == 0x0203)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 25))
			{
				Referee_Update_RobotPos();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 10 && cmdID == 0x0204)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 10))
			{
				Referee_Update_Buff();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 12 && cmdID == 0x0205)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 12))
			{
				Referee_Update_AerialEnergy();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}

		if(buffercnt == 10 && cmdID == 0x0206)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 10)) 
			{
				Referee_Update_RobotHurt();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}

		if(buffercnt == 15 && cmdID == 0x0207)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 15))
			{
				Referee_Update_ShootData();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}	
		}
		
		if(buffercnt == 11 && cmdID == 0x0208)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 11))
			{
				Referee_Update_BulletRemaining();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}	
		}
	}

	else 
	{
		if(tmp_judge == 0xA5)
		{
			receiving = 1;
			buffercnt = 0;
			buffer[0] = tmp_judge;
			buffercnt++;
		}
	}
	if(HAL_UART_Receive_DMA(&JUDGE_UART, &tmp_judge, 1) != HAL_OK)
	{
		Error_Handler();
	}
}

void getJudgeState(void)
{
	static int s_count_judge = 0;
	if(JUDGE_Received==1)
	{
		s_count_judge = 0;
		JUDGE_State = ONLINE;
		JUDGE_Received = 0;
	}
	else
	{
		s_count_judge++;
		if(s_count_judge > 300)//300ms
		{
			JUDGE_State = OFFLINE;
		}
	}
}

#define getbit(x,y) (x >> y) & 1)//获取某一位的值
#define jointbyte(x,y) ((uint16_t)(x << 8) | y)

void Referee_Update_GameStatus()
{
	RefereeData.GameStatus.game_type = (uint8_t)buffer[7] & 0x0f;
	RefereeData.GameStatus.game_progress = (uint8_t)(buffer[7] >> 4) & 0x0f;
	uint8_t* gd2 = (uint8_t*)&RefereeData.GameStatus.stage_remain_time;
	int8_t tmp1[2] = {buffer[8],buffer[9]};
	for(int i = 0; i<2; i++){gd2[i] = (uint8_t)tmp1[i];}
}

void Referee_Update_RobotHP()
{
	RefereeData.RobotHP.red_1_robot_HP = (uint16_t)buffer[7] | (buffer[8] << 8);
	RefereeData.RobotHP.red_2_robot_HP = (uint16_t)buffer[9] | (buffer[10] << 8);
	RefereeData.RobotHP.red_3_robot_HP = (uint16_t)buffer[11] | (buffer[12] << 8);
	RefereeData.RobotHP.red_4_robot_HP = (uint16_t)buffer[13] | (buffer[14] << 8);
	RefereeData.RobotHP.red_5_robot_HP = (uint16_t)buffer[15] | (buffer[16] << 8);
	RefereeData.RobotHP.red_7_robot_HP = (uint16_t)buffer[17] | (buffer[18] << 8);
	RefereeData.RobotHP.red_base_HP = (uint16_t)buffer[19] | (buffer[20] << 8);
	RefereeData.RobotHP.blue_1_robot_HP = (uint16_t)buffer[21] | (buffer[22] << 8);
	RefereeData.RobotHP.blue_2_robot_HP = (uint16_t)buffer[23] | (buffer[24] << 8);
	RefereeData.RobotHP.blue_3_robot_HP = (uint16_t)buffer[25] | (buffer[26] << 8);
	RefereeData.RobotHP.blue_4_robot_HP = (uint16_t)buffer[27] | (buffer[28] << 8);
	RefereeData.RobotHP.blue_5_robot_HP = (uint16_t)buffer[29] | (buffer[30] << 8);
	RefereeData.RobotHP.blue_7_robot_HP = (uint16_t)buffer[31] | (buffer[32] << 8);
	RefereeData.RobotHP.blue_base_HP = (uint16_t)buffer[33] | (buffer[34] << 8);
}

void Referee_Update_EventData()
{
	
}

void Referee_Update_Warning()
{
	RefereeData.refereeWarning.level = (uint8_t)buffer[7];
	RefereeData.refereeWarning.foul_robot_id = (uint8_t)buffer[8];
}

void Referee_Update_RobotState()
{
	uint8_t* grs0 = (uint8_t*)&RefereeData.GameRobotState.robot_id;
	int8_t tmp0[1] = {buffer[7]};
	grs0[0] = (uint8_t)tmp0[0];
	
	uint8_t* grs1 = (uint8_t*)&RefereeData.GameRobotState.robot_level;
	int8_t tmp1[1] = {buffer[8]};
	grs1[0] = (uint8_t)tmp1[0];
	
	uint8_t* grs2 = (uint8_t*)&RefereeData.GameRobotState.remain_HP;
	int8_t tmp2[2] = {buffer[9],buffer[10]};
	for(int i = 0; i<2; i++){
		grs2[i] = (uint8_t)tmp2[i];
	}
	
	uint8_t* grs3 = (uint8_t*)&RefereeData.GameRobotState.max_HP;
	int8_t tmp3[2] = {buffer[11], buffer[12]};
	for(int i = 0; i<2; i++){
		grs3[i] = (uint8_t)tmp3[i];
	}
	
	uint8_t* grs4 = (uint8_t*)&RefereeData.GameRobotState.shooter_heat0_cooling_rate;
	int8_t tmp4[2] = {buffer[13], buffer[14]};
	for(int i = 0; i<2; i++){
		grs4[i] = (uint8_t)tmp4[i];
	}
	
	uint8_t* grs5 = (uint8_t*)&RefereeData.GameRobotState.shooter_heat0_cooling_limit;
	int8_t tmp5[2] = {buffer[15], buffer[16]};
	for(int i = 0; i<2; i++){
		grs5[i] = (uint8_t)tmp5[i];
	}
	
	uint8_t* grs6 = (uint8_t*)&RefereeData.GameRobotState.shooter_heat1_cooling_rate;
	int8_t tmp6[2] = {buffer[17], buffer[18]};
	for(int i = 0; i<2; i++){
		grs6[i] = (uint8_t)tmp6[i];
	}	
	
	uint8_t* grs7 = (uint8_t*)&RefereeData.GameRobotState.shooter_heat1_cooling_limit;
	int8_t tmp7[2] = {buffer[19], buffer[20]};
	for(int i = 0; i<2; i++){
		grs7[i] = (uint8_t)tmp7[i];
	}
}

void Referee_Update_PowerHeatData()
{
	uint8_t* ph0 = (uint8_t*)&RefereeData.PowerHeat.chassis_volt;
	for(int i = 0; i<2; i++){
	ph0[i] = (uint8_t)buffer[i+7];
	}
	
	uint8_t* ph1 = (uint8_t*)&RefereeData.PowerHeat.chassis_current;
	for(int i = 0; i<2; i++){
	ph1[i] = (uint8_t)buffer[i+9];
	}
	
	uint8_t* ph2 = (uint8_t*)&RefereeData.PowerHeat.chassis_power;
	for(int i = 0; i<4; i++){
	ph2[i] = (uint8_t)buffer[i+11];
	}
	
	uint8_t * ph3 = (uint8_t*)&RefereeData.PowerHeat.chassis_power_buffer;
	for(int i = 0; i<2; i++){
	ph3[i] = (uint8_t)buffer[i+15];
	}
	
	uint8_t * ph4 = (uint8_t*)&RefereeData.PowerHeat.shooter_heat0;
	for(int i = 0; i<2; i++){
	ph4[i] = (uint8_t)buffer[i+17];
	}
	
	uint8_t * ph5 = (uint8_t*)&RefereeData.PowerHeat.shooter_heat1;
	for(int i = 0; i<2; i++){
	ph5[i] = (uint8_t)buffer[i+19];
	}
}

void Referee_Update_RobotPos()
{
	uint8_t * rp0 = (uint8_t*)&RefereeData.RobotPos.x;
	for(int i = 0; i<4; i++){
	rp0[i] = (uint8_t)buffer[i+7];
	}
	
	uint8_t * rp1 = (uint8_t*)&RefereeData.RobotPos.y;
	for(int i = 0; i<4; i++){
	rp1[i] = (uint8_t)buffer[i+11];
	}
	
	uint8_t * rp2 = (uint8_t*)&RefereeData.RobotPos.z;
	for(int i = 0; i<4; i++){
	rp2[i] = (uint8_t)buffer[i+15];
	}
	
	uint8_t * rp3 = (uint8_t*)&RefereeData.RobotPos.yaw;
	for(int i = 0; i<4; i++){
	rp3[i] = (uint8_t)buffer[i+19];
	}
}

void Referee_Update_Buff()
{
	RefereeData.Buff.power_rune_buff = (uint8_t)buffer[7];
}

void Referee_Update_AerialEnergy()
{
	uint8_t * ae0 = (uint8_t*)&RefereeData.ShootData.bullet_type;
	ae0[0] = (uint8_t)buffer[7];
	
	uint8_t * ae1 = (uint8_t*)&RefereeData.ShootData.bullet_type;
	for(int i = 0; i<2; i++){
	ae1[i] = (uint8_t)buffer[i+8];
	}
}

void Referee_Update_RobotHurt()
{
	RefereeData.RobotHurt.armor_id = (uint8_t)(buffer[7] & 0xf);
	RefereeData.RobotHurt.hurt_type = (uint8_t)((buffer[7] >> 4) & 0xf);
}

void Referee_Update_ShootData()
{
	uint8_t * sd0 = (uint8_t*)&RefereeData.ShootData.bullet_type;
	sd0[0] = (uint8_t)buffer[7];
	
	uint8_t * sd1 = (uint8_t*)&RefereeData.ShootData.bullet_freq;
	sd1[0] = (uint8_t)buffer[8];
	
	uint8_t * sd2 = (uint8_t*)&RefereeData.ShootData.bullet_speed;
	for(int i = 0; i<4; i++){
	sd2[i] = (uint8_t)buffer[i+9];
	}
}

void Referee_Update_BulletRemaining()
{
	RefereeData.bulletRemaining.bullet_remaining_num = (uint16_t)buffer[7] | (buffer[8] << 8);
}

uint8_t check_buffer[28] = {0};
void Referee_Transmit_UserData()
{	
	uint8_t buffer[28] = {0};
	uint8_t * ud1 = (uint8_t*)&custom_data.data1;
	uint8_t * ud2 = (uint8_t*)&custom_data.data2;
	uint8_t * ud3 = (uint8_t*)&custom_data.data3;
	
	//帧头
	buffer[0] = 0xA5;//数据帧起始字节，固定值为 0xA5
	buffer[1] = 19;//数据帧中 data 的长度,占两个字节
	buffer[2] = 0;
	buffer[3] = 1;//包序号
	buffer[4] = myGet_CRC8_Check_Sum(&buffer[0], 5-1, myCRC8_INIT);//帧头 CRC8 校验
	//cmd ID: 0x0301
	buffer[5] = 0x01;
	buffer[6] = 0x03;
	//数据的内容 ID:0xD180  ,占两个字节
	buffer[7] = 0x80;
	buffer[8] = 0xD1;
	//发送者的 ID, 占两个字节
	buffer[9] = RefereeData.GameRobotState.robot_id;
	buffer[10] = 0;
	//客户端的 ID, 只能为发送者机器人对应的客户端,  占两个字节
	buffer[11] = RefereeData.GameRobotState.robot_id;
	if(buffer[11]>9&&buffer[11]<16)buffer[11]+=6;
	buffer[12] = 0x01;
	//自定义数据
	for(int i=13;i<17;i++) buffer[i] = ud1[i-13];
	for(int i=17;i<21;i++) buffer[i] = ud2[i-17];
	for(int i=21;i<25;i++) buffer[i] = ud3[i-21];
	buffer[25] = custom_data.masks;
	//CRC16校验位，两个字节
	Append_CRC16_Check_Sum(buffer,sizeof(buffer));
	
	for(int i = 0; i < 28; i++)
	{
		check_buffer[i] = buffer[i];
	}

	if(tx_free)
	{
		if(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&buffer,sizeof(buffer))!=HAL_OK)
		{
			Error_Handler();
		}
		tx_free = 0;
	}
}

void Referee_Transmit_ClientGraph()
{
	uint8_t buffer[70] = {0};
	uint8_t * cg1 = (uint8_t*)&client_graph.graphic_name;
	uint8_t * cg2 = (uint8_t*)&client_graph.start_x;
	uint8_t * cg3 = (uint8_t*)&client_graph.start_y;
	uint8_t * cg4 = (uint8_t*)&client_graph.radius;
	uint8_t * cg5 = (uint8_t*)&client_graph.end_x;
	uint8_t * cg6 = (uint8_t*)&client_graph.end_y;
	uint8_t * cg7 = (uint8_t*)&client_graph.start_angle;
	uint8_t * cg8 = (uint8_t*)&client_graph.end_angle;
	uint8_t * cg9 = (uint8_t*)&client_graph.text;
	
	//帧头
	buffer[0] = 0xA5;//数据帧起始字节，固定值为 0xA5
	buffer[1] = 61;//数据帧中 data 的长度,占两个字节
	buffer[2] = 0;
	buffer[3] = 1;//包序号
	buffer[4] = myGet_CRC8_Check_Sum(&buffer[0], 5-1, myCRC8_INIT);//帧头 CRC8 校验
	//cmd ID: 0x0301
	buffer[5] = 0x01;
	buffer[6] = 0x03;
	//数据的内容 ID:0x0100  ,占两个字节
	buffer[7] = 0x80;
	buffer[8] = 0xD1;
	//发送者的 ID, 占两个字节
	buffer[9] = RefereeData.GameRobotState.robot_id;
	buffer[10] = 0;
	//客户端的 ID, 只能为发送者机器人对应的客户端,  占两个字节
	buffer[11] = RefereeData.GameRobotState.robot_id;
	if(buffer[11]>9&&buffer[11]<16)buffer[11]+=6;
	buffer[12] = 0x01;
	//自定义图形数据
	buffer[13] = client_graph.operate_type;
	client_graph.operate_type = 0;
	buffer[14] = client_graph.graphic_type;
	for(int i=15;i<20;i++) buffer[i] = cg1[i-15];
	buffer[20] = client_graph.layer;
	buffer[21] = client_graph.color;
	buffer[22] = client_graph.width;
	for(int i=23;i<25;i++) buffer[i] = cg2[i-23];
	for(int i=25;i<27;i++) buffer[i] = cg3[i-25];
	for(int i=27;i<29;i++) buffer[i] = cg4[i-27];
	for(int i=29;i<31;i++) buffer[i] = cg5[i-29];
	for(int i=31;i<33;i++) buffer[i] = cg6[i-31];
	for(int i=33;i<35;i++) buffer[i] = cg7[i-33];
	for(int i=35;i<37;i++) buffer[i] = cg8[i-35];
	buffer[37] = client_graph.text_length;
	for(int i=38;i<68;i++) buffer[i] = cg9[i-38];	
	Append_CRC16_Check_Sum(buffer,sizeof(buffer));
	
	for(int i = 0; i < sizeof(buffer); i++)
	{
		check_buffer[i] = buffer[i];
	}

	if(tx_free)
	{
		if(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&buffer,sizeof(buffer))!=HAL_OK)
		{
			Error_Handler();
		}
		tx_free = 0;
	}
}

void fakeHeatCalc(void)
{
	maxHeat0 = RefereeData.GameRobotState.shooter_heat0_cooling_limit;
	cooldown0 = RefereeData.GameRobotState.shooter_heat0_cooling_rate;
	maxHeat1 = RefereeData.GameRobotState.shooter_heat1_cooling_limit;
	cooldown1 = RefereeData.GameRobotState.shooter_heat1_cooling_rate;

	if(auto_counter_heat0==0)
	{
		fakeHeat0 = RefereeData.PowerHeat.shooter_heat0;
	}
	if(auto_counter_heat1==0)
	{
		fakeHeat1 = RefereeData.PowerHeat.shooter_heat1;
	}
	
	if(fakeHeat0 >= cooldown0/1000) fakeHeat0 -= cooldown0/1000;
	else fakeHeat0 = 0;
	if(fakeHeat1 >= cooldown1/1000) fakeHeat1 -= cooldown1/1000;
	else fakeHeat1 = 0;
}

void Refresh_Client_Data(void)
{
	custom_data.data1 = (float)GMP.RealAngle;
	custom_data.data2 = (float)GMY.RealAngle;
	custom_data.data3 = 0x01;
	custom_data.masks = Client_Show_SuperCap_Voltage();//二进制最左位对应灯条最右灯
}

void Refresh_Client_Graph(void)//Todo
{
	client_graph.operate_type = 1;
	client_graph.graphic_type = 3;
	for(int i = 0; i < 5; i++)
	{
		client_graph.graphic_name[i] = 0;
	}
	client_graph.layer = 0;
	client_graph.color = 0;
	client_graph.width = 10;
	client_graph.start_x = 960;
	client_graph.start_y = 540;
	client_graph.radius = 100;
	client_graph.end_x = 0;
	client_graph.end_y = 0;
	client_graph.start_angle = 0;
	client_graph.end_angle = 0;
	client_graph.text_length = 0;
	//client_graph.text
}
