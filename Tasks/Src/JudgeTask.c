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

static void Judge_Refresh_Hit(void);
static void Judge_Refresh_Result(void);
static void Referee_Update_RobotState(void);
static void Referee_Update_PowerHeatData(void);
static void Referee_Update_BuffMask(void);
static void Referee_Update_ShootData(void);
static void Referee_Update_GameData(void);

uint8_t tmp_judge;
void InitJudgeUart(void){
	tx_free = 1;
	Referee_Transmit_UserData();
	if(HAL_UART_Receive_DMA(&JUDGE_UART, &tmp_judge, 1) != HAL_OK){
			Error_Handler();
	}
}
uint8_t receiving = 0;
uint8_t received = 0;
uint8_t buffer[80] = {0}; 
uint8_t buffercnt = 0;
uint16_t cmdID;

void judgeUartRxCpltCallback(void)
{
//	fw_printfln("judge receive");
		if(receiving) 
		{
			if(buffercnt >40)buffercnt = 4;
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
					Referee_Update_GameData();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 10 && cmdID == 0x0002)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 10))
				{
					Judge_Refresh_Result();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 11 && cmdID == 0x0003)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 11))
				{
					Judge_Refresh_Result();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 13 && cmdID == 0x0101)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 13))
				{
					Judge_Refresh_Result();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 12 && cmdID == 0x0102)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 12))
				{
					Judge_Refresh_Result();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 11 && cmdID == 0x0103)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 11))
				{
					Judge_Refresh_Result();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 24 && cmdID == 0x0201)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 24))
				{
					Referee_Update_RobotState();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 23 && cmdID == 0x0202)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 23))
				{
					Referee_Update_PowerHeatData();
					receiving = 0;
					buffercnt = 0;
				}					
			}

 			if(buffercnt == 25 && cmdID == 0x0203)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 25))
				{
					Judge_Refresh_Result();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 10 && cmdID == 0x0204)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 10))
				{
					Referee_Update_BuffMask();
					receiving = 0;
					buffercnt = 0;
				}					
			}
			
			if(buffercnt == 12 && cmdID == 0x0205)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 12))
				{
					Judge_Refresh_Result();
					receiving = 0;
					buffercnt = 0;
				}					
			}

 			if(buffercnt == 10 && cmdID == 0x0206)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 10)) 
				{
					Judge_Refresh_Hit();
					receiving = 0;
					buffercnt = 0;
				}
			}

			if(buffercnt == 15 && cmdID == 0x0207)
			{
				if (myVerify_CRC16_Check_Sum(buffer, 15))
				{
					Referee_Update_ShootData();
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


uint8_t JUDGE_Received = 0;
JudgeState_e JUDGE_State = OFFLINE;

uint16_t maxHP = MAXHP3;
uint16_t remainHP = MAXHP3;
uint16_t maxHeat0 = MAXHEAT03;
int16_t remainHeat0 = MAXHEAT03;
uint16_t maxHeat1 = MAXHEAT03;
int16_t remainHeat1 = MAXHEAT03;
uint16_t realHeat0 = 0;
float fakeHeat0 = 0;
float fakeHeat1 = 0;
float realBulletSpeed0 = 18.2f;
float cooldown0 = COOLDOWN03;
float cooldown1 = COOLDOWN13;
uint8_t shoot0Cnt = 0;
uint8_t shoot1Cnt = 0;
uint8_t syncCnt0 = 0;

void Judge_Refresh_Result()
{
	JUDGE_Received = 1;
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
		if(s_count_judge > 150)
		{//300ms
			JUDGE_State = OFFLINE;
		}
	}
}

#define getbit(x,y) ((x) >> (y)&1)//获取某一位的值

ext_game_state_t GameData;
ext_game_robot_state_t GameRobotState;
ext_power_heat_data_t PowerHeat;
ext_buff_musk_t BuffMask;
ext_shoot_data_t ShootData;

void Referee_Update_GameData()
{
	unsigned char* gd1 = (unsigned char*)&GameData.stage_remain_time;
	char tmp1[2] = {buffer[8],buffer[9]};
	for(int i = 0; i<2; i++){gd1[i] = (unsigned char)tmp1[i];}
}

void Referee_Update_RobotState()
{
	unsigned char* grs0 = (unsigned char*)&GameRobotState.robot_id;
	char tmp0[1] = {buffer[7]};
	grs0[0] = (unsigned char)tmp0[0];
	
	unsigned char* grs1 = (unsigned char*)&GameRobotState.robot_level;
	char tmp1[1] = {buffer[8]};
	grs1[0] = (unsigned char)tmp1[0];
	
	unsigned char* grs2 = (unsigned char*)&GameRobotState.remain_HP;
	char tmp2[2] = {buffer[9],buffer[10]};
	for(int i = 0; i<2; i++){
		grs2[i] = (unsigned char)tmp2[i];
	}
	
	unsigned char* grs3 = (unsigned char*)&GameRobotState.max_HP;
	char tmp3[2] = {buffer[11], buffer[12]};
	for(int i = 0; i<2; i++){
		grs3[i] = (unsigned char)tmp3[i];
	}
	
	unsigned char* grs4 = (unsigned char*)&GameRobotState.shooter_heat0_cooling_rate;
	char tmp4[2] = {buffer[13], buffer[14]};
	for(int i = 0; i<2; i++){
		grs4[i] = (unsigned char)tmp4[i];
	}
	
	unsigned char* grs5 = (unsigned char*)&GameRobotState.shooter_heat0_cooling_limit;
	char tmp5[2] = {buffer[15], buffer[16]};
	for(int i = 0; i<2; i++){
		grs5[i] = (unsigned char)tmp5[i];
	}
	
	unsigned char* grs6 = (unsigned char*)&GameRobotState.shooter_heat1_cooling_rate;
	char tmp6[2] = {buffer[17], buffer[18]};
	for(int i = 0; i<2; i++){
		grs6[i] = (unsigned char)tmp6[i];
	}	
	
	unsigned char* grs7 = (unsigned char*)&GameRobotState.shooter_heat1_cooling_limit;
	char tmp7[2] = {buffer[19], buffer[20]};
	for(int i = 0; i<2; i++){
		grs7[i] = (unsigned char)tmp7[i];
	}
	
	maxHeat0 = GameRobotState.shooter_heat0_cooling_limit;
	cooldown0 = GameRobotState.shooter_heat0_cooling_rate;
	maxHeat1 = GameRobotState.shooter_heat1_cooling_limit;
	cooldown1 = GameRobotState.shooter_heat1_cooling_rate;

	JUDGE_Received = 1;
	//电源输出情况TBD
	
//	unsigned char* grs = (unsigned char*)&GameRobotState;
//	char tmp[15];
//	for(int i = 0; i<15; i++){
//		 //strcpy(tmp, buffer[i+7]);
////		sprintf(tmp, "%s", (char *)buffer[i+7]);
//	}
}

void Referee_Update_PowerHeatData()
{
	//电压tbd
	//电流tbd
	unsigned char * ph0 = (unsigned char*)&PowerHeat.chassis_power;
	for(int i = 0; i<4; i++){
	ph0[i] = (unsigned char)buffer[i+11];
	}
	
	unsigned char * ph1 = (unsigned char*)&PowerHeat.chassis_power_buffer;
	for(int i = 0; i<2; i++){
	ph1[i] = (unsigned char)buffer[i+15];
	}
	
	unsigned char * ph2 = (unsigned char*)&PowerHeat.shooter_heat0;
	for(int i = 0; i<2; i++){
	ph2[i] = (unsigned char)buffer[i+17];
	}
	
	unsigned char * ph3 = (unsigned char*)&PowerHeat.shooter_heat1;
	for(int i = 0; i<2; i++){
	ph3[i] = (unsigned char)buffer[i+19];
	}
	realHeat0 = PowerHeat.shooter_heat0;
	remainHeat0 = maxHeat0 - PowerHeat.shooter_heat0;
	remainHeat1 = maxHeat1 - PowerHeat.shooter_heat1;
	if(auto_counter_heat0==0)
	{
		fakeHeat0 = PowerHeat.shooter_heat0;
	}
	if(auto_counter_heat1==0)
	{
		fakeHeat1 = PowerHeat.shooter_heat1;
	}
	JUDGE_Received = 1;
}

void Referee_Update_BuffMask()
{
	unsigned char * bm = (unsigned char*)&BuffMask.power_rune_buff;
	for(int i = 0; i<2; i++){
	bm[i] = (unsigned char)buffer[i+7];
	}
	JUDGE_Received = 1;
}

uint8_t HitCnt=0;
void Judge_Refresh_Hit()
{
	JUDGE_Received = 1;
}

void Referee_Update_ShootData()
{
	unsigned char * sd0 = (unsigned char*)&ShootData.bullet_type;
	sd0[1] = (unsigned char)buffer[7];
	
	JUDGE_Received = 1;unsigned char * sd1 = (unsigned char*)&ShootData.bullet_freq;
	sd1[1] = (unsigned char)buffer[8];
	
	unsigned char * sd2 = (unsigned char*)&ShootData.bullet_speed;
	for(int i = 0; i<4; i++){
	sd2[i] = (unsigned char)buffer[i+9];
	}

	//realBulletSpeed0 = (ShootData.bullet_speed < 8)?8:ShootData.bullet_speed;//Todo:注意弹丸类型
	JUDGE_Received = 1;
}

client_custom_data_t custom_data;
void Referee_Transmit_UserData()
{
	uint8_t buffer[28]={0};
	custom_data.data1 = (float)(- GMP.RealAngle);
	custom_data.data2 = 0x01;
	custom_data.data3 = 0x01;
	custom_data.masks = Client_Show_SuperCap_Voltage();//二进制最左位对应灯条最右灯
	
	unsigned char * bs1 = (unsigned char*)&custom_data.data1;
	unsigned char * bs2 = (unsigned char*)&custom_data.data2;
	unsigned char * bs3 = (unsigned char*)&custom_data.data3;
	buffer[0] = 0xA5;//数据帧起始字节，固定值为 0xA5
	buffer[1] = 19;//数据帧中 data 的长度,占两个字节
	
	buffer[3] = 1;//包序号
	buffer[4] = myGet_CRC8_Check_Sum(&buffer[0], 5-1, myCRC8_INIT);//帧头 CRC8 校验
	buffer[5] = 0x01;
	buffer[6] = 0x03;
	
	buffer[7] = 0x80;//数据的内容 ID:0xD180  ,占两个字节
	buffer[8] = 0xD1;
	buffer[9] = GameRobotState.robot_id;//发送者的 ID, 占两个字节
	buffer[10] = 0;//存疑
	buffer[11] = GameRobotState.robot_id;//客户端的 ID, 只能为发送者机器人对应的客户端,  占两个字节
	if(buffer[11]>9&&buffer[11]<16)buffer[11]+=6;
	buffer[12] = 0x01;//
	for(int i=13;i<17;i++) buffer[i] = bs1[i-13];
	for(int i=17;i<21;i++) buffer[i] = bs2[i-17];
	for(int i=21;i<25;i++) buffer[i] = bs3[i-21];
	buffer[25] = custom_data.masks;
	static uint16_t CRC16=0;
	CRC16 = myGet_CRC16_Check_Sum(buffer, 26, myCRC16_INIT);
	buffer[26] = CRC16 & 0x00ff;//0xff
	buffer[27] = (CRC16 >> 8) & 0xff;

	tx_free = 0;
	while(HAL_UART_Transmit_DMA(&JUDGE_UART,(uint8_t *)&buffer,28)!=HAL_OK);
}
