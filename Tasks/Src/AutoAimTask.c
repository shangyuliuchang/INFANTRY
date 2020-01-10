/**
  ******************************************************************************
  *FileName				: AutoAimTask.c
  *Description		: 自瞄程序
  *Author					: 管易恒
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
*/

#include "includes.h"

#ifdef	USE_AUTOAIM

//通信系数
#define k_angle			(180.0f/(32768.0f-1.0f))
#define k_distance	(1000.0f/(32768.0f-1.0f))
#define K_YAW				(1.5f)
#define ENEMY_RED		(1)
#define ENEMY_BLUE	(0)
//角度补偿
#ifdef INFANTRY_3
	#define YAW_ADJUST												(0.5f)
	#define PITCH_ADJUST_CONST_SLOW_NORMAL		(0.5f)
	#define PITCH_ADJUST_COEF_SLOW_NORMAL			(0.02f)
	#define PITCH_ADJUST_CONST_SLOW_CLOSE			(2.5f)
	#define PITCH_ADJUST_COEF_SLOW_CLOSE			(0)
	#define PITCH_ADJUST_CONST_SLOW_UP				(-0.5f)
	#define PITCH_ADJUST_COEF_SLOW_UP					(0.025f)
	#define PITCH_ADJUST_CONST_FAST_NORMAL		(5.5f)
	#define PITCH_ADJUST_COEF_FAST_NORMAL			(0)
	#define PITCH_ADJUST_CONST_FAST_CLOSE			(1.5f)
	#define PITCH_ADJUST_COEF_FAST_CLOSE			(0)
	#define PITCH_ADJUST_CONST_FAST_UP				(-1.1f)
	#define PITCH_ADJUST_COEF_FAST_UP					(0.014)
#endif
//预测参数
#ifdef INFANTRY_3
	#define DELAY_T														(60)
	#define PREDICT_STEPS											(13.5)
	#define PREDICT_SHOOTABLE_BOUND_BELOW			(0.6f)
	#define PREDICT_SHOOTABLE_BOUND_ABOVE			(0.6f)
#endif
//反陀螺参数
#define ANTI_GYRO_THRES											(6.0f)
#define ANTI_GYRO_SHOOTABLE_BOUND						(10)
#ifdef INFANTRY_3
	#define SHOOT_DELAY												(20)
#endif
#define SHOOT_DELAY_TIME										(shoot_delay+enemy_dist*10.0f/BulletSpeed)
//自瞄pid
#ifdef INFANTRY_3
	#define AUTOAIM_YAW_PID_INIT() 						fw_PID_INIT(1.2f, 0.1f, 0.0f, 10.0, 8.0, 3.0, 10.0)
	#define AUTOAIM_PITCH_PID_INIT()					fw_PID_INIT(1.0f, 0.05f, 0.2f, 10.0, 8.0, 3.0, 10.0)
#endif

uint8_t Enemy_INFO[8];
float enemy_dist = 200;
GMAngle_t aim;
GMAngle_t aim_real;
GMAngle_t aim_output;
GMAngle_t adjust;
GMAngle_t GM_RealAngle_RCD;
uint8_t delay_t = DELAY_T;
uint8_t aim_mode = 0;
uint8_t	find_enemy = 0;
uint8_t	aim_finish = 1;
uint8_t auto_shoot_flag = 0;
uint16_t auto_counter_autoaim = 0;
uint16_t receive_cnt = 0;
uint16_t receive_rcd = 0;
uint16_t shoot_delay = SHOOT_DELAY;
imu_t imu_w_RCD;
fw_PID_Regulator_t AutoAim_Yaw_pid = AUTOAIM_YAW_PID_INIT();
fw_PID_Regulator_t AutoAim_Pitch_pid = AUTOAIM_PITCH_PID_INIT();
int16_t last_change_time1 = 0;
int16_t last_change_time2 = 0;
bool isPeriod1 = true;
float period1 = 0;
float	period2 = 0;

//自瞄初始化
//Called in main.c & RemoteTask.c
void InitAutoAim()
{
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK)
	{
		Error_Handler();
	}
	adjust.yaw = YAW_ADJUST;
}

//自瞄UART回调函数
//Called in RemoteTask.c
void AutoAimUartRxCpltCallback()
{
	if(RX_ENEMY_START == 0x7f && RX_ENEMY_END == 0x26)
	{
		aim.yaw=YAW_DIR*(float)((((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)>0x7fff) ? (((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)-0xffff) : (RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2 )*k_angle;
		aim.pitch=PIT_DIR*(float)((((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)>0x7fff) ? (((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)-0xffff) : (RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2 )*k_angle;
		enemy_dist=(float)((RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2)*k_distance;
		
		find_enemy = 1;
		aim_finish = 0;
		receive_cnt++;
	}
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK)
	{
		Error_Handler();
	}
}

//卡尔曼滤波
//Called in AutoAimDataProcess()
float P = 2.7f;
float Q = 1.0f;
float R = 10.0f;
float Real_Position_Kalman_Filter(float input)
{
	static float rcd = 0;
	float output;
	P = P + Q;
	float K = P / (P + R);
	output = rcd + K * (input - rcd);
	rcd = output;
	P = (1 - K) * P;
	return output;
}

float Pv = 0.62f;
float Qv = 1.0f;
float Rv = 1.0f;
float Real_Speed_Kalman_Filter(float input)
{
	static float rcd = 0;
	float output;
	Pv = Pv + Qv;
	float Kv = Pv / (Pv + Rv);
	output = rcd + Kv * (input - rcd);
	MINMAX(output, -1.0f, 1.0f);
	rcd = output;
	Pv = (1 - Kv) * Pv;
	return output;
}

float Pa = 1.0f;
float Qa = 1.0f;
float Ra = 40.0f;
float Real_Accelerate_Kalman_Filter(float input)
{
	static float rcd = 0;
	float output;
	Pa = Pa + Qa;
	float Ka = Pa / (Pa + Ra);
	output = rcd + Ka * (input - rcd);
	MINMAX(output, -0.02f, 0.02f);
	rcd = output;
	Pa = (1 - Ka) * Pa;
	return output;
}

float P_final = 4.0f;
float Q_final = 1.0f;
float R_final = 20.0f;
float Final_Kalman_Filter(float input)
{
	static float rcd = 0;
	float output;
	P_final = P_final + Q_final;
	float K_final = P_final / (P_final + R_final);
	output = rcd + K_final *(input - rcd);
	rcd = output;
	P_final = (1-K_final) * P_final;
	return output;
}

float P_gyro = 0.62f;
float Q_gyro = 1.0f;
float R_gyro = 1.0f;
float Gyro_Kalman_Filter(float input)
{
	static float rcd = 0;
	float output;
	P_gyro = P_gyro + Q_gyro;
	float K_gyro = P_gyro / (P_gyro + R_gyro);
	output = rcd + K_gyro *(input - rcd);
	rcd = output;
	P_gyro = (1-K_gyro) * P_gyro;
	return output;
}

//自瞄数据处理
//Called in AutoAim()
float pit_const = 0;
float pit_coef = 0;
float steps = PREDICT_STEPS;
float thres = ANTI_GYRO_THRES;
float cur_v = 0;
float cur_a = 0;
float kalman_last_yaw = 0;
float kalman_last_v = 0;
float window;
void AutoAimDataProcess()
{
	//pitch角度补偿计算
	if(BulletSpeed < 20)
	{
		pit_const = PITCH_ADJUST_CONST_SLOW_NORMAL;
		pit_coef = PITCH_ADJUST_COEF_SLOW_NORMAL;
		if(GMP.RealAngle > 7)
		{
			pit_const = PITCH_ADJUST_CONST_SLOW_UP;
			pit_coef = PITCH_ADJUST_COEF_SLOW_UP;
		}
		else if(enemy_dist < 250)
		{
			pit_const = PITCH_ADJUST_CONST_SLOW_CLOSE;
			pit_coef = PITCH_ADJUST_COEF_SLOW_CLOSE;
		}
	}
	else
	{
		pit_const = PITCH_ADJUST_CONST_FAST_NORMAL;
		pit_coef = PITCH_ADJUST_COEF_FAST_NORMAL;
		if(GMP.RealAngle > 7)
		{
			pit_const = PITCH_ADJUST_CONST_FAST_UP;
			pit_coef = PITCH_ADJUST_COEF_FAST_UP;
		}
		else if(enemy_dist < 250)
		{
			pit_const = PITCH_ADJUST_CONST_FAST_CLOSE;
			pit_coef = PITCH_ADJUST_COEF_FAST_CLOSE;
		}
	}
	adjust.pitch = pit_const + pit_coef * enemy_dist;

	if(!aim_finish)
	{
		//角度滤波
		aim_real.yaw = GM_RealAngle_RCD.yaw + aim.yaw * K_YAW + adjust.yaw;
		float cur_yaw =  Real_Position_Kalman_Filter(aim_real.yaw);
		//角速度滤波
		cur_v = cur_yaw - kalman_last_yaw;
		kalman_last_yaw = cur_yaw;
		cur_v = Real_Speed_Kalman_Filter(cur_v);
		//角加速度滤波
		cur_a = cur_v - kalman_last_v;
		kalman_last_v = cur_v;
		cur_a = Real_Accelerate_Kalman_Filter(cur_a);
		//预测角度滤波
		float final_prediction = Final_Kalman_Filter(cur_yaw + cur_v * steps + 0.5f * cur_a * steps * steps);
		
		//获得发射窗口参数
		window = - (aim.yaw * K_YAW + adjust.yaw) / (cur_v * steps);
		
		//陀螺中心滤波
		float gyro_center = cur_yaw;
		if(aim_mode == ANTI_GYRO_MODE)
		{
			gyro_center = Gyro_Kalman_Filter(cur_yaw);
		}
		//计算陀螺周期
		static float last_final= 0;
		float final_diff = aim_real.yaw - last_final;
		last_final = aim_real.yaw;
		if(fabs(final_diff) > thres)
		{
			if (last_change_time1 < auto_counter_autoaim)
				last_change_time1 += 2000;
			if (last_change_time2 < auto_counter_autoaim)
				last_change_time2 += 2000;
			
			if(isPeriod1)
			{
				period1 = (last_change_time1 - auto_counter_autoaim);
				isPeriod1 = false;
				last_change_time1 = auto_counter_autoaim;
			}
			else
			{
				period2 = (last_change_time2 - auto_counter_autoaim);
				isPeriod1 = true;
				last_change_time2 = auto_counter_autoaim;
			}
		}
		
		//自瞄输出值
		if(aim_mode == 0)
		{
			AutoAim_Yaw_pid.Reset(&AutoAim_Yaw_pid);
			AutoAim_Pitch_pid.Reset(&AutoAim_Pitch_pid);
			kalman_last_yaw = 0;
			kalman_last_v = 0;
		}
		else
		{
			if(aim_mode == PREDICT_MODE)
			{
				aim_output.yaw = PID_PROCESS(&AutoAim_Yaw_pid, final_prediction, GM_RealAngle_RCD.yaw);
			}
			else if(aim_mode == ANTI_GYRO_MODE)
			{
				aim_output.yaw = PID_PROCESS(&AutoAim_Yaw_pid, gyro_center, GM_RealAngle_RCD.yaw);
			}
			aim_output.pitch = PID_PROCESS(&AutoAim_Pitch_pid, 0, aim.pitch * 0.1f + adjust.pitch);
		}
	}
	//自瞄最大输出值限制
	MINMAX(aim_output.yaw, -10.0f, 10.0f);
	MINMAX(aim_output.pitch, -5.0f, 5.0f);
}

//云台控制
//Called in AutoAim()
void AutoAimControl()
{
	if(!aim_finish)
	{
		GMY.TargetAngle = GM_RealAngle_RCD.yaw + aim_output.yaw;
		GMP.TargetAngle = GM_RealAngle_RCD.pitch + aim_output.pitch;
		aim_finish = 1;
	}
}

//自瞄串口通信发送函数
//Called in AutoAim()
void AutoAimUartTxInfo()
{
	uint8_t data[5];
	int16_t imu_yaw_tmp = (int16_t)(imu.yaw / k_angle);
	
	data[0] = ((RefereeData.GameRobotState.robot_id >= 9) ? ENEMY_RED : ENEMY_BLUE);
	data[1] = (uint8_t)((imu_yaw_tmp >> 8) & 0xff);
	data[2] = (uint8_t)((imu_yaw_tmp) & 0xff);
	data[3] = 0;
	data[4] = '\n';
	
	freq_div(if(HAL_UART_Transmit_DMA(&AUTOAIM_UART, (uint8_t*)&data, sizeof(data)) != HAL_OK){Error_Handler();}, 10);
}

//自瞄帧率检测
//Called in AutoAim()
void AutoAimFpsCalc()
{
	if(auto_counter_autoaim == 0)
	{
		receive_rcd = receive_cnt / 2;
		receive_cnt = 0;
		auto_counter_autoaim = 2000;
	}
}
	 
//自瞄
//Called in ControlTask.c
void AutoAim()
{
	//自瞄数据处理
	AutoAimDataProcess();
	//串口数据发送
	AutoAimUartTxInfo();
	//自瞄帧率检测
	AutoAimFpsCalc();
	//云台控制
	switch(aim_mode)
	{
		case PREDICT_MODE:
		case ANTI_GYRO_MODE:
		{
			AutoAimControl();
		}break;
	}
	
	//200ms未收到自瞄数据find_enemy置0
	if(auto_counter_autoaim % 200 == 0)
	{
		find_enemy = 0;
	}
}

//自动发射
//Called in FunctionTask.c
int16_t anti_gyro_shootable_bound = ANTI_GYRO_SHOOTABLE_BOUND;
void AutoShoot(uint8_t mode)
{
	//预测模式自动发射判定
	if(aim_mode == PREDICT_MODE)
	{
		//发射窗口条件判定
		if(cur_v >= 0)
		{
			if(aim.yaw * K_YAW + adjust.yaw <= - cur_v * steps * PREDICT_SHOOTABLE_BOUND_BELOW && aim.yaw * K_YAW + adjust.yaw >= - cur_v * steps * PREDICT_SHOOTABLE_BOUND_ABOVE)
				auto_shoot_flag = 1;
			else
				auto_shoot_flag = 0;
		}
		else
		{
			if(aim.yaw * K_YAW + adjust.yaw >= - cur_v * steps * PREDICT_SHOOTABLE_BOUND_BELOW && aim.yaw * K_YAW + adjust.yaw <= - cur_v * steps * PREDICT_SHOOTABLE_BOUND_ABOVE)
				auto_shoot_flag = 1;
			else
				auto_shoot_flag = 0;
		}
	}
	//反陀螺模式自动发射判定
	else if(aim_mode == ANTI_GYRO_MODE)
	{
		if(last_change_time1 < auto_counter_autoaim)
			last_change_time1 += 2000;
		if(last_change_time2 < auto_counter_autoaim)
			last_change_time2 += 2000;
		//判断周期条件
		if(period1/period2 < 1.2f && period1/period2 > 0.8f)
		{
			//判断发射窗口条件
			if(fabs(last_change_time1-auto_counter_autoaim+SHOOT_DELAY_TIME-(period1+period2)*0.375f) < anti_gyro_shootable_bound)
				auto_shoot_flag = 1;
			else if(fabs(last_change_time2-auto_counter_autoaim+SHOOT_DELAY_TIME-(period1+period2)*0.375f) < anti_gyro_shootable_bound)
				auto_shoot_flag = 1;
			else
				auto_shoot_flag = 0;
		}
		else
			auto_shoot_flag = 0;
	}
	
	//目标静止
	if(fabs(cur_v) < 0.3f && aim_output.yaw < 0.5f)
		auto_shoot_flag = 1;
	
	//pitch到位判定
	if(fabs(aim_output.pitch) > 5.0f)
		auto_shoot_flag = 0;
	
	if(shoot_cd == 0)
	{
		if(find_enemy && auto_shoot_flag == 1)
		{
			ShootOneBullet(BULLET_TYPE);
		}
	}
}

//云台历史角度记录
//Called in ControlTask.c
#define rcd_amt 90
GMAngle_t GM_RealAngle_Rcd(MotorINFO *GMY, MotorINFO *GMP, int delay_t)
{
	static GMAngle_t GM_RealAngle_Rcd[rcd_amt];
	static uint8_t i = 0;
	if(GMYReseted == 1 && GMPReseted == 1)
	{
		GM_RealAngle_Rcd[i].yaw = GMY->RealAngle;
		GM_RealAngle_Rcd[i].pitch = GMP->RealAngle;
		i = (i + 1) % rcd_amt;
	}
	return GM_RealAngle_Rcd[(i + rcd_amt - delay_t) % rcd_amt];
}

//陀螺仪历史角速度记录
//Called in ControlTask.c
imu_t imu_w_rcd(imu_t *imu_current, int delay_t)
{
	static imu_t imu_Rcd[rcd_amt] = {0};
	static uint8_t i = 0;
	if(GMYReseted == 1 && GMPReseted == 1)
	{
		imu_Rcd[i].wx = imu_current->wx;
		imu_Rcd[i].wy = imu_current->wy;
		imu_Rcd[i].wz = imu_current->wz;
		i = (i + 1) % rcd_amt;
	}
	return imu_Rcd[(i + rcd_amt - delay_t) % rcd_amt];
}

#endif /*USE_AUTOAIM*/
