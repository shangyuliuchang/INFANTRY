/**
  ******************************************************************************
  * File Name          : ControlTask.c
  * Description        : 主控制任务
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

WorkState_e WorkState = PREPARE_STATE;
WorkState_e RxWorkState = PREPARE_STATE;
uint16_t prepare_time = 0;
uint16_t counter = 0;
double rotate_speed = 0;
MusicNote SuperMario[] = {
	{H3, 100}, {0, 50}, 
	{H3, 250}, {0, 50}, 
	{H3, 100}, {0, 50}, 
	{0, 150},
	{H1, 100}, {0, 50},  
	{H3, 250}, {0, 50},
	{H5, 250}, {0, 50},
	{0, 300},
	{M5, 250}, {0, 50},
	{0, 300},
	{H1, 250}, {0, 50}
};

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 

void playMusicSuperMario(void){
	HAL_TIM_PWM_Start(&BUZZER_TIM, TIM_CHANNEL_1);
	for(int i = 0; i < sizeof(SuperMario) / sizeof(MusicNote); i++){
			PLAY(SuperMario[i].note, SuperMario[i].time);
	}
	HAL_TIM_PWM_Stop(&BUZZER_TIM, TIM_CHANNEL_1);
}

//状态机切换
void WorkStateFSM(void)
{
	#ifndef SUB_BOARD
	switch (WorkState)
	{
		case PREPARE_STATE:				//准备模式
		{
			//if (inputmode == STOP) WorkState = STOP_STATE;
			if(prepare_time < 500) prepare_time++;	
			if(prepare_time >= 500 && imu.InitFinish == 1 && isCan11FirstRx == 1 && isCan12FirstRx == 1 && isCan21FirstRx == 1 && isCan22FirstRx == 1)//imu初始化完成且所有can电机上电完成后进入正常模式
			{
				//playMusicSuperMario();
				CMRotatePID.Reset(&CMRotatePID);
				WorkState = NORMAL_STATE;
				#ifdef SUB_BOARD
				WorkState = RxWorkState;
				#endif
				prepare_time = 0;
			}
			for(int i=0;i<8;i++) 
			{
				{
					InitMotor(can1[i]);
					InitMotor(can2[i]);
				}
			}
			#ifdef CAN11
			setCAN11();
			#endif
			#ifdef CAN12
			setCAN12();
			#endif
			#ifdef CAN21
			setCAN21();
			#endif
			#ifdef CAN22
			setCAN22();
			#endif
			FunctionTaskInit();
		}break;
		case NORMAL_STATE:				//正常模式
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			
			if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
			if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
		}break;
		case ADDITIONAL_STATE_ONE:		//附加模式一
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			
			if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
			if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
		}break;
		case ADDITIONAL_STATE_TWO:		//附加模式二
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			
			if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
			if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
		}break;
		case STOP_STATE:				//紧急停止
		{
			for(int i=0;i<8;i++) 
			{
				if(can1[i]==&FRICL || can1[i]==&FRICR)
				{
					CAN1_SHUTDOWN(i);
					InitMotor(can2[i]);
				}
				else if(can2[i]==&FRICL || can2[i]==&FRICR)
				{
					InitMotor(can1[i]);
					CAN2_SHUTDOWN(i);
				}
				else
				{
					InitMotor(can1[i]);
					InitMotor(can2[i]);
				}
			}
			#ifdef CAN11
			setCAN11();
			#endif
			#ifdef CAN12
			setCAN12();
			#endif
			#ifdef CAN21
			setCAN21();
			#endif
			#ifdef CAN22
			setCAN22();
			#endif
			if (inputmode == REMOTE_INPUT || inputmode == KEY_MOUSE_INPUT)
			{
				WorkState = PREPARE_STATE;
				GMYReseted=0;
				GMPReseted=0;
				ChassisTwistState = 0;
				FunctionTaskInit();
			}
		}break;
		default: break;
	}
	#else
	WorkState = RxWorkState;
	#endif
}
void ControlRotate(void)
{	
	#ifdef USE_CHASSIS_FOLLOW
		switch (ChassisTwistState)
		{
			case 1: 
			{
				ChassisSpeedRef.rotate_ref = 50;
				break;
			}
			case 2: 
			{
				ChassisSpeedRef.rotate_ref = -50;
				break;
			}
			default: ChassisSpeedRef.rotate_ref = (GMY.RxMsgC6x0.angle - chassis_follow_center) * 360 / 8192.0f - ChassisTwistGapAngle; break;
		}
		NORMALIZE_ANGLE180(ChassisSpeedRef.rotate_ref);
	#endif
	CMRotatePID.ref = 0;
	CMRotatePID.fdb = ChassisSpeedRef.rotate_ref;
	CMRotatePID.Calc(&CMRotatePID);
	if(ChassisTwistState) MINMAX(CMRotatePID.output,-50,50);
	rotate_speed = CMRotatePID.output * 30;
}

void Chassis_Data_Decoding()
{
	if(!chassis_lock)
	{
		ControlRotate();
		
		float cosPlusSin, cosMinusSin, GMYEncoderAngle;

//		if(ChassisTwistState == 0)
//		{
//			GMYEncoderAngle = (GMY.RxMsgC6x0.angle - GM_YAW_ZERO) * 6.28f / 8192.0f;
//		}
//		else
//		{
//			//有功率限制情况下调整陀螺走直线
//			if(Cap_Get_Cap_State() != CAP_STATE_RELEASE)
//			{
//				GMYEncoderAngle = (GMY.RxMsgC6x0.angle - GM_YAW_ZERO) * 6.28f / 8192.0f - ChassisSpeedRef.rotate_ref / 57.3f / 3.0f;
//			}
//			else
//			{
//				GMYEncoderAngle = (GMY.RxMsgC6x0.angle - GM_YAW_ZERO) * 6.28f / 8192.0f;
//			}
//			ChassisSpeedRef.forward_back_ref /= 1.1f;
//			ChassisSpeedRef.left_right_ref /= 1.1f;
//		}
		GMYEncoderAngle = (GMY.RxMsgC6x0.angle - GM_YAW_ZERO) * 6.28f / 8192.0f;
		
		cosPlusSin = cos(GMYEncoderAngle) + sin(GMYEncoderAngle);
		cosMinusSin = cos(GMYEncoderAngle) - sin(GMYEncoderAngle);

		#ifdef USE_CHASSIS_FOLLOW
		CMFL.TargetAngle = (	ChassisSpeedRef.forward_back_ref * cosMinusSin + ChassisSpeedRef.left_right_ref * cosPlusSin + rotate_speed)*12;
		CMFR.TargetAngle = (- ChassisSpeedRef.forward_back_ref * cosPlusSin + ChassisSpeedRef.left_right_ref * cosMinusSin + rotate_speed)*12;
		CMBL.TargetAngle = (  ChassisSpeedRef.forward_back_ref * cosPlusSin - ChassisSpeedRef.left_right_ref * cosMinusSin + rotate_speed)*12;
		CMBR.TargetAngle = (- ChassisSpeedRef.forward_back_ref * cosMinusSin - ChassisSpeedRef.left_right_ref * cosPlusSin + rotate_speed)*12;
		#else
		CMFL.TargetAngle = (  ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref	+ rotate_speed)*12;
		CMFR.TargetAngle = (- ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + rotate_speed)*12;
		CMBL.TargetAngle = (  ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + rotate_speed)*12;
		CMBR.TargetAngle = (- ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref	+ rotate_speed)*12;
		#endif
	}
	else
	{
		CMFL.TargetAngle = 0;
		CMFR.TargetAngle = 0;
		CMBL.TargetAngle = 0;
		CMBR.TargetAngle = 0;
	}
}

//主控制循环
void controlLoop()
{
	getJudgeState();
	WorkStateFSM();
	#ifdef DOUBLE_BOARD_CAN1
	CANTxInfo(&hcan1);
	#endif
	#ifdef DOUBLE_BOARD_CAN2
	CANTxInfo(&hcan2);
	#endif
	
	if(WorkState > 0)
	{
		Chassis_Data_Decoding();
		
		for(int i=0;i<8;i++) if(can1[i]!=0) (can1[i]->Handle)(can1[i]);
		for(int i=0;i<8;i++) if(can2[i]!=0) (can2[i]->Handle)(can2[i]);
		#ifdef USE_CHASSIS_ADJUST
		if(CAP_STATE_RELEASE){chassisMixingPID(12,0,2, 4,0.01,0);}
		else{chassisMixingPID(12,0,2, 3,0.4,0);}
		#endif
		
		OptionalFunction();

		#ifdef CAN11
		setCAN11();
		#endif
		#ifdef CAN12
		setCAN12();
		#endif
		#ifdef CAN21
		setCAN21();
		#endif
		#ifdef CAN22
		setCAN22();
		#endif
	}
}

void heatCalc()//1ms
{
	if(fakeHeat0 >= cooldown0/1000) fakeHeat0 -= cooldown0/1000;
	else fakeHeat0 = 0;
	if(fakeHeat1 >= cooldown1/1000) fakeHeat1 -= cooldown1/1000;
	else fakeHeat1 = 0;
}

int delay_t = 60;
//时间中断入口函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)//1ms时钟`
	{
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		#ifndef SUB_BOARD
			//imu解算
			mpu_get_data();
			imu_ahrs_update();
			imu_attitude_update();
		#endif
		//主循环在时间中断中启动
		controlLoop();
		heatCalc();
		static uint8_t cap_time_cnt = 0;
		cap_time_cnt += 1;
		if (cap_time_cnt >= 2){
		   Cap_Run();
		   cap_time_cnt = 0;
		}
		
		#ifndef SUB_BOARD
		//自瞄数据解算（5ms）
		static int aim_cnt=0;
		aim_cnt++;
		GM_RealAngle_RCD = GM_RealAngle_Rcd(&GMY, &GMP, delay_t);
		imu_w_RCD = imu_w_rcd(&imu, delay_t);
		if(aim_cnt >= 5)
		{
			AutoAim();
			aim_cnt=0;
		}
		#endif
		
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}
	else if (htim->Instance == htim7.Instance)//ims时钟
	{
		rc_cnt++;
		if(auto_counter > 0) auto_counter--;
		if(auto_counter_stir > 0) auto_counter_stir--;
		if(auto_counter_autoaim > 0) auto_counter_autoaim--;
		if(auto_counter_heat0 > 0) auto_counter_heat0--;
		if(auto_counter_heat1 > 0) auto_counter_heat1--;
		if(shoot_cd > 0) shoot_cd--;
		
		#ifndef SUB_BOARD
		if (rx_free == 1 && tx_free == 1)
		{
			if( (rc_cnt <= 17) && (rc_first_frame == 1))
			{
				RemoteDataProcess(rc_data);				//遥控器数据解算
				HAL_UART_AbortReceive(&RC_UART);
				rx_free = 0;
				while(HAL_UART_Receive_DMA(&RC_UART, rc_data, 18)!= HAL_OK);
				if (counter == 10) 
				{
					tx_free = 0;
					Referee_Transmit_UserData();
					counter = 0;
				}
				else counter++;				
					rc_cnt = 0;
			}
			else
			{
				if(rc_first_frame == 0) 
				{
				   WorkState = PREPARE_STATE;
				   HAL_UART_AbortReceive(&RC_UART);
				   while(HAL_UART_Receive_DMA(&RC_UART, rc_data, 18)!= HAL_OK);
  				 rc_cnt = 0;
				   rc_first_frame = 1;
				}
			}
			rc_update = 0;
		}
		#else
		HAL_IWDG_Refresh(&hiwdg);
		#endif
	}
}
