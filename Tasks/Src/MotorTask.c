/**
  ******************************************************************************
  * File Name          : CANMotot.c
  * Description        : CAN电机统一驱动任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

//底盘电机pid
#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,0,{0,0},\
	16.0f,0.0f,2.0f,\
	0,0,0,\
	20000,20000,20000,\
	0,16384,0,0,0,\
	&PID_Calc,&PID_Reset,\
}
//摩擦轮pid
#define FRIC_MOTOR_SPEED_PID_DEFAULT \
{\
	0,0,{0,0},\
	30.0f,0.0f,5.0f,\
	0,0,0,\
	15000,15000,15000,\
	0,10000,0,0,0,\
	&PID_Calc,&PID_Reset,\
}

#define Normal_MOTORINFO_Init(rdc,func,ppid,spid)\
{\
	ESC_C6x0,0,0,0,rdc,\
	{0,0,0},{0,0,0},0,0,1,0,0,func,\
	ppid,spid,CHASSIS_MOTOR_SPEED_PID_DEFAULT,0 \
}

#define Chassis_MOTORINFO_Init(func,spid)\
{\
	ESC_C6x0,0,0,0,1,\
	{0,0,0},{0,0,0},0,0,1,0,0,func,\
	FW_PID_DEFAULT,FW_PID_DEFAULT,spid,0 \
}

#define Gimbal_MOTORINFO_Init(rdc,func,ppid,spid)\
{\
	ESC_C6x0,0,0,0,rdc,\
	{0,0,0},{0,0,0},0,0,1,0,0,func,\
	ppid,spid,CHASSIS_MOTOR_SPEED_PID_DEFAULT,0 \
}

void ControlNM(MotorINFO *id);
void ControlCM(MotorINFO *id);
void ControlGMY(MotorINFO *id);
void ControlGMP(MotorINFO *id);

uint8_t GMYReseted = 0;
uint8_t GMPReseted = 0;

//**********************************************************************
//					pid(kp,ki,kd,kprM,kirM,kdrM,rM)
//						kprM:kp result Max
//**********************************************************************

//**********************************************************************
//				Chassis_MOTORINFO_Init(func,spid)
//**********************************************************************
MotorINFO CMFL = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMFR = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBL = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBR = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICL = Chassis_MOTORINFO_Init(&ControlCM,FRIC_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICR = Chassis_MOTORINFO_Init(&ControlCM,FRIC_MOTOR_SPEED_PID_DEFAULT);
//************************************************************************
//		     Gimbal_MOTORINFO_Init(rdc,func,ppid,spid)
//************************************************************************
//使用云台电机时，请务必确定校准过零点
#ifdef INFANTRY
MotorINFO GMP  = Normal_MOTORINFO_Init(1.0,&ControlGMP,
           fw_PID_INIT(0.3,0.0,0.2, 100.0, 100.0, 100.0, 10.0),
           fw_PID_INIT(8000.0,200.0,100.0, 50000.0, 50000.0, 50000.0, 30000.0));
MotorINFO GMY  = Normal_MOTORINFO_Init(1.0,&ControlGMY,
										 fw_PID_INIT(0.3,0.0,0.6,100.0, 100.0, 100.0, 10.0),
										 fw_PID_INIT(10000.0,200.0,100.0, 50000.0, 50000.0, 50000.0, 30000.0));
#endif
//*************************************************************************
//			Normal_MOTORINFO_Init(rdc,func,ppid,spid)
//*************************************************************************
MotorINFO STIR = Normal_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT(20.0, 0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(80.0, 0.5, 0.0, 		10000.0, 5000.0, 10000.0, 10000.0));

#ifdef INFANTRY_3
MotorINFO* can1[8]={&CMFR,&CMFL,&CMBL,&CMBR,&GMP,&GMY,0,&STIR};
MotorINFO* can2[8]={&FRICL,&FRICR,0,0,0,0,0,0};
#endif

void ControlNM(MotorINFO* id)
{
	if(id==0) return;
	if(id->s_count == 1)
	{		
		uint16_t 	ThisAngle;	
		double 		ThisSpeed;	
		ThisAngle = id->RxMsgC6x0.angle;				//未处理角度
		if(id->FirstEnter==1) {id->lastRead = ThisAngle;id->FirstEnter = 0;return;}
		if(ThisAngle<=id->lastRead)
		{
			if((id->lastRead-ThisAngle)>4000)//编码器上溢
				id->RealAngle = id->RealAngle + (ThisAngle+8192-id->lastRead) * 360 / 8192.0 / id->ReductionRate;
			else//正常
				id->RealAngle = id->RealAngle - (id->lastRead - ThisAngle) * 360 / 8192.0 / id->ReductionRate;
		}
		else
		{
			if((ThisAngle-id->lastRead)>4000)//编码器下溢
				id->RealAngle = id->RealAngle - (id->lastRead+8192-ThisAngle) *360 / 8192.0 / id->ReductionRate;
			else//正常
				id->RealAngle = id->RealAngle + (ThisAngle - id->lastRead) * 360 / 8192.0 / id->ReductionRate;
		}
		ThisSpeed = id->RxMsgC6x0.RotateSpeed * 6 / id->ReductionRate;
		
		id->Intensity = PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,ThisSpeed);
		
		id->s_count = 0;
		id->lastRead = ThisAngle;
	}
	else
	{
		id->s_count++;
	}		
}

void ControlCM(MotorINFO* id)
{
	//TargetAngle 代作为目标速度
	if(id==0) return;
	id->offical_speedPID.ref = (float)(id->TargetAngle);
	id->offical_speedPID.fdb = id->RxMsgC6x0.RotateSpeed;
	id->offical_speedPID.Calc(&(id->offical_speedPID));
	id->Intensity=(1.30f)*id->offical_speedPID.output;
}

void ControlGMY(MotorINFO* id)
{
	if(id==0) return;

	#ifdef USE_CHASSIS_FOLLOW
		#ifndef USE_GIMBAL_ENCODER
			float 	ThisAngle = -imu.yaw;
			float 	ThisSpeed = imu.wz;
			static	uint8_t	ChassisLockRCD = 0;
			if(chassis_lock)
			{
				ThisAngle = - (float)(GM_YAW_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
				ThisSpeed = imu.wz;
			}
		#else
			float 	ThisAngle = - (float)(GM_YAW_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
			float 	ThisSpeed = imu.wz;
		#endif
	#else
		double 	ThisAngle = - (double)(GM_YAW_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
		float 	ThisSpeed = imu.wz;
	#endif
			
	int8_t 	dir;
	if(id->ReductionRate>=0) dir=1;
	else dir=-1;

	if(id->FirstEnter==1) {
		id->lastRead = ThisAngle;
		id->RealAngle = - (double)(GM_YAW_ZERO - id->RxMsgC6x0.angle) * 360.0 / 8192.0 / id->ReductionRate;
		NORMALIZE_ANGLE180(id->RealAngle);
		id->FirstEnter = 0;
		return;
	}
	
	if(ThisAngle <= id->lastRead)
	{
		if((id->lastRead-ThisAngle) > 180)
			 id->RealAngle += (ThisAngle + 360 - id->lastRead)*dir;
		else
			 id->RealAngle -= (id->lastRead - ThisAngle)*dir;
	}
	else
	{
		if((ThisAngle-id->lastRead) > 180)
			 id->RealAngle -= (id->lastRead + 360 - ThisAngle)*dir;
		else
			 id->RealAngle += (ThisAngle - id->lastRead)*dir;
	}
	
	if(chassis_lock != ChassisLockRCD)
	{
		id->TargetAngle = id->RealAngle;
		ChassisLockRCD = chassis_lock;
	}
	
	#ifdef INFANTRY_3
		if(fabs((GMY.RxMsgC6x0.angle - chassis_follow_center) * 360 / 8192.0f) < 1) GMYReseted = 1;
	#endif
	
	if(GMYReseted==0) id->positionPID.outputMax = 1.0;
	else id->positionPID.outputMax = 10.0;
	
	id->lastRead = ThisAngle;
	id->Intensity = PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,ThisSpeed);	
}

void ControlGMP(MotorINFO* id)
{
	if(id==0) return;

	#ifdef USE_CHASSIS_FOLLOW
		#ifndef USE_GIMBAL_ENCODER
			float 	ThisAngle = imu.pit;
			float 	ThisSpeed = -imu.wy;
			static	uint8_t	ChassisLockRCD = 0;
			if(chassis_lock)
			{
				ThisAngle = - (float)(GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
			}
		#else
			float 	ThisAngle = - (float)(GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
			float 	ThisSpeed = -imu.wy;
		#endif
	#else
		double 	ThisAngle = - (double)(GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
	#endif
	int8_t 	dir;
	if(id->ReductionRate>=0) dir=1;
	else dir=-1;
	
	if(id->FirstEnter==1) {
		id->lastRead = ThisAngle;
		id->RealAngle = - (double)(GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f / id->ReductionRate;
		NORMALIZE_ANGLE180(id->RealAngle);
		id->FirstEnter = 0;
		return;
	}
	
	if(ThisAngle <= id->lastRead)
	{
		if((id->lastRead-ThisAngle) > 180)
			 id->RealAngle += (ThisAngle + 360 - id->lastRead)*dir;
		else
			 id->RealAngle -= (id->lastRead - ThisAngle)*dir;
	}
	else
	{
		if((ThisAngle-id->lastRead) > 180)
			 id->RealAngle -= (id->lastRead + 360 - ThisAngle)*dir;
		else
			 id->RealAngle += (ThisAngle - id->lastRead)*dir;
	}
	
	if(fabs(id->RealAngle-id->TargetAngle)<1) GMPReseted = 1;
	
	
	if(chassis_lock != ChassisLockRCD)
	{
		id->TargetAngle = id->RealAngle;
		ChassisLockRCD = chassis_lock;
	}

	#ifdef INFANTRY_3
		MINMAX(id->TargetAngle, id->RealAngle + (GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0 / 8192.0 / id->ReductionRate - 30.0f, id->RealAngle + (GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0 / 8192.0 / id->ReductionRate + 42.0f);
	#endif
	
	if(GMPReseted==0) id->positionPID.outputMax = 1.0;
	else id->positionPID.outputMax = 10.0;
	
	id->lastRead = ThisAngle ;
	id->Intensity = GM_PITCH_GRAVITY_COMPENSATION + PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,ThisSpeed);
}

//CAN
void setCAN11()
{
	CanTxMsgTypeDef pData;
	hcan1.pTxMsg = &pData;
	
	hcan1.pTxMsg->StdId = 0x200;
	hcan1.pTxMsg->ExtId = 0;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can1[i]==0) {
			hcan1.pTxMsg->Data[i*2]   = 0;
			hcan1.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan1.pTxMsg->Data[i*2]   = (uint8_t)(can1[i]->Intensity >> 8);
			hcan1.pTxMsg->Data[i*2+1] = (uint8_t)can1[i]->Intensity;
		}
	}

	if(can1_update == 1 && can1_type == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK)
		{
			Error_Handler();
		}
		
		can1_update = 0;
		
		#ifdef CAN12
			can1_type = 2;
		#else
		#ifdef DOUBLE_BOARD_CAN1
			can1_type = 3;
		#endif
		#endif
		
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}
}
void setCAN12()
{
	CanTxMsgTypeDef pData;
	hcan1.pTxMsg = &pData;
	
	hcan1.pTxMsg->StdId = 0x1ff;
	hcan1.pTxMsg->ExtId = 0;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can1[i+4]==0) {
			hcan1.pTxMsg->Data[i*2]   = 0;
			hcan1.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan1.pTxMsg->Data[i*2]   = (uint8_t)(can1[i+4]->Intensity >> 8);
			hcan1.pTxMsg->Data[i*2+1] = (uint8_t)can1[i+4]->Intensity;
		}
	}

	if(can1_update == 1 && can1_type == 2)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK)
		{
			Error_Handler();
		}
		
		can1_update = 0;
		
		#ifdef DOUBLE_BOARD_CAN1
			can1_type = 3;
		#else
		#ifdef CAN11
			can1_type = 1;
		#endif
		#endif
		
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
  }
}
void setCAN21()
{
	CanTxMsgTypeDef pData;
	hcan2.pTxMsg = &pData;
	
	hcan2.pTxMsg->StdId = 0x200;
	hcan2.pTxMsg->ExtId = 0;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can2[i]==0) {
			hcan2.pTxMsg->Data[i*2]   = 0;
			hcan2.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan2.pTxMsg->Data[i*2]   = (uint8_t)(can2[i]->Intensity >> 8);
			hcan2.pTxMsg->Data[i*2+1] = (uint8_t)can2[i]->Intensity;
		}
	}

	if(can2_update == 1 && can2_type == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		if(HAL_CAN_Transmit_IT(&hcan2) != HAL_OK)
		{
			Error_Handler();
		}
		
		can2_update = 0;
		
		#ifdef CAN22
			can2_type = 2;
		#else
		#ifdef DOUBLE_BOARD_CAN2
			can2_type = 3;
		#endif
		#endif
		
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
  }
}
void setCAN22()
{
	CanTxMsgTypeDef pData;
	hcan2.pTxMsg = &pData;
	
	hcan2.pTxMsg->StdId = 0x1ff;
	hcan2.pTxMsg->ExtId = 0;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can2[i+4]==0) {
			hcan2.pTxMsg->Data[i*2]   = 0;
			hcan2.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan2.pTxMsg->Data[i*2]   = (uint8_t)(can2[i+4]->Intensity >> 8);
			hcan2.pTxMsg->Data[i*2+1] = (uint8_t)can2[i+4]->Intensity;
		}
	}

	if(can2_update == 1 && can2_type == 2)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		if(HAL_CAN_Transmit_IT(&hcan2) != HAL_OK)
		{
			Error_Handler();
		}
		
		can2_update = 0;
		
		#ifdef DOUBLE_BOARD_CAN2
			can2_type = 3;
		#else
		#ifdef CAN21
			can2_type = 1;
		#endif
		#endif
		
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
  }
}

void InitMotor(MotorINFO *id)
{
	if(id==0) return;
	id->FirstEnter=1;
	id->lastRead=0;
	id->RealAngle=0;
	id->TargetAngle=0;
	id->offical_speedPID.Reset(&(id->offical_speedPID));
	(id->Handle)(id);
	id->Intensity=0;
}

void Motor_ID_Setting()
{
	for(int i=0;i<4;i++)
	{
		if(can1[i]!=0) 
		{
			can1[i]->CAN_TYPE=&hcan1;
			can1[i]->RXID = 0x201+i;
			can1[i]->TXID = 0x200;
		}
		if(can2[i]!=0) 
		{
			can2[i]->CAN_TYPE=&hcan2;
			can2[i]->RXID = 0x201+i;
			can2[i]->TXID = 0x200;
		}
	}
	for(int i=4;i<8;i++)
	{
		if(can1[i]!=0) 
		{
			can1[i]->CAN_TYPE=&hcan1;
			can1[i]->RXID = 0x201+i;
			can1[i]->TXID = 0x1ff;
		}
		if(can2[i]!=0) 
		{
			can2[i]->CAN_TYPE=&hcan2;
			can2[i]->RXID = 0x201+i;
			can2[i]->TXID = 0x1ff;
		}
	}
}
