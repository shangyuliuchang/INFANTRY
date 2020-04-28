/**
  ******************************************************************************
  * File Name      	: PowerLimitationTask.c
  * Description    	: 底盘功率限制算法实现
  * Author			:
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#include "math.h"
#define POWER_LIMITATION_DEBUG

void PowerLimitation(void)
{
	double kIntensityToMoment=200000.0;
	double PowerMax=(double)Cap_Get_Aim_Power()*0.7;
	double rotateSpeedRate=0.0;
	double PowerSum=CMFL.offical_speedPID.kp*((CMFL.TargetAngle-CMFL.RxMsgC6x0.RotateSpeed)*CMFL.RxMsgC6x0.RotateSpeed+\
																						(CMFR.TargetAngle-CMFR.RxMsgC6x0.RotateSpeed)*CMFR.RxMsgC6x0.RotateSpeed+\
																						(CMBL.TargetAngle-CMBL.RxMsgC6x0.RotateSpeed)*CMBL.RxMsgC6x0.RotateSpeed+\
																						(CMBR.TargetAngle-CMBR.RxMsgC6x0.RotateSpeed)*CMBR.RxMsgC6x0.RotateSpeed)/kIntensityToMoment;
	
	#ifdef POWER_LIMITATION_DEBUG
	PowerMax=30.0;
	#else
	if(Cap_Get_Cap_State()==CAP_STATE_RELEASE && Cap_Get_Cap_Voltage()>12.0){
		PowerMax = INT16_MAX;
	}
	#endif
	if(PowerSum>PowerMax){
		rotateSpeedRate=1.0-(PowerSum-PowerMax)/(CMFL.offical_speedPID.kp*(CMFL.TargetAngle*CMFL.RxMsgC6x0.RotateSpeed+\
																																			 CMFR.TargetAngle*CMFR.RxMsgC6x0.RotateSpeed+\
																																			 CMBL.TargetAngle*CMBL.RxMsgC6x0.RotateSpeed+\
																																			 CMBR.TargetAngle*CMBR.RxMsgC6x0.RotateSpeed)/kIntensityToMoment);
		CMFL.TargetAngle*=rotateSpeedRate;
		CMFR.TargetAngle*=rotateSpeedRate;
		CMBL.TargetAngle*=rotateSpeedRate;
		CMBR.TargetAngle*=rotateSpeedRate;
	}
}
