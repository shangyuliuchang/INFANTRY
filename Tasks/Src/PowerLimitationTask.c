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
int32_t IntensitySum;
//#define POWER_LIMITATION_DEBUG

void PowerLimitation(void)
{
	#ifndef POWER_LIMITATION_DEBUG
	float realPowerBuffer=0;
	#endif
	
	int16_t CMFLIntensity = __fabs((CMFL.TargetAngle - CMFL.RxMsgC6x0.RotateSpeed) * CMFL.offical_speedPID.kp);
	int16_t CMFRIntensity = __fabs((CMFR.TargetAngle - CMFR.RxMsgC6x0.RotateSpeed) * CMFR.offical_speedPID.kp);
	int16_t CMBLIntensity = __fabs((CMBL.TargetAngle - CMBL.RxMsgC6x0.RotateSpeed) * CMBL.offical_speedPID.kp);
	int16_t CMBRIntensity = __fabs((CMBR.TargetAngle - CMBR.RxMsgC6x0.RotateSpeed) * CMBR.offical_speedPID.kp);
	
	int32_t IntensityMax = (Cap_Get_Aim_Power()*0.8f*819.2f/25.0f<14000?Cap_Get_Aim_Power()*0.8f*819.2f/25.0f:14000);
	int32_t IntensitySum = CMFLIntensity + CMFRIntensity + CMBLIntensity + CMBRIntensity;
	
	
	#ifndef POWER_LIMITATION_DEBUG
	if(Cap_Get_Cap_State()==CAP_STATE_RELEASE && Cap_Get_Cap_Voltage()>12){
		IntensityMax = INT16_MAX;
	}
	else if(Cap_Get_Cap_State == CAP_STATE_EMERGENCY){
		realPowerBuffer = RefereeData.PowerHeat.chassis_power_buffer>0?RefereeData.PowerHeat.chassis_power_buffer:0;
		IntensityMax = 2500 + 192*realPowerBuffer;
	}
	#else
	IntensityMax=2000;
	#endif
	
	if(IntensitySum>IntensityMax){
		CMFL.TargetAngle = (CMFL.TargetAngle - CMFL.RxMsgC6x0.RotateSpeed) * IntensityMax / IntensitySum + CMFL.RxMsgC6x0.RotateSpeed;
		CMFR.TargetAngle = (CMFR.TargetAngle - CMFR.RxMsgC6x0.RotateSpeed) * IntensityMax / IntensitySum + CMFR.RxMsgC6x0.RotateSpeed;
		CMBL.TargetAngle = (CMBL.TargetAngle - CMBL.RxMsgC6x0.RotateSpeed) * IntensityMax / IntensitySum + CMBL.RxMsgC6x0.RotateSpeed;
		CMBR.TargetAngle = (CMBR.TargetAngle - CMBR.RxMsgC6x0.RotateSpeed) * IntensityMax / IntensitySum + CMBR.RxMsgC6x0.RotateSpeed;
	}
}
