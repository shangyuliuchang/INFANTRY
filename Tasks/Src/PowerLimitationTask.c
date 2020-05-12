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
//#define POWER_LIMITATION_DEBUG

void PowerLimitation(void)
{
	double kIntensityToMoment=200000.0;
	double PowerMax=(double)Cap_Get_Aim_Power()*0.7;
	double rotateSpeedRate=0.0;
	double CMFLRotateSpeed=(double)CMFL.RxMsgC6x0.RotateSpeed;
	double CMFRRotateSpeed=(double)CMFR.RxMsgC6x0.RotateSpeed;
	double CMBLRotateSpeed=(double)CMBL.RxMsgC6x0.RotateSpeed;
	double CMBRRotateSpeed=(double)CMBR.RxMsgC6x0.RotateSpeed;
	double PowerSum=CMFL.offical_speedPID.kp*((CMFL.TargetAngle-CMFLRotateSpeed)*CMFLRotateSpeed+\
																						(CMFR.TargetAngle-CMFRRotateSpeed)*CMFRRotateSpeed+\
																						(CMBL.TargetAngle-CMBLRotateSpeed)*CMBLRotateSpeed+\
																						(CMBR.TargetAngle-CMBRRotateSpeed)*CMBRRotateSpeed)/kIntensityToMoment;
	
	#ifdef POWER_LIMITATION_DEBUG
	PowerMax=30.0;
	#else
		#ifdef USE_CAP3
			if(Cap_Get_Cap_State()==CAP_STATE_RELEASE && Cap_Get_Cap_Voltage()>12.0){
				PowerMax = 10000.0;
			}
		#endif
	#endif
	if(PowerSum>PowerMax){
		rotateSpeedRate=1.0-(PowerSum-PowerMax)/(CMFL.offical_speedPID.kp*(CMFL.TargetAngle*CMFLRotateSpeed+\
																																			 CMFR.TargetAngle*CMFRRotateSpeed+\
																																			 CMBL.TargetAngle*CMBLRotateSpeed+\
																																			 CMBR.TargetAngle*CMBRRotateSpeed))*kIntensityToMoment;
		if(rotateSpeedRate>1.0)rotateSpeedRate=1.0;
		if(rotateSpeedRate<0.0)rotateSpeedRate=0.0;
		CMFL.TargetAngle*=rotateSpeedRate;
		CMFR.TargetAngle*=rotateSpeedRate;
		CMBL.TargetAngle*=rotateSpeedRate;
		CMBR.TargetAngle*=rotateSpeedRate;
	}
}
