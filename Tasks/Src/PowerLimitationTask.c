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


void PowerLimitation(void)
{
	float realPowerBuffer=0;
	int16_t CMFLIntensity = CMFL.Intensity;
	int16_t CMFRIntensity = CMFR.Intensity;
	int16_t CMBLIntensity = CMBL.Intensity;
	int16_t CMBRIntensity = CMBR.Intensity;
	
	int32_t IntensityMax = (Cap_Get_Aim_Power()*0.8f*2000.0f/Cap_Get_Power_Voltage()<14000?Cap_Get_Aim_Power()*0.8f*2000.0f/Cap_Get_Power_Voltage():14000);
	int32_t IntensitySum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
	
	
	if(Cap_Get_Cap_State()==CAP_STATE_RELEASE && Cap_Get_Cap_Voltage()<15){
		if(IntensitySum > IntensityMax){
			CMFLIntensity = CMFLIntensity * IntensityMax / IntensitySum;
			CMBLIntensity = CMBLIntensity * IntensityMax / IntensitySum;
			CMBRIntensity = CMBRIntensity * IntensityMax / IntensitySum;
			CMFRIntensity = CMFRIntensity * IntensityMax / IntensitySum;
		}
	}
	else if(Cap_Get_Cap_State == CAP_STATE_EMERGENCY){
		realPowerBuffer = RefereeData.PowerHeat.chassis_power_buffer>0?RefereeData.PowerHeat.chassis_power_buffer:0;
		IntensityMax = 2500 + 192*realPowerBuffer;
		if(IntensitySum > IntensityMax){
			CMFLIntensity = CMFLIntensity * IntensityMax / IntensitySum;
			CMBLIntensity = CMBLIntensity * IntensityMax / IntensitySum;
			CMBRIntensity = CMBRIntensity * IntensityMax / IntensitySum;
			CMFRIntensity = CMFRIntensity * IntensityMax / IntensitySum;
		}
	}
	
	CMFL.Intensity = CMFLIntensity;
	CMBL.Intensity = CMBLIntensity;
	CMBR.Intensity = CMBRIntensity;
	CMFR.Intensity = CMFRIntensity;
}
