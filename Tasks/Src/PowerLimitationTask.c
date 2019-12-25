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

float SpeedAttenuation = 1.0f;
float LimitFactor = 1.0f;
uint8_t flag = 1;
fw_PID_Regulator_t PowerLimitationPID = POWER_LIMITATION_PID_DEFAULT;

//底盘功率限制
void PowerLimitation(void)
{
	uint16_t sum = 0;
	int16_t CM_current_max;
	int16_t CMFLIntensity = CMFL.Intensity;
	int16_t CMFRIntensity = CMFR.Intensity;
	int16_t CMBLIntensity = CMBL.Intensity;
	int16_t CMBRIntensity = CMBR.Intensity;
	
	sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
	static int16_t FLILast,FRILast,BLILast,BRILast;
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
	}
	else if(PowerHeat.chassis_power_buffer-((PowerHeat.chassis_power-105)>0?(PowerHeat.chassis_power-105):0)*0.5f < 50.0f)
	{
		float realPowerBuffer = PowerHeat.chassis_power_buffer;
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	else if(Cap_Get_Cap_State()!=CAP_STATE_RELEASE||Cap_Get_Cap_Voltage()<12)
	{
		CM_current_max = 11000;
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+0.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+0.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+0.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+0.0f))*CM_current_max;
		}
	}
	else if(sum>11000)
	{
	  FLILast=(CMFLIntensity>0?1:-1)*abs(FLILast);
	  FRILast=(CMFRIntensity>0?1:-1)*abs(FRILast);
	  BLILast=(CMBLIntensity>0?1:-1)*abs(BLILast);
	  BRILast=(CMBRIntensity>0?1:-1)*abs(BRILast);
		if(abs(CMFLIntensity-FLILast)>2000)
		{
			CMFLIntensity=FLILast+(CMFLIntensity-FLILast)*0.01;
			CMFRIntensity=FRILast+(CMFRIntensity-FRILast)*0.01;
			CMBLIntensity=BLILast+(CMBLIntensity-BLILast)*0.01;
			CMBRIntensity=BRILast+(CMBRIntensity-BRILast)*0.01;
		}
		else if(abs(CMFLIntensity-FLILast)>1000)
		{
			CMFLIntensity=FLILast+(CMFLIntensity-FLILast)*0.02;
			CMFRIntensity=FRILast+(CMFRIntensity-FRILast)*0.02;
			CMBLIntensity=BLILast+(CMBLIntensity-BLILast)*0.02;
			CMBRIntensity=BRILast+(CMBRIntensity-BRILast)*0.02;
		}
	}
	FLILast=CMFLIntensity;
	FRILast=CMFRIntensity;
	BLILast=CMBLIntensity;
	BRILast=CMBRIntensity;
	
	CMFL.Intensity = CMFLIntensity;
	CMFR.Intensity = CMFRIntensity;
	CMBL.Intensity = CMBLIntensity;
	CMBR.Intensity = CMBRIntensity;
}


//用于常态的基于自检测功率的功率限制
void CurBased_PowerLimitation(void)
{
	int32_t sum = 0;
	int32_t CM_current_max;
	int32_t CMFLIntensity = CMFL.Intensity;
	int32_t CMFRIntensity = CMFR.Intensity;
	int32_t CMBLIntensity = CMBL.Intensity;
	int32_t CMBRIntensity = CMBR.Intensity;
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
	}
	
	//仿桂电策略
	else if((PowerHeat.chassis_power_buffer-((Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-65)>0?(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-65):0)*0.5f < 50.0f))
	{
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		float realPowerBuffer = PowerHeat.chassis_power_buffer;
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	CMFL.Intensity = CMFLIntensity;
	CMFR.Intensity = CMFRIntensity;
	CMBL.Intensity = CMBLIntensity;
	CMBR.Intensity = CMBRIntensity;
	rlease_flag = 0;
}


//用于放电模式下的基于自检测功率的功率限制
void CapBased_PowerLimitation(void)
{
	int32_t sum = 0;
	int32_t CM_current_max;
	int32_t CMFLIntensity = CMFL.Intensity;
	int32_t CMFRIntensity = CMFR.Intensity;
	int32_t CMBLIntensity = CMBL.Intensity;
	int32_t CMBRIntensity = CMBR.Intensity;
	
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
	}
	else if( (PowerHeat.chassis_power_buffer-((Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-70)>0?(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-70):0)*1.0f < 50.0f))
	{
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		float realPowerBuffer = PowerHeat.chassis_power_buffer;
		
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		LimitFactor = 2500 + 200*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	else if(Cap_Get_Cap_Voltage() < 12.5 && (PowerHeat.chassis_power_buffer-((Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-70)>0?(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-70):0)*1.0f < 20.0f))
	{
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		float realPowerBuffer = PowerHeat.chassis_power_buffer;
		
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	
	CMFL.Intensity = CMFLIntensity;
	CMFR.Intensity = CMFRIntensity;
	CMBL.Intensity = CMBLIntensity;
	CMBR.Intensity = CMBRIntensity;
	rlease_flag = 0;
}
