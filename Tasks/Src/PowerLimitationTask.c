/**
  ******************************************************************************
  * File Name      	: PowerLimitationTask.c
  * Description    	: ���̹��������㷨ʵ��
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

float LimitFactor = 1.0f;

//���̹�������
void No_Cap_PowerLimitation(void)
{
	uint16_t sum = 0;
	int16_t CM_current_max;
	int16_t CMFLIntensity = CMFL.Intensity;
	int16_t CMFRIntensity = CMFR.Intensity;
	int16_t CMBLIntensity = CMBL.Intensity;
	int16_t CMBRIntensity = CMBR.Intensity;
	
	sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
	static int16_t FLILast,FRILast,BLILast,BRILast;
	//����ģʽ
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
	else if(RefereeData.PowerHeat.chassis_power_buffer-((RefereeData.PowerHeat.chassis_power-cur_robot_status.power_limit+15)>0?(RefereeData.PowerHeat.chassis_power-cur_robot_status.power_limit+15):0)*0.5f < 50.0f)
	{
		float realPowerBuffer = RefereeData.PowerHeat.chassis_power_buffer;
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


//���ڳ�̬�Ļ����Լ�⹦�ʵĹ�������
void CurBased_PowerLimitation(void)
{
	int32_t sum = 0;
	int32_t CM_current_max;
	int32_t CMFLIntensity = CMFL.Intensity;
	int32_t CMFRIntensity = CMFR.Intensity;
	int32_t CMBLIntensity = CMBL.Intensity;
	int32_t CMBRIntensity = CMBR.Intensity;
	//����ģʽ
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
	
	//�¹�����
	else if((RefereeData.PowerHeat.chassis_power_buffer-((Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-cur_robot_status.power_limit+15)>0?(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-cur_robot_status.power_limit+15):0)*0.5f < 50.0f))
	{
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		float realPowerBuffer = RefereeData.PowerHeat.chassis_power_buffer;
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


//���ڷŵ�ģʽ�µĻ����Լ�⹦�ʵĹ�������
void CapBased_PowerLimitation(void)
{
	int32_t sum = 0;
	int32_t CM_current_max;
	int32_t CMFLIntensity = CMFL.Intensity;
	int32_t CMFRIntensity = CMFR.Intensity;
	int32_t CMBLIntensity = CMBL.Intensity;
	int32_t CMBRIntensity = CMBR.Intensity;
	
	//����ģʽ
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
	else if( (RefereeData.PowerHeat.chassis_power_buffer-((Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-cur_robot_status.power_limit+10)>0?(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-cur_robot_status.power_limit+10):0)*1.0f < 50.0f))
	{
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		float realPowerBuffer = RefereeData.PowerHeat.chassis_power_buffer;
		
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		LimitFactor = 2500 + 200*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	else if(Cap_Get_Cap_Voltage() < 12.5 && (RefereeData.PowerHeat.chassis_power_buffer-((Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-70)>0?(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-70):0)*1.0f < 20.0f))
	{
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		float realPowerBuffer = RefereeData.PowerHeat.chassis_power_buffer;
		
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

void PowerLimitation(void)
{
//	if(Cap_Get_Cap_State() == CAP_STATE_STOP)
//	{
//		No_Cap_PowerLimitation(); //�����Բ⹦�ʵĹ������ƣ������ڳ���ֹͣ״̬
//	}
//	else
//	{
//		if (Cap_Get_Cap_State() == CAP_STATE_RECHARGE || Cap_Get_Cap_State() == CAP_STATE_TEMP_RECHARGE)
//			CurBased_PowerLimitation();//�����Բ⹦�ʵĹ������ƣ������ڳ���ֹͣ״̬
//		else
//		{
//			if (Cap_Get_Cap_State() == CAP_STATE_RELEASE)
//				CapBased_PowerLimitation();//�������ݹ���ģʽ�µĹ�������
//		}
//	}
	No_Cap_PowerLimitation(); //�޵��ݵĹ�������
}