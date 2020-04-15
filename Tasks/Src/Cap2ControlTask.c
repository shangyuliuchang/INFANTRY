/**
  ******************************************************************************
  *FileName     : Cap2ControlTask.c                                            *
  *Description  : 超级电容开关控制程序                                        *
  *Author       : 唐欣阳                                                       *
  ******************************************************************************
  *                                                                            *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University               *
  * All rights reserved.                                                       *
  *                                                                            *
  ******************************************************************************
  */

#include <math.h>
#include <string.h>
#include "includes.h"

/*使用之前注意：
1.按照硬件版本配置Cube
2.调用Cap_Init()、Cap_Run()
3.检查与裁判系统的通信，否则电容无法正常工作
*/

//启用电容的版本,请务必与车上的硬件版本相一致
//#define USE_CAP1
//#define USE_CAP2


//各版本下启用运行模式


#ifdef USE_CAP3
  #define CAP_LED_SHOW
#endif /* USE_CAP3 */
//Program Begin!

#define ADC_CHANNALS         (4)
#define ADC_HITS             (50)
	
#define AIM_POWER            (JUDGE_State == OFFLINE?(80.0f):(80.0f))
#define Cap_MOS_1_GPIO_PORT  GPIOE
#define Cap_MOS_2_GPIO_PORT  GPIOC

#define VAL_POWER_Voltage    (((float)ADC_val[1]*3.3f*11.0f)/4095.0f)//PB0
#define VAL_CAP_Voltage		   ((float)ADC_val[2]*3.3f*11.0f/4095.0f)							//PB1
#define VAL_POWER_CUR        ((VAL_CAP_Voltage>0.0f?AIM_POWER*0.95f/VAL_CAP_Voltage:10.0f)<=10.0f?(VAL_CAP_Voltage>0.0f?AIM_POWER*0.95f/VAL_CAP_Voltage:10.0f):10.0f)
#define DAC_OUT		           (uint32_t)(VAL_POWER_CUR*4095.0f/3.3f/5.0f)

float Iset=0;
int disableInput=0;
#ifdef USE_CAP3	
static uint16_t mos[4]={GPIO_PIN_12,GPIO_PIN_6,GPIO_PIN_2,GPIO_PIN_3};
#endif

static int16_t ADC_hits_val[ADC_HITS][ADC_CHANNALS];
static int32_t ADC_tmp[ADC_CHANNALS];
int16_t ADC_val[ADC_CHANNALS];

cap_state CapState = CAP_STATE_STOP;

static void Cap_State(void);
static void Cap_Ctr(void);


void Cap_Init(void) {
	#ifdef USE_CAP3
	HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[0],GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[1],GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[2],GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[3],GPIO_PIN_RESET);
	
	HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1,DAC_ALIGN_12B_R,0);
	HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
	#endif /* USE_CAP3 */
	memset(ADC_hits_val, 0, sizeof(ADC_hits_val));
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_hits_val, ADC_CHANNALS*ADC_HITS);
}
//在控制主循环中被调用
void Cap_Run(void) {
	int cnt1, cnt2;
	memset(ADC_tmp, 0, sizeof(ADC_tmp));
	for (cnt1 = 0; cnt1 < ADC_CHANNALS; cnt1++) {
		for (cnt2 = 0; cnt2 < ADC_HITS; cnt2++) {
			ADC_tmp[cnt1] += ADC_hits_val[cnt2][cnt1];
		}
		ADC_val[cnt1] = ADC_tmp[cnt1] / ADC_HITS;
	}
	#ifdef CAP_LED_SHOW
	  LED_Show_SuperCap_Voltage(1);
	#endif /* CAP_LED_SHOW */
	Iset=VAL_POWER_CUR;
	Cap_Ctr();
	Cap_State();
}
float Cap_Get_Aim_Power(void){
	return AIM_POWER;
}

//负责切换状态并控制硬件做出相应切换动作
static void Cap_State_Switch(cap_state State) {
	switch (State) {
	case CAP_STATE_STOP:
		#ifdef USE_CAP3
					CapState = CAP_STATE_STOP;
					HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[0],GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[1],GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[2],GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[3],GPIO_PIN_RESET);
		#endif /* USE_CAP3 */
		break;
	case CAP_STATE_RELEASE:
		#ifdef USE_CAP3
					CapState = CAP_STATE_RELEASE;
					HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[0],GPIO_PIN_SET);
					HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[1],GPIO_PIN_SET);
					HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[2],GPIO_PIN_RESET);
					//HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[3],GPIO_PIN_RESET);
		#endif /* USE_CAP3 */
		break;
	case CAP_STATE_EMERGENCY:
		#ifdef USE_CAP3
			CapState=CAP_STATE_EMERGENCY;
			HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[0],GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[1],GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[2],GPIO_PIN_SET);
			//HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[3],GPIO_PIN_RESET);
		#endif /* USE_CAP3 */
		break;
	
	case CAP_STATE_PREPARE:
		#ifdef USE_CAP3
			CapState=CAP_STATE_PREPARE;
			HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[0],GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[1],GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[2],GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[3],GPIO_PIN_RESET);
		#endif /* USE_CAP3 */
		break;
	}
}

double Cap_Get_Cap_Voltage(void) {
	#ifdef BOARD_SLAVE
		return rx_cap_voltage;
	#else
		#ifdef USE_CAP3
			return VAL_CAP_Voltage;
		#else
			return 25;
		#endif
	#endif
}


double Cap_Get_Power_Voltage(void){
	#ifdef BOARD_SLAVE
		return rx_power_voltage;
	#else
		#ifdef USE_CAP3
			if(VAL_POWER_Voltage>0)
				return VAL_POWER_Voltage;
			else
				return 0.1;
		#else
			return 25;
		#endif
	#endif
}

double Cap_Get_Power_CURR(void){
	#ifdef BOARD_SLAVE
		return rx_power_current;
	#else
		#ifdef USE_CAP3
			return VAL_POWER_CUR;
		#else
			return 1;
		#endif
	#endif
}

cap_state Cap_Get_Cap_State(void) {
	return CapState;
}


/**
  * @brief  Set the pwm compare or dac output according to the state.
  * @param  None
  * @retval None
  */

//处于某个状态时的硬件控制
static void Cap_State(void) { // called with period of 2 ms
	switch (CapState) {
	case CAP_STATE_STOP:
		HAL_DAC_SetValue(&hdac,	DAC1_CHANNEL_1,	DAC_ALIGN_12B_R,	0);
		break;
	case CAP_STATE_RELEASE:
			if(disableInput==0){
				HAL_DAC_SetValue(&hdac,	DAC_CHANNEL_1,	DAC_ALIGN_12B_R,	DAC_OUT);
			}else{
				HAL_DAC_SetValue(&hdac,	DAC_CHANNEL_1,	DAC_ALIGN_12B_R,	0);
			}
		break;
	case CAP_STATE_EMERGENCY:
		HAL_DAC_SetValue(&hdac,	DAC1_CHANNEL_1,	DAC_ALIGN_12B_R,	0);
		break;
	
	case CAP_STATE_PREPARE:
		HAL_DAC_SetValue(&hdac,	DAC1_CHANNEL_1,	DAC_ALIGN_12B_R,	DAC_OUT);
		break;
	}
}

/**
  * @brief  The detailed capacitance control logic.
  * @param  None
  * @retval None
  */
static void Cap_Ctr_STOP() {
	if(VAL_POWER_Voltage>10){
		Cap_State_Switch(CAP_STATE_PREPARE);
	}
}

static void Cap_Ctr_RELEASE() {
		if(VAL_POWER_Voltage<9)
		{
			Cap_State_Switch(CAP_STATE_STOP);
		}
		else
		{
			//if(VAL_CAP_Voltage<12)
				//Cap_State_Switch(CAP_STATE_EMERGENCY);
			//else if(VAL_CAP_Voltage<24)
				//Cap_State_Switch(CAP_STATE_BOOST);
		}
}
static void Cap_Ctr_EMERGENCY(){
	
}

static void Cap_Ctr_PREPARE(){
	if(VAL_POWER_Voltage<9){
		Cap_State_Switch(CAP_STATE_STOP);
	}else if(VAL_CAP_Voltage>15)
	{
		Cap_State_Switch(CAP_STATE_RELEASE);
	}
}
/**
  * @brief  Control the release and recharge progress.
  * @param  None
  * @retval None
  */
//状态切换判断
static void Cap_Ctr() { // called with period of 2 ms
	#ifdef BOARD_SLAVE
	if (WorkState == STOP_STATE) {
		Cap_State_Switch(CAP_STATE_STOP);
	}
	#else
	if ((RefereeData.GameRobotState.remain_HP < 1 && JUDGE_State==ONLINE) || WorkState == STOP_STATE /*|| WorkState == PREPARE_STATE*/) {
		Cap_State_Switch(CAP_STATE_STOP);
	}
	#endif
	else {
		switch (CapState) {
		case CAP_STATE_STOP:
			Cap_Ctr_STOP();
			break;
		case CAP_STATE_RELEASE:
			Cap_Ctr_RELEASE();
			break;
		case CAP_STATE_EMERGENCY:
			Cap_Ctr_EMERGENCY();
			break;
		case CAP_STATE_PREPARE:
			Cap_Ctr_PREPARE();
			break;
		}
	}
}

/**
  * @brief  Regular conversion complete callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	
}
#ifdef USE_CAP3
static void LED_Show_SuperCap_Voltage(uint8_t flag)
{
	if (flag == 0)
	{
		HAL_GPIO_WritePin(GPIOG, 0x1fe, GPIO_PIN_SET);
		return;
	}
	if (Cap_Get_Cap_Voltage() < 11)
		HAL_GPIO_WritePin(GPIOG, 0x1fe, GPIO_PIN_SET);
	else {
		HAL_GPIO_WritePin(GPIOG, 0x1fe, GPIO_PIN_SET);
		int unlight = 7 - (int) ( ((Cap_Get_Cap_Voltage() * Cap_Get_Cap_Voltage() - 121) / (23*23 - 121)) * 6);
		if (unlight < 0) unlight = 0;
		HAL_GPIO_WritePin(GPIOG, 0x1fe >> unlight, GPIO_PIN_RESET);
	}
}
#endif

int light;
uint8_t Client_Show_SuperCap_Voltage(void)
{
	if (Cap_Get_Cap_Voltage() < 11)
		return 0;
	else
	{
		light = (int)(6.0f * (Cap_Get_Cap_Voltage() * Cap_Get_Cap_Voltage() - 121.0f) / (21.0f * 21.0f - 121.0f));
		return (uint8_t)(0x0fc0 >> light);
	}
}
