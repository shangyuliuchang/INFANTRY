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
#ifdef USE_CAP1
  #define CAP_AUTO_RECHARGE
  //#define CAP_DEBUG
  #define CAP_LED_SHOW
#endif /* USE_CAP1 */

#ifdef USE_CAP2
  //#define CAP_USE_CURR
  //#define CAP_AUTO_RECHARGE
  //#define CAP_DEBUG
  #define CAP_LED_SHOW
#endif /* USE_CAP2 */

#ifdef USE_CAPex
  //#define CAP_DEBUG
  #define CAP_LED_SHOW
  #define CAP_FINAL_TEST
#endif /* USE_CAPex */

#ifdef USE_CAP3
  //#define CAP_DEBUG
  #define CAP_LED_SHOW
#endif /* USE_CAP3 */
//Program Begin!

#define ADC_CHANNALS            (4)
#define ADC_HITS                (50)

#define VREFINT                 (0)
#define ICOUT                   (1)       //PB0 Channel 8
#define ICIN                    (2)       //PB1 Channel 9
#define UCK1                    (3)       //PC0 Channel 10



#ifdef USE_CAP1
  #define RECHARGE_VOLTAGE_MAX    (21.5)
  #define RE_RECHARGE_VOLTAGE     (20.0)
  #define RELEASE_VOLTAGE_MIN     (8.0)//6
  #define VAL__CAP_VOLTAGE        (FUNC__Get_Voltage(UCK1)  * 9.2 * 13.33 / 11.01)
  #define VAL__CAP_Power_CURR     (FUNC__Get_Voltage(ICIN)  / 0.152)
  #define VAL__CAP_Power_Voltage    (FUNC__Get_Voltage(ICOUT)  * 9.2 * 13.33 / 11.01) //Power Voltage
  #define FUNC__Get_Voltage(dev)              (1.2 * ADC_val[dev] / ADC_val[VREFINT])
  #endif /* USE_CAP1 */


#ifdef USE_CAP2
  #define PWM_CMP_MAX             (42000-1)
  #define RECHARGE_POWER_MAX      (65)
  #define RELEASE_POWER_MAX       (70)//70
  #define RECHARGE_CURR_MAX       (3.0)
  #define RELEASE_CURR_MAX        (3.0)
  #define RELEASE_POW_RATE        (1.5)//1.3
  #define RECHARGE_POW_RATE       ((RefereeData.PowerHeat.chassis_power > RECHARGE_POWER_MAX)?\
	                                ((RECHARGE_VOLTAGE_MAX - VAL__CAP_VOLTAGE) / (RECHARGE_VOLTAGE_MAX - RELEASE_VOLTAGE_MIN) * (1.4-0.8) + 0.8):\
																	((RECHARGE_VOLTAGE_MAX - VAL__CAP_VOLTAGE) / (RECHARGE_VOLTAGE_MAX - RELEASE_VOLTAGE_MIN) * (1.4-1.0) + 1.0))
  #define INPUT_PWM_PERCENT_MAX   (100)
  #define OUTPUT_PWM_PERCENT_MAX  (100)
  #define RECHARGE_VOLTAGE_MAX    (21.5)
  #define RELEASE_VOLTAGE_MIN     (8.0)//6
  #define RE_RECHARGE_VOLTAGE     (20.0)
  #define OUT_VOL_PWM_TIM         htim8
  #define OUT_VOL_PWM_CHANNEL     TIM_CHANNEL_2
  #define IN_VOL_PWM_TIM          htim8
  #define IN_VOL_PWM_CHANNEL      TIM_CHANNEL_3

  #define VAL__INPUT_PWM_MAX      (INPUT_PWM_PERCENT_MAX * PWM_CMP_MAX / 100)
  #define VAL__OUTPUT_PWM_MAX     (OUTPUT_PWM_PERCENT_MAX * PWM_CMP_MAX / 100)
  #define VAL__OUTPUT_PWM_PERCENT (output_pwm_cmp * 100.0 / PWM_CMP_MAX)
  #define VAL__INPUT_PWM_PERCENT  (input_pwm_cmp * 100.0 / PWM_CMP_MAX)
  #define VAL__CAP_VOLTAGE        (FUNC__Get_Voltage(UCK1)  * 9.2 * 13.33 / 11.01)
  #define VAL__CAP_Power_CURR     (FUNC__Get_Voltage(ICIN)  / 0.152)
  #define VAL__CAP_Power_Voltage    (FUNC__Get_Voltage(ICOUT)  * 9.2 * 13.33 / 11.01) //Power Voltage

  #define FUNC__Get_Voltage(dev)              (1.2 * ADC_val[dev] / ADC_val[VREFINT])

  #define FUNC__ADD_INPUT_PWM_PERCENT(rate)   (input_pwm_cmp += (rate)*PWM_CMP_MAX/100)
  #define FUNC__ADD_OUTPUT_PWM_PERCENT(rate)  (output_pwm_cmp += (rate)*PWM_CMP_MAX/100)
  #define FUNC__Cap_Set_Output_Percent(rate)  (output_pwm_cmp = PWM_CMP_MAX*rate/100)
  #define FUNC__Cap_Set_Input_Percent(rate)   (input_pwm_cmp  = PWM_CMP_MAX*rate/100)

  #ifdef CAP_USE_CURR
    #define FUNC__RECAL_INPUT_PWM(rate)       FUNC__ADD_INPUT_PWM_PERCENT(rate*CAL_RECHARGE(RECHARGE_POWER_MAX, \
                                              Cap_Get_Power_Voltage()*Cap_Get_Power_CURR() + (60 - RefereeData.PowerHeat.chassis_power_buffer) / 2));
    #define FUNC__RECAL_OUTPUT_PWM(rate)      FUNC__ADD_OUTPUT_PWM_PERCENT(-rate*CAL_RELEASE(RELEASE_POWER_MAX, \
                                              Cap_Get_Power_Voltage()*Cap_Get_Power_CURR() + (60 - RefereeData.PowerHeat.chassis_power_buffer) / 2));
  #else
    #define FUNC__RECAL_INPUT_PWM(rate)       FUNC__ADD_INPUT_PWM_PERCENT(rate*CAL_RECHARGE(RECHARGE_POWER_MAX, \
                                              RefereeData.PowerHeat.chassis_power + (60 - RefereeData.PowerHeat.chassis_power_buffer) / 2));
    #define FUNC__RECAL_OUTPUT_PWM(rate)      FUNC__ADD_OUTPUT_PWM_PERCENT(-rate*CAL_RELEASE(RELEASE_POWER_MAX, \
                                              RefereeData.PowerHeat.chassis_power + (60 - RefereeData.PowerHeat.chassis_power_buffer) / 2));
  #endif /* CAP_USE_CURR */

  #define CAL_RELEASE(max, x) (((max)>(x))?(pow((max)-(x), RELEASE_POW_RATE)):(-pow((x)-(max), RELEASE_POW_RATE)))
  #define CAL_RECHARGE(max, x) (((max)>(x))?(pow((max)-(x), RECHARGE_POW_RATE)):(-pow((x)-(max), RECHARGE_POW_RATE)))

#endif /* USE_CAP2 */



#ifdef USE_CAPex
  #define DAC_PER_MAX             (4095)
  #define RECHARGE_VOLTAGE_MAX    (22.5)
  #define RE_RECHARGE_VOLTAGE     (17.0)
  #define RELEASE_VOLTAGE_MIN     (11.0)
  #define VAL__INPUT_DAC_MAX      (4095)
  #define RE_RELASE_VOLTAGE_MIN   (15.5)
  double VAL__CAP_VOLTAGE;
  #define VAL__CAP_Power_CURR     (FUNC__Get_Voltage(ICIN)  / 0.14)
  #define VAL__CAP_Power_Voltage    (FUNC__Get_Voltage(ICOUT)  * 35.2 / 2.2) //Power Voltage
  double FUNC__Get_Voltage(uint8_t dev);
  #ifdef BOARD_SLAVE
    #define FUNC__RECAL_INPUT_DAC        ((70 - VAL__CAP_Power_CURR*VAL__CAP_Power_Voltage)/VAL__CAP_VOLTAGE)*4095*0.02*10/3.3
    #define FUNC__RECAL_INPUT_DAC_T        ((70 - VAL__CAP_Power_CURR*VAL__CAP_Power_Voltage)/VAL__CAP_VOLTAGE)*4095*0.02*10/3.3
	#else
	  #define FUNC__RECAL_INPUT_DAC        ((59 - VAL__CAP_Power_CURR*VAL__CAP_Power_Voltage)/VAL__CAP_VOLTAGE)*4095*0.02*10/3.3
    #define FUNC__RECAL_INPUT_DAC_T        ((59 - VAL__CAP_Power_CURR*VAL__CAP_Power_Voltage)/VAL__CAP_VOLTAGE)*4095*0.02*10/3.3
	#endif 
	
    /* 备份
	#define FUNC__RECAL_OUTPUT_DAC       4095*0.14*78/VAL__CAP_Power_Voltage/3.3
	*/																		
  #define FUNC__RECAL_OUTPUT_DAC       4095	
																							
  double FUNC_NEW_Get_Voltage(void);
#endif /* USE_CAPex */
	
#ifdef USE_CAP3
	#define AIM_POWER            (JUDGE_State == OFFLINE?(80.0f):(80.0f))
	#define Cap_MOS_1_GPIO_PORT  GPIOE
	#define Cap_MOS_2_GPIO_PORT  GPIOC
	
	#define VAL_POWER_Voltage    (((float)ADC_val[1]*3.3f*11.0f)/4095.0f)//PB0
	#define VAL_CAP_Voltage		   ((float)ADC_val[2]*3.3f*11.0f/4095.0f)							//PB1
	#define VAL_POWER_CUR        ((VAL_CAP_Voltage>0.0f?AIM_POWER*0.95f/VAL_CAP_Voltage:10.0f)<=10.0f?(VAL_CAP_Voltage>0.0f?AIM_POWER*0.95f/VAL_CAP_Voltage:10.0f):10.0f)
	#define DAC_OUT		           (uint32_t)(VAL_POWER_CUR*4095.0f/3.3f/5.0f)
	
	int disableInput=0;
	
static uint16_t mos[4]={GPIO_PIN_12,GPIO_PIN_6,GPIO_PIN_2,GPIO_PIN_3};
#endif /* USE_CAP3 */

static int16_t ADC_hits_val[ADC_HITS][ADC_CHANNALS];
static int32_t ADC_tmp[ADC_CHANNALS];
int16_t ADC_val[ADC_CHANNALS];
uint8_t rlease_flag = 0;

cap_state CapState = CAP_STATE_STOP;
CapControl_t Control_SuperCap = { 0,0 };
float Iset=0;

static void Cap_State(void);
static void Cap_Ctr(void);

#ifdef USE_CAP2
  static int32_t input_pwm_cmp = 0;
  static int32_t output_pwm_cmp = 0;
  static int16_t recharge_cnt = 0;
  static int16_t release_cnt = 0;
  static int32_t output_pwm_cmp_max = 0;
#endif /* USE_CAP2 */

#ifdef USE_CAPex
  static int32_t input_dac_per = 0;
  static int32_t output_dac_per = 0;
	uint8_t cap_move_state = 0;
  double rx_cap_voltage = 0;
  double rx_power_voltage = 0;
  double rx_power_current = 0;
#endif /* USE_CAPex */


#ifdef CAP_DEBUG
  #define DEBUG_HITS    (4000)
  #define DEBUG_AMOUNT  (4)
  static uint8_t cnt=0;
  int32_t cps[DEBUG_AMOUNT][DEBUG_HITS];
  static uint32_t ncnt=0;
  #if DEBUG_HITS * DEBUG_AMOUNT * 4 > 65535
    #error Cap debug info bytes must below 65535, try to change the DEBUG_AMOUNT and DEBUG_HITS
  #endif
#endif /* CAP_DEBUG */

void Cap_Init(void) {
	#ifdef USE_CAP3
	HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[0],GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[1],GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[2],GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[3],GPIO_PIN_RESET);
	
	HAL_DAC_SetValue(&hdac,DAC1_CHANNEL_1,DAC_ALIGN_12B_R,0);
	HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
	#else
	HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
	#endif /* USE_CAP3 */
	memset(ADC_hits_val, 0, sizeof(ADC_hits_val));
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_hits_val, ADC_CHANNALS*ADC_HITS);
	#ifdef USE_CAP2
	  HAL_TIM_PWM_Start(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL);
	  __HAL_TIM_SET_COMPARE(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL, 0);
	  HAL_TIM_PWM_Start(&IN_VOL_PWM_TIM, IN_VOL_PWM_CHANNEL);
	  __HAL_TIM_SET_COMPARE(&IN_VOL_PWM_TIM, IN_VOL_PWM_CHANNEL, 0);
	#endif /* USE_CAP2 */
	
	#ifdef USE_CAPex
	  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	#endif /* USE_CAPex */
}

#ifdef USE_CAPex
  double FUNC__Get_Voltage(uint8_t dev)
  {
	  static double adc_val_output = 0;
	  if (ADC_val[VREFINT] != 0){ 
	  adc_val_output =  (1.2 * ADC_val[dev] / ADC_val[VREFINT]);}
	  return adc_val_output;
  }

  double FUNC_NEW_Get_Voltage(void)
  {
	  static double last_cap_voltage = 0;
	static double new_cap_voltage = 0;
	if (ADC_val[0]  != 0){
	new_cap_voltage = 1.2*0.001 * ADC_val[3] / ADC_val[0] + last_cap_voltage*0.999;
	}
	last_cap_voltage = new_cap_voltage;
	  
	return new_cap_voltage;
  }	
#endif /* USE_CAPex */

void Cap_Run(void) {
	#ifdef USE_CAPex
	VAL__CAP_VOLTAGE = FUNC_NEW_Get_Voltage() * 35.2 / 2.2;
	Control_SuperCap.C_voltage = 100*Cap_Get_Cap_Voltage();
	Control_SuperCap.P_voltage = 100*Cap_Get_Power_Voltage();
	Control_SuperCap.P_Power = 100*Cap_Get_Power_Voltage()*Cap_Get_Power_CURR(); //
	#endif /* USE_CAPex */
	#ifdef CAP_LED_SHOW
	  LED_Show_SuperCap_Voltage(1);
	#endif /* CAP_LED_SHOW */
	Iset=DAC_OUT;
	Cap_Ctr();
	Cap_State();
	//custom_data.masks = 0xC0 | ((1 << ((int)(((VAL__CAP_VOLTAGE*VAL__CAP_VOLTAGE - 121) / (RECHARGE_VOLTAGE_MAX*RECHARGE_VOLTAGE_MAX - 121)) * 6 + 1))) - 1);
}
float Cap_Get_Aim_Power(void){
	return AIM_POWER;
}
void Cap_State_Switch(cap_state State) {
	switch (State) {
	case CAP_STATE_STOP:
		#ifdef USE_CAP1
	        CapState = CAP_STATE_STOP;
          HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
    #endif /* USE_CAP1 */
	
	  #ifdef USE_CAP2
          CapState = CAP_STATE_STOP, output_pwm_cmp = 0, input_pwm_cmp = 0;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
    #endif /* USE_CAP2 */
	
	  #ifdef USE_CAPex
          CapState = CAP_STATE_STOP, input_dac_per = 0;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
    #endif /* USE_CAPex */
		
		#ifdef USE_CAP3
					CapState = CAP_STATE_STOP;
					HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[0],GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[1],GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[2],GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[3],GPIO_PIN_RESET);
		#endif /* USE_CAP3 */
		break;
	case CAP_STATE_RELEASE:
		#ifdef USE_CAP1
          CapState = CAP_STATE_RELEASE;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
	      	HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_SET);
    #endif /* USE_CAP1 */
	
	  #ifdef USE_CAP2
          CapState = CAP_STATE_RELEASE, input_pwm_cmp = 0,release_cnt = 0;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_SET);
    #endif /* USE_CAP2 */
	
	  #ifdef USE_CAPex
	        rlease_flag = 1;
          CapState = CAP_STATE_RELEASE, input_dac_per = 0;
	        #ifdef CAP_FINAL_TEST
	            HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
	        #else
	            HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
	        #endif /* CAP_FINAL_TEST */
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_SET);
    #endif /* USE_CAPex */
		
		#ifdef USE_CAP3
					CapState = CAP_STATE_RELEASE;
					HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[0],GPIO_PIN_SET);
					HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[1],GPIO_PIN_SET);
					HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[2],GPIO_PIN_RESET);
					//HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[3],GPIO_PIN_RESET);
		#endif /* USE_CAP3 */
		break;
	case CAP_STATE_RECHARGE:
		#ifdef USE_CAP1
					CapState = CAP_STATE_RECHARGE;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
    #endif /* USE_CAP1 */
	
	  #ifdef USE_CAP2
          CapState = CAP_STATE_RECHARGE, output_pwm_cmp = 0, recharge_cnt = 0;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
    #endif /* USE_CAP2 */
	
	  #ifdef USE_CAPex
          CapState = CAP_STATE_RECHARGE;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
    #endif /* USE_CAPex */
		
		break;
	
	case CAP_STATE_TEMP_RECHARGE:
	
	  #ifdef USE_CAPex
          CapState = CAP_STATE_TEMP_RECHARGE;
		      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
		      HAL_GPIO_WritePin(Cap_Out_GPIO_Port, Cap_Out_Pin, GPIO_PIN_RESET);
    #endif /* USE_CAPex */
		
		break;
	#ifdef USE_CAP3
	
	case CAP_STATE_BOOST:
			CapState=CAP_STATE_BOOST;
			HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[0],GPIO_PIN_SET);
			HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[1],GPIO_PIN_SET);
			HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[2],GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[3],GPIO_PIN_RESET);
		break;
	
	case CAP_STATE_EMERGENCY:
			CapState=CAP_STATE_EMERGENCY;
			HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[0],GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[1],GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[2],GPIO_PIN_SET);
			//HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[3],GPIO_PIN_RESET);
		break;
	
	case CAP_STATE_PREPARE:
			CapState=CAP_STATE_PREPARE;
			HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[0],GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[1],GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[2],GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[3],GPIO_PIN_RESET);
		break;
	
	#endif /* USE_CAP3 */
	}
}

double Cap_Get_Cap_Voltage(void) {
	#ifdef USE_CAP3
		#ifdef BOARD_SLAVE
			return rx_cap_voltage;
		#else
			return VAL_CAP_Voltage;
		#endif
	#else
	#ifndef BOARD_MAIN
	  return VAL__CAP_VOLTAGE;
	#else
		return rx_cap_voltage;
	#endif 
	#endif
}


double Cap_Get_Power_Voltage(void){
	#ifdef USE_CAP3
		#ifdef BOARD_SLAVE
			return rx_power_voltage;
		#else
			return VAL_POWER_Voltage;
		#endif
	#else
	#ifndef BOARD_MAIN
		return VAL__CAP_Power_Voltage;
	#else
		return rx_power_voltage;
	#endif 	
	#endif
}

double Cap_Get_Power_CURR(void){
	#ifdef USE_CAP3
		#ifdef BOARD_SLAVE
			return rx_power_current;
		#else
			return VAL_POWER_CUR;
		#endif
	#else
	#ifndef BOARD_MAIN
		return VAL__CAP_Power_CURR;
	#else
		return rx_power_current;
	#endif
	#endif
}

//#endif /* USE_CAPex */
cap_state Cap_Get_Cap_State(void) {
	return CapState;
}


/**
  * @brief  Set the pwm compare or dac output according to the state.
  * @param  None
  * @retval None
  */
void Cap_State() { // called with period of 2 ms
	switch (CapState) {
	case CAP_STATE_STOP:
		#ifdef USE_CAP2
		  __HAL_TIM_SET_COMPARE(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL, 0);
		  __HAL_TIM_SET_COMPARE(&IN_VOL_PWM_TIM, IN_VOL_PWM_CHANNEL, 0);
	  #endif /* USE_CAP2 */
	
	  #ifdef USE_CAPex
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);//
	  #endif /* USE_CAPex */ 
		#ifdef USE_CAP3
		HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[0],GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Cap_MOS_1_GPIO_PORT,mos[1],GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Cap_MOS_2_GPIO_PORT,mos[2],GPIO_PIN_RESET);
		#endif /*USE_CAP3*/
		break;
	case CAP_STATE_RECHARGE:
		#ifdef USE_CAP2 
		  __HAL_TIM_SET_COMPARE(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL, 0);
		  __HAL_TIM_SET_COMPARE(&IN_VOL_PWM_TIM, IN_VOL_PWM_CHANNEL, input_pwm_cmp); //
	  #endif /* USE_CAP2 */
	 
	  #ifdef USE_CAPex
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, input_dac_per );//input_dac_per 4095*3*0.02*10/3.3
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
	  #endif /* USE_CAPex */
	
		break;
	case CAP_STATE_RELEASE:
		#ifdef USE_CAP2
		  __HAL_TIM_SET_COMPARE(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL, output_pwm_cmp);
		  //__HAL_TIM_SET_COMPARE(&OUT_VOL_PWM_TIM, OUT_VOL_PWM_CHANNEL, output_pwm_cmp_max);
		  __HAL_TIM_SET_COMPARE(&IN_VOL_PWM_TIM, IN_VOL_PWM_CHANNEL, 0);
	  #endif /* USE_CAP2 */
	  
	  #ifdef USE_CAPex
	    #ifdef CAP_FINAL_TEST
		      HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, input_dac_per);
	    #else
	        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	    #endif /* CAP_FINAL_TEST */
	    if (Cap_Get_Cap_Voltage() > 11){
		    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, output_dac_per); //
			}
			else{
				HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095); 
			}
				
	  #endif /* USE_CAPex */
		#ifdef USE_CAP3
			if(disableInput==0){
				HAL_DAC_SetValue(&hdac,	DAC_CHANNEL_1,	DAC_ALIGN_12B_R,	DAC_OUT);
			}else{
				HAL_DAC_SetValue(&hdac,	DAC_CHANNEL_1,	DAC_ALIGN_12B_R,	0);
			}
		#endif /*USE_CAP3*/
		break;
			
	case CAP_STATE_TEMP_RECHARGE:
	 
	  #ifdef USE_CAPex
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, input_dac_per );//input_dac_per 4095*3*0.02*10/3.3
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
	  #endif /* USE_CAPex */
	
		break;
	#ifdef USE_CAP3
	
	case CAP_STATE_BOOST:
		HAL_DAC_SetValue(&hdac,	DAC1_CHANNEL_1,	DAC_ALIGN_12B_R,	DAC_OUT);
		break;
	
	case CAP_STATE_EMERGENCY:
		HAL_DAC_SetValue(&hdac,	DAC1_CHANNEL_1,	DAC_ALIGN_12B_R,	0);
		break;
	
	case CAP_STATE_PREPARE:
		HAL_DAC_SetValue(&hdac,	DAC1_CHANNEL_1,	DAC_ALIGN_12B_R,	DAC_OUT);
		break;
	
	#endif /* USE_CAP3 */
	}
}

/**
  * @brief  The detailed capacitance control logic.
  * @param  None
  * @retval None
  */
static void Cap_Ctr_STOP() {
  #ifdef CAP_AUTO_RECHARGE
	  if (VAL__CAP_VOLTAGE < RE_RECHARGE_VOLTAGE) {
		  #ifdef USE_CAP1
		    if (VAL__CAP_VOLTAGE < RECHARGE_VOLTAGE_MAX  && fabs(CMFL.offical_speedPID.fdb) < 1000 && fabs(CMFR.offical_speedPID.fdb) < 1000 && \
				  		  fabs(CMBL.offical_speedPID.fdb) < 1000 && fabs(CMBR.offical_speedPID.fdb) < 1000 && RefereeData.PowerHeat.chassis_power_buffer > 59.0f) {
		        Cap_State_Switch(CAP_STATE_RECHARGE);
	      }
		
		  #endif /* USE_CAP1 */
		
		  #ifdef USE_CAP2
		    Cap_State_Switch(CAP_STATE_RECHARGE);
		  #endif /* USE_CAP2 */
		
	}
  #else
	  (void)0;
	
	  #ifdef CAP_DEBUG
	
		  Control_SuperCap.C_voltage = 100*Cap_Get_Cap_Voltage();
	    Control_SuperCap.P_voltage = 100*Cap_Get_Power_Voltage();
	    Control_SuperCap.P_Power = 100*Cap_Get_Power_Voltage()*Cap_Get_Power_CURR();*/
      if(++cnt >=5 && ncnt < DEBUG_HITS){
        cps[0][ncnt] = RefereeData.PowerHeat.chassis_power * 100;
        cps[1][ncnt] = RefereeData.PowerHeat.chassis_power_buffer  * 100;
        cps[2][ncnt] = Cap_Get_Power_Voltage()*Cap_Get_Power_CURR()*100;
        cps[3][ncnt++] = VAL__INPUT_PWM_PERCENT * 100;
        cnt = 0;
      }else if(ncnt >= DEBUG_HITS){
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
      }
    #endif /* CAP_DEBUG */
		
  #endif /*CAP_AUTO_RECHARGE */
	
	#ifdef USE_CAP3
	HAL_DAC_SetValue(&hdac,	DAC1_CHANNEL_1,	DAC_ALIGN_12B_R,	0);
	if(VAL_POWER_Voltage>10){
		Cap_State_Switch(CAP_STATE_PREPARE);
	}
	#endif /* USE_CAP3 */
	return;
	
}

static void Cap_Ctr_RECHARGE() {
	#ifdef USE_CAP1
	  if (VAL__CAP_VOLTAGE > RECHARGE_VOLTAGE_MAX  || fabs(CMFL.offical_speedPID.fdb) > 1000 || fabs(CMFR.offical_speedPID.fdb) > 1000 || \
	  					  fabs(CMBL.offical_speedPID.fdb) > 1000 || fabs(CMBR.offical_speedPID.fdb) > 1000 || RefereeData.PowerHeat.chassis_power_buffer < 59.0f) {
	  	Cap_State_Switch(CAP_STATE_STOP);
	  }
	  else 
	  {
	  	;
	  }
	
	#endif /* USE_CAP1 */
	
	#ifdef USE_CAP2
	  if (VAL__CAP_VOLTAGE > RECHARGE_VOLTAGE_MAX) {
		  Cap_State_Switch(CAP_STATE_STOP);
	  }
	  else {
        #ifdef CAP_DEBUG
	          Control_SuperCap.C_voltage = 100*Cap_Get_Cap_Voltage();
	          Control_SuperCap.P_voltage = 100*Cap_Get_Power_Voltage();
	          Control_SuperCap.P_Power = 100*Cap_Get_Power_Voltage()*Cap_Get_Power_CURR();
            if(++cnt >=5 && ncnt < DEBUG_HITS){
							cps[0][ncnt] = RefereeData.PowerHeat.chassis_power * 100;
							cps[1][ncnt] = RefereeData.PowerHeat.chassis_power_buffer  * 100;
							cps[2][ncnt] = Cap_Get_Power_Voltage()*Cap_Get_Power_CURR()*100;
							cps[3][ncnt++] = VAL__INPUT_PWM_PERCENT * 100;
							cnt = 0;
            }else if(ncnt >= DEBUG_HITS){
               HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
            }
         #endif /* CAP_DEBUG */
		if (recharge_cnt < 250) {
			recharge_cnt++;
			FUNC__RECAL_INPUT_PWM(0.0004f);//0.0005f
		}
		else {
			FUNC__RECAL_INPUT_PWM(0.001f);//0.002f
		}
		if (input_pwm_cmp > VAL__INPUT_PWM_MAX) {
			input_pwm_cmp = VAL__INPUT_PWM_MAX;
		}
		else if (input_pwm_cmp < 0){
			input_pwm_cmp = 0;
		}
	}
  #endif /* USE_CAP2 */
	
	#ifdef USE_CAPex
		#ifndef BOARD_SLAVE
		if (VAL__CAP_VOLTAGE > RECHARGE_VOLTAGE_MAX  || fabs(CMFL.offical_speedPID.fdb - CMFL.offical_speedPID.ref) > 300 || fabs(CMFR.offical_speedPID.fdb - CMFR.offical_speedPID.ref) > 300 || \
					  fabs(CMBL.offical_speedPID.fdb - CMBL.offical_speedPID.ref) > 300 || fabs(CMBR.offical_speedPID.fdb - CMBR.offical_speedPID.ref) > 300 || RefereeData.PowerHeat.chassis_power_buffer < 59.0f ){
		        HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
		  	}else{
				    HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
	  }
		#else
		if (Cap_Get_Cap_Voltage() > RECHARGE_VOLTAGE_MAX  || cap_move_state){
			      HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
		  	}else{
				    HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
	  }
		#endif
	  
		#ifndef BOARD_MAIN		
	  if (Cap_Get_Cap_Voltage() > RECHARGE_VOLTAGE_MAX) {
		  Cap_State_Switch(CAP_STATE_STOP);
	  }
	  else {
		  input_dac_per = FUNC__RECAL_INPUT_DAC;
		  if (input_dac_per > VAL__INPUT_DAC_MAX) {
			  input_dac_per = VAL__INPUT_DAC_MAX;
		  }
		  else if (input_dac_per < 0){
			  input_dac_per = 0;
		  }
	  }
		#endif
	
	#endif /* USE_CAPex */
}

static void Cap_Ctr_TEMP_RECHARGE() {
	
	#ifdef USE_CAPex
	#ifndef BOARD_SLAVE
	if (VAL__CAP_VOLTAGE > RECHARGE_VOLTAGE_MAX  || fabs(CMFL.offical_speedPID.fdb - CMFL.offical_speedPID.ref) > 300 || fabs(CMFR.offical_speedPID.fdb - CMFR.offical_speedPID.ref) > 300 || \
					fabs(CMBL.offical_speedPID.fdb - CMBL.offical_speedPID.ref) > 300 || fabs(CMBR.offical_speedPID.fdb - CMBR.offical_speedPID.ref) > 300 || RefereeData.PowerHeat.chassis_power_buffer < 59.0f ){
					HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
			}else{
					HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
	}
	#else
	if (Cap_Get_Cap_Voltage() > RECHARGE_VOLTAGE_MAX  || cap_move_state){
					HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
			}else{
					HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
	}
	#endif
				
	if (Cap_Get_Cap_Voltage() > 15){
		Cap_State_Switch(CAP_STATE_RELEASE);
	}else{
			input_dac_per = FUNC__RECAL_INPUT_DAC;
			if (input_dac_per > VAL__INPUT_DAC_MAX) {
				input_dac_per = VAL__INPUT_DAC_MAX;
			}
			else if (input_dac_per < 0){
				input_dac_per = 0;
			}
	}
	#endif /* USE_CAPex */
}

static void Cap_Ctr_RELEASE() {
  #ifdef USE_CAP1
	  if (VAL__CAP_VOLTAGE < RELEASE_VOLTAGE_MIN) {
		  Cap_State_Switch(CAP_STATE_STOP);
	  }
  #endif /* USE_CAP1 */
	
  #ifdef USE_CAP2
	  if (VAL__CAP_VOLTAGE < RELEASE_VOLTAGE_MIN) {
		  Cap_State_Switch(CAP_STATE_STOP);
	  }
	  else {

      #ifdef CAP_DEBUG
         if(++cnt>=5 && ncnt<DEBUG_HITS){
          cps[0][ncnt] = RefereeData.PowerHeat.chassis_power * 100;
          cps[1][ncnt] = RefereeData.PowerHeat.chassis_power_buffer  * 100;
          cps[2][ncnt] = Cap_Get_Power_Voltage()*Cap_Get_Power_CURR()*100;
          cps[3][ncnt++] = VAL__OUTPUT_PWM_PERCENT * 100;
          cnt = 0;
         }else if(ncnt >= DEBUG_HITS){
          HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,  GPIO_PIN_SET);
         }
      #endif /* CAP_DEBUG */
		  if (release_cnt < 50) { 
			  release_cnt++;
			  output_pwm_cmp_max = 0;
			  output_pwm_cmp = 0;
		  }
		  else {
			  output_pwm_cmp_max = PWM_CMP_MAX;
			  FUNC__RECAL_OUTPUT_PWM(0.0004f);//0.002f
		  }
		
		  if (output_pwm_cmp > VAL__OUTPUT_PWM_MAX) {
			  output_pwm_cmp = VAL__OUTPUT_PWM_MAX;
		  }
		  else if (output_pwm_cmp < 0) {
			  output_pwm_cmp = 0;
		  }
	  }
  #endif /* USE_CAP2 */
	
	#ifdef USE_CAPex
		  #ifdef CAP_FINAL_TEST
		    
				if (Cap_Get_Cap_Voltage() < RELEASE_VOLTAGE_MIN) {
		      Cap_State_Switch(CAP_STATE_TEMP_RECHARGE);
	      }
	      else {
					#ifndef BOARD_SLAVE
					if (VAL__CAP_VOLTAGE > RECHARGE_VOLTAGE_MAX  || fabs(CMFL.offical_speedPID.fdb - CMFL.offical_speedPID.ref) > 300 || fabs(CMFR.offical_speedPID.fdb - CMFR.offical_speedPID.ref) > 300 || \
									fabs(CMBL.offical_speedPID.fdb - CMBL.offical_speedPID.ref) > 300 || fabs(CMBR.offical_speedPID.fdb - CMBR.offical_speedPID.ref) > 300 || RefereeData.PowerHeat.chassis_power_buffer < 59.0f ){
									HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
							}else{
									HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
					}
					#else
					if (Cap_Get_Cap_Voltage() > RECHARGE_VOLTAGE_MAX  || cap_move_state){
									HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_RESET);
							}else{
									HAL_GPIO_WritePin(Cap_In_GPIO_Port, Cap_In_Pin, GPIO_PIN_SET);
					}
					#endif
		      input_dac_per = FUNC__RECAL_INPUT_DAC_T;
		      if (input_dac_per > VAL__INPUT_DAC_MAX) {
			      input_dac_per = VAL__INPUT_DAC_MAX;
		      }
		      else if (input_dac_per < 0){
			      input_dac_per = 0;
		      }
          output_dac_per = FUNC__RECAL_OUTPUT_DAC;
			  }
		  #else
				if (VAL__CAP_VOLTAGE < RELEASE_VOLTAGE_MIN) {
		      Cap_State_Switch(CAP_STATE_STOP);
	      }
	      else {
		    output_dac_per = FUNC__RECAL_OUTPUT_DAC;
			  }
		  #endif /* CAP_FINAL_TEST */
	#endif /* USE_CAPex */

	#ifdef USE_CAP3
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
	#endif /* USE_CAP3 */
}

#ifdef USE_CAP3
static void Cap_Ctr_BOOST(){
	if(VAL_POWER_Voltage<10)
		{
			Cap_State_Switch(CAP_STATE_STOP);
		}
		else
		{
			if(VAL_CAP_Voltage<12)
				Cap_State_Switch(CAP_STATE_EMERGENCY);
			else if(VAL_CAP_Voltage>24)
				Cap_State_Switch(CAP_STATE_RELEASE);
		}
}

static void Cap_Ctr_EMERGENCY(){
	if(VAL_POWER_Voltage<9)
		{
			Cap_State_Switch(CAP_STATE_STOP);
		}
}

static void Cap_Ctr_PREPARE(){
	if(VAL_POWER_Voltage<9){
		Cap_State_Switch(CAP_STATE_STOP);
	}else if(VAL_CAP_Voltage>15)
	{
		Cap_State_Switch(CAP_STATE_RELEASE);
	}
}
#endif /* USE_CAP3 */
/**
  * @brief  Control the release and recharge progress.
  * @param  None
  * @retval None
  */
cap_state cap_check;
void Cap_Ctr() { // called with period of 2 ms
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
		case CAP_STATE_RECHARGE:
			Cap_Ctr_RECHARGE();
			break;
		case CAP_STATE_RELEASE:
			Cap_Ctr_RELEASE();
			break;
		case CAP_STATE_TEMP_RECHARGE:
			Cap_Ctr_TEMP_RECHARGE();
			break;
		#ifdef USE_CAP3
		
		case CAP_STATE_EMERGENCY:
			Cap_Ctr_EMERGENCY();
			break;
		case CAP_STATE_BOOST:
			Cap_Ctr_BOOST();
			break;
		case CAP_STATE_PREPARE:
			Cap_Ctr_PREPARE();
			break;
		#endif
		}
	}
	cap_check = Cap_Get_Cap_State();
}

/**
  * @brief  Regular conversion complete callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if(hadc == &hadc1)
	{
		int cnt1, cnt2;
		memset(ADC_tmp, 0, sizeof(ADC_tmp));
		for (cnt1 = 0; cnt1 < ADC_CHANNALS; cnt1++) {
			for (cnt2 = 0; cnt2 < ADC_HITS; cnt2++) {
				ADC_tmp[cnt1] += ADC_hits_val[cnt2][cnt1];
			}
			ADC_val[cnt1] = ADC_tmp[cnt1] / ADC_HITS;
		}
	}
}
#ifdef CAP_DEBUG
  uint8_t sendfinish=1;
  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(sendfinish){
      sendfinish = 0;
      HAL_UART_Transmit_DMA(&huart8, (uint8_t*)cps, sizeof(cps));
    }
  }
#endif /* CAP_DEBUG */

void LED_Show_SuperCap_Voltage(uint8_t flag)
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
