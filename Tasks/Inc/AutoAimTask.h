/**
  ******************************************************************************
  * File Name          : AutoAimtask.h
  * Description        : 
  ******************************************************************************
  *
  * Copyright (c) 2019 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */

#ifndef __AUTOAIMTASK_H
#define __AUTOAIMTASK_H

#include "bsp_imu.h"
#include "includes.h"

#ifdef	USE_AUTOAIM

#define RX_ENEMY_START		Enemy_INFO[0]
#define RX_ENEMY_YAW1			Enemy_INFO[1]
#define RX_ENEMY_YAW2 		Enemy_INFO[2]
#define RX_ENEMY_PITCH1 	Enemy_INFO[3]
#define RX_ENEMY_PITCH2		Enemy_INFO[4]
#define RX_ENEMY_DIS1			Enemy_INFO[5]
#define RX_ENEMY_DIS2 		Enemy_INFO[6]
#define RX_ENEMY_END 			Enemy_INFO[7]

#define PREDICT_MODE 1
#define ANTI_GYRO_MODE 2

typedef struct GMAngle_t
{
	float yaw;
	float pitch;
}GMAngle_t;

typedef struct Coordinate_t
{
	float x;
	float y;
	float z;
}Coordinate_t;

extern uint8_t aim_mode;
extern uint8_t find_enemy;
extern uint16_t auto_counter_autoaim;
extern GMAngle_t GM_RealAngle_RCD;
extern uint8_t delay_t;
extern imu_t imu_w_RCD;
extern uint8_t auto_shoot_flag;

void InitAutoAim(void);
void AutoAimUartRxCpltCallback(void);
void AutoAim(void);
void AutoShoot(uint8_t mode);
GMAngle_t GM_RealAngle_Rcd(MotorINFO *GMY, MotorINFO *GMP, int delay_t);
imu_t imu_w_rcd(imu_t *imu_current, int delay_t);

#endif /*USE_AUTOAIM*/

#endif /*__AUTOAIMTASK_H*/
