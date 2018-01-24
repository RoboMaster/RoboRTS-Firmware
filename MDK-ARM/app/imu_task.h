/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file imu_task.h
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief imu attitude calculation task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "stm32f4xx_hal.h"
#include "bsp_imu.h"

/* imu task period time (ms) */
#define IMU_TASK_PERIOD 1

typedef struct
{
  int roll_cnt;
  int pitch_cnt;
  int yaw_cnt;
  
  float last_roll;
  float last_pitch;
  float last_yaw;

  float roll;
  float pitch;
  float yaw;
} imu_attitude_t;

extern mpu_data_t     mpu_data;
extern imu_data_t     imu;
extern imu_attitude_t atti;

void imu_task(void const *argu);
void imu_param_init(void);

static void imu_temp_ctrl_init(void);
static void imu_temp_keep(void);

#endif
