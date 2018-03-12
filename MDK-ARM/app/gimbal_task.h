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
/** @file gimbal_task.h
 *  @version 1.1
 *  @date Oct 2017
 *
 *  @brief gimbal control task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "stm32f4xx_hal.h"

/* gimbal control period time (ms) */
#define GIMBAL_PERIOD 5

typedef enum
{
  GIMBAL_RELAX         = 0,
  GIMBAL_INIT          = 1,
  GIMBAL_NO_ARTI_INPUT = 2,
  GIMBAL_FOLLOW_ZGYRO  = 3,
  GIMBAL_TRACK_ARMOR   = 4,
  GIMBAL_PATROL_MODE   = 5,
  GIMBAL_SHOOT_BUFF    = 6,
  GIMBAL_POSITION_MODE = 7,
} gimbal_mode_e;

typedef enum
{
  NO_ACTION = 0,
  IS_ACTION,
} action_mode_e;

typedef enum
{
  CMD_CALI_FIVE = 0,
  CMD_CALI_NINE,
  CMD_TARGET_NUM
} gimbal_cmd_e;

typedef struct
{  
  int32_t pit_offset;
  int32_t yaw_offset;
  
  uint8_t target_num;
  uint8_t last_num;
  
  float yaw_calied_5;
  float pit_calied_5;
  float yaw_calied_9;
  float pit_calied_9;
  
} big_buff_t;

typedef struct
{
  /* position loop */
  float yaw_angle_ref;
  float pit_angle_ref;
  float yaw_angle_fdb;
  float pit_angle_fdb;
  /* speed loop */
  float yaw_speed_ref;
  float pit_speed_ref;
  float yaw_speed_fdb;
  float pit_speed_fdb;
} gim_pid_t;

typedef struct
{
  /* unit: degree */
  float pit_relative_angle;
  float yaw_relative_angle;
  float gyro_angle;
  /* uint: degree/s */
  float yaw_palstance;
  float pit_palstance;
} gim_sensor_t;

typedef struct
{
  action_mode_e ac_mode;
  float         action_angle;
  uint8_t       no_action_flag;
  uint32_t      no_action_time;
} no_action_t;

typedef struct
{
  /* ctrl mode */
  gimbal_mode_e ctrl_mode;
  gimbal_mode_e last_ctrl_mode;
  
  /* gimbal information */
  gim_sensor_t  sensor;
  float         ecd_offset_angle;
  float         yaw_offset_angle;
  
  /* gimbal ctrl parameter */
  gim_pid_t     pid;
  no_action_t   input;
  
  /* read from flash */
  int32_t       pit_center_offset;
  int32_t       yaw_center_offset;
  
  //gimbal_cmd_e  auto_ctrl_cmd;
} gimbal_t;

extern gimbal_t gim;

static void no_action_handle(void);
static void init_mode_handle(void);
static void close_loop_handle(void);

static void track_aimor_handle(void);
static void gimbal_patrol_handle(void);
static void big_buff_handle(void);
static void pc_position_ctrl_handle(void);

void gimbal_param_init(void);
void gimbal_back_param(void);

void gimbal_task(void const *argu);
void gimbal_self_check(void);

#endif

