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
/** @file modeswitch_task.h
 *  @version 1.0
 *  @date Oct 2017
 *
 *  @brief infantry control mode switch
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
#ifndef __MODE_SW_TASK_H__
#define __MODE_SW_TASK_H__

#include "stm32f4xx_hal.h"

typedef enum
{
  MANUAL_CTRL_MODE,
  SEMI_AUTO_MODE,
  AUTO_CTRL_MODE,
  SAFETY_MODE,
} infantry_mode_e;

void mode_switch_task(void const *argu);

static void get_main_ctrl_mode(void);
static void get_gimbal_mode(void);
static void get_chassis_mode(void);
static void get_shoot_mode(void);

static void get_global_last_mode(void);

uint8_t gimbal_is_controllable(void);
uint8_t chassis_is_controllable(void);


#endif 
