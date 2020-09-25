/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
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

#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "sys.h"

#define NORMAL_MODE      0
#define ADJUST_MODE      1
#define INIT_MODE        2
#define YAW_DEBUG_MODE   3
#define PITCH_DEBUG_MODE 4

#define GIMBAL_PARAM_KEY "ECD_CENTER"

void gimbal_init_start(void);
uint8_t gimbal_get_work_mode(void);
void gimbal_set_work_mode(uint8_t mode);
void gimbal_task(void const *argument);
struct gimbal *get_gimbal(void);
void gimbal_gyro_yaw_update(uint16_t std_id, uint8_t *data);

#endif // __GIMBAL_TASK_H__
