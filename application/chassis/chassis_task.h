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

#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "chassis.h"

struct chassis *get_chassis(void);
void chassis_task(void const *argument);
int32_t chassis_set_relative_angle(float angle);
int32_t follow_angle_info_rcv(uint8_t *buff, uint16_t len);
void set_follow_relative(float val);

#endif // __CHASSIS_TASK_H__
