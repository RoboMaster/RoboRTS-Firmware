/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
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

#ifdef CHASSIS_TASK_H_GLOBAL
  #define CHASSIS_TASK_H_EXTERN 
#else
  #define CHASSIS_TASK_H_EXTERN extern
#endif

#include "chassis.h"

void chassis_task(void const * argument);
int32_t chassis_set_relative_angle(float angle);

#endif // __CHASSIS_TASK_H__
