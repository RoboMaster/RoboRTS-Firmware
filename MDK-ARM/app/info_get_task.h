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
/** @file info_get_task.h
 *  @version 1.0
 *  @date Oct 2017
 *
 *  @brief get infantry sensor and control information
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __INFO_GET_TASK_H__
#define __INFO_GET_TASK_H__

#include "stm32f4xx_hal.h"
#include "infantry_info.h"

/* get information task period time (ms) */
#define INFO_GET_PERIOD 5

extern infantry_structure_t glb_struct;

static void get_global_last_info(void);

void info_get_task(void const *argu);

#endif
