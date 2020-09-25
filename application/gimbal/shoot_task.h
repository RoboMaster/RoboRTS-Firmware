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

#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

#ifdef SHOOT_TASK_H_GLOBAL
    #define SHOOT_TASK_H_EXTERN
#else
    #define SHOOT_TASK_H_EXTERN extern
#endif

void shoot_task(void const *argument);
struct shoot *get_shoot(void);

#endif // __SHOOT_TASK_H__
