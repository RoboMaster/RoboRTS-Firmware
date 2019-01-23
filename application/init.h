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

#ifndef __INIT_H__
#define __INIT_H__

#ifdef INIT_H_GLOBAL
  #define INIT_H_EXTERN 
#else
  #define INIT_H_EXTERN extern
#endif

#define CHASSIS_APP 0
#define GIMBAL_APP  1

uint8_t get_sys_cfg(void);
void hw_init(void);
void task_init(void);

#endif // __INIT_H__
