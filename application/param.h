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

#ifndef __PARAM_H__
#define __PARAM_H__

#ifdef PARAM_H_GLOBAL
  #define PARAM_H_EXTERN 
#else
  #define PARAM_H_EXTERN extern
#endif

#include "sys.h"
#include "drv_flash.h"

#define CALIED_FLAG 0x55

typedef struct
{
  uint16_t yaw_offset;
  uint16_t pitch_offset;
  uint8_t calied_done; //0x55:already calied
} gim_cali_t;

typedef struct
{
  uint32_t   firmware_version;
  gim_cali_t gim_cali_data;
} cali_sys_t;

void cali_param_init(void);
cali_sys_t *get_cali_param(void);
void cali_data_read(void);
void save_cali_data(void);
void gimbal_save_data(uint16_t pit_ecd, uint16_t yaw_ecd);

#endif // __PARAM_H__
