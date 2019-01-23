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
/** @file param.c
 *  @version 1.0
 *  @date Jan 2018
 *
 *  @brief  
 *
 *  @copyright 2019 DJI RoboMaster. All rights reserved.
 *
 */

#include "param.h"

static cali_sys_t cali_param;

cali_sys_t *get_cali_param(void)
{
  return &cali_param;
}

/**
  * @brief save calibrate data cali_param, write this structure in chip flash
  * @usage called when calibrate data be changed in calibrate loop
  */
void save_cali_data(void)
{
  taskENTER_CRITICAL();
  BSP_FLASH_Write((uint8_t*)&cali_param, sizeof(cali_sys_t));
  taskEXIT_CRITICAL();
}
/**
  * @brief read calibrate data cali_param from chip flash
  * @usage called after cali_param_init() in main() initialize part.
  */
void cali_data_read(void)
{
  memcpy((void*)&cali_param, (void*)PARAM_SAVED_START_ADDRESS, sizeof(cali_sys_t));
}

void cali_param_init(void)
{
  cali_data_read();
}

/**
  * @brief save gimbal pitch and yaw center point offset
  *        the offset is absolute encoder value
  * @usage called in gimbal task loop 
  */
void gimbal_save_data(uint16_t yaw_ecd, uint16_t pitch_ecd)
{
  cali_param.gim_cali_data.yaw_offset   = yaw_ecd;
  cali_param.gim_cali_data.pitch_offset = pitch_ecd;
  cali_param.gim_cali_data.calied_done  = CALIED_FLAG;
  save_cali_data();
}
