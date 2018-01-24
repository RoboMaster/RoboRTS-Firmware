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
/** @file calibrate.c
 *  @version 1.0
 *  @date June 2017
 *
 *  @brief  provides gimbal_offset/imu_data calibrate, 
 *          and save these calibration data in flash
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "calibrate.h"
#include "bsp_can.h"
#include "bsp_flash.h"
#include "string.h"
#include "cmsis_os.h"

cali_sys_t cali_param;

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
  cali_param.imu_cali_list[CALI_GYRO].name = "gyro";
  cali_param.imu_cali_list[CALI_ACC].name  = "acc";
  cali_param.imu_cali_list[CALI_MAG].name  = "mag";
  
  cali_data_read();
}

/**
  * @brief     save gimbal pitch and yaw center point offset
  *            when parameter cali_cmd = 1
  *            the offset is absolute encoder value
  * @usage     called in gimbal task loop 
  */
void gimbal_cali_hook(int32_t pit_ecd, int32_t yaw_ecd)
{
  if (cali_param.gim_cali_data[CALI_GIMBAL_CENTER].cali_cmd == 1)
  {
    cali_param.gim_cali_data[CALI_GIMBAL_CENTER].pitch_offset = pit_ecd;
    cali_param.gim_cali_data[CALI_GIMBAL_CENTER].yaw_offset   = yaw_ecd;
    cali_param.gim_cali_data[CALI_GIMBAL_CENTER].calied_done  = CALIED_FLAG;
    cali_param.gim_cali_data[CALI_GIMBAL_CENTER].cali_cmd     = 0;
    save_cali_data();
  }
  if (cali_param.gim_cali_data[CALI_CAMERA_CENTER].cali_cmd == 1)
  {
    cali_param.gim_cali_data[CALI_CAMERA_CENTER].pitch_offset = pit_ecd;
    cali_param.gim_cali_data[CALI_CAMERA_CENTER].yaw_offset   = yaw_ecd;
    cali_param.gim_cali_data[CALI_CAMERA_CENTER].calied_done  = CALIED_FLAG;
    cali_param.gim_cali_data[CALI_CAMERA_CENTER].cali_cmd     = 0;
    save_cali_data();
  }
}

/**
  * @brief     save imu static offset 
  * @param[in] cali_id  : acceleration/palstance/compass
  * @param[in] raw_xyz[]: raw x axis data address
  * @retval    none
  * @attention called in mpu_get_data() function
  */
void imu_cali_hook(cali_imu_e cali_id, int16_t raw_xyz[])
{
  static int sum[3];
  static int cnt = 0; //global / static var init = 0
  if (cali_param.imu_cali_list[cali_id].cali_cmd == 1)
  {
    cali_param.imu_cali_list[cali_id].calied_done = 0;
    sum[0] += raw_xyz[0];
    sum[1] += raw_xyz[1];
    sum[2] += raw_xyz[2];
    if (++cnt >= 100)
    {
      cnt = 0;
      cali_param.imu_cali_list[cali_id].offset[0] = sum[0] / 100.0f;
      cali_param.imu_cali_list[cali_id].offset[1] = sum[1] / 100.0f;
      cali_param.imu_cali_list[cali_id].offset[2] = sum[2] / 100.0f;
      sum[0] = 0;
      sum[1] = 0;
      sum[2] = 0;
      cali_param.imu_cali_list[cali_id].cali_cmd = 0;
      cali_param.imu_cali_list[cali_id].calied_done = CALIED_FLAG;
      save_cali_data();
    }
  }
}


