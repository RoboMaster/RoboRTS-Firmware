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
/** @file calibrate.h
 *  @version 1.0
 *  @date June 2017
 *
 *  @brief  provides gimbal_offset/imu_data calibrate, 
 *          and save these calibration data in flash
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
#ifndef __CALIBRATE_H__
#define __CALIBRATE_H__

#include "stm32f4xx_hal.h"

#define CALIED_FLAG 0x55

typedef enum 
{
  CALI_GYRO    = 0,
  CALI_ACC     = 1,
  CALI_MAG     = 2,
  CALI_IMU_NUM = 3,
} cali_imu_e;

typedef enum
{
  GIMBAL_CALI_START = 1,
  GIMBAL_CALI_END   = 2,
  CAMERA_CALI_START = 3,
  CAMERA_CALI_END   = 4,
} gimbal_cali_type_t;

typedef enum
{
  CALI_GIMBAL_CENTER = 0,
  CALI_CAMERA_CENTER = 1,
  CALI_GIMBAL_NUM    = 2,
} cali_gimbal_e;

typedef __packed struct
{
  int32_t yaw_offset;
  int32_t pitch_offset;
  uint8_t cali_cmd;    //1:calibrae  0:no operate
  uint8_t calied_done; //0x55:already calied
} gim_cali_t;

typedef __packed struct
{
  int16_t offset[3];   //x,y,z
  uint8_t cali_cmd;    //1:calibrae  0:not
  uint8_t calied_done; 
  char*   name;
} imu_cali_t;

typedef __packed struct
{
  uint32_t   firmware_version;
  gim_cali_t gim_cali_data[CALI_GIMBAL_NUM];
  imu_cali_t imu_cali_list[CALI_IMU_NUM];
} cali_sys_t;

extern cali_sys_t cali_param;

void gimbal_cali_hook(int32_t pit_ecd, int32_t yaw_ecd);
void imu_cali_hook(cali_imu_e cali_id, int16_t raw_xyz[]);
void cali_param_init(void);
void cali_data_read(void);

#endif

