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
/** @file bsp_imu.h
 *  @version 1.0
 *  @date Apr 2017
 *
 *  @brief Configuration MPU6500 and Read the Accelerator
 *         and Gyrometer data using SPI interface
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __BSP_IMU_H__
#define __BSP_IMU_H__

#include "stm32f4xx_hal.h"

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t mx;
    int16_t my;
    int16_t mz;

    int16_t temp;

    int16_t gx;
    int16_t gy;
    int16_t gz;
    
    int16_t ax_offset;
    int16_t ay_offset;
    int16_t az_offset;
  
    int16_t gx_offset;
    int16_t gy_offset;
    int16_t gz_offset;
    
} mpu_data_t;

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t mx;
    int16_t my;
    int16_t mz;

    float temp;
    float temp_ref;
  
    float wx;
    float wy;
    float wz;

    float vx;
    float vy;
    float vz;
  
    float rol;
    float pit;
    float yaw;
} imu_data_t;

uint8_t mpu_device_init(void);
void    mpu_get_data(void);
void    mpu_offset_cal(void);

#endif
