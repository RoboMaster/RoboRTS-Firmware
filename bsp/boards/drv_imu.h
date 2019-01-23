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

#ifndef __DRV_IMU_H__
#define __DRV_IMU_H__

#ifdef DRV_IMU_H_GLOBAL
  #define DRV_IMU_H_EXTERN
#else
  #define DRV_IMU_H_EXTERN extern
#endif

#include "stm32f4xx_hal.h"
#include "ahrs.h"

#define MPU_IO_PROBE() //HAL_GPIO_TogglePin(IO_PROBE_GPIO_Port, IO_PROBE_Pin);

struct mpu_calibrate
{
  uint8_t gyro_flag;
  uint8_t acc_flag;
  uint8_t mag_flag;
};

struct mpu_data_info
{
  int16_t ax;
  int16_t ay;
  int16_t az;

  int16_t gx;
  int16_t gy;
  int16_t gz;

  int16_t mx;
  int16_t my;
  int16_t mz;

  int16_t temp;

  int16_t ax_offset;
  int16_t ay_offset;
  int16_t az_offset;

  int16_t gx_offset;
  int16_t gy_offset;
  int16_t gz_offset;

  int16_t mx_offset;
  int16_t my_offset;
  int16_t mz_offset;
};

uint8_t mpu_device_init(void);
void mpu_get_data(struct ahrs_sensor *sensor);
void mpu_get_temp(float *tmp);

#endif // __DRV_IMU_H__
