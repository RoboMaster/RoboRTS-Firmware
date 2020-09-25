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

#ifndef __DRV_IMU_H__
#define __DRV_IMU_H__

#ifdef DRV_IMU_H_GLOBAL
    #define DRV_IMU_H_EXTERN
#else
    #define DRV_IMU_H_EXTERN extern
#endif

#include "stm32f4xx_hal.h"
#include "ahrs.h"

#define BMI088_PARAM_KEY "BMI088_PARAM"

void bmi088_get_data(struct ahrs_sensor *sensor);
void bmi088_get_temp(float *tmp);
uint8_t bmi088_device_init(void);

int ahrs_update(struct ahrs_sensor *atti, uint8_t period_ms);

int32_t imu_temp_keep(void *argc);
void imu_temp_ctrl_init(void);
uint8_t bmi088_set_offset(void);
uint8_t bmi088_get_offset(void);

#endif // __DRV_IMU_H__
