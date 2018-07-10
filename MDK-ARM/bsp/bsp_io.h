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
/** @file bsp_io.h
 *  @version 1.0
 *  @date June 2017
 *
 *  @brief basic IO port operation
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __BSP_IO_H__
#define __BSP_IO_H__

#include "stm32f4xx_hal.h"

#define LEFT_FRICTION    TIM1->CCR1
#define RIGHT_FIRCTION   TIM1->CCR4

#define IMU_PWM_PULSE    TIM3->CCR2

#define BEEP_TUNE        TIM12->ARR
#define BEEP_CTRL        TIM12->CCR1
  
  
void turn_on_laser(void);
void turn_off_laser(void);
void turn_on_friction_wheel(uint16_t spd);
void turn_off_friction_wheel(void);
void beep_ctrl(uint16_t tune, uint16_t ctrl);
void pwm_device_init(void);
void mpu_heat_ctrl(uint16_t pwm_pulse);

uint8_t get_trigger_key_state(void);
uint8_t sd_insert(void);

#endif

