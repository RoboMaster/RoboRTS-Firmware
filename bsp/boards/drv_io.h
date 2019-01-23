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

#ifndef __DRV_IO_H__
#define __DRV_IO_H__

#ifdef DRV_IO_H_GLOBAL
  #define DRV_IO_H_EXTERN 
#else
  #define DRV_IO_H_EXTERN extern
#endif

#include "stm32f4xx_hal.h"
#include "main.h"
  
#define DEFAULT_IMU_TEMP    50

#define LEFT_FRICTION        TIM1->CCR1
#define RIGHT_FRICTION       TIM1->CCR4

#define IMU_PWM_PULSE    TIM3->CCR2

#define BEEP_TUNE        TIM12->ARR
#define BEEP_CTRL        TIM12->CCR1

#define LED_G_ON()      HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET)
#define LED_G_OFF()     HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET)
#define LED_G_TOGGLE()  HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin)

#define LED_R_ON()      HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET)
#define LED_R_OFF()     HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET)
#define LED_R_TOGGLE()  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin)

#define get_trig_status() HAL_GPIO_ReadPin(TRIG_GPIO_Port, TRIG_Pin)

void pwm_device_init(void);
void fric_set_output(uint16_t  fric_spd1, uint16_t  fric_spd2);
void fric_get_speed(uint16_t  *fric_spd1, uint16_t  *fric_spd2);

void mpu_heat_output(uint16_t pwm_pulse);
void beep_set_tune(uint16_t tune, uint16_t ctrl);
int32_t beep_set_times(uint8_t times);
int32_t beep_ctrl_times(void *argc);
int32_t led_toggle_300ms(void *argc);

#endif // __DRV_IO_H__
