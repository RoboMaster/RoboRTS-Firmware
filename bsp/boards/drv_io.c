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

#include "tim.h"
#include "sys.h"
#include "drv_io.h"
#include "pid.h"

void pwm_device_init(void)
{
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1); // ctrl imu temperature
    HAL_TIM_PWM_Start(&htim4,  TIM_CHANNEL_3); // beep
    HAL_TIM_PWM_Start(&htim8,  TIM_CHANNEL_2); // friction wheel
    HAL_TIM_PWM_Start(&htim8,  TIM_CHANNEL_3);

    fric_set_output(1000, 1000);
    mpu_heat_output(0);
}

void fric_set_output(uint16_t  fric_spd1, uint16_t  fric_spd2)
{
    LEFT_FRICTION = fric_spd1;
    RIGHT_FRICTION = fric_spd2;
}

void fric_get_speed(uint16_t  *fric_spd1, uint16_t  *fric_spd2)
{
    *fric_spd1 = LEFT_FRICTION;
    *fric_spd2 = RIGHT_FRICTION;
}

void mpu_heat_output(uint16_t pwm_pulse)
{
    IMU_PWM_PULSE = pwm_pulse;
}

static uint8_t beep_times;

int32_t beep_set_times(uint8_t times)
{
    beep_times = times;
    return 0;
}

void beep_set_tune(uint16_t tune, uint16_t ctrl)
{
    BEEP_TUNE = tune;
    BEEP_CTRL = ctrl;
}

/**
  * @brief  called by cycle, control beep times.(one BEEP_PERIOD)
  * @param  NULL
  * @retval
  */
int32_t beep_ctrl_times(void *argc)
{
    static uint32_t beep_tick;
    static uint32_t times_tick;
    static uint8_t times;

    /* The beep works after the system starts 3s */
    if (get_time_ms() / 1000 < 3)
    {
        return 0;
    }

    if (get_time_ms() - beep_tick > BEEP_PERIOD)
    {
        times = beep_times;
        beep_tick = get_time_ms();
        times_tick = get_time_ms();
    }
    else if (times != 0)
    {
        if (get_time_ms() - times_tick < BEEP_ON_TIME)
        {
            beep_set_tune(BEEP_TUNE_VALUE, BEEP_CTRL_VALUE);
            LED_R_ON();
        }
        else if (get_time_ms() - times_tick < BEEP_ON_TIME + BEEP_OFF_TIME)
        {
            beep_set_tune(0, 0);
            LED_R_OFF();
        }
        else
        {
            times--;
            times_tick = get_time_ms();
        }
    }

    return 0;
}

/**
  * @brief  toggle led when system is normal.
  * @param  toggle period(int), unit:ms
  * @retval
  */
int32_t green_led_toggle(void *argc)
{
    static uint32_t led_tick;

    if (get_time_ms() - led_tick > *(int *)argc)
    {
        LED_G_TOGGLE();
        led_tick = get_time_ms();
    }

    return 0;
}
