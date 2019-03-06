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

#include "can.h"
#include "motor.h"
#include "dbus.h"
#include "detect.h"
#include "board.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "single_gyro.h"
#include "init.h"
#include "communicate.h"
#include "timer_task.h"
#include "gimbal_task.h"
#include "offline_check.h"

int32_t can1_motor_msg_rec(CAN_RxHeaderTypeDef *header, uint8_t *data)
{
  motor_device_data_update(DEVICE_CAN1, header->StdId, data);
  return 0;
}

int32_t can2_single_gyro_rec(CAN_RxHeaderTypeDef *header, uint8_t *data)
{
  struct single_gyro gyro = {0x401, 0, 0};
  int32_t err;

  chassis_t pchassis;
  pchassis = chassis_find("chassis");

  gimbal_t pgimbal;
  pgimbal = gimbal_find("gimbal");

  err = single_gyro_update(&(gyro), header->StdId, data);

  if(err != RM_OK)
  goto end;
  
  if ((pchassis != NULL) && (err == RM_OK))
  {
    chassis_gyro_updata(pchassis, gyro.yaw_gyro_angle, gyro.yaw_gyro_rate);
  }

  if ((pgimbal != NULL) && (err == RM_OK))
  {
    gimbal_yaw_gyro_update(pgimbal, gyro.yaw_gyro_angle + pgimbal->ecd_angle.yaw);
  }

  return RM_OK;
end:
  return err;
}

int32_t motor_canstd_send(enum device_can can, struct can_msg msg)
{
  if (can == DEVICE_CAN1)
    can_msg_bytes_send(&hcan1, msg.data, 8, msg.id);
  else if (can == DEVICE_CAN2)
    can_msg_bytes_send(&hcan2, msg.data, 8, msg.id);
  return 0;
}

int32_t gyro_can_std_send(uint32_t std_id, uint8_t *can_rx_data)
{
  can_msg_bytes_send(&hcan1, can_rx_data, 8, std_id);
  return RM_OK;
}

int32_t dr16_rx_data_by_uart(uint8_t *buff, uint16_t len)
{
  struct detect_device *rc_offline;
  rc_offline = get_offline_dev();
  detect_device_update(rc_offline, RC_OFFLINE_EVENT); 

  rc_device_t rc_dev;
  rc_dev = (rc_device_t)device_find("uart_rc");
  rc_device_date_update(rc_dev, buff);
  return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_10)
  {
    shoot_t pshoot;
    pshoot = shoot_find("shoot");
    if (pshoot != NULL)
    {
      shoot_state_update(pshoot);
    }
  }
  if (GPIO_Pin == GPIO_PIN_2)
  {
    uint8_t app;
    app = get_sys_cfg();

    if (app == CHASSIS_APP)
    {
      gimbal_adjust();
    }
    else
    {
      gimbal_auto_adjust_start();
    }
  }
}

uint32_t get_time_us(void)
{
  return TIM5->CNT;
}

uint32_t get_time_ms(void)
{
  return HAL_GetTick();
}

float get_time_ms_us(void)
{
  return get_time_ms() + get_time_us() / 1000.0f;
}

int32_t motor_can1_output_1ms(void *argc)
{
  motor_device_can_output(DEVICE_CAN1);
  return 0;
}

void board_config(void)
{
  soft_timer_init();

  usart6_manage_init();
	usart3_manage_init();
  can_manage_init();
  pwm_device_init();
  mpu_device_init();

  dr16_uart_init();
  dr16_rx_uart_callback_register(dr16_rx_data_by_uart);

  soft_timer_register(motor_can1_output_1ms, NULL, 1);
  soft_timer_register(beep_ctrl_times, NULL, 1);  
  soft_timer_register(led_toggle_300ms, NULL, 1); 

  motor_device_can_send_register(motor_canstd_send);
  single_gyro_can_send_register(gyro_can_std_send);

  can_fifo0_rx_callback_register(&can1_manage, can1_motor_msg_rec);
  can_fifo0_rx_callback_register(&can2_manage, can2_single_gyro_rec);
}
