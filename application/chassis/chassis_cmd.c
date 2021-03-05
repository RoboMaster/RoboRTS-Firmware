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

#include "protocol.h"
#include "chassis_cmd.h"
#include "chassis_task.h"
#include "infantry_cmd.h"
#include "referee_system.h"
#include "offline_service.h"

uint8_t chassis_sdk_state = CHASSIS_SDK_OFF;
uint8_t chassis_heart_state = CHASSIS_HEART_OFF;

void set_chassis_sdk_mode(uint8_t state)
{
    chassis_sdk_state = state;
}

void set_chassis_heart_mode(uint8_t state)
{
    chassis_heart_state = state;
}

uint8_t get_chassis_sdk_mode(void)
{
    return (chassis_sdk_state && chassis_heart_state);
}

int32_t chassis_manifold_heart(uint8_t *buff, uint16_t len)
{
    offline_event_time_update(OFFLINE_MANIFOLD2_HEART);
    return 0;
}

/**
  * @brief  speed control
  * @param
  * @retval int32_t
  */
int32_t chassis_speed_ctrl(uint8_t *buff, uint16_t len)
{
    if (get_chassis_sdk_mode() != CHASSIS_SDK_ON)
    {
        return -1;
    }

    if (len == sizeof(struct cmd_chassis_speed))
    {
        chassis_t p_chassis = get_chassis();
        struct cmd_chassis_speed *p_speed = (struct cmd_chassis_speed *)buff;
        chassis_set_offset(p_chassis, p_speed->rotate_x_offset, p_speed->rotate_x_offset);
        chassis_set_acc(p_chassis, 0, 0, 0);
        chassis_set_speed(p_chassis, p_speed->vx, p_speed->vy, p_speed->vw / 10.0f);

        offline_event_time_update(OFFLINE_CONTROL_CMD);
    }
    return 0;
}

/**
  * @brief  speed control with acceleration
  * @param
  * @retval int32_t
  */
int32_t chassis_spd_acc_ctrl(uint8_t *buff, uint16_t len)
{
    if (get_chassis_sdk_mode() != CHASSIS_SDK_ON)
    {
        return -1;
    }

    if (len == sizeof(struct cmd_chassis_spd_acc))
    {
        chassis_t p_chassis = get_chassis();
        struct cmd_chassis_spd_acc *p_acc = (struct cmd_chassis_spd_acc *)buff;
        chassis_set_offset(p_chassis, p_acc->rotate_x_offset, p_acc->rotate_x_offset);
        chassis_set_acc(p_chassis, p_acc->ax, p_acc->ay, p_acc->wz / 10.0f);
        chassis_set_speed(p_chassis, p_acc->vx, p_acc->vy, p_acc->vw / 10.0f);

        offline_event_time_update(OFFLINE_CONTROL_CMD);
    }
    return 0;
}

/**
  * @brief  send student user data to referee system
  * @param
  * @retval int32_t
  */
int32_t student_data_transmit(uint8_t *buff, uint16_t len)
{
    uint16_t cmd_id = *(uint16_t *)buff;
    referee_protocol_tansmit(cmd_id, buff + 2, len - 2);
    return 0;
}

/**
  * @brief  send chassis infomation to pc
  * @param
  * @retval int32_t
  */
int32_t chassis_info_push(void *argc)
{
    struct chassis_info info;
    struct cmd_chassis_info cmd_chassis_info;
    chassis_t p_chassis = (chassis_t)argc;
    chassis_get_info(p_chassis, &info);

    cmd_chassis_info.angle_deg = info.angle_deg * 10;
    cmd_chassis_info.gyro_angle = info.yaw_gyro_angle * 10;
    cmd_chassis_info.gyro_palstance = info.yaw_gyro_rate * 10;
    cmd_chassis_info.position_x_mm = info.position_x_mm;
    cmd_chassis_info.position_y_mm = info.position_y_mm;
    cmd_chassis_info.v_x_mm = info.v_x_mm;
    cmd_chassis_info.v_y_mm = info.v_y_mm;

    protocol_send(MANIFOLD2_ADDRESS, CMD_PUSH_CHASSIS_INFO, &cmd_chassis_info, sizeof(cmd_chassis_info));

    return 0;
}
