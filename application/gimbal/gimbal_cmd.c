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

#include "gimbal_cmd.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "infantry_cmd.h"
#include "gimbal.h"
#include "shoot.h"
#include "protocol.h"
#include "offline_service.h"

#include "appcfg.h"

uint8_t gimbal_sdk_state = GIMBAL_SDK_OFF;
uint8_t gimbal_heart_state = GIMBAL_HEART_OFF;

void gimbal_user_key_handle(void);

void set_gimbal_sdk_mode(uint8_t state)
{
    gimbal_sdk_state = state;
}

void set_gimbal_heart_mode(uint8_t state)
{
    gimbal_heart_state = state;
}

uint8_t get_gimbal_sdk_mode(void)
{
    return (gimbal_sdk_state && gimbal_heart_state);
}

int32_t gimbal_manifold_heart(uint8_t *buff, uint16_t len)
{
    offline_event_time_update(OFFLINE_MANIFOLD2_HEART);
    return 0;
}

int32_t gimbal_adjust_cmd(uint8_t *buff, uint16_t len)
{
    gimbal_user_key_handle();
    return 0;
}

int32_t gimbal_angle_ctrl(uint8_t *buff, uint16_t len)
{
    if (get_gimbal_sdk_mode() != GIMBAL_SDK_ON)
    {
        return -1;
    }

    if (len == sizeof(struct cmd_gimbal_angle))
    {
        struct cmd_gimbal_angle *p_cmd;
        p_cmd = (struct cmd_gimbal_angle *)buff;

        gimbal_t p_gimbal;
        p_gimbal = get_gimbal();

        if (p_cmd->ctrl.bit.pitch_mode == 0)
        {
            gimbal_set_pitch_angle(p_gimbal, p_cmd->pitch / 10.0f);
        }
        else
        {
            gimbal_set_pitch_speed(p_gimbal, p_cmd->pitch / 10.0f);
        }

        if (p_cmd->ctrl.bit.yaw_mode == 0)
        {
            gimbal_set_yaw_angle(p_gimbal, p_cmd->yaw / 10.0f, 0);
        }
        else
        {
            gimbal_set_yaw_speed(p_gimbal, p_cmd->yaw / 10.0f);
        }

        offline_event_time_update(OFFLINE_CONTROL_CMD);
    }
    return 0;
}

int32_t shoot_firction_ctrl(uint8_t *buff, uint16_t len)
{
    if (get_gimbal_sdk_mode() != GIMBAL_SDK_ON)
    {
        return -1;
    }

    if (len == sizeof(struct cmd_firction_speed))
    {
        shoot_t p_shoot;
        p_shoot = get_shoot();

        struct cmd_firction_speed *p_cmd;
        p_cmd = (struct cmd_firction_speed *)buff;
        shoot_set_fric_speed(p_shoot, p_cmd->left, p_cmd->right);

        offline_event_time_update(OFFLINE_CONTROL_CMD);
    }
    return 0;
}

/**
  * @brief  set shoot number
  * @param
  * @retval void
  */
int32_t shoot_num_ctrl(uint8_t *buff, uint16_t len)
{
    if (get_gimbal_sdk_mode() != GIMBAL_SDK_ON)
    {
        return -1;
    }

    if (len == sizeof(struct cmd_shoot_num))
    {
        struct cmd_shoot_num *p_cmd;
        p_cmd = (struct cmd_shoot_num *)buff;
        shoot_t p_shoot;
        p_shoot = get_shoot();
        shoot_set_cmd(p_shoot, p_cmd->shoot_cmd, p_cmd->shoot_add_num);
        shoot_set_turn_speed(p_shoot, p_cmd->shoot_freq);

        offline_event_time_update(OFFLINE_CONTROL_CMD);
    }
    return 0;
}

/**
  * @brief  gimbal information push
  * @param
  * @retval void
  */
int32_t gimbal_info_push(void *argc)
{
    struct gimbal_info info;
    struct cmd_gimbal_info cmd_gimbal_info;
    gimbal_t p_gimbal = (gimbal_t)argc;
    gimbal_get_info(p_gimbal, &info);

    cmd_gimbal_info.mode = info.mode;
#ifdef ICRA2019
    cmd_gimbal_info.pitch_ecd_angle = -info.pitch_ecd_angle * 10;
#else
    cmd_gimbal_info.pitch_ecd_angle = info.pitch_ecd_angle * 10;
#endif
    cmd_gimbal_info.pitch_gyro_angle = info.pitch_gyro_angle * 10;
    cmd_gimbal_info.pitch_rate = info.pitch_rate * 10;
    cmd_gimbal_info.yaw_ecd_angle = info.yaw_ecd_angle * 10;
    cmd_gimbal_info.yaw_gyro_angle = info.yaw_gyro_angle * 10;
    cmd_gimbal_info.yaw_rate = info.yaw_rate * 10;

    if (gimbal_get_work_mode() != NORMAL_MODE)
    {
        cmd_gimbal_info.yaw_ecd_angle = 0;
    }

    /* gimbal motor is offline */
    if (get_system_status() != STATE_ONLINE)
    {
        cmd_gimbal_info.yaw_ecd_angle = 0;
    }

    protocol_send(PROTOCOL_BROADCAST_ADDR, CMD_PUSH_GIMBAL_INFO, &cmd_gimbal_info, sizeof(cmd_gimbal_info));
    return 0;
}
