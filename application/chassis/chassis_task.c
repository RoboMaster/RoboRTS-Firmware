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

#include "dbus.h"
#include "chassis_task.h"
#include "chassis_cmd.h"
#include "os_timer.h"
#include "infantry_cmd.h"
#include "board.h"
#include "event_mgr.h"
#include "event.h"
#include "chassis.h"
#include "offline_service.h"

struct pid_param chassis_motor_param =
{
    .p = 6.5f,
    .i = 0.1f,
    .max_out = 15000,
    .integral_limit = 500,
};

static void chassis_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp);
static int32_t chassis_angle_broadcast(void *argv);

struct chassis chassis;
struct rc_device chassis_rc;
struct ahrs_sensor chassis_gyro;

/* chassis speed */
static float vx, vy, wz;

/* fllow control */
struct pid pid_follow = {0};
float follow_relative_angle;

void chassis_task(void const *argument)
{
    rc_info_t p_rc_info;

    subscriber_t listSubs;
    subscriber_t nolistSubs;

    EventSubscribeInit(&listSubs, SUBS_MODE_NORMAL);
    EventSubscribe(&listSubs, DBUS_MSG, DBUS_MSG_LEN, 3, chassis_dr16_data_update);

    EventSubscribeInit(&nolistSubs, SUBS_MODE_NOLIST);
    EventSubscribe(&nolistSubs, AHRS_MSG, AHRS_MSG_LEN, 0, NULL);

    rc_device_register(&chassis_rc, "Chassis RC");
    p_rc_info = rc_device_get_info(&chassis_rc);

    chassis_pid_init(&chassis, "Chassis", chassis_motor_param, DEVICE_CAN2);

    soft_timer_register((soft_timer_callback)chassis_pid_calculate, (void *)&chassis, 5);
    soft_timer_register((soft_timer_callback)chassis_angle_broadcast, (void *)NULL, 10);

    pid_struct_init(&pid_follow, MAX_CHASSIS_VW_SPEED, 50, 8.0f, 0.0f, 2.0f);

    while (1)
    {
        /* dr16 data update */
        EventMsgProcess(&listSubs, 0);
        /* gyro data update */
        EventMsgGetLast(&nolistSubs, AHRS_MSG, &chassis_gyro, NULL);

        chassis_gyro_updata(&chassis, chassis_gyro.yaw * RAD_TO_DEG, chassis_gyro.gz * RAD_TO_DEG);

        if (rc_device_get_state(&chassis_rc, RC_S2_UP) == E_OK)
        {
            vx = (float)p_rc_info->ch2 / 660 * MAX_CHASSIS_VX_SPEED;
            vy = -(float)p_rc_info->ch1 / 660 * MAX_CHASSIS_VY_SPEED;
            wz = -pid_calculate(&pid_follow, follow_relative_angle, 0);
            chassis_set_offset(&chassis, ROTATE_X_OFFSET, ROTATE_Y_OFFSET);
            chassis_set_acc(&chassis, 0, 0, 0);
            chassis_set_speed(&chassis, vx, vy, wz);
        }

        if (rc_device_get_state(&chassis_rc, RC_S2_MID) == E_OK)
        {
            vx = (float)p_rc_info->ch2 / 660 * MAX_CHASSIS_VX_SPEED;
            vy = -(float)p_rc_info->ch1 / 660 * MAX_CHASSIS_VY_SPEED;
            wz = -(float)p_rc_info->ch3 / 660 * MAX_CHASSIS_VW_SPEED;
            chassis_set_offset(&chassis, 0, 0);
            chassis_set_acc(&chassis, 0, 0, 0);
            chassis_set_speed(&chassis, vx, vy, wz);
        }

        if (rc_device_get_state(&chassis_rc, RC_S2_MID2DOWN) == E_OK)
        {
            chassis_set_speed(&chassis, 0, 0, 0);
            chassis_set_acc(&chassis, 0, 0, 0);
        }

        if (rc_device_get_state(&chassis_rc, RC_S2_MID2UP) == E_OK)
        {
            chassis_set_speed(&chassis, 0, 0, 0);
            chassis_set_acc(&chassis, 0, 0, 0);
        }

        if (rc_device_get_state(&chassis_rc, RC_S2_DOWN) == E_OK)
        {
            set_chassis_sdk_mode(CHASSIS_SDK_ON);
            offline_event_enable(OFFLINE_MANIFOLD2_HEART);
            offline_event_enable(OFFLINE_CONTROL_CMD);

            if ((p_rc_info->ch1 < -400) && (p_rc_info->ch2 < -400) && (p_rc_info->ch3 > 400) && (p_rc_info->ch4 < -400))
            {
                static int cnt = 0;
                cnt++;
                /* 2 second */
                if (cnt > 400)
                {
                    motor_auto_set_id(DEVICE_CAN2);
                }
            }
        }
        else
        {
            /* disable sdk */
            set_chassis_sdk_mode(CHASSIS_SDK_OFF);
            offline_event_disable(OFFLINE_MANIFOLD2_HEART);
            offline_event_disable(OFFLINE_CONTROL_CMD);

            offline_event_enable(OFFLINE_CHASSIS_MOTOR1);
            offline_event_enable(OFFLINE_CHASSIS_MOTOR2);
            offline_event_enable(OFFLINE_CHASSIS_MOTOR3);
            offline_event_enable(OFFLINE_CHASSIS_MOTOR4);
        }

        osDelay(5);
    }
}

/**
  * @brief  send chassis angle to gimbal
  * @param
  * @retval void
  */
int32_t chassis_angle_broadcast(void *argv)
{
    int32_t s_yaw, s_yaw_rate;

    s_yaw = chassis.mecanum.gyro.yaw_gyro_angle * 1000;
    s_yaw_rate = chassis.mecanum.gyro.yaw_gyro_rate * 1000;

    uint8_t data[8];
    data[0] = s_yaw >> 24;
    data[1] = s_yaw >> 16;
    data[2] = s_yaw >> 8;
    data[3] = s_yaw;
    data[4] = s_yaw_rate >> 24;
    data[5] = s_yaw_rate >> 16;
    data[6] = s_yaw_rate >> 8;
    data[7] = s_yaw_rate;

    can1_std_transmit(0x401, data, 8);
    return 0;
}

struct chassis *get_chassis(void)
{
    return &chassis;
}

/**
  * @brief  subscrib dr16 event, update
  * @param
  * @retval void
  */
static void chassis_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp)
{
    rc_device_date_update(&chassis_rc, pMsgData);
}

/**
  * @brief  follow mode angle update
  * @param
  * @retval void
  */
int32_t follow_angle_info_rcv(uint8_t *buff, uint16_t len)
{
    struct cmd_gimbal_info *info;
    info = (struct cmd_gimbal_info *)buff;
    follow_relative_angle = info->yaw_ecd_angle / 10.0f;
    offline_event_time_update(OFFLINE_GIMBAL_INFO);
    return 0;
}

void set_follow_relative(float val)
{
    follow_relative_angle = val;
}
