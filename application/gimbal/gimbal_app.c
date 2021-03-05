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

#include "gimbal.h"
#include "shoot.h"
#include "gimbal_app.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "infantry_cmd.h"
#include "app_manage.h"
#include "os_timer.h"
#include "gimbal_cmd.h"

#include "protocol.h"
#include "board.h"

#include "easyflash.h"
#include "offline_service.h"

osThreadId gimbal_task_t;
osThreadId shoot_task_t;

static void gimbal_can1_callback(uint16_t std_id, uint8_t *data, uint8_t dlc);
static void gimbal_can2_callback(uint16_t std_id, uint8_t *data, uint8_t dlc);

void gimbal_user_key_handle(void);
static void gimbal_dbus_rx_complete(void);
static void gimbal_input_handle(void);

void gimbal_online(void);
void gimbal_offline(void);
void gimbal_dbus_online(void);
void gimbal_heart_offline(void);
void gimbal_heart_online(void);

void gimbal_control_offline(void);
void gimbal_control_online(void);

/* no send cmd need ack */
struct protocol_send_cfg_obj gimbal_send_cfg_table[] = {0};

struct protocol_recv_cmd_obj gimbal_recv_cmd_table[] =
{
    /* CMD | callback function */
    {CMD_SET_GIMBAL_ANGLE, gimbal_angle_ctrl},
    {CMD_SET_FRICTION_SPEED, shoot_firction_ctrl},
    {CMD_SET_SHOOT_FREQUENTCY, shoot_num_ctrl},
    {CMD_GIMBAL_ADJUST, gimbal_adjust_cmd},
    {CMD_MANIFOLD2_HEART, gimbal_manifold_heart},
};

struct offline_obj gimbal_offline_table[] =
{
    /* event | enable | beep_times | offline time | offline_first_func| offline_func | online_first_func | online_func */
    {SYSTEM_PROTECT, ENABLE, 0, 0, 0, NULL, gimbal_offline, gimbal_dbus_online, gimbal_online},

    {OFFLINE_DBUS, ENABLE, OFFLINE_ERROR_LEVEL, 0, 100, NULL, NULL, NULL, NULL},
    {OFFLINE_GIMBAL_YAW, ENABLE, OFFLINE_ERROR_LEVEL, 5, 100, NULL, NULL, NULL, NULL},
    {OFFLINE_GIMBAL_PITCH, ENABLE, OFFLINE_ERROR_LEVEL, 6, 100, NULL, NULL, NULL, NULL},
    {OFFLINE_GIMBAL_TURN_MOTOR, ENABLE, OFFLINE_ERROR_LEVEL, 7, 100, NULL, NULL, NULL, NULL},
    {OFFLINE_MANIFOLD2_HEART, DISABLE, OFFLINE_WARNING_LEVEL, BEEP_DISABLE, 700, gimbal_heart_offline, NULL, NULL, gimbal_heart_online},
    {OFFLINE_CONTROL_CMD, ENABLE, OFFLINE_WARNING_LEVEL, BEEP_DISABLE, 700, NULL, gimbal_control_offline, NULL, gimbal_control_online},
};

struct route_obj gimbal_route_table[] =
{
    {GIMBAL_ADDRESS, "can1_0x600_to_0x500"},
    {MANIFOLD2_ADDRESS, "can1_0x600_to_0x500"},
};

/**
  * @brief  chassis app init
  * @param
  * @retval void
  */
void gimbal_app_init(void)
{
    struct app_manage *app;
    gimbal_t p_gimbal;

    app = get_current_app();
    p_gimbal = get_gimbal();

    protocol_can_interface_register("can1_0x600_to_0x500", 1024, 1, CAN1_PORT, CHASSIS_CAN_ID, GIMBAL_CAN_ID, can1_std_transmit);

    app->local_addr = GIMBAL_ADDRESS;
    app->recv_cmd_table = gimbal_recv_cmd_table;
    app->recv_cmd_tab_size = sizeof(gimbal_recv_cmd_table) / sizeof(struct protocol_recv_cmd_obj);

    app->send_cfg_table = NULL;

    app->offline_table = gimbal_offline_table;
    app->offline_tab_size = sizeof(gimbal_offline_table) / sizeof(struct offline_obj);

    app->route_table = gimbal_route_table;
    app->route_tab_size = sizeof(gimbal_route_table) / sizeof(struct route_obj);

    app->can1_msg_callback = gimbal_can1_callback;
    app->can2_msg_callback = gimbal_can2_callback;
    app->dbus_rx_complete = gimbal_dbus_rx_complete;

    app->user_input_callback = gimbal_input_handle;
    app->user_key_callback = gimbal_user_key_handle;

    soft_timer_register(gimbal_info_push, p_gimbal, 10);

    osThreadDef(GIMBAL_TASK, gimbal_task, osPriorityNormal, 0, 512);
    gimbal_task_t = osThreadCreate(osThread(GIMBAL_TASK), NULL);

    osThreadDef(SHOOT_TASK, shoot_task, osPriorityNormal, 0, 512);
    gimbal_task_t = osThreadCreate(osThread(SHOOT_TASK), NULL);
}

void gimbal_can1_callback(uint16_t std_id, uint8_t *data, uint8_t dlc)
{
    gimbal_gyro_yaw_update(std_id, data);
}

/**
  * @brief  chassis can2 receive interupt
  * @param
  * @retval void
  */
void gimbal_can2_callback(uint16_t std_id, uint8_t *data, uint8_t dlc)
{
    switch (std_id)
    {
    case 0x205:
        offline_event_time_update(OFFLINE_GIMBAL_YAW);
        break;
    case 0x206:
        offline_event_time_update(OFFLINE_GIMBAL_PITCH);
        break;
    case 0x207:
        offline_event_time_update(OFFLINE_GIMBAL_TURN_MOTOR);
        break;
    }
}

/**
  * @brief  chassis adjust key
  * @param
  * @retval void
  */
void gimbal_user_key_handle(void)
{
    MASTER_INT_DISABLE();

    ef_del_env(BMI088_PARAM_KEY);
    ef_del_env(GIMBAL_PARAM_KEY);
    HAL_Delay(2000);

    /* reboot */
    NVIC_SystemReset();
}

void gimbal_input_handle(void)
{
    shoot_t p_shoot;

    p_shoot = get_shoot();

    shoot_state_update(p_shoot);
}

void gimbal_dbus_rx_complete(void)
{
    offline_event_time_update(OFFLINE_DBUS);
}

void gimbal_online(void)
{
    struct shoot *p_shoot;
    struct gimbal *p_gimbal;
    p_shoot = get_shoot();
    p_gimbal = get_gimbal();

    shoot_enable(p_shoot);
    gimbal_yaw_enable(p_gimbal);
    gimbal_pitch_enable(p_gimbal);
}

void gimbal_dbus_online(void)
{
    gimbal_init_start();
}

void gimbal_offline(void)
{
    struct shoot *p_shoot;
    struct gimbal *p_gimbal;
    p_shoot = get_shoot();
    p_gimbal = get_gimbal();

    shoot_disable(p_shoot);
    gimbal_yaw_disable(p_gimbal);
    gimbal_pitch_disable(p_gimbal);
}

void gimbal_heart_offline(void)
{
    struct gimbal *p_gimbal;
    p_gimbal = get_gimbal();
    gimbal_set_pitch_mode(p_gimbal, ENCODER_MODE);
    gimbal_set_pitch_angle(p_gimbal, 0);
    gimbal_set_yaw_mode(p_gimbal, ENCODER_MODE);
    gimbal_set_yaw_angle(p_gimbal, 0, 0);

    struct shoot *p_shoot;
    p_shoot = get_shoot();
    shoot_disable(p_shoot);

    set_gimbal_heart_mode(GIMBAL_HEART_OFF);
    offline_event_enable(OFFLINE_GIMBAL_PITCH);
    offline_event_enable(OFFLINE_GIMBAL_YAW);
    offline_event_enable(OFFLINE_GIMBAL_TURN_MOTOR);

    LED_B_ON();
}

void gimbal_heart_online(void)
{
    //  struct gimbal *p_gimbal;
    //  p_gimbal = get_gimbal();

    struct shoot *p_shoot;
    p_shoot = get_shoot();
    shoot_enable(p_shoot);

    set_gimbal_heart_mode(GIMBAL_HEART_ON);
    offline_event_disable(OFFLINE_GIMBAL_PITCH);
    offline_event_disable(OFFLINE_GIMBAL_YAW);
    offline_event_disable(OFFLINE_GIMBAL_TURN_MOTOR);

    LED_B_OFF();
}

void gimbal_control_offline(void)
{
}

void gimbal_control_online(void)
{
}
