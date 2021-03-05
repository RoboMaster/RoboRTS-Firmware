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

#include "os_timer.h"
#include "app_manage.h"
#include "chassis_cmd.h"
#include "chassis_task.h"
#include "protocol.h"
#include "board.h"
#include "easyflash.h"

osThreadId chassis_task_t;

static void chassis_can2_callback(uint16_t std_id, uint8_t *data, uint8_t dlc);
static void chassis_user_key_handle(void);
static void chassis_offline(void);
static void chassis_online(void);
static void chassis_dbus_rx_complete(void);
static void chassis_input_handle(void);

void chassis_heart_offline(void);
void chassis_heart_online(void);

void chassis_control_online(void);
void chassis_control_offline(void);

void gimbal_info_offline(void);
void gimbal_info_online(void);

void referee_data_send2pc(uint16_t cmd_id, uint8_t *pdata, uint16_t len);

/* no send cmd need ack */
struct protocol_send_cfg_obj chassis_send_cfg_table[] = {0};

struct protocol_recv_cmd_obj chassis_recv_cmd_table[] =
{
    /* CMD | callback function */
    {CMD_STUDENT_DATA, student_data_transmit},
    {CMD_PUSH_GIMBAL_INFO, follow_angle_info_rcv},
    {CMD_SET_CHASSIS_SPEED, chassis_speed_ctrl},
    {CMD_SET_CHASSIS_SPD_ACC, chassis_spd_acc_ctrl},
    {CMD_MANIFOLD2_HEART, chassis_manifold_heart},
};

struct offline_obj chassis_offline_table[] =
{
    /* event | enable | beep_times | offline time | offline_first_func| offline_func | online_first_func | online_func */
    {SYSTEM_PROTECT, ENABLE, 0, 0, 0, 0, chassis_offline, chassis_online, NULL},

    {OFFLINE_DBUS, ENABLE, OFFLINE_ERROR_LEVEL, 0, 100, NULL, NULL, NULL, NULL},

    {OFFLINE_CHASSIS_MOTOR1, ENABLE, OFFLINE_ERROR_LEVEL, 1, 100, NULL, NULL, NULL, NULL},
    {OFFLINE_CHASSIS_MOTOR2, ENABLE, OFFLINE_ERROR_LEVEL, 2, 100, NULL, NULL, NULL, NULL},
    {OFFLINE_CHASSIS_MOTOR3, ENABLE, OFFLINE_ERROR_LEVEL, 3, 100, NULL, NULL, NULL, NULL},
    {OFFLINE_CHASSIS_MOTOR4, ENABLE, OFFLINE_ERROR_LEVEL, 4, 100, NULL, NULL, NULL, NULL},
    {OFFLINE_MANIFOLD2_HEART, DISABLE, OFFLINE_WARNING_LEVEL, BEEP_DISABLE, 700, NULL, chassis_heart_offline, NULL, chassis_heart_online},
    {OFFLINE_CONTROL_CMD, ENABLE, OFFLINE_WARNING_LEVEL, BEEP_DISABLE, 700, NULL, chassis_control_offline, NULL, chassis_control_online},

    {OFFLINE_GIMBAL_INFO, ENABLE, APP_PROTECT_LEVEL, 0, 700, NULL, gimbal_info_offline, NULL, gimbal_info_online},
};

struct route_obj chassis_route_table[] =
{
    {GIMBAL_ADDRESS, "can1_0x500_to_0x600"},
    {MANIFOLD2_ADDRESS, "usb"},
};

/**
  * @brief  chassis app init
  * @param
  * @retval void
  */
void chassis_app_init(void)
{
    struct app_manage *app;
    chassis_t p_chassis;

    protocol_can_interface_register("can1_0x500_to_0x600", 1024, 1, CAN1_PORT, GIMBAL_CAN_ID, CHASSIS_CAN_ID, can1_std_transmit);

    app = get_current_app();
    p_chassis = get_chassis();

    app->local_addr = CHASSIS_ADDRESS;
    app->recv_cmd_table = chassis_recv_cmd_table;
    app->recv_cmd_tab_size = sizeof(chassis_recv_cmd_table) / sizeof(struct protocol_recv_cmd_obj);

    app->send_cfg_table = NULL;

    app->offline_table = chassis_offline_table;
    app->offline_tab_size = sizeof(chassis_offline_table) / sizeof(struct offline_obj);

    app->route_table = chassis_route_table;
    app->route_tab_size = sizeof(chassis_route_table) / sizeof(struct route_obj);

    app->can2_msg_callback = chassis_can2_callback;
    app->dbus_rx_complete = chassis_dbus_rx_complete;

    app->user_input_callback = chassis_input_handle;
    app->user_key_callback = chassis_user_key_handle;

    app->referee_cmd_callback = referee_data_send2pc;

    soft_timer_register(chassis_info_push, p_chassis, 10);

    osThreadDef(CHASSIS_TASK, chassis_task, osPriorityNormal, 0, 512);
    chassis_task_t = osThreadCreate(osThread(CHASSIS_TASK), NULL);
}

/**
  * @brief  chassis can2 receive interupt
  * @param
  * @retval void
  */
void chassis_can2_callback(uint16_t std_id, uint8_t *data, uint8_t dlc)
{
    switch (std_id)
    {
    case 0x201:
        offline_event_time_update(OFFLINE_CHASSIS_MOTOR1);
        break;
    case 0x202:
        offline_event_time_update(OFFLINE_CHASSIS_MOTOR2);
        break;
    case 0x203:
        offline_event_time_update(OFFLINE_CHASSIS_MOTOR3);
        break;
    case 0x204:
        offline_event_time_update(OFFLINE_CHASSIS_MOTOR4);
        break;
    }
}

uint32_t key_time_now = 0;
/**
  * @brief  chassis adjust key
  * @param
  * @retval void
  */
void chassis_user_key_handle(void)
{
    MASTER_INT_DISABLE();

    LED_B_ON();
    LED_R_OFF();
    LED_G_OFF();

    ef_del_env(BMI088_PARAM_KEY);
    HAL_Delay(2000);

    /* reboot */
    NVIC_SystemReset();
}

void chassis_input_handle(void)
{
}

void chassis_dbus_rx_complete(void)
{
    offline_event_time_update(OFFLINE_DBUS);
}

/**
  * @brief  chassis protection
  * @param
  * @retval void
  */
void chassis_offline(void)
{
    chassis_t p_chassis;
    p_chassis = get_chassis();
    chassis_disable(p_chassis);
}

/**
  * @brief  chassis enable
  * @param
  * @retval void
  */
void chassis_online(void)
{
    chassis_t p_chassis;
    p_chassis = get_chassis();

    chassis_enable(p_chassis);
}

void chassis_heart_offline(void)
{
    set_chassis_heart_mode(CHASSIS_HEART_OFF);
    chassis_t p_chassis = get_chassis();
    LED_B_ON();
    chassis_set_acc(p_chassis, 0, 0, 0);
    chassis_set_speed(p_chassis, 0, 0, 0);
    offline_event_enable(OFFLINE_CHASSIS_MOTOR1);
    offline_event_enable(OFFLINE_CHASSIS_MOTOR2);
    offline_event_enable(OFFLINE_CHASSIS_MOTOR3);
    offline_event_enable(OFFLINE_CHASSIS_MOTOR4);
}

void chassis_heart_online(void)
{
    LED_B_OFF();
    set_chassis_heart_mode(CHASSIS_HEART_ON);
    offline_event_disable(OFFLINE_CHASSIS_MOTOR1);
    offline_event_disable(OFFLINE_CHASSIS_MOTOR2);
    offline_event_disable(OFFLINE_CHASSIS_MOTOR3);
    offline_event_disable(OFFLINE_CHASSIS_MOTOR4);
}

void chassis_control_offline(void)
{
    chassis_t p_chassis = get_chassis();

    chassis_set_acc(p_chassis, 0, 0, 0);
    chassis_set_speed(p_chassis, 0, 0, 0);
}

void chassis_control_online(void)
{
    LED_B_OFF();
}

void gimbal_info_offline(void)
{
    set_follow_relative(0);
}

void gimbal_info_online(void)
{
    return;
}

void referee_data_send2pc(uint16_t cmd_id, uint8_t *pdata, uint16_t len)
{
    protocol_send(MANIFOLD2_ADDRESS, cmd_id + 0x4000, pdata, len);
}
