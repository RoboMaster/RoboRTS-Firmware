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

#ifndef __APP_MANAGE__
#define __APP_MANAGE__

#include "sys.h"
#include "device.h"
#include "protocol.h"
#include "offline_service.h"

struct protocol_recv_cmd_obj
{
    uint16_t cmd;
    rcv_handle_fn_t rcv_callback;
};

struct protocol_send_cfg_obj
{
    uint16_t cmd;
    uint8_t resend_times;
    uint16_t resend_timeout;
    uint8_t ack_enable;
    ack_handle_fn_t ack_callback;
    no_ack_handle_fn_t no_ack_callback;
};

struct offline_obj
{
    offline_event event;
    uint8_t enable;
    uint8_t level;
    uint8_t beep_times;
    uint32_t offline_time;
    offline_t offline_first_func;
    offline_t offline_func;
    offline_t online_first_func;
    offline_t online_func;
};

struct route_obj
{
    uint8_t address;
    char interface[PROTOCOL_OBJ_NAME_MAX_LEN];
};

struct app_manage
{
    uint8_t local_addr;

    struct protocol_recv_cmd_obj *recv_cmd_table;
    uint16_t recv_cmd_tab_size;

    struct protocol_send_cfg_obj *send_cfg_table;
    uint16_t send_cfg_tab_size;

    struct offline_obj *offline_table;
    uint16_t offline_tab_size;

    struct route_obj *route_table;
    uint16_t route_tab_size;

    void (*referee_cmd_callback)(uint16_t cmd_id, uint8_t *pdata, uint16_t len);
    void (*dbus_rx_complete)(void);

    void (*user_key_callback)(void);
    void (*user_input_callback)(void);

    void (*can1_msg_callback)(uint16_t std_id, uint8_t *data, uint8_t dlc);
    void (*can2_msg_callback)(uint16_t std_id, uint8_t *data, uint8_t dlc);
};

struct app_manage *get_current_app(void);
void app_protocol_init(void);
#endif
