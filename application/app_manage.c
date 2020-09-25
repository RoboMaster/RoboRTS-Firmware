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

#include "app_manage.h"

struct app_manage current_app;

struct app_manage *get_current_app(void)
{
    return &current_app;
}

/**
  * @brief  protocol tab, offline tab init
  * @param
  * @retval void
  */
void app_protocol_init(void)
{
    struct offline_manage_obj obj;

    protocol_set_local_address(current_app.local_addr);

    if (current_app.recv_cmd_table != NULL)
    {
        for (int i = 0; i < current_app.recv_cmd_tab_size; i++)
        {
            protocol_rcv_cmd_register(current_app.recv_cmd_table[i].cmd,
                                      current_app.recv_cmd_table[i].rcv_callback);
        }
    }

    if (current_app.send_cfg_table != NULL)
    {
        for (int i = 0; i < current_app.send_cfg_tab_size; i++)
        {
            protocol_send_cmd_config(current_app.send_cfg_table[i].cmd,
                                     current_app.send_cfg_table[i].resend_times,
                                     current_app.send_cfg_table[i].resend_timeout,
                                     current_app.send_cfg_table[i].ack_enable,
                                     current_app.send_cfg_table[i].ack_callback,
                                     current_app.send_cfg_table[i].no_ack_callback);
        }
    }

    if (current_app.offline_table != NULL)
    {
        for (int i = 0; i < current_app.offline_tab_size; i++)
        {
            obj.event = current_app.offline_table[i].event;
            obj.enable = current_app.offline_table[i].enable;
            obj.error_level = current_app.offline_table[i].level;

            obj.online_first_func = current_app.offline_table[i].online_first_func;
            obj.offline_first_func = current_app.offline_table[i].offline_first_func;
            obj.online_func = current_app.offline_table[i].online_func;
            obj.offline_func = current_app.offline_table[i].offline_func;

            obj.beep_times = current_app.offline_table[i].beep_times;
            obj.offline_time = current_app.offline_table[i].offline_time;
            offline_event_init(obj);
        }
    }

    if (current_app.route_table != NULL)
    {
        for (int i = 0; i < current_app.route_tab_size; i++)
        {
            protocol_set_route(current_app.route_table[i].address,
                               current_app.route_table[i].interface);
        }
    }
}
