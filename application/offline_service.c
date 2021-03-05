
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

#include "cmsis_os.h"
#include "drv_io.h"
#include "offline_service.h"

static void offline_service(void const *argument);
static void offline_event_callback(struct offline_manage_obj *obj);

struct offline_manage_obj offline_manage[OFFLINE_EVENT_MAX_NUM] = {NO_OFFLINE};

osThreadId offline_service_task_t;

void offline_service_task_init(void)
{
    for (int i = 1; i < OFFLINE_EVENT_MAX_NUM; i++)
    {
        offline_manage[i].online_state = STATE_OFFLINE;
    }

    osThreadDef(OFFLINE_TASK, offline_service, osPriorityNormal, 0, 512);
    offline_service_task_t = osThreadCreate(osThread(OFFLINE_TASK), NULL);
}

/**
  * @brief  check device offline.
  * @param
  * @retval void
  */
void offline_service(void const *argument)
{
    offline_event display_event;
    uint32_t time_now;

    while (1)
    {
        time_now = get_time_ms();
        uint8_t beep_time = 0xFF;
        uint8_t error_level = 0XFF;

        display_event = NO_OFFLINE;

        for (int i = 1; i < OFFLINE_EVENT_MAX_NUM; i++)
        {
            if ((time_now - offline_manage[i].last_time > offline_manage[i].offline_time) && (offline_manage[i].enable))
            {
                offline_manage[i].online_state = STATE_OFFLINE;
                if (error_level > offline_manage[i].error_level)
                {
                    error_level = offline_manage[i].error_level;
                }

                if (offline_manage[i].beep_times < beep_time)
                {
                    if (offline_manage[i].error_level <= OFFLINE_WARNING_LEVEL)
                    {
                        beep_time = offline_manage[i].beep_times;
                        display_event = (offline_event)i;
                    }
                }
            }
            else
            {
                offline_manage[i].online_state = STATE_ONLINE;
            }
        }

        if ((error_level == OFFLINE_ERROR_LEVEL) && (offline_manage[SYSTEM_PROTECT].enable))
        {
            offline_manage[SYSTEM_PROTECT].online_state = STATE_OFFLINE;
            offline_event_callback(&offline_manage[SYSTEM_PROTECT]);
        }
        else
        {
            offline_manage[SYSTEM_PROTECT].online_state = STATE_ONLINE;
            offline_event_callback(&offline_manage[SYSTEM_PROTECT]);

            for (int i = 1; i < OFFLINE_EVENT_MAX_NUM; i++)
            {
                offline_event_callback(&offline_manage[i]);
            }
        }

        if (display_event != NO_OFFLINE)
        {
            beep_set_times(offline_manage[display_event].beep_times);
            if (offline_manage[display_event].beep_times == 0)
            {
                LED_R_ON();
            }
        }
        else
        {
            display_event = NO_OFFLINE;
            beep_set_times(0);
            LED_R_OFF();
        }

        osDelay(20);
    }
}

/**
  * @brief  get system offline event
  * @param
  * @retval void
  */
uint8_t get_system_status(void)
{
    return offline_manage[SYSTEM_PROTECT].online_state;
}

/**
  * @brief  register offline event
  * @param
  * @retval void
  */
void offline_event_init(struct offline_manage_obj obj)
{
    offline_event event = obj.event;

    offline_manage[event].event = event;
    offline_manage[event].enable = obj.enable;
    offline_manage[event].error_level = obj.error_level;
    offline_manage[event].online_first_func = obj.online_first_func;
    offline_manage[event].offline_first_func = obj.offline_first_func;
    offline_manage[event].online_func = obj.online_func;
    offline_manage[event].offline_func = obj.offline_func;

    offline_manage[event].beep_times = obj.beep_times;
    offline_manage[event].offline_time = obj.offline_time;
}

/**
  * @brief  called when offline happened.
  * @param
  * @retval void
  */
void offline_event_callback(struct offline_manage_obj *obj)
{
    if (obj->online_state == STATE_OFFLINE)
    {
        if (obj->last_state == STATE_ONLINE)
        {
            if (obj->offline_first_func != NULL)
            {
                obj->offline_first_func();
            }
        }
        else
        {
            if (obj->offline_func != NULL)
            {
                obj->offline_func();
            }
        }
    } // obj->online_state == STATE_OFFLINE
    else
    {
        obj->online_state = STATE_ONLINE;
        if (obj->last_state == STATE_OFFLINE)
        {
            if (obj->online_first_func != NULL)
            {
                obj->online_first_func();
            }
        }
        else
        {
            if (obj->online_func != NULL)
            {
                obj->online_func();
            }
        }
    } // obj->online_state != STATE_OFFLINE
    obj->last_state = obj->online_state;
}

/**
  * @brief  update the information receive time.
  * @param
  * @retval void
  */
void offline_event_time_update(offline_event event)
{
    offline_manage[event].last_time = get_time_ms();
}

/**
  * @brief  enable event offline check
  * @param
  * @retval void
  */
void offline_event_enable(offline_event event)
{
    offline_manage[event].enable = 1;
}

void offline_event_disable(offline_event event)
{
    offline_manage[event].enable = 0;
}
