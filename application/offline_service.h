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

#ifndef __OFFLINE_SERVICE__
#define __OFFLINE_SERVICE__

#include "sys.h"

#define STATE_ONLINE 0
#define STATE_OFFLINE 1

/* system protection uses event 0. */
#define SYSTEM_PROTECT NO_OFFLINE
#define OFFLINE_ERROR_LEVEL 0
#define OFFLINE_WARNING_LEVEL 1
#define APP_PROTECT_LEVEL 2

#define BEEP_DISABLE 0xFF

typedef void (*offline_t)(void);

typedef enum
{
    NO_OFFLINE = 0, /*!< system is normal */
    OFFLINE_DBUS,
    OFFLINE_CHASSIS_MOTOR1,
    OFFLINE_CHASSIS_MOTOR2,
    OFFLINE_CHASSIS_MOTOR3,
    OFFLINE_CHASSIS_MOTOR4,
    OFFLINE_GIMBAL_PITCH,
    OFFLINE_GIMBAL_YAW,
    OFFLINE_GIMBAL_TURN_MOTOR,
    OFFLINE_SINGLE_GYRO,
    OFFLINE_MANIFOLD2_HEART,
    OFFLINE_CONTROL_CMD,
    OFFLINE_GIMBAL_INFO,
    OFFLINE_EVENT_MAX_NUM,
} offline_event;

struct offline_manage_obj
{
    offline_event event;
    uint8_t enable;
    uint8_t online_state;
    uint8_t last_state;
    uint8_t error_level;
    offline_t offline_first_func;
    offline_t offline_func;
    offline_t online_first_func;
    offline_t online_func;

    /* if offline event number is more than 1, beep_times equal minimum of all events. */
    /* max value is 0xFE */
    uint8_t beep_times;
    uint32_t last_time;
    uint32_t offline_time;
};

void offline_service_task_init(void);

void offline_event_init(struct offline_manage_obj obj);
void offline_event_time_update(offline_event event);
void offline_event_enable(offline_event event);
void offline_event_disable(offline_event event);

uint8_t get_system_status(void);

#endif
