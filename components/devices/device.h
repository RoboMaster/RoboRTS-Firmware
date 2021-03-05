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

#ifndef __DEVICE_H__
#define __DEVICE_H__

#include "sys.h"

//This macro must be greater than 16.
#define OBJECT_NAME_MAX_LEN 50

#if OBJECT_NAME_MAX_LEN < 16
    #error "Macro OBJECT_NAME_MAX_LEN must be greater than 16."
#endif

enum device_can
{
    DEVICE_CAN1 = 0,
    DEVICE_CAN2,
    DEVICE_CAN_ALL,
    DEVICE_CAN_NUM = DEVICE_CAN_ALL,
};

enum device_type
{
    DEVICE_INIT = 0,
    DEVICE_MOTOR,
    DEVICE_DBUS,
    DEVICE_SINGLE_GYRO,
    DEVICE_UNKNOW
};

struct device
{
    char name[OBJECT_NAME_MAX_LEN];
    uint8_t type;
    list_t  list;
    void *param;
    void *user_data;

    void (*device_init)(void *param);
};

typedef struct device *device_t;

struct device_information
{
    list_t object_list;          /**< object list */
};

device_t device_find(const char *name, uint8_t type);
int32_t device_init(struct device *object, const char *name);
void device_detach(device_t object);
struct device_information *get_device_information(void);

#endif // __DEVICE_H__
