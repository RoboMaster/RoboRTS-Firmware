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

#include "device.h"
#include "errno.h"

#define LOG_TAG "device"
#define LOG_OUTPUT_LEVEL  5
#include "log.h"

char *device_name[DEVICE_UNKNOW] =
{
    "NULL",
    "MOTOR",
    "DBUS",
    "SINGLE_GYRO",
};

/* Device Infonation Link Table */
static struct device_information
    object_container = {{&(object_container.object_list), &(object_container.object_list)}};

/* return device infornation pointer */
struct device_information *get_device_information(void)
{
    return &object_container;
}

/**
  * @brief  device intialize, all object is static.
  * @param  int32_t
  * @retval error code
  */
int32_t device_init(struct device *object,
                    const char *name)
{
    var_cpu_sr();

    device_assert(object != NULL);

    /* copy name */
    if (strlen(name) > OBJECT_NAME_MAX_LEN - 1)
    {
        return -1;
    }

    strcpy(object->name, name);

    /* lock interrupt */
    enter_critical();

    {
        /* insert object into information object list */
        list_add(&(object->list), &(object_container.object_list));
    }
    log_i("%s register successful, type: %s.", name, device_name[object->type]);
    /* unlock interrupt */
    exit_critical();
    return 0;
}

/**
  * @brief  find a device by name, type
  * @param  int32_t
  * @retval error code
  */
device_t device_find(const char *name, uint8_t type)
{
    struct device *object = NULL;
    list_t *node = NULL;

    var_cpu_sr();

    /* parameter check */
    if ((name == NULL) || (type >= DEVICE_UNKNOW))
    {
        return NULL;
    }

    /* enter critical */
    enter_critical();

    /* try to find object */
    for (node = object_container.object_list.next;
            node != &(object_container.object_list);
            node = node->next)
    {
        object = list_entry(node, struct device, list);
        if ((strncmp(object->name, name, strlen(name)) == 0) && (type == object->type))
        {
            /* leave critical */
            exit_critical();

            return object;
        }
    }

    /* leave critical */
    exit_critical();

    return NULL;
}

void device_detach(device_t object)
{
    var_cpu_sr();

    /* object check */
    device_assert(object != NULL);

    /* reset object type */
    object->type = DEVICE_UNKNOW;
    /* lock interrupt */
    enter_critical();

    /* remove from old list */
    list_del(&(object->list));

    /* unlock interrupt */
    exit_critical();
}
