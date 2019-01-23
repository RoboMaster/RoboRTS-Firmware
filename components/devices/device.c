/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
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

int32_t device_register(struct device *dev,
                        const char *name,
                        uint16_t flags)
{
  if (dev == NULL)
    return -RM_INVAL;
  if (device_find(name) != NULL)
    return -RM_EXISTED;

  object_init(&(dev->parent), Object_Class_Device, name);

  dev->flag = flags;
  dev->ref_count = 0;
  dev->open_flag = 0;

  return RM_OK;
}

int32_t device_unregister(struct device *dev)
{
  if (dev == NULL)
    return -RM_INVAL;
  if (device_find(((object_t)dev)->name) == NULL)
    return RM_OK;

  object_detach(&(dev->parent));

  return RM_OK;
}

device_t device_find(const char *name)
{
  struct object *object;

  object = object_find(name, Object_Class_Device);

  return (device_t)object;
}

