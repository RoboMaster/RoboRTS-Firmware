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

#include "detect.h"

int32_t detect_device_register(detect_device_t detect_dev,
                               const char *name,
                               uint16_t flags,
                               uint8_t callback_mode)
{
  if (detect_dev == NULL)
    return -RM_INVAL;

  if (device_find(name) != NULL)
    return -RM_EXISTED;

  ((device_t)detect_dev)->type = Device_Class_Detect;

  detect_dev->callback_mode = callback_mode;

  device_register(&(detect_dev->parent), name, flags);

  return RM_OK;
}

int32_t detect_device_update(detect_device_t detect_dev, uint32_t event)
{
  if (detect_dev == NULL)
    return -RM_INVAL;

  uint32_t temp = 1;

  event &= detect_dev->enable;

  for (int i = 0; i < 32; i++)
  {
    temp = 1 << i;
    if ((event & temp) == temp)
    {
      detect_dev->last_time[i] = offline_get_ms();
    }
  }

  return RM_OK;
}

int32_t detect_device_set_mode(detect_device_t detect_dev, uint8_t callback_mode)
{
  if (detect_dev == NULL)
    return -RM_INVAL;

  detect_dev->callback_mode = callback_mode;

  return RM_OK;
}

int32_t detect_device_check(detect_device_t detect_dev, uint32_t event)
{
  if (detect_dev == NULL)
    return -RM_INVAL;

  uint8_t callback_execute = 0;
  uint32_t temp = 1;

  event = event & detect_dev->enable;

  for (int i = 0; i < 32; i++)
  {
    temp = 1 << i;
    if ((event & temp) == temp)
    {
      if ((offline_get_ms() - detect_dev->last_time[i]) > detect_dev->timeout[i])
      {
        detect_dev->event |= temp;

        if ((detect_dev->offline_callback[i] != NULL) && (callback_execute == 0))
        {
          detect_dev->offline_callback[i](detect_dev->argc[i]);

          if (detect_dev->callback_mode == HIGHEST_PRIORITY)
          {
            callback_execute = 1;
          }
        }
      }
      else
      {
        detect_dev->event &= ~temp;
      }
    }
  }

  return RM_OK;
}

uint32_t detect_device_get_event(detect_device_t detect_dev)
{
  if (detect_dev == NULL)
    return NULL;

  return detect_dev->event;
}

int32_t detect_device_get_state_or(detect_device_t detect_dev, int32_t event)
{
  if (detect_dev == NULL)
    return NULL;
  
  uint32_t temp = 1;
  event |= detect_dev->enable;

  for (int i = 0; i < 32; i++)
  {
    temp = 1 << i;
    if ((event & temp) == temp)
    {
      return RM_OK;
    }
  }

  return RM_NOSTATE;
}

int32_t detect_device_add_event(detect_device_t detect_dev,
                                uint32_t event,
                                uint32_t timeout,
                                int32_t (*offline_callback)(void *argc),
                                void *argc)
{
  if (detect_dev == NULL)
    return -RM_INVAL;

  uint32_t temp = 1;
  detect_dev->enable |= event;

  for (int i = 0; i < 32; i++)
  {
    temp = 1 << i;
    if ((event & temp) == temp)
    {
      detect_dev->timeout[i] = timeout;
      detect_dev->argc[i] = argc;
      detect_dev->offline_callback[i] = offline_callback;
    }
  }

  return RM_OK;
}

int32_t detect_device_modify_timeout(detect_device_t detect_dev, uint32_t event, uint32_t timeout)
{
  if (detect_dev == NULL)
    return -RM_INVAL;

  uint32_t temp = 1;

  for (int i = 0; i < 32; i++)
  {
    temp = 1 << i;
    if ((event & temp) == temp)
    {
      detect_dev->timeout[i] = timeout;
    }
  }

  return RM_OK;
}

detect_device_t detect_device_find(const char *name)
{
  device_t dev;
  enum device_type type;

  dev = device_find(name);
  
  if(dev == NULL) 
    return NULL;

  type = dev->type;

  if (type == Device_Class_Detect)
  {
    return (detect_device_t)dev;
  }
  else
  {
    return NULL;
  }
}

int32_t detect_device_enable_event(detect_device_t detect_dev, uint32_t event)
{
  if (detect_dev == NULL)
    return -RM_INVAL;

  detect_dev->enable |= event;

  return RM_OK;
}

int32_t detect_device_disable_event(detect_device_t detect_dev, uint32_t event)
{
  if (detect_dev == NULL)
    return -RM_INVAL;

  detect_dev->enable &= ~event;
  detect_dev->event &= ~event;

  return RM_OK;
}
