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

#ifndef __DETECT_H__
#define __DETECT_H__

#ifdef DETECT_H_GLOBAL
  #define DETECT_H_EXTERN 
#else
  #define DETECT_H_EXTERN extern
#endif

#include "device.h"

#define offline_get_ms HAL_GetTick

#define HIGHEST_PRIORITY 0
#define ALL_PRIORITY     1

#define EVENT_0BIT  (1<<0u)
#define EVENT_1BIT  (1<<1u)
#define EVENT_2BIT  (1<<2u)
#define EVENT_3BIT  (1<<3u)
#define EVENT_4BIT  (1<<4u)
#define EVENT_5BIT  (1<<5u)
#define EVENT_6BIT  (1<<6u)
#define EVENT_7BIT  (1<<7u)
#define EVENT_8BIT  (1<<8u)
#define EVENT_9BIT  (1<<9u)
#define EVENT_10BIT (1<<10u)
#define EVENT_11BIT (1<<11u)
#define EVENT_12BIT (1<<12u)
#define EVENT_13BIT (1<<13u)
#define EVENT_14BIT (1<<14u)
#define EVENT_15BIT (1<<15u)
#define EVENT_16BIT (1<<16u)
#define EVENT_17BIT (1<<17u)
#define EVENT_18BIT (1<<18u)
#define EVENT_19BIT (1<<19u)
#define EVENT_20BIT (1<<20u)
#define EVENT_21BIT (1<<21u)
#define EVENT_22BIT (1<<22u)
#define EVENT_23BIT (1<<23u)
#define EVENT_24BIT (1<<24u)
#define EVENT_25BIT (1<<25u)
#define EVENT_26BIT (1<<26u)
#define EVENT_27BIT (1<<27u)
#define EVENT_28BIT (1<<28u)
#define EVENT_29BIT (1<<29u)
#define EVENT_30BIT (1<<30u)
#define EVENT_31BIT (1<<31u)

typedef struct detect_device *detect_device_t;

struct detect_device
{
  struct device parent;
  uint8_t callback_mode;
  uint32_t event;
  uint32_t enable;
  uint32_t last_time[32];
  uint32_t timeout[32];
  void *argc[32];
  int32_t (*offline_callback[32])(void *argc);
};

int32_t detect_device_register(detect_device_t detect_dev,
                               const char *name,
                               uint16_t flags,
                               uint8_t callback_mode);
int32_t detect_device_update(detect_device_t detect_dev, uint32_t event);
int32_t detect_device_check(detect_device_t detect_dev, uint32_t event);
uint32_t detect_device_get_event(detect_device_t detect_dev);
int32_t detect_device_get_state_or(detect_device_t detect_dev, int32_t event);
int32_t detect_device_add_event(detect_device_t detect_dev,
                                uint32_t event,
                                uint32_t timeout,
                                int32_t (*offline_callback)(void *argc),
                                void *argc);
int32_t detect_device_set_mode(detect_device_t detect_dev, uint8_t callback_mode);
int32_t detect_device_modify_timeout(detect_device_t detect_dev, uint32_t event, uint32_t timeout);
int32_t detect_device_enable_event(detect_device_t detect_dev, uint32_t event);
int32_t detect_device_disable_event(detect_device_t detect_dev, uint32_t event);
detect_device_t detect_device_find(const char *name);
#endif // __DETECT_H__
