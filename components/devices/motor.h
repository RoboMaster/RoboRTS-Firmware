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

#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef MOTOR_H_GLOBAL
    #define MOTOR_H_EXTERN
#else
    #define MOTOR_H_EXTERN extern
#endif

#include "device.h"

#ifndef ENCODER_ANGLE_RATIO
    #define ENCODER_ANGLE_RATIO (8192.0f / 360.0f)
#endif

#define MOTOR_FLAG_UNINITIALIZED (1 << 0)
#define MOTOR_FLAG_OFFLINE       (1 << 7)

typedef struct motor_data *motor_data_t;
typedef struct motor_device *motor_device_t;

struct motor_data
{
    uint16_t ecd;
    uint16_t last_ecd;

    int16_t speed_rpm;
    int16_t given_current;

    int32_t round_cnt;
    int32_t total_ecd;
    int32_t total_angle;

    int32_t ecd_raw_rate;

    uint32_t msg_cnt;
    uint16_t offset_ecd;
};

struct can_msg
{
    uint32_t id : 29;
    uint32_t ide : 1;
    uint32_t rtr : 1;
    uint32_t rsv : 1;
    uint32_t len : 8;
    uint32_t priv : 8;
    uint32_t hdr : 8;
    uint32_t reserved : 8;
    uint8_t data[8];
};

struct motor_device
{
    struct device parent;
    struct motor_data data;

    enum device_can can_periph;
    uint16_t can_id;
    uint16_t init_offset_f;

    int16_t current;

    void (*get_data)(motor_device_t, uint8_t *);
};

typedef int32_t (*fn_can_send)(enum device_can can, struct can_msg);

void motor_device_can_send_register(fn_can_send fn);

motor_device_t motor_find(const char *name);
motor_device_t motor_find_by_canid(enum device_can can, uint16_t can_id);
int32_t motor_register(motor_device_t motor_dev, const char *name);
void motor_can_send_register(fn_can_send fn);
motor_data_t motor_get_data(motor_device_t motor_dev);
int32_t motor_set_current(motor_device_t motor_dev, int16_t current);
int32_t motor_data_update(enum device_can can, uint16_t can_id, uint8_t can_rx_data[]);
int32_t motor_can_output(enum device_can m_can);
int32_t motor_auto_set_id(enum device_can can);
#endif // __MOTOR_H__
