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

#ifndef __DBUS_H__
#define __DBUS_H__

#include "device.h"

#define RC_S1_UP2MID   (1 << 0u)
#define RC_S1_MID2UP   (1 << 1u)
#define RC_S1_DOWN2MID (1 << 2u)
#define RC_S1_MID2DOWN (1 << 3u)

#define RC_S2_UP2MID   (1 << 4u)
#define RC_S2_MID2UP   (1 << 5u)
#define RC_S2_DOWN2MID (1 << 6u)
#define RC_S2_MID2DOWN (1 << 7u)

#define RC_S1_UP       (1 << 8u)
#define RC_S1_MID      (1 << 9u)
#define RC_S1_DOWN     (1 << 10u)
#define RC_S2_UP       (1 << 11u)
#define RC_S2_MID      (1 << 12u)
#define RC_S2_DOWN     (1 << 13u)

typedef struct rc_device *rc_device_t;
typedef struct rc_info *rc_info_t;

// #pragma pack(push, 1)

/**
  * @brief  remote control information
  */
struct rc_info
{
    /* rocker channel information */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    /* left and right lever information */
    uint8_t sw1;
    uint8_t sw2;
    /* mouse movement and button information */
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;

        uint8_t l;
        uint8_t r;
    } mouse;
    /* keyboard key information */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W : 1;
            uint16_t S : 1;
            uint16_t A : 1;
            uint16_t D : 1;
            uint16_t SHIFT : 1;
            uint16_t CTRL : 1;
            uint16_t Q : 1;
            uint16_t E : 1;
            uint16_t R : 1;
            uint16_t F : 1;
            uint16_t G : 1;
            uint16_t Z : 1;
            uint16_t X : 1;
            uint16_t C : 1;
            uint16_t V : 1;
            uint16_t B : 1;
        } bit;
    } kb;
    int16_t wheel;
};

// #pragma pack(pop)

struct rc_device
{
    struct device parent;
    struct rc_info rc_info;
    struct rc_info last_rc_info;
    uint16_t state;
    void (*get_data)(rc_device_t, uint8_t *);
    void (*get_state)(rc_device_t);
};

int32_t rc_device_register(rc_device_t rc_dev, const char *name);
int32_t rc_device_date_update(rc_device_t rc_dev, uint8_t *buff);
int32_t rc_device_get_state(rc_device_t rc_dev, uint16_t state);
rc_info_t rc_device_get_info(rc_device_t rc_dev);
rc_device_t rc_device_find(const char *name);


#endif // __DBUS_H__
