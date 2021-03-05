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

#include "dbus.h"
#include "sys.h"

#define LOG_TAG "drv.dbus"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"

static void get_dr16_data(rc_device_t rc_dev, uint8_t *buff);
static void get_dr16_state(rc_device_t rc_dev);

int32_t rc_device_register(rc_device_t rc_dev, const char *name)
{
    device_assert(rc_dev != NULL);

    ((device_t)rc_dev)->type = DEVICE_DBUS;

    rc_dev->get_data = get_dr16_data;
    rc_dev->get_state = get_dr16_state;

    device_init(&(rc_dev->parent), name);

    return E_OK;
}

int32_t rc_device_date_update(rc_device_t rc_dev, uint8_t *buff)
{
    if (rc_dev != NULL)
    {
        rc_dev->get_data(rc_dev, buff);
        rc_dev->get_state(rc_dev);
        return E_OK;
    }
    return -E_UNREGISTERED;
}

int32_t rc_device_get_state(rc_device_t rc_dev, uint16_t state)
{
    var_cpu_sr();

    enter_critical();

    if (rc_dev != NULL)
    {
        if ((rc_dev->state & state) == state)
        {
            rc_dev->state &= (~(state & 0x00FF));
            exit_critical();
            return E_OK;
        }
        else
        {
            exit_critical();
            return -E_NOSTATE;
        }
    }

    exit_critical();

    return -E_UNREGISTERED;
}

rc_info_t rc_device_get_info(rc_device_t rc_dev)
{
    device_assert(rc_dev != NULL);

    return &(rc_dev->rc_info);
}

static void get_dr16_data(rc_device_t rc_dev, uint8_t *buff)
{

    memcpy(&(rc_dev->last_rc_info), &rc_dev->rc_info, sizeof(struct rc_info));

    rc_info_t rc = &rc_dev->rc_info;

    rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3 -= 1024;
    rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc->ch4 -= 1024;

    /* prevent remote control zero deviation */
    if (rc->ch1 <= 5 && rc->ch1 >= -5)
    {
        rc->ch1 = 0;
    }
    if (rc->ch2 <= 5 && rc->ch2 >= -5)
    {
        rc->ch2 = 0;
    }
    if (rc->ch3 <= 5 && rc->ch3 >= -5)
    {
        rc->ch3 = 0;
    }
    if (rc->ch4 <= 5 && rc->ch4 >= -5)
    {
        rc->ch4 = 0;
    }

    rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buff[5] >> 4) & 0x0003;

    if ((abs(rc->ch1) > 660) || \
            (abs(rc->ch2) > 660) || \
            (abs(rc->ch3) > 660) || \
            (abs(rc->ch4) > 660))
    {
        memset(rc, 0, sizeof(struct rc_info));
        return ;
    }

    rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
    rc->mouse.y = buff[8] | (buff[9] << 8);
    rc->mouse.z = buff[10] | (buff[11] << 8);

    rc->mouse.l = buff[12];
    rc->mouse.r = buff[13];

    rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
    rc->wheel = (buff[16] | buff[17] << 8) - 1024;
}

static void get_dr16_state(rc_device_t rc_dev)
{

    if (rc_dev->rc_info.sw1 == 3)
    {
        rc_dev->state |= RC_S1_MID;
        rc_dev->state &= ~RC_S1_UP;
        rc_dev->state &= ~RC_S1_DOWN;
        if (rc_dev->last_rc_info.sw1 == 1)
        {
            rc_dev->state |= RC_S1_UP2MID;
        }
        else if (rc_dev->last_rc_info.sw1 == 2)
        {
            rc_dev->state |= RC_S1_DOWN2MID;
        }
    }
    else if (rc_dev->rc_info.sw1 == 1)
    {
        rc_dev->state &= ~RC_S1_MID;
        rc_dev->state |= RC_S1_UP;
        rc_dev->state &= ~RC_S1_DOWN;
        if (rc_dev->last_rc_info.sw1 == 3)
        {
            rc_dev->state |= RC_S1_MID2UP;
        }
    }
    else if (rc_dev->rc_info.sw1 == 2)
    {
        rc_dev->state &= ~RC_S1_MID;
        rc_dev->state &= ~RC_S1_UP;
        rc_dev->state |= RC_S1_DOWN;
        if (rc_dev->last_rc_info.sw1 == 3)
        {
            rc_dev->state |= RC_S1_MID2DOWN;
        }
    }

    if (rc_dev->rc_info.sw2 == 3)
    {
        rc_dev->state |= RC_S2_MID;
        rc_dev->state &= ~RC_S2_UP;
        rc_dev->state &= ~RC_S2_DOWN;
        if (rc_dev->last_rc_info.sw2 == 1)
        {
            rc_dev->state |= RC_S2_UP2MID;
        }
        else if (rc_dev->last_rc_info.sw2 == 2)
        {
            rc_dev->state |= RC_S2_DOWN2MID;
        }
    }
    else if (rc_dev->rc_info.sw2 == 1)
    {
        rc_dev->state &= ~RC_S2_MID;
        rc_dev->state |= RC_S2_UP;
        rc_dev->state &= ~RC_S2_DOWN;
        if (rc_dev->last_rc_info.sw2 == 3)
        {
            rc_dev->state |= RC_S2_MID2UP;
        }
    }
    else if (rc_dev->rc_info.sw2 == 2)
    {
        rc_dev->state &= ~RC_S2_MID;
        rc_dev->state &= ~RC_S2_UP;
        rc_dev->state |= RC_S2_DOWN;
        if (rc_dev->last_rc_info.sw2 == 3)
        {
            rc_dev->state |= RC_S2_MID2DOWN;
        }
    }
}
