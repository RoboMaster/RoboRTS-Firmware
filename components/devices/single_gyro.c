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

#include "single_gyro.h"
#include "errno.h"

#define LOG_TAG "drv.gyro"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"

/**
  * @brief  gyro device initialize
  * @param
  * @retval error code
  */
int32_t single_gyro_init(struct single_gyro *gyro, char *name, uint16_t std_id)
{
    device_assert(gyro != NULL);

    gyro->std_id = std_id;

    ((device_t)gyro)->type = DEVICE_SINGLE_GYRO;

    device_init(&(gyro->parent), name);
    return E_OK;
}

/**
  * @brief  update gyro by can
  * @param
  * @retval error code
  */
int32_t single_gyro_update(struct single_gyro *gyro, uint16_t std_id, uint8_t can_rx_data[])
{
    device_assert(gyro != NULL);

    if (std_id == gyro->std_id)
    {
        gyro->yaw_gyro_angle = 0.001f * ((int32_t)(can_rx_data[0] << 24) |
                                         (can_rx_data[1] << 16) |
                                         (can_rx_data[2] << 8) |
                                         (can_rx_data[3]));

        gyro->yaw_gyro_rate = 0.001f * ((int32_t)(can_rx_data[4] << 24) |
                                        (can_rx_data[5] << 16) |
                                        (can_rx_data[6] << 8) |
                                        (can_rx_data[7]));
        return E_OK;
    }
    return E_INVAL;
}

