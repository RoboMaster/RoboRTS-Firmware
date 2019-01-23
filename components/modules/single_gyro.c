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

#include "single_gyro.h"
#include "errno.h"

gyro_can_send_t gyro_can_send = NULL;

int32_t single_gyro_can_send_register(gyro_can_send_t send)
{
  if (send != NULL)
  {
    gyro_can_send = send;
    return RM_OK;
  }
  return -RM_INVAL;
}

int32_t single_gyro_set_stdid(struct single_gyro *gyro, uint32_t std_id)
{
  if(gyro == NULL)
  {
    return -RM_INVAL;
  }
  gyro->std_id = std_id;
  return RM_OK;
}

int32_t single_gyro_update(struct single_gyro *gyro, uint32_t std_id, uint8_t can_rx_data[])
{
  if(gyro == NULL)
  {
    return -RM_INVAL;
  }
  if (std_id == gyro->std_id)
  {
    gyro->yaw_gyro_angle = -0.001f * ((int32_t)(can_rx_data[0] << 24) |
                                      (can_rx_data[1] << 16) |
                                      (can_rx_data[2] << 8) |
                                      (can_rx_data[3]));

    gyro->yaw_gyro_rate = -0.001f * ((int32_t)(can_rx_data[4] << 24) |
                                     (can_rx_data[5] << 16) |
                                     (can_rx_data[6] << 8) |
                                     (can_rx_data[7]));
    return RM_OK;
  }
  return RM_UNREGISTERED;
}

int32_t single_gyro_reset(struct single_gyro *gyro)
{
  if(gyro == NULL)
  {
    return -RM_INVAL;
  }
  if(gyro_can_send != NULL)
  {
    uint8_t can_rx_data[8] = {0,1,2,3,4,5,6,7};
    gyro_can_send(0x406, can_rx_data);
  }
  return RM_OK;
}

int32_t single_gyro_adjust(struct single_gyro *gyro)
{
  if(gyro == NULL)
  {
    return -RM_INVAL;
  }
  if(gyro_can_send != NULL)
  { 
    uint8_t can_rx_data[8] = {0,1,2,3,4,5,6,7};
    gyro_can_send(0x408, can_rx_data);
    return RM_OK;
  }
  return RM_UNREGISTERED;
}
