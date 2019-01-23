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

#ifndef __SINGLE_GYRO_H__
#define __SINGLE_GYRO_H__

#ifdef SINGLE_GYRO_H_GLOBAL
  #define SINGLE_GYRO_H_EXTERN 
#else
  #define SINGLE_GYRO_H_EXTERN extern
#endif

#include "stdint.h"

struct single_gyro
{
  uint32_t std_id;
  float yaw_gyro_angle;
  float yaw_gyro_rate;
};

typedef int32_t (*gyro_can_send_t)(uint32_t std_id, uint8_t *can_rx_data);
typedef int32_t (*gyro_can_forword_t)(uint32_t std_id, uint8_t *can_rx_data);

int32_t single_gyro_can_send_register(gyro_can_send_t send);
int32_t single_gyro_set_stdid(struct single_gyro *gyro, uint32_t std_id);
int32_t single_gyro_update(struct single_gyro *gyro, uint32_t std_id, uint8_t can_rx_data[]);
int32_t single_gyro_reset(struct single_gyro *gyro);
int32_t single_gyro_adjust(struct single_gyro *gyro);

#endif // __SINGLE_GYRO_H__
