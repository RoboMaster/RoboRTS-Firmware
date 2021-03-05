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
#ifndef __CHASSIS_CMD_H__
#define __CHASSIS_CMD_H__

#include "infantry_cmd.h"

#define CHASSIS_SDK_ON  1
#define CHASSIS_SDK_OFF 0

#define CHASSIS_HEART_ON  1
#define CHASSIS_HEART_OFF 0

void set_chassis_sdk_mode(uint8_t state);
void set_chassis_heart_mode(uint8_t state);
uint8_t get_chassis_sdk_mode(void);

int32_t chassis_manifold_heart(uint8_t *buff, uint16_t len);
int32_t chassis_info_push(void *argc);
int32_t chassis_spd_acc_ctrl(uint8_t *buff, uint16_t len);
int32_t chassis_speed_ctrl(uint8_t *buff, uint16_t len);
int32_t student_data_transmit(uint8_t *buff, uint16_t len);

#endif
