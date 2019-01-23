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

#ifndef __OFFLINE_CHECK_H__
#define __OFFLINE_CHECK_H__

#ifdef OFFLINE_CHECK_H_GLOBAL
  #define OFFLINE_CHECK_H_EXTERN 
#else
  #define OFFLINE_CHECK_H_EXTERN extern
#endif

#define RC_OFFLINE_EVENT     EVENT_0BIT
#define MOTOR1_OFFLINE_EVENT EVENT_1BIT
#define MOTOR2_OFFLINE_EVENT EVENT_2BIT
#define MOTOR3_OFFLINE_EVENT EVENT_3BIT
#define MOTOR4_OFFLINE_EVENT EVENT_4BIT
#define YAW_OFFLINE_EVENT    EVENT_5BIT
#define PITCH_OFFLINE_EVENT  EVENT_6BIT
#define TURN_OFFLINE_EVENT   EVENT_7BIT
#define GYRO_OFFLINE_EVENT   EVENT_8BIT

void offline_init(void);
struct detect_device *get_offline_dev(void);
int32_t offline_check(void *argc);
int32_t get_offline_state(void);
int32_t can1_detect_update(CAN_RxHeaderTypeDef *header, uint8_t *rx_data);
int32_t can2_detect_update(CAN_RxHeaderTypeDef *header, uint8_t *rx_data);

#endif // __OFFLINE_CHECK_H__
