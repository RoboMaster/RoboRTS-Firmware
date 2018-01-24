/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
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
/** @file comm_task.h
 *  @version 1.0
 *  @date Oct 2017
 *
 *  @brief communicate with computer task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __COMM_TASK_H__
#define __COMM_TASK_H__

#include "stm32f4xx_hal.h"
#include "data_fifo.h"

/* communication task period time (ms) */
#define COMM_TASK_PERIOD 10

#define JUDGE_UART_TX_SIGNAL   ( 1 << 0 )
#define JUDGE_UART_IDLE_SIGNAL ( 1 << 1 )
#define JUDGE_DMA_FULL_SIGNAL  ( 1 << 2 )

#define PC_UART_TX_SIGNAL      ( 1 << 3 )
#define PC_UART_IDLE_SIGNAL    ( 1 << 4 )
#define PC_DMA_FULL_SIGNAL     ( 1 << 5 )

#define GIMBAL_MOTOR_MSG_SEND  ( 1 << 6 )
#define CHASSIS_MOTOR_MSG_SEND ( 1 << 7 )

#define SHOT_TASK_EXE_SIGNAL   ( 1 << 8 )
#define INFO_GET_EXE_SIGNAL    ( 1 << 9 )

typedef struct
{
  /* 4 chassis motor current */
  int16_t chassis_cur[4];
  /* yaw/pitch/trigger motor current */
  int16_t gimbal_cur[3];
} motor_current_t;

void freq_info_task(void const *argu);

void judge_unpack_task(void const *argu);
void pc_unpack_task(void const *argu);

void can_msg_send_task(void const *argu);

void communicate_param_init(void);

void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof);

static void get_upload_data(void);

extern motor_current_t glb_cur;

#endif
