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
/** @file info_interactive.h
 *  @version 1.0
 *  @date Oct 2017
 *
 *  @brief get hardware peripheral information
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __INFO_INTER_H__
#define __INFO_INTER_H__

#include "stm32f4xx_hal.h"

int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);

void get_gimbal_info(void);
void get_shoot_info(void);
void send_gimbal_motor_ctrl_message(int16_t gimbal_cur[]);

void get_chassis_info(void);
void send_chassis_motor_ctrl_message(int16_t chassis_cur[]);
void chassis_position_measure(void);

void write_uart_noblocking(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size);
void write_uart_blocking(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size);

void read_uart_noblocking(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size);
void read_uart_blocking(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size);

void uart_write_completed_signal(UART_HandleTypeDef *huart);
void uart_read_completed_signal(UART_HandleTypeDef *huart);
  
void uart_idle_interrupt_signal(UART_HandleTypeDef *huart);
void uart_dma_full_signal(UART_HandleTypeDef *huart);

void get_dma_memory_msg(DMA_Stream_TypeDef *dma_stream, uint8_t *mem_id, uint16_t *remain_cnt);

void no_cali_data_handle(void);
void get_infantry_info(void);
void get_custom_data_info(void);

uint8_t read_gimbal_offset(int32_t *pit_offset, int32_t *yaw_offset,
                           int32_t *pit_buff_offset, int32_t *yaw_buff_offset);

static void get_structure_param(void);
static void gimbal_cali_msg_hook(uint8_t cur_type, uint8_t last_type);

#endif
