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
/** @file bsp_uart.h
 *  @version 1.0
 *  @date Apr 2017
 *
 *  @brief uart receive data from DBus/judge_system/manifold etc.
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"

#define UART_RX_DMA_SIZE       1024

void dbus_uart_init(void);
void computer_uart_init(void);
void judgement_uart_init(void);

void uart_receive_handler(UART_HandleTypeDef *huart);

uint8_t dma_current_memory_target(DMA_Stream_TypeDef *dma_stream);
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream);

extern uint8_t judge_dma_rxbuff[2][UART_RX_DMA_SIZE];
extern uint8_t pc_dma_rxbuff[2][UART_RX_DMA_SIZE];

#endif
