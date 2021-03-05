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

#ifndef __DRV_DR16_H__
#define __DRV_DR16_H__

#ifdef DRV_DR16_H_GLOBAL
    #define DRV_DR16_H_EXTERN
#else
    #define DRV_DR16_H_EXTERN extern
#endif

#include "fifo.h"

typedef int32_t (*dr16_rx_callback_t)(uint8_t *buff, uint16_t len);

#define DR16_RX_BUFFER_SIZE      (50u)
#define DR16_DATA_LEN            (18u)

void dr16_uart_init(dr16_rx_callback_t rx_fn,
                    dr16_rx_callback_t forword_fn);

uint32_t dr16_uart_rx_data_handle(UART_HandleTypeDef *huart);

#endif // __DRV_DR16_H__
