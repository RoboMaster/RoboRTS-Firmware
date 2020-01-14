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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

/* Includes ------------------------------------------------------------------*/
#include "protocol_common.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ---------------GERR-----------------------------------------*/
int32_t protocol_send_cmd_config(uint16_t cmd,
                                 uint8_t resend_times,
                                 uint16_t resend_timeout,
                                 uint8_t ack_enable,
                                 ack_handle_fn_t ack_callback,
                                 no_ack_handle_fn_t no_ack_callback);

int32_t protocol_rcv_cmd_register(uint16_t cmd, rcv_handle_fn_t rcv_callback);

void protocol_set_local_address(uint8_t address);
uint32_t protocol_local_object_init(void);

uint32_t protocol_send(uint8_t reciver, uint16_t cmd, void *p_data, uint32_t data_len);

uint32_t protocol_ack(uint8_t reciver, uint8_t session, void *p_data,
                      uint32_t data_len, uint16_t ack_seq);

uint32_t protocol_send_flush(void);

uint32_t protocol_unpack_flush(void);

uint32_t protocol_rcv_data(void *p_data, uint32_t data_len, struct perph_interface *perph);

uint32_t protocol_send_list_add_callback_reg(void_fn_t fn);

#endif /* SUPPORT_PROTOCOL_H_ */
