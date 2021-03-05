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
#ifndef _PROTOCOL_DATALINK_H_
#define _PROTOCOL_DATALINK_H_

/* Includes ------------------------------------------------------------------*/
#include "protocol_common.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

uint8_t protocol_get_session(struct perph_interface *interface);
int32_t protocol_release_session(struct perph_interface *interface, uint8_t id);
struct send_cmd_info *protocol_get_send_cmd_info(uint16_t cmd);

uint32_t protocol_s_add_sendnode(uint8_t reciver, uint8_t session, uint8_t pack_type,
                                 void *p_data, uint32_t data_len, uint16_t cmd, uint16_t ack_seq);
uint32_t protocol_s_broadcast_add_node(void *p_data, uint32_t data_len, uint16_t cmd);
uint32_t protocol_s_fill_pack(send_ctx_t *ctx, uint8_t *p_data,
                              uint32_t data_len, uint8_t *pack_zone, uint16_t seq, uint16_t cmd);
uint32_t protocol_s_interface_send_data(send_list_node_t *cur_send_node, struct perph_interface *obj);
uint32_t protocol_s_interface_normal_send_flush(struct perph_interface *obj);
uint32_t protocol_s_interface_ack_send_flush(struct perph_interface *obj);
uint32_t protocol_s_broadcast_send_flush(void);

struct perph_interface *protocol_s_get_route(uint8_t tar_add);

uint32_t protocol_s_pack_forward(protocol_pack_desc_t *p_pack, struct perph_interface *src_obj);
uint32_t protocol_s_unpack_data_handle(struct perph_interface *obj);
send_list_node_t *protocol_s_session_get_node(struct perph_interface *obj,
        uint8_t address, uint8_t session);
uint32_t protocol_s_extract(struct perph_interface *obj);
uint32_t protocol_s_find_pack_header(rcvd_desc_t *rcvd);
uint32_t protocol_s_auth_pack_header(rcvd_desc_t *rcvd);
uint32_t protocol_s_fetch_pack_data(rcvd_desc_t *rcvd);
ver_data_len_t protocol_s_get_ver_datalen(void *pack);
void protocol_s_error_info_printf(uint32_t status, char *file, int line);

#endif
