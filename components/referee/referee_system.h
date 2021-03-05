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

#ifndef __REFEREE_SYSTEM_H__
#define __REFEREE_SYSTEM_H__

#include "sys.h"
#include "fifo.h"
#define REFEREE_FIFO_BUFLEN 500

typedef void (*ref_send_handler_t)(uint8_t *buf, uint16_t len);
typedef void (*ref_rx_complete_callabck_t)(uint8_t *buf, uint16_t len);
typedef void (*ref_cmd_callback_t)(uint16_t cmd_id, uint8_t *pdata, uint16_t len);

#define REF_PROTOCOL_HEADER                 0xA5
#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#define REF_PROTOCOL_FRAME_MAX_SIZE         128
#define REF_PROTOCOL_CMD_MAX_NUM            20

#define REF_USER_TO_SERVER_MAX_DATA_LEN     64
#define REF_SERVER_TO_USER_MAX_DATA_LEN     32

#pragma pack(push,1)

typedef struct
{
    uint8_t  sof;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  crc8;
} frame_header_t;

#pragma pack(pop)

typedef enum
{
    STEP_HEADER_SOF  = 0,
    STEP_LENGTH_LOW  = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ   = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
    fifo_s_t       *data_fifo;
    frame_header_t *p_header;
    uint16_t       data_len;
    uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
    unpack_step_e  unpack_step;
    uint16_t       index;
} unpack_data_t;

void referee_param_init(ref_send_handler_t send_t,
                        ref_rx_complete_callabck_t rx_callback,
                        ref_cmd_callback_t cmd_callback);
void     referee_unpack_fifo_data(void);
uint32_t referee_uart_rx_data_handle(uint8_t *data, uint16_t len);
void     referee_protocol_tansmit(uint16_t cmd_id, void *p_buf, uint16_t len);

#endif // __REFEREE_SYSTEM_H__
