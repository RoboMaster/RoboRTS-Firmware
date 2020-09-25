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

#ifndef __PROTOCOL_INTERFACE_H__
#define __PROTOCOL_INTERFACE_H__

#include "fifo.h"
#include "linux_list.h"
#include "mem_mang.h"
#include "mf_crc.h"

typedef enum
{
    INVAILD_COM = 0,
    USB_COM,
    UART1_PORT,
    UART2_PORT,
    UART3_PORT,
    UART4_PORT,
    UART5_PORT,
    UART6_PORT,
    UART7_PORT,
} com_port_t;

typedef enum
{
    INVAILD_CAN = 0,
    CAN1_PORT,
    CAN2_PORT,
} can_port_t;

enum interface_type
{
    INVAILD_PORT = 0,
    COM_PORT = 1,
    CAN_PORT,
};

typedef struct
{
    fifo_s_t fifo;         /*!< Receive Buffer */
    uint8_t *p_data;       /*!< Pointer To A Temp Memory When Unpack */
    uint16_t rcvd_num;     /*!< The Length Of Data That Has Been Received */
    uint16_t total_num;    /*!< The Total Data Length Of Current Package */
    uint8_t state;         /*!< Current Unpack state */
    uint16_t last_rcv_seq; /*!< Last Recvice Seq Num */
    uint32_t last_rcv_crc; /*!< Last Recvice CRC Num */
} rcvd_desc_t;

typedef struct
{
    list_t normal_list_header; /*!< Noramal Pack List Header */
    list_t ack_list_header;    /*!< Ack Pack List Header */
    uint16_t send_seq;         /*!< Send Sequence */
    uint8_t normal_node_num;   /*!< Current Node Num In Normal List */
    uint8_t ack_node_num;      /*!< Current Node Num In Ack List */
    MUTEX_DECLARE(mutex_lock);
} send_desc_t;

/* Pointer to send function */
union interface_send_fn_u
{
    void *fun;
    uint32_t (*com_send_fn)(uint8_t *p_data, uint16_t len);
    uint32_t (*can_send_fn)(uint16_t std_id, uint8_t *p_data, uint16_t len);
};

union interface_user_data
{
    struct
    {
        com_port_t port;
    } com;
    struct
    {
        can_port_t port;
        uint32_t send_id;
        uint32_t rcv_id;
    } can;
} __attribute__((aligned(4)));

struct perph_interface
{
    char object_name[PROTOCOL_OBJ_NAME_MAX_LEN]; /*!< Name Of Object */
    rcvd_desc_t rcvd;                            /*!< Receive Describe */
    send_desc_t send;                            /*!< About Send Describe */
    uint8_t idx;                                 /*!< interface */
    uint8_t is_valid;                            /*!< Valid */
    uint8_t broadcast_output_enable;             /*!< Broadcast Output Enable */
    uint8_t session[31];
    enum interface_type type;

    union interface_send_fn_u send_callback;
    union interface_user_data user_data;
};

int32_t protocol_set_route(uint8_t tar_add, const char *name);
struct perph_interface *protocol_get_interface(const char *name);
int32_t protocol_interface_send_data(struct perph_interface *perph, uint8_t *buff, uint16_t len);
uint32_t protocol_can_rcv_data(can_port_t can_port, uint32_t rcv_id, void *p_data, uint32_t data_len);
uint32_t protocol_uart_rcv_data(com_port_t com_port, void *p_data, uint32_t data_len);
int32_t protocol_can_interface_register(char *interface_name,
                                        uint16_t rcv_buf_size,
                                        uint8_t broadcast_output_enable,
                                        can_port_t can_port,
                                        uint32_t can_tx_id,
                                        uint32_t can_rx_id,
                                        uint32_t (*can_send_fn)(uint16_t std_id, uint8_t *p_data, uint16_t len));
int32_t protocol_uart_interface_register(char *interface_name,
        uint16_t rcv_buf_size,
        uint8_t broadcast_output_enable,
        com_port_t com_port,
        uint32_t (*com_send_fn)(uint8_t *p_data, uint16_t len));

#endif // __PROTOCOL_INTERFACE_H__
