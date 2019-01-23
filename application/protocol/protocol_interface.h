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

#ifndef __PROTOCOL_INTERFACE_H__
#define __PROTOCOL_INTERFACE_H__

#ifdef PROTOCOL_INTERFACE_H_GLOBAL
  #define PROTOCOL_INTERFACE_H_EXTERN 
#else
  #define PROTOCOL_INTERFACE_H_EXTERN extern
#endif

#include "fifo.h"
#include "linux_list.h"
#include "mem_mang.h"
#include "macro_mutex.h"
#include "MF_CRC.h"

#define PROTOCOL_CAN_PORT1 0
#define PROTOCOL_CAN_PORT2 1

#define PROTOCOL_USB_PORT  0
#define PROTOCOL_COM1_PORT 1

enum interface_type
{
  COM_PORT = 0,
  CAN_PORT,
  SOCKET,
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
    int32_t (*com_send_fn)(uint8_t *p_data, uint32_t len);
    int32_t (*can_send_fn)(uint32_t std_id, uint8_t *p_data, uint32_t len);
};

union interface_user_data 
{
  struct
  {
    uint32_t port;
  } com;
  struct 
  {
    uint8_t can_port;
    uint32_t send_id;
    uint32_t rcv_id;
  } can;
  struct
  {
    uint32_t address;
    uint32_t port;
  } socket;
};

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
uint32_t protocol_can_rcv_data(uint8_t can_port, uint32_t rcv_id, void *p_data, uint32_t data_len);
uint32_t protocol_uart_rcv_data(uint8_t com_port, void *p_data, uint32_t data_len);
int32_t protocol_can_interface_register(char *interface_name,
                                        uint16_t rcv_buf_size,
                                        uint8_t boardcast_output_enable,
                                        uint8_t can_port,
                                        uint32_t can_tx_id,
                                        uint32_t can_rx_id,
                                        int (*can_send_fn)(uint32_t std_id, uint8_t *p_data, uint32_t len));
int32_t protocol_uart_interface_register(char *interface_name,
                                        uint16_t rcv_buf_size,
                                        uint8_t boardcast_output_enable,
                                        uint8_t com_port,
                                        int (*com_send_fn)(uint8_t *p_data, uint32_t len));
                                        
#endif // __PROTOCOL_INTERFACE_H__
