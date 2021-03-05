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
#ifndef PROTOCOL_COMMON_H_
#define PROTOCOL_COMMON_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "protocol_cfg.h"
#include "protocol_interface.h"

/* Exported define -----------------------------------------------------------*/
/********************DEFINE COMMON************************/
#define PROTOCOL_DISENABLE (0)
#define PROTOCOL_ENABLE (1)

/********************DEFINE PACK**************************/
#define PROTOCOL_PACK_HEAD_TAIL_SIZE (sizeof(protocol_pack_desc_t) + sizeof(crc32_t))
#define PROTOCOL_PACK_HEAD_SIZE      (sizeof(protocol_pack_desc_t))
#define PROTOCOL_PACK_CMD_SIZE       (2)
#define PROTOCOL_PACK_TAIL_SIZE      (sizeof(crc32_t))
#define PROTOCOL_SEND_NODE_SIZE      (sizeof(send_list_node_t))

#define PROTOCOL_BROADCAST_ADDR      (0xFF)

/********************DEFINE ERROR**************************/
#define PROTOCOL_SUCCESS               (0u)
#define PROTOCOL_ERR_DATA_TOO_LONG     (1u)
#define PROTOCOL_ERR_NOT_ENOUGH_MEM    (2u)
#define PROTOCOL_ERR_SESSION_NOT_FOUND (3u)
#define PROTOCOL_ERR_SESSION_FULL      (4u)
#define PROTOCOL_ERR_SESSION_IS_USE    (5u)
#define PROTOCOL_ERR_ROUTE_NOT_FOUND   (6u)
#define PROTOCOL_ERR_FIFO_EMPTY        (7u)
#define PROTOCOL_ERR_AUTH_FAILURE      (8u)
#define PROTOCOL_ERR_NOT_FIND_HEADER   (9u)
#define PROTOCOL_ERR_DATA_NOT_ENOUGH   (10u)
#define PROTOCOL_ERR_FIFO_FULL         (11u)
#define PROTOCOL_ERR_OBJECT_NOT_FOUND  (12u)
#define PROTOCOL_ERR_UNSUPPORT_CPU     (13u)
#define PROTOCOL_ERR_ROUTEU_SET_BEYOND (14u)
#define PROTOCOL_ERR_INTER_NOT_FOUND   (15u)
#define PROTOCOL_ERR_PROTOCOL_NOT_INIT (16u)
#define PROTOCOL_ERR_SESSION_ERROR     (17u)
#define PROTOCOL_ERR_REGISTER_FAILED   (18u)

/* Exported types ------------------------------------------------------------*/
/********************CALLBACK TYPEDEF**********************/
typedef void (*pack_handle_fn_t)(uint8_t *pack_data, uint16_t cmd,
                                 uint8_t session, uint8_t source_add);
typedef void (*void_fn_t)(void);
typedef int32_t (*ack_handle_fn_t)(int32_t err);
typedef int32_t (*no_ack_handle_fn_t)(uint16_t cmd);
typedef int32_t (*rcv_handle_fn_t)(uint8_t *buff, uint16_t len);

struct rcv_cmd_info
{
    uint8_t used;
    uint16_t cmd;
    rcv_handle_fn_t rcv_callback;
};

struct send_cmd_info
{
    uint8_t used;
    uint16_t cmd;
    uint8_t ack_enable;
    uint8_t resend_times;    /*!< Send Times */
    uint16_t resend_timeout; /*!< Time Interval */
    ack_handle_fn_t ack_callback;
    no_ack_handle_fn_t no_ack_callback;
};

/********************FRAME STRUCT**************************/
#ifdef __CC_ARM /* for keil compiler */
    #pragma anon_unions
#endif

#pragma pack(push)
#pragma pack(1)

typedef enum
{
    PROTOCOL_PACK_NOR = 0,
    PROTOCOL_PACK_ACK = 1,
} pack_type_e;

typedef struct
{
    uint16_t data_len : 10; /*!< Data Length, Exclude Header And Crc16 */
    uint16_t version : 6;   /*!< Protocol Version */
} ver_data_len_t;

typedef struct
{
    uint8_t session : 5;   /*!< Need(0~1) Or Not Need(2~31) Ack */
    uint8_t pack_type : 1; /*!< Ack Package Or Normal Package */
    uint8_t res : 2;       /*!< Reserve */
} S_A_R_t;

/* This Struct Is Used To Describe A Package Header */
typedef struct
{
    uint8_t sof; /*!< Identify Of A Package */
    union
    {
        struct
        {
            uint16_t data_len : 10; /*!< Data Length, Include Header And Crc */
            uint16_t version : 6;   /*!< Protocol Version */
        };
        uint16_t ver_data_len;
    };
    union
    {
        struct
        {
            uint8_t session : 5;   /*!< Need(0~1) Or Not Need(2~63) Ack */
            uint8_t pack_type : 1; /*!< Ack Package Or Normal Package */
            uint8_t res : 2;       /*!< Reserve */
        };
        uint8_t S_A_R_c;
    };
    uint8_t sender;   /*!< Sender Module Information */
    uint8_t reciver;  /*!< Receiver Module Information */
    uint16_t res1;    /*!< Reserve 1 */
    uint16_t seq_num; /*!< Sequence Number */
    uint16_t crc_16;  /*!< CRC16 */
    uint8_t pdata[];
} protocol_pack_desc_t;

typedef uint32_t crc32_t;

#pragma pack(pop)

#define PACK_HEADER_TAIL_LEN (16 + 2)

/********************PROTOCL INFO STRUCT********************/
typedef enum
{
    UNPACK_PACK_STAGE_FIND_SOF = 0,
    UNPACK_PACK_STAGE_AUTH_HEADER = 1,
    UNPACK_PACK_STAGE_RECV_DATA = 2,
    UNPACK_PACK_STAGE_AUTH_PACK = 3,
    UNPACK_PACK_STAGE_DATA_HANDLE = 4,
} unpack_status_e;

typedef struct
{
    list_t send_list_header; /*!< Send List Header */
    uint8_t send_node_num;   /*!< Current Node Num In Send List */
    uint8_t is_valid;        /*!< Valid */
    MUTEX_DECLARE(mutex_lock);
} broadcast_object_t;

typedef struct send_list_node
{
    list_t send_list;                        /*!< Support List */
    uint8_t *p_data;                         /*!< Pointer To Data Include Pack Header */
    uint16_t len;                            /*!< Length Of Data Include Pack Header */
    uint8_t seq;                             /*!< Sequence Number */
    uint8_t is_got_ack;                      /*!< Is Got Ack */
    uint8_t is_ready_realse;                 /*!< Is Ready Realse */
    uint8_t session;                         /*!< Session Number */
    uint8_t address;                         /*!< Address */
    uint16_t cmd;                            /*!< CMD */
    uint8_t pack_type;                       /*!< pack_type */
    uint8_t rest_cnt;                        /*!< The Remaining Number Of Transmissions */
    uint16_t timeout;                        /*!< Time Interval Between Each Transmissions*/
    uint32_t pre_timestamp;                  /*!< Last Sent Timestamp */
    uint8_t is_first_send;                   /*!< Is First Send */
    struct perph_interface *forward_src_obj; /*!< Foward Src Interface Object */
    ack_handle_fn_t ack_callback;
    no_ack_handle_fn_t no_ack_callback;
} send_list_node_t;

typedef struct
{
    uint8_t address; /*!< Locol Address */

    uint8_t route_table[PROTOCOL_ROUTE_TABLE_MAX_NUM];
    /*!< Route Table */

    pack_handle_fn_t rcv_nor_callBack; /*!< Rcv Nor CallBack */
    void_fn_t send_list_add_callBack;  /*!< Send List Add CallBack */

    struct rcv_cmd_info rcv_cmd_info[PROTOCOL_RECV_CMD_MAX_NUM];
    struct send_cmd_info send_cmd_info[PROTOCOL_SEND_CMD_MAX_NUM];

    struct perph_interface interface[PROTOCOL_INTERFACE_MAX];

    uint8_t is_valid; /*!< Valid */
    MUTEX_DECLARE(mutex_lock);
} local_info_t;

/* This Struct Is Used To Describe Send Information When Sending */
typedef struct
{
    uint16_t version; /*!< Version */
    uint8_t reciver;  /*!< Receiver Receiver Information */
    union
    {
        S_A_R_t s_a_r;
        uint8_t S_A_R_c; /*!< Ack And Session */
    };
} send_ctx_t;

/* Exported functions --------------------------------------------------------*/
void *protocol_p_malloc(uint32_t size);
void protocol_p_free(void *ptr);
uint32_t protocol_p_get_time(void);
void protocol_p_printf(const char *format, ...);

#endif /* PROTOCOL_COMMON_H_ */
