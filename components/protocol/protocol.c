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

/* Includes ------------------------------------------------------------------*/
#include "protocol.h"
#include "protocol_transmit.h"
#include "protocol_cfg.h"
#include "protocol_log.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

broadcast_object_t broadcast_object;
local_info_t protocol_local_info =
{
    .is_valid = 0
};

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief     get send node param
  * @param[in] cmd code
  * @retval    send_cmd_info pointer
  */
struct send_cmd_info *protocol_get_send_cmd_info(uint16_t cmd)
{
    for (int i = 0; i < PROTOCOL_SEND_CMD_MAX_NUM; i++)
    {
        if ((protocol_local_info.send_cmd_info[i].cmd == cmd) && (protocol_local_info.send_cmd_info[i].used == 1))
        {
            return &protocol_local_info.send_cmd_info[i];
        }
    }
    return NULL;
}

/**
  * @brief     protocol data handle(internal)
  * @param[in]
  * @retval    void
  */
static void protocol_rcv_pack_handle(uint8_t *pack_data, uint16_t cmd, uint8_t session, uint8_t source_add)
{
    protocol_pack_desc_t *pack;
    uint16_t rcv_seq;
    int32_t err;
    pack = (protocol_pack_desc_t *)(pack_data);
    rcv_seq = pack->seq_num;

    for (int i = 0; i < PROTOCOL_RECV_CMD_MAX_NUM; i++)
    {
        if ((protocol_local_info.rcv_cmd_info[i].cmd == cmd) && (protocol_local_info.rcv_cmd_info[i].rcv_callback != NULL) && (protocol_local_info.rcv_cmd_info[i].used == 1))
        {
            err = protocol_local_info.rcv_cmd_info[i].rcv_callback(pack->pdata + 2, pack->data_len - PACK_HEADER_TAIL_LEN);
            if (session != 0)
            {
                protocol_ack(source_add, session, &err, sizeof(err), rcv_seq);
            }
        }
    }

    return;
}

/**
  * @brief     register a cmd callback function.
  * @param[in] cmd code
  * @param[in] callback
  * @retval    error code
  */
int32_t protocol_rcv_cmd_register(uint16_t cmd, rcv_handle_fn_t rcv_callback)
{
    for (int i = 0; i < PROTOCOL_RECV_CMD_MAX_NUM; i++)
    {
        if (protocol_local_info.rcv_cmd_info[i].used == 0)
        {
            protocol_local_info.rcv_cmd_info[i].used = 1;
            protocol_local_info.rcv_cmd_info[i].cmd = cmd;
            protocol_local_info.rcv_cmd_info[i].rcv_callback = rcv_callback;
            PROTOCOL_OTHER_INFO_PRINTF("Receive function register. cmd: 0x%04x", cmd);
            return 0;
        }
    }
    PROTOCOL_ERR_INFO_PRINTF(PROTOCOL_ERR_REGISTER_FAILED, __FILE__, __LINE__);
    return -1;
}

/**
  * @brief
  * @param[in] send node configration
  * @param[in] callback
  * @retval    error code
  */
int32_t protocol_send_cmd_config(uint16_t cmd,
                                 uint8_t resend_times,
                                 uint16_t resend_timeout,
                                 uint8_t ack_enable,
                                 ack_handle_fn_t ack_callback,
                                 no_ack_handle_fn_t no_ack_callback)
{
    for (int i = 0; i < PROTOCOL_SEND_CMD_MAX_NUM; i++)
    {
        if (protocol_local_info.send_cmd_info[i].used == 0)
        {
            protocol_local_info.send_cmd_info[i].used = 1;
            protocol_local_info.send_cmd_info[i].cmd = cmd;
            protocol_local_info.send_cmd_info[i].resend_times = resend_times;
            protocol_local_info.send_cmd_info[i].resend_timeout = resend_timeout;
            protocol_local_info.send_cmd_info[i].ack_enable = ack_enable;
            protocol_local_info.send_cmd_info[i].ack_callback = ack_callback;
            protocol_local_info.send_cmd_info[i].no_ack_callback = no_ack_callback;
            PROTOCOL_OTHER_INFO_PRINTF("send cmd comfigration. cmd: 0x%04x, resend times:%d timeout: %d", cmd, resend_times, resend_timeout);
            return 0;
        }
    }
    PROTOCOL_ERR_INFO_PRINTF(PROTOCOL_ERR_REGISTER_FAILED, __FILE__, __LINE__);
    return -1;
}

/**
  * @brief     local address init
  * @param[in]
  * @retval    error code
  */
void protocol_set_local_address(uint8_t address)
{
    protocol_local_info.address = address;
    PROTOCOL_OTHER_INFO_PRINTF("Local address: %d", address);
}

/**
  * @brief     local objects initialize
  * @param[in] module address, this address is unique in a same network.
  * @retval    error code
  */
uint32_t protocol_local_object_init(void)
{

    uint32_t status;

    status = PROTOCOL_SUCCESS;

    /* little endian */
    const uint16_t endian_test = 0xAABB;
    if (*((uint8_t *)(&endian_test)) == 0xAA)
    {
        /* big endian is not support */
        status = PROTOCOL_ERR_UNSUPPORT_CPU;
        PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);

        while (1)
            ;
    }

    MUTEX_INIT(protocol_local_info.mutex_lock);

    memset(protocol_local_info.route_table, 0xFF, PROTOCOL_ROUTE_TABLE_MAX_NUM);

#if (PROTOCOL_AUTO_LOOPBACK == PROTOCOL_ENABLE)
    struct perph_interface *interface;

    interface = &protocol_local_info.interface[0];

    memset(interface, 0, sizeof(struct perph_interface));

    strcpy(interface->object_name, "Local Loop Interface");

    uint8_t *rcv_buf = protocol_p_malloc(PROTOCOL_LOOPBACK_SIZE);
    if (rcv_buf == NULL)
    {
        status = PROTOCOL_ERR_NOT_ENOUGH_MEM;
        PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
        return status;
    }
    fifo_s_init(&interface->rcvd.fifo, rcv_buf, PROTOCOL_LOOPBACK_SIZE);

    INIT_LIST_HEAD(&interface->send.normal_list_header);
    INIT_LIST_HEAD(&interface->send.ack_list_header);
    MUTEX_INIT(interface->send.mutex_lock);

    interface->broadcast_output_enable = 0;
    interface->idx = 0;
    interface->is_valid = 1;
#endif

    for (int i = 0; i < PROTOCOL_SEND_CMD_MAX_NUM; i++)
    {
        /* initalization cmd is 0xFFFF */
        memset(&protocol_local_info.send_cmd_info[i].cmd, 0xFFFF, 2);
    }

    for (int i = 0; i < PROTOCOL_RECV_CMD_MAX_NUM; i++)
    {
        /* initalization cmd is 0xFFFF */
        memset(&protocol_local_info.rcv_cmd_info[i].cmd, 0xFFFF, 2);
    }

    protocol_local_info.address = 0xFE;
    protocol_local_info.rcv_nor_callBack = protocol_rcv_pack_handle;

    MUTEX_INIT(broadcast_object.mutex_lock);
    INIT_LIST_HEAD(&broadcast_object.send_list_header);
    broadcast_object.is_valid = 1;
    protocol_local_info.is_valid = 1;
    PROTOCOL_OTHER_INFO_PRINTF("Local objects has been initialized.");

    return status;
}

/**
  * @brief     send a normal frame.
  * @param[in] reciver
  *            session 0:no ack 1~31: need ack
  *  @retval   error code
  */
uint32_t protocol_send(uint8_t reciver, uint16_t cmd, void *p_data, uint32_t data_len)
{
    uint32_t status;
    uint8_t session = 0;
    uint8_t ack = 0;

    struct send_cmd_info *cmd_info;
    cmd_info = protocol_get_send_cmd_info(cmd);

    struct perph_interface *int_obj;
    int_obj = protocol_s_get_route(reciver);

    if (cmd_info != NULL)
    {
        ack = cmd_info->ack_enable;
    }

    if (reciver == PROTOCOL_BROADCAST_ADDR)
    {
        status = protocol_s_broadcast_add_node(p_data, data_len, cmd);
    }
    else
    {
        if (ack == 1)
        {
            session = protocol_get_session(int_obj);
        }
        status = protocol_s_add_sendnode(reciver, session, PROTOCOL_PACK_NOR, p_data,
                                         data_len, cmd, 0);
    }
    if (status == PROTOCOL_SUCCESS)
    {
        if (protocol_local_info.send_list_add_callBack != NULL)
        {
            protocol_local_info.send_list_add_callBack();
        }
    }
    else
    {
        if (ack == 1)
        {
            protocol_release_session(int_obj, session);
        }
    }
    return status;
}

/**
  * @brief     send a ack frame when session is not 0. (internal)
  * @param[in] reciver address
  *            session: this function is called when receiver session is not zero.
  * @retval    error code
  */
uint32_t protocol_ack(uint8_t reciver, uint8_t session, void *p_data, uint32_t data_len, uint16_t ack_seq)
{
    uint32_t status;
    status = protocol_s_add_sendnode(reciver, session, PROTOCOL_PACK_ACK, p_data,
                                     data_len, 0, ack_seq);
    if (status == PROTOCOL_SUCCESS)
    {
        if (protocol_local_info.send_list_add_callBack != NULL)
        {
            protocol_local_info.send_list_add_callBack();
        }
    }
    return status;
}

/**
  * @brief  this function should be called after sending data or periodic.
  * @param  void
  * @retval error code
  */
uint32_t protocol_send_flush(void)
{
    for (uint8_t i = 0; i < PROTOCOL_INTERFACE_MAX; i++)
    {
        if (protocol_local_info.interface[i].is_valid)
        {
            if (protocol_local_info.interface[i].send.normal_node_num > 0)
            {
                protocol_s_interface_normal_send_flush(protocol_local_info.interface + i);
            }
            if (protocol_local_info.interface[i].send.ack_node_num > 0)
            {
                protocol_s_interface_ack_send_flush(protocol_local_info.interface + i);
            }
        }
    }

    if (broadcast_object.is_valid)
    {
        if (broadcast_object.send_node_num > 0)
        {
            protocol_s_broadcast_send_flush();
        }
    }
    return 0;
}

/**
  * @brief  this function should be called after receiving data or periodic.
  * @param  void
  * @retval 0
  */
uint32_t protocol_unpack_flush(void)
{
    for (uint8_t i = 0; i < PROTOCOL_INTERFACE_MAX; i++)
    {
        if (protocol_local_info.interface[i].is_valid)
        {
            protocol_s_extract(&(protocol_local_info.interface[i]));
        }
    }
    return 0;
}

/**
  * @brief     receive data from different perphs
  * @param[in] perph interface index
  * @retval    error code
  */
uint32_t protocol_rcv_data(void *p_data, uint32_t data_len, struct perph_interface *perph)
{
    struct perph_interface *obj;
    uint32_t rcv_length;
    uint32_t status;

    //Interrupt Off;

    status = PROTOCOL_SUCCESS;

    if (protocol_local_info.is_valid == 0)
    {
        status = PROTOCOL_ERR_PROTOCOL_NOT_INIT;
        //Interrupt On
        return status;
    }

    obj = &(protocol_local_info.interface[perph->idx]);

    rcv_length = fifo_s_puts_noprotect(&(obj->rcvd.fifo), p_data, data_len);

    if (rcv_length < data_len)
    {
        status = PROTOCOL_ERR_FIFO_FULL;
        PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
    }
    //Interrupt On

    return status;
}

/**
  * @brief     callback after completing send task.
  * @param[in] pack_handle_fn_t
  * @retval    0
  */
uint32_t protocol_send_list_add_callback_reg(void_fn_t fn)
{
    protocol_local_info.send_list_add_callBack = fn;
    return 0;
}
