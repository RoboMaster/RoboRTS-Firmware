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

#include "protocol.h"
#include "protocol_common.h"
#include "protocol_transmit.h"
#include "protocol_log.h"

extern local_info_t protocol_local_info;

/**
  * @brief     low layer interface initialize(internal)
  * @param[in] perph: perph interface
  *            interface_name MAX:PROTOCOL_OBJ_NAME_MAX_LEN
  *            rcv_buf_size: fifo size
  *            broadcast_output_enable
  *            send_fn  send function pointer
  * @retval    error code
  */
int32_t protocol_interface_init(struct perph_interface *perph,
                                char *interface_name,
                                uint8_t broadcast_output_enable,
                                uint16_t rcv_buf_size)
{
    struct perph_interface *interface;

    uint32_t status;
    int32_t idx = PROTOCOL_INTERFACE_MAX;

    status = PROTOCOL_SUCCESS;

    /* zero is local loop interface */
    for (int i = 1; i < PROTOCOL_INTERFACE_MAX; i++)
    {
        if (protocol_local_info.interface[i].is_valid == 0)
        {
            idx = i;
            break;
        }
    }

    if (idx == PROTOCOL_INTERFACE_MAX)
    {
        status = PROTOCOL_ERR_OBJECT_NOT_FOUND;
        PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
        return status;
    }

    interface = &protocol_local_info.interface[idx];

    memcpy(interface, perph, sizeof(struct perph_interface));

    if ((interface_name != NULL) && (strlen(interface_name) < PROTOCOL_OBJ_NAME_MAX_LEN))
    {
        strncpy(interface->object_name, (const char *)interface_name, strlen(interface_name));
    }
    else
    {
        strcpy(interface->object_name, "NULL");
    }

    uint8_t *rcv_buf = protocol_p_malloc(rcv_buf_size);
    if (rcv_buf == NULL)
    {
        status = PROTOCOL_ERR_NOT_ENOUGH_MEM;
        PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
        return status;
    }
    fifo_s_init(&interface->rcvd.fifo, rcv_buf, rcv_buf_size);

    INIT_LIST_HEAD(&interface->send.normal_list_header);
    INIT_LIST_HEAD(&interface->send.ack_list_header);
    MUTEX_INIT(interface->send.mutex_lock);

    interface->broadcast_output_enable = broadcast_output_enable;
    interface->idx = idx;
    interface->is_valid = 1;

    PROTOCOL_OTHER_INFO_PRINTF("Interface %s idx=[%d] has been initialized.",
                               interface->object_name, interface->idx);

    return status;
}

/**
  * @brief     can interface init
  * @param[in]
  * @retval    error code
  */
int32_t protocol_can_interface_register(char *interface_name,
                                        uint16_t rcv_buf_size,
                                        uint8_t broadcast_output_enable,
                                        can_port_t can_port,
                                        uint32_t can_tx_id,
                                        uint32_t can_rx_id,
                                        uint32_t (*can_send_fn)(uint16_t std_id, uint8_t *p_data, uint16_t len))
{
    struct perph_interface interface = {0};
    uint32_t status;
    status = PROTOCOL_SUCCESS;

    interface.type = CAN_PORT;
    interface.send_callback.can_send_fn = can_send_fn;
    interface.user_data.can.send_id = can_tx_id;
    interface.user_data.can.rcv_id = can_rx_id;
    interface.user_data.can.port = can_port;

    status = protocol_interface_init(&interface, interface_name, broadcast_output_enable, rcv_buf_size);
    return status;
}

/**
  * @brief     uart interface init
  * @param[in]
  * @retval    error code
  */
int32_t protocol_uart_interface_register(char *interface_name,
        uint16_t rcv_buf_size,
        uint8_t broadcast_output_enable,
        com_port_t com_port,
        uint32_t (*com_send_fn)(uint8_t *p_data, uint16_t len))
{
    struct perph_interface interface = {0};
    uint32_t status;
    status = PROTOCOL_SUCCESS;

    interface.type = COM_PORT;
    interface.send_callback.com_send_fn = com_send_fn;
    interface.user_data.com.port = com_port;

    status = protocol_interface_init(&interface, interface_name, broadcast_output_enable, rcv_buf_size);
    return status;
}

/**
  * @brief     low interface send data
  * @param[in]
  * @retval    error code
  */
int32_t protocol_interface_send_data(struct perph_interface *perph, uint8_t *buff, uint16_t len)
{
    uint32_t status;
    status = PROTOCOL_SUCCESS;

    if (perph == NULL)
    {
        status = PROTOCOL_ERR_INTER_NOT_FOUND;
        PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
        return status;
    }

    if (perph->type == CAN_PORT)
    {
        if (perph->send_callback.can_send_fn != NULL)
        {
            perph->send_callback.can_send_fn(perph->user_data.can.send_id, buff, len);
        }
        else
        {
            status = PROTOCOL_ERR_INTER_NOT_FOUND;
            PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
        }
    }
    else if (perph->type == COM_PORT)
    {
        if (perph->send_callback.com_send_fn != NULL)
        {
            perph->send_callback.com_send_fn(buff, len);
        }
        else
        {
            status = PROTOCOL_ERR_INTER_NOT_FOUND;
            PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
        }
    }
    return status;
}

/**
  * @brief     can rcv data handle
  * @param[in]
  * @retval    error code
  */
uint32_t protocol_can_rcv_data(can_port_t can_port, uint32_t rcv_id, void *p_data, uint32_t data_len)
{
    uint32_t status = PROTOCOL_SUCCESS;

    for (int i = 1; i < PROTOCOL_INTERFACE_MAX; i++)
    {
        if ((protocol_local_info.interface[i].type == CAN_PORT)
                && (protocol_local_info.interface[i].user_data.can.rcv_id == rcv_id)
                && (protocol_local_info.interface[i].user_data.can.port == can_port))
        {
            protocol_rcv_data(p_data, data_len, &protocol_local_info.interface[i]);
        }
    }
    return status;
}

/**
  * @brief     uart rcv data handle
  * @param[in]
  * @retval    error code
  */
uint32_t protocol_uart_rcv_data(com_port_t com_port, void *p_data, uint32_t data_len)
{
    uint32_t status = PROTOCOL_SUCCESS;

    for (int i = 1; i < PROTOCOL_INTERFACE_MAX; i++)
    {
        if ((protocol_local_info.interface[i].type == COM_PORT)
                && (protocol_local_info.interface[i].user_data.com.port == com_port))
        {
            protocol_rcv_data(p_data, data_len, &protocol_local_info.interface[i]);
        }
    }
    return status;
}

/**
  * @brief  router binding
  * @param
  * @retval
  */
int32_t protocol_set_route(uint8_t tar_add, const char *name)
{
    uint32_t status;
    struct perph_interface *perph;
    perph = protocol_get_interface(name);

    status = PROTOCOL_SUCCESS;
    if (perph == NULL)
    {
        status = PROTOCOL_ERR_INTER_NOT_FOUND;
        PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
        return status;
    }
    if (tar_add > PROTOCOL_ROUTE_TABLE_MAX_NUM)
    {
        status = PROTOCOL_ERR_ROUTEU_SET_BEYOND;
        PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
        return status;
    }

    if (perph->is_valid == 0)
    {
        status = PROTOCOL_ERR_INTER_NOT_FOUND;
        PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
        return status;
    }

    protocol_local_info.route_table[tar_add] = perph->idx;

    PROTOCOL_OTHER_INFO_PRINTF("Route has been set, Address 0x%02X next jump is %s. index:[%d].",
                               tar_add, perph->object_name, perph->idx);

    return status;
}

struct perph_interface *protocol_get_interface(const char *name)
{
    var_cpu_sr();
    enter_critical();
    for (int i = 0; i < PROTOCOL_INTERFACE_MAX; i++)
    {
        if (strncmp(protocol_local_info.interface[i].object_name, name, strlen(name)) == 0)
        {
            exit_critical();
            return &protocol_local_info.interface[i];
        }
    }
    exit_critical();
    return NULL;
}

