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

/* Includes ------------------------------------------------------------------*/
#include "protocol.h"
#include "protocol_transmit.h"
#include "protocol_log.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern local_info_t protocol_local_info;
extern boardcast_object_t boardcast_object;

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

uint8_t protocol_get_session(struct perph_interface * interface)
{
  for (int i = 0; i < 31; i++)
  {
    if (interface->session[i] == 0)
    {
      interface->session[i] = 1;
      return i + 1;
    }
  }
  return 0;
}

int32_t protocol_release_session(struct perph_interface * interface, uint8_t id)
{
  if ((id > 0) && (id < 32))
  {
    interface->session[id - 1] = 0;
    return 0;
  }
  return -1;
}

//添加协议帧
uint32_t protocol_s_add_sendnode(uint8_t reciver, uint8_t session, uint8_t pack_type,
                                 void *p_data, uint32_t data_len, uint16_t cmd, uint16_t ack_seq)
{
  send_ctx_t ctx = {0};
  struct perph_interface *int_obj;
  uint32_t status;
  uint32_t malloc_size;
  uint8_t *malloc_zone;
  uint32_t pack_head_offset;
  protocol_pack_desc_t *pack_head;
  send_list_node_t *send_node;
  uint16_t seq;

  status = PROTOCOL_SUCCESS;

  if (data_len > PROTOCOL_MAX_DATA_LEN)
  {
    status = PROTOCOL_ERR_DATA_TOO_LONG;
    PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
    return status;
  }

  //配置发送参数
  ctx.s_a_r.pack_type = pack_type;
  ctx.s_a_r.session = session;
  ctx.s_a_r.res = 0;
  ctx.reciver = reciver;
  ctx.version = PROTOCOL_VERSION;

  //获取路由接口
  int_obj = protocol_s_get_route(reciver);

  if (int_obj == NULL)
  {
    status = PROTOCOL_ERR_ROUTE_NOT_FOUND;
    PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);

    return status;
  }

  if ((pack_type == PROTOCOL_PACK_NOR) && (session != 0))
  {
    if (protocol_s_session_get_node(int_obj, reciver, session) != NULL)
    {
      status = PROTOCOL_ERR_SESSION_IS_USE;
      PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
      return status;
    }
  }

  //分配数据帧所需内存
  if (pack_type == PROTOCOL_PACK_ACK)
  {
    malloc_size = PROTOCOL_PACK_HEAD_TAIL_SIZE + PROTOCOL_SEND_NODE_SIZE +
                  data_len;
  }
  else
  {
    malloc_size = PROTOCOL_PACK_HEAD_TAIL_SIZE + PROTOCOL_SEND_NODE_SIZE +
                  data_len + PROTOCOL_PACK_CMD_SIZE;
  }
  malloc_zone = protocol_p_malloc(malloc_size);
  if (malloc_zone == NULL)
  {
    status = PROTOCOL_ERR_NOT_ENOUGH_MEM;
    PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
    return status;
  }

  if (pack_type == PROTOCOL_PACK_NOR)
  {
    MUTEX_LOCK(int_obj->send.mutex_lock);
    seq = int_obj->send.send_seq++;
    MUTEX_UNLOCK(int_obj->send.mutex_lock);
  }
  else
  {
    seq = ack_seq;
  }

  pack_head_offset = PROTOCOL_SEND_NODE_SIZE;
  pack_head = (protocol_pack_desc_t *)&malloc_zone[pack_head_offset];
  send_node = (send_list_node_t *)&malloc_zone[0];

  //填充帧数据部分
  protocol_s_fill_pack(&ctx, p_data, data_len, (uint8_t *)(pack_head), seq, cmd);

  //填充send_node
  send_node->session = ctx.s_a_r.session;
  send_node->p_data = &malloc_zone[pack_head_offset];
  send_node->len = malloc_size - PROTOCOL_SEND_NODE_SIZE;
  send_node->pre_timestamp = 0;
  send_node->is_got_ack = 0;
  send_node->is_first_send = 1;
  send_node->address = reciver;
  send_node->pack_type = pack_type;
  send_node->is_ready_realse = 0;
  send_node->cmd = cmd;
  send_node->forward_src_obj = NULL;

  struct send_cmd_info *cmd_info;
  cmd_info = protocol_get_send_cmd_info(cmd);
  if (cmd_info != NULL)
  {
    send_node->rest_cnt = cmd_info->resend_times;
    send_node->timeout = cmd_info->resend_timeout;
    send_node->ack_callback = cmd_info->ack_callback;
    send_node->no_ack_callback = cmd_info->no_ack_callback;
  }
  else
  {
    send_node->rest_cnt = 1;
    send_node->timeout = 0;
    send_node->ack_callback = NULL;
    send_node->no_ack_callback = NULL;
  }

  //添加至发送链表
  MUTEX_LOCK(int_obj->send.mutex_lock);

  if ((pack_type == PROTOCOL_PACK_NOR) && (session != 0))
  {
    if (protocol_s_session_get_node(int_obj, reciver, session) != NULL)
    {
      status = PROTOCOL_ERR_SESSION_IS_USE;
      MUTEX_UNLOCK(int_obj->send.mutex_lock);
      PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
      return status;
    }
  }

  if (pack_type == PROTOCOL_PACK_NOR)
  {
    list_add(&(send_node->send_list), &(int_obj->send.normal_list_header));
    int_obj->send.normal_node_num++;
  }
  else
  {
    list_add(&(send_node->send_list), &(int_obj->send.ack_list_header));
    int_obj->send.ack_node_num++;
  }

  MUTEX_UNLOCK(int_obj->send.mutex_lock);

  if (pack_type == PROTOCOL_PACK_NOR)
  {
      PROTOCOL_SEND_DBG_PRINTF("Send pack, Address:0x%02X, Cmd:0x%04X, Session: %d Normal pack.",
                              reciver, cmd, session);
  }
  else
  {
    PROTOCOL_SEND_DBG_PRINTF("Send pack, Address:0x%02X, Session: %d Ack pack.",
                              reciver, session);
  }

  return status;
}

//广播包添加处理函数
uint32_t protocol_s_broadcast_add_node(void *p_data, uint32_t data_len, uint16_t cmd)
{
  send_ctx_t ctx;
  uint32_t status;
  uint32_t malloc_size;
  uint8_t *malloc_zone;
  uint32_t pack_head_offset;
  protocol_pack_desc_t *pack_head;
  send_list_node_t *send_node;

  status = PROTOCOL_SUCCESS;

  if (data_len > PROTOCOL_MAX_DATA_LEN)
  {
    status = PROTOCOL_ERR_DATA_TOO_LONG;
    PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
    return status;
  }

  //配置发送参数
  ctx.s_a_r.pack_type = PROTOCOL_PACK_NOR;
  ctx.s_a_r.session = 0;
  ctx.s_a_r.res = 0;
  ctx.reciver = PROTOCOL_BROADCAST_ADDR;
  ctx.version = PROTOCOL_VERSION;

  malloc_size = PROTOCOL_PACK_HEAD_TAIL_SIZE + PROTOCOL_SEND_NODE_SIZE +
                data_len + PROTOCOL_PACK_CMD_SIZE;

  malloc_zone = protocol_p_malloc(malloc_size);
  if (malloc_zone == NULL)
  {
    status = PROTOCOL_ERR_NOT_ENOUGH_MEM;
    PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
    return status;
  }

  pack_head_offset = PROTOCOL_SEND_NODE_SIZE;
  pack_head = (protocol_pack_desc_t *)&malloc_zone[pack_head_offset];
  send_node = (send_list_node_t *)&malloc_zone[0];

  //填充帧数据部分
  protocol_s_fill_pack(&ctx, p_data, data_len, (uint8_t *)(pack_head), 0, cmd);

  //填充send_node
  send_node->session = 0;
  send_node->p_data = &malloc_zone[pack_head_offset];
  send_node->len = malloc_size - PROTOCOL_SEND_NODE_SIZE;
  send_node->rest_cnt = 1;
  send_node->pre_timestamp = 0;
  send_node->timeout = 0;
  send_node->is_got_ack = 0;
  send_node->is_first_send = 1;
  send_node->address = PROTOCOL_BROADCAST_ADDR;
  send_node->pack_type = PROTOCOL_PACK_NOR;
  send_node->is_ready_realse = 0;
  send_node->cmd = cmd;
  send_node->forward_src_obj = NULL;

  //添加至发送链表
  MUTEX_LOCK(boardcast_object.mutex_lock);

  list_add(&(send_node->send_list), &(boardcast_object.send_list_header));
  boardcast_object.send_node_num++;

  MUTEX_UNLOCK(boardcast_object.mutex_lock);

  PROTOCOL_SEND_DBG_PRINTF("Send broadcast pack, Cmd:0x%04X, Normal pack.", cmd);

  return status;
}

//帧填充
uint32_t protocol_s_fill_pack(send_ctx_t *ctx, uint8_t *p_data,
                              uint32_t data_len, uint8_t *pack_zone, uint16_t seq, uint16_t cmd)
{
  uint32_t status = 0;
  protocol_pack_desc_t *p_pack_head;

  p_pack_head = (protocol_pack_desc_t *)pack_zone;

  /* get local module */

  p_pack_head->sof = PROTOCOL_HEADER;
  p_pack_head->version = ctx->version;
  p_pack_head->sender = protocol_local_info.address;
  p_pack_head->reciver = ctx->reciver;
  p_pack_head->S_A_R_c = ctx->S_A_R_c;
  p_pack_head->seq_num = seq;
  p_pack_head->ver_data_len = p_pack_head->ver_data_len;
  p_pack_head->res1 = 0;

  /* cpy data */
  if (ctx->s_a_r.pack_type == PROTOCOL_PACK_ACK)
  {
    p_pack_head->data_len = data_len + PROTOCOL_PACK_HEAD_TAIL_SIZE;
    memcpy(pack_zone + PROTOCOL_PACK_HEAD_SIZE, p_data, data_len);
  }
  else
  {
    p_pack_head->data_len = data_len + PROTOCOL_PACK_HEAD_TAIL_SIZE + PROTOCOL_PACK_CMD_SIZE;
    *((uint16_t *)(pack_zone + PROTOCOL_PACK_HEAD_SIZE)) = cmd;
    memcpy(pack_zone + PROTOCOL_PACK_HEAD_SIZE + PROTOCOL_PACK_CMD_SIZE, p_data, data_len);
  }

  /* crc */
  append_crc16(pack_zone, 12);

  if (ctx->s_a_r.pack_type == PROTOCOL_PACK_ACK)
  {
    append_crc32(pack_zone, data_len + PROTOCOL_PACK_HEAD_TAIL_SIZE);
  }
  else
  {
    append_crc32(pack_zone, data_len + PROTOCOL_PACK_HEAD_TAIL_SIZE + PROTOCOL_PACK_CMD_SIZE);
  }

  return status;
}

//通过接口发送数据
uint32_t protocol_s_interface_send_data(send_list_node_t *cur_send_node, struct perph_interface *obj)
{

#if (PROTOCOL_AUTO_LOOKBACK == PROTOCOL_ENABLE)

  if (cur_send_node->address != protocol_local_info.address)
  {
    //发送地址与本地地址不相同，外发
    protocol_interface_send_data(obj, cur_send_node->p_data, cur_send_node->len);
  }
  else
  {
    //发送地址与本地地址相同，直接内部回环

    protocol_rcv_data(cur_send_node->p_data, cur_send_node->len, &protocol_local_info.interface[0]);

    PROTOCOL_OTHER_INFO_PRINTF("Reciver is local, Loop back.");
  }

#else
  if (obj->send.send_fn != NULL)
  {
    (*(obj->send.send_fn))(cur_send_node->p_data,
                           cur_send_node->len,
                           obj->interface);
  }
#endif

  return PROTOCOL_SUCCESS;
}

//清空发送列表
uint32_t protocol_s_interface_normal_send_flush(struct perph_interface *obj)
{
  list_t *head_node;
  list_t *cur_node;
  list_t *store_list;
  send_list_node_t *cur_send_node;
  uint32_t timeout;

  head_node = &(obj->send.normal_list_header);
  list_for_each_prev_safe(cur_node, store_list, head_node)
  {

    cur_send_node = (send_list_node_t *)cur_node;

    MUTEX_LOCK(obj->send.mutex_lock);
    //得到ACK，删除
    if (cur_send_node->is_got_ack)
    {
      list_del(cur_node);
      protocol_p_free(cur_send_node);
      obj->send.normal_node_num--;

      protocol_release_session(obj, cur_send_node->session);

      MUTEX_UNLOCK(obj->send.mutex_lock);
      continue;
    }

    //超过重发次数释放
    if (cur_send_node->is_ready_realse)
    {
      list_del(cur_node);
      obj->send.normal_node_num--;

      if (cur_send_node->no_ack_callback != NULL)
      {
        cur_send_node->no_ack_callback(cur_send_node->cmd);
      }

      protocol_release_session(obj, cur_send_node->session);

      protocol_p_free(cur_send_node);

      MUTEX_UNLOCK(obj->send.mutex_lock);
      continue;
    }
    else
    {
      MUTEX_UNLOCK(obj->send.mutex_lock);
    }

    timeout = protocol_p_get_time() - cur_send_node->pre_timestamp;

    //超时重发或者初次发送
    if ((timeout > cur_send_node->timeout || cur_send_node->is_first_send) &&
        cur_send_node->rest_cnt >= 1)
    {
      cur_send_node->is_first_send = 0;

      cur_send_node->rest_cnt--;

      //发送数据
      protocol_s_interface_send_data(cur_send_node, obj);

      if (cur_send_node->session == 0)
      {
        //session为0,不需要重发和ACK回复
        MUTEX_LOCK(obj->send.mutex_lock);
        list_del(cur_node);
        obj->send.normal_node_num--;
        protocol_p_free(cur_send_node);
        MUTEX_UNLOCK(obj->send.mutex_lock);
        continue;
      }
      else
      {
        //session不为0,需要重发和ACK确认
        if (cur_send_node->rest_cnt == 0)
        {
          //发送次数用完
          cur_send_node->is_ready_realse = 1;
        }
        else
        {
          cur_send_node->pre_timestamp = protocol_p_get_time();
        }
      }
    }
  }

  return 0;
}

//清空ACK帧发送列表
uint32_t protocol_s_interface_ack_send_flush(struct perph_interface *obj)
{
  list_t *head_node;
  list_t *cur_node;
  list_t *store_list;
  send_list_node_t *cur_send_node;

  head_node = &(obj->send.ack_list_header);
  list_for_each_prev_safe(cur_node, store_list, head_node)
  {

    cur_send_node = (send_list_node_t *)cur_node;

    protocol_s_interface_send_data(cur_send_node, obj);

    //包为ACK类型,不需要重发和确认
    MUTEX_LOCK(obj->send.mutex_lock);
    list_del(cur_node);
    obj->send.ack_node_num--;
    protocol_p_free(cur_send_node);
    MUTEX_UNLOCK(obj->send.mutex_lock);
  }

  return 0;
}

//清空广播包发送列表
uint32_t protocol_s_broadcast_send_flush(void)
{
  list_t *head_node;
  list_t *cur_node;
  list_t *store_list;
  send_list_node_t *cur_send_node;

  head_node = &(boardcast_object.send_list_header);
  list_for_each_prev_safe(cur_node, store_list, head_node)
  {
    cur_send_node = (send_list_node_t *)cur_node;

    for (uint8_t i = 0; i < PROTOCOL_INTERFACE_MAX; i++)
    {
      if (cur_send_node->forward_src_obj == protocol_local_info.interface + i)
        continue;
      if (!cur_send_node->forward_src_obj->is_valid)
        continue;
      if (!protocol_local_info.interface[i].broadcast_output_enable)
        continue;

      protocol_s_interface_send_data(cur_send_node, protocol_local_info.interface + i);
    }

    MUTEX_LOCK(boardcast_object.mutex_lock);
    list_del(cur_node);
    boardcast_object.send_node_num--;
    protocol_p_free(cur_send_node);
    MUTEX_UNLOCK(boardcast_object.mutex_lock);
  }

  return 0;
}

//获取路由
struct perph_interface *protocol_s_get_route(uint8_t tar_add)
{
  uint8_t int_obj_idx;

  int_obj_idx = protocol_local_info.route_table[tar_add];
  if (int_obj_idx > PROTOCOL_INTERFACE_MAX)
  {
    return NULL;
  }

  if (protocol_local_info.interface[int_obj_idx].is_valid)
  {
    return &(protocol_local_info.interface[int_obj_idx]);
  }
  else
  {
    return NULL;
  }
}

//获得指定地址和session的节点
send_list_node_t *protocol_s_session_get_node(struct perph_interface *obj,
                                              uint8_t address, uint8_t session)
{
  list_t *head_node;
  list_t *cur_node;
  list_t *store_list;
  send_list_node_t *cur_send_node;

  head_node = &(obj->send.normal_list_header);

  MUTEX_LOCK(protocol_local_info.mutex_lock);
  list_for_each_prev_safe(cur_node, store_list, head_node)
  {
    cur_send_node = (send_list_node_t *)cur_node;

    if ((cur_send_node->session == session) &&
        (cur_send_node->address == address))
    {
      MUTEX_UNLOCK(protocol_local_info.mutex_lock);
      return cur_send_node;
    }
  }
  MUTEX_UNLOCK(protocol_local_info.mutex_lock);
  return NULL;
}

//包转发函数
uint32_t protocol_s_pack_forward(protocol_pack_desc_t *p_pack, struct perph_interface *src_obj)
{

  struct perph_interface *tar_inter;
  uint8_t *malloc_zone;
  uint32_t status;
  uint32_t pack_head_offset;
  send_list_node_t *send_node;

  status = PROTOCOL_SUCCESS;

  //寻找包的目的地
  if (p_pack->reciver != PROTOCOL_BROADCAST_ADDR)
  {
    tar_inter = protocol_s_get_route(p_pack->reciver);

    if (tar_inter == NULL)
    {
      PROTOCOL_RCV_ERR_PRINTF("Pack forward error, Route to address 0x%02x does not exist.", p_pack->reciver);

      return PROTOCOL_ERR_ROUTE_NOT_FOUND;
    }
  }

  //分配转发包所需的内存
  malloc_zone = protocol_p_malloc(p_pack->data_len + PROTOCOL_SEND_NODE_SIZE);
  if (malloc_zone == NULL)
  {
    status = PROTOCOL_ERR_NOT_ENOUGH_MEM;
    PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
    return status;
  }

  pack_head_offset = PROTOCOL_SEND_NODE_SIZE;
  send_node = (send_list_node_t *)&malloc_zone[0];

  send_node->session = 0;
  send_node->p_data = &malloc_zone[pack_head_offset];
  send_node->len = p_pack->data_len;
  send_node->rest_cnt = 1;
  send_node->pre_timestamp = 0;
  send_node->timeout = 0;
  send_node->is_got_ack = 0;
  send_node->is_first_send = 1;
  send_node->address = p_pack->reciver;
  send_node->pack_type = PROTOCOL_PACK_ACK; //把转发包当做ACK包发送更快捷
  send_node->is_ready_realse = 0;
  send_node->cmd = 0;
  send_node->forward_src_obj = src_obj;

  memcpy(send_node->p_data, p_pack, p_pack->data_len);
  if (p_pack->reciver != PROTOCOL_BROADCAST_ADDR)
  {
    //非广播包处理
    MUTEX_LOCK(tar_inter->send.mutex_lock);
    list_add(&(send_node->send_list), &(tar_inter->send.ack_list_header)); //把转发包当做ACK包发送更快捷
    tar_inter->send.ack_node_num++;
    MUTEX_UNLOCK(tar_inter->send.mutex_lock);

    PROTOCOL_RCV_DBG_PRINTF("Pack forward to address 0x%02x, Next jump is %s.",
                             p_pack->reciver, tar_inter->object_name);
  }
  else
  {
    //广播包处理
    MUTEX_LOCK(boardcast_object.mutex_lock);

    list_add(&(send_node->send_list), &(boardcast_object.send_list_header));
    boardcast_object.send_node_num++;

    MUTEX_UNLOCK(boardcast_object.mutex_lock);

    PROTOCOL_RCV_DBG_PRINTF("Broadcast pack forward.");
  }

  return status;
}

//解包处理函数
uint32_t protocol_s_unpack_data_handle(struct perph_interface *obj)
{
  uint32_t status;
  uint16_t cmd;
  protocol_pack_desc_t *p_pack;
  send_list_node_t *session_node;

  status = PROTOCOL_SUCCESS;
  p_pack = (protocol_pack_desc_t *)(obj->rcvd.p_data);

#if PROTOCOL_ROUTE_FOWARD == PROTOCOL_ENABLE

  //若接收地址不符合本地地址，则进行转发
  if (p_pack->reciver != protocol_local_info.address)
  {
    status = protocol_s_pack_forward(p_pack, obj);
    if (p_pack->reciver != PROTOCOL_BROADCAST_ADDR)
    {
      return status;
    }
  }

#endif

  if (p_pack->pack_type == PROTOCOL_PACK_ACK)
  {
    session_node = protocol_s_session_get_node(obj,
                                               p_pack->sender,
                                               p_pack->session);

    if (session_node == NULL)
    {
      status = PROTOCOL_ERR_SESSION_NOT_FOUND;
      PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);

      return status;
    }
    session_node->is_got_ack = 1;

    cmd = session_node->cmd;

    PROTOCOL_RCV_DBG_PRINTF("Rcv pack, Address:0x%02X, Cmd:0x%04X, Session:%d Ack pack.",
                             p_pack->sender, cmd, p_pack->session);

    if (session_node->ack_callback != NULL)
    {
      session_node->ack_callback(*(int32_t *)(p_pack->pdata));
    }
  }
  else
  {
    cmd = *((uint16_t *)(p_pack->pdata));
    PROTOCOL_RCV_DBG_PRINTF("Rcv pack, Address:0x%02X, Cmd:0x%04X, Normal pack.",
                             p_pack->sender, cmd);
    if (protocol_local_info.rcv_nor_callBack != NULL)
    {
      protocol_local_info.rcv_nor_callBack((uint8_t *)p_pack,
                                           cmd,
                                           p_pack->session,
                                           p_pack->sender);
    }
  }

  return status;
}

//解包
uint32_t protocol_s_extract(struct perph_interface *obj)
{
  uint32_t status = 0;
  rcvd_desc_t *rcvd;

  rcvd = &obj->rcvd;
  if (fifo_s_isempty(&rcvd->fifo))
  {
    status = PROTOCOL_ERR_FIFO_EMPTY;
    return status;
  }

  while (1)
  {
    switch (rcvd->state)
    {
    case UNPACK_PACK_STAGE_FIND_SOF:

      status = protocol_s_find_pack_header(rcvd);
      if (status == PROTOCOL_SUCCESS)
      {
        rcvd->state = UNPACK_PACK_STAGE_AUTH_HEADER;
      }
      break;

    case UNPACK_PACK_STAGE_AUTH_HEADER:

      status = protocol_s_auth_pack_header(rcvd);

      if (status == PROTOCOL_SUCCESS)
      { /* malloc memory size equal to header size adding data size */
        rcvd->state = UNPACK_PACK_STAGE_RECV_DATA;
        rcvd->p_data = protocol_p_malloc(rcvd->total_num);
        if (rcvd->p_data == NULL)
        {
          status = PROTOCOL_ERR_NOT_ENOUGH_MEM;
          PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
          return status;
        }
        memset(rcvd->p_data, 0, rcvd->rcvd_num);
      }
      else if (status == PROTOCOL_ERR_AUTH_FAILURE)
      {

        fifo_s_get(&rcvd->fifo);
        /* this is a pseudo header, remove this from fifo */
        rcvd->state = UNPACK_PACK_STAGE_FIND_SOF;

        PROTOCOL_RCV_ERR_PRINTF("Pack header auth failure.");
      }
      break;

    case UNPACK_PACK_STAGE_RECV_DATA:

      if (rcvd->p_data)
      {
        status = protocol_s_fetch_pack_data(rcvd);
      }
      else
      {
        status = PROTOCOL_ERR_NOT_ENOUGH_MEM;
        PROTOCOL_ERR_INFO_PRINTF(status, __FILE__, __LINE__);
      }

      if (status == PROTOCOL_SUCCESS)
      {
        rcvd->state = UNPACK_PACK_STAGE_AUTH_PACK;
      }

      break;

    case UNPACK_PACK_STAGE_AUTH_PACK:

      if (verify_crc32(rcvd->p_data, rcvd->total_num))
      {
        rcvd->state = UNPACK_PACK_STAGE_DATA_HANDLE;
      }
      else
      {
        rcvd->rcvd_num = 0;
        rcvd->total_num = 0;
        protocol_p_free(rcvd->p_data);
        rcvd->state = UNPACK_PACK_STAGE_FIND_SOF;

        PROTOCOL_RCV_ERR_PRINTF("Pack data auth failure.");
      }
      break;

    case UNPACK_PACK_STAGE_DATA_HANDLE:

      status = protocol_s_unpack_data_handle(obj);

      protocol_p_free(rcvd->p_data);
      rcvd->state = UNPACK_PACK_STAGE_FIND_SOF;
      break;

    default:
      break;
    }

    if (status == PROTOCOL_ERR_DATA_NOT_ENOUGH ||
        status == PROTOCOL_ERR_NOT_ENOUGH_MEM ||
        status == PROTOCOL_ERR_NOT_FIND_HEADER)
    {
      break; /* break from while */
    }
  }

  return status;
}

//找帧头
uint32_t protocol_s_find_pack_header(rcvd_desc_t *rcvd)
{
  uint32_t status;

  while (fifo_s_isempty(&rcvd->fifo) == 0)
  { // if fifo not empty, loop
    if ((uint8_t)(fifo_s_preread(&rcvd->fifo, 0)) == PROTOCOL_HEADER)
    {
      status = PROTOCOL_SUCCESS;
      goto END;
    }
    else
    {
      fifo_s_get(&rcvd->fifo); //remove one byte from fifo
    }
  }
  //if fifo not empty, loop

  status = PROTOCOL_ERR_NOT_FIND_HEADER;

END:
  return status;
}

//校验包头
uint32_t protocol_s_auth_pack_header(rcvd_desc_t *rcvd)
{
  uint32_t status;
  uint8_t auth_array[12];
  ver_data_len_t ver_len;

  if (fifo_s_prereads(&rcvd->fifo, (char *)auth_array, 0, 12) == 12)
  {
    ver_len = protocol_s_get_ver_datalen(auth_array);
    if (ver_len.data_len - PROTOCOL_PACK_HEAD_TAIL_SIZE > PROTOCOL_MAX_DATA_LEN)
    {
      status = PROTOCOL_ERR_AUTH_FAILURE;
    }
    else
    {
      if ((ver_len.version == 0) && (verify_crc16(auth_array, 12) == 1))
      {
        status = PROTOCOL_SUCCESS;
        rcvd->total_num = ver_len.data_len;
        rcvd->rcvd_num = 0;
      }
      else
      {
        status = PROTOCOL_ERR_AUTH_FAILURE;
      }
    }
  }
  else
  {
    status = PROTOCOL_ERR_DATA_NOT_ENOUGH;
  }

  return status;
}

//获取包数据
uint32_t protocol_s_fetch_pack_data(rcvd_desc_t *rcvd)
{
  uint32_t status;
  uint32_t length;
  uint32_t want_len;

  want_len = rcvd->total_num - rcvd->rcvd_num;
  length = fifo_s_gets(&rcvd->fifo,
                       (char *)rcvd->p_data + rcvd->rcvd_num,
                       want_len);
  rcvd->rcvd_num += length;

  if (rcvd->rcvd_num < rcvd->total_num)
  {
    status = PROTOCOL_ERR_DATA_NOT_ENOUGH;
  }
  else
  {
    status = PROTOCOL_SUCCESS;
  }
  return status;
}

//获取版本号和数据长度
ver_data_len_t protocol_s_get_ver_datalen(void *pack)

{
  ver_data_len_t ver_len;
  uint16_t *tmp = (uint16_t *)&ver_len;
  uint8_t *ptr = (uint8_t *)pack;

  *tmp = ptr[2] << 8 | ptr[1];

  return ver_len;
}

//答应错误信息
void protocol_s_error_info_printf(uint32_t status, char *file, int line)
{
  char *err_info;
  switch (status)
  {
  case PROTOCOL_SUCCESS:
    err_info = "PROTOCOL_SUCCESS";
    break;
  case PROTOCOL_ERR_DATA_TOO_LONG:
    err_info = "PROTOCOL_ERR_DATA_TOO_LONG";
    break;
  case PROTOCOL_ERR_NOT_ENOUGH_MEM:
    err_info = "PROTOCOL_ERR_NOT_ENOUGH_MEM";
    break;
  case PROTOCOL_ERR_SESSION_NOT_FOUND:
    err_info = "PROTOCOL_ERR_SESSION_NOT_FOUND";
    break;
  case PROTOCOL_ERR_SESSION_FULL:
    err_info = "PROTOCOL_ERR_SESSION_FULL";
    break;
  case PROTOCOL_ERR_SESSION_IS_USE:
    err_info = "PROTOCOL_ERR_SESSION_IS_USE";
    break;
  case PROTOCOL_ERR_ROUTE_NOT_FOUND:
    err_info = "PROTOCOL_ERR_ROUTE_NOT_FOUND";
    break;
  case PROTOCOL_ERR_FIFO_EMPTY:
    err_info = "PROTOCOL_ERR_FIFO_EMPTY";
    break;
  case PROTOCOL_ERR_AUTH_FAILURE:
    err_info = "PROTOCOL_ERR_AUTH_FAILURE";
    break;
  case PROTOCOL_ERR_NOT_FIND_HEADER:
    err_info = "PROTOCOL_ERR_NOT_FIND_HEADER";
    break;
  case PROTOCOL_ERR_DATA_NOT_ENOUGH:
    err_info = "PROTOCOL_ERR_DATA_NOT_ENOUGH";
    break;
  case PROTOCOL_ERR_FIFO_FULL:
    err_info = "PROTOCOL_ERR_FIFO_FULL";
    break;
  case PROTOCOL_ERR_OBJECT_NOT_FOUND:
    err_info = "PROTOCOL_ERR_OBJECT_NOT_FOUND";
    break;
  case PROTOCOL_ERR_UNSUPPORT_CPU:
    err_info = "PROTOCOL_ERR_UNSUPPORT_CPU";
    break;
  case PROTOCOL_ERR_ROUTEU_SET_BEYOND:
    err_info = "PROTOCOL_ERR_ROUTEU_SET_BEYOND";
    break;
  case PROTOCOL_ERR_INTER_NOT_FOUND:
    err_info = "PROTOCOL_ERR_INTER_NOT_FOUND";
    break;
  case PROTOCOL_ERR_PROTOCOL_NOT_INIT:
    err_info = "PROTOCOL_ERR_PROTOCOL_NOT_INIT";
    break;
  case PROTOCOL_ERR_SESSION_ERROR:
    err_info = "PROTOCOL_ERR_SESSION_ERROR(Boardcast session can only be 0)";
    break;
  case PROTOCOL_ERR_REGISTER_FAILED:
    err_info = "PROTOCOL_ERR_REGISTER_FAILED";
    break;
  default:
    err_info = "PROTOCOL_ERR_NOT_FOUND";
  }
  protocol_log_e("%s[%u].(File:%s,Line:%d)",
                 err_info, status, file, line);

  return;
}
