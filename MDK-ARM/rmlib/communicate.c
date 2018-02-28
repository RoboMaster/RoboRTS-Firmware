/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
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
/** @file communicate.c
 *  @version 1.0
 *  @date Oct 2017
 *
 *  @brief the communication interface of main control with
 *         judge system and computer
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "communicate.h"
#include "comm_task.h"
#include "judgement_info.h"
#include "infantry_info.h"
#include "data_fifo.h"
#include "protocol.h"
#include "info_interactive.h"
#include "sys_config.h"
#include "string.h"
#include "usart.h"

void unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof)
{
  uint8_t byte = 0;
  
  while ( fifo_used_count(p_obj->data_fifo) )
  {
    //independent thread, need not mutex
    byte = fifo_s_get_no_mutex(p_obj->data_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
    
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == HEADER_LEN)
        {
          if ( verify_crc8_check_sum(p_obj->protocol_packet, HEADER_LEN) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  

      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_crc16_check_sum(p_obj->protocol_packet, HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN) )
          {
            if (sof == UP_REG_ID)
            {
              pc_data_handler(p_obj->protocol_packet);
            }
            else  //DN_REG_ID
            {
              judgement_data_handler(p_obj->protocol_packet);
            }
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}

void dma_buffer_to_unpack_buffer(uart_dma_rxdata_t *dma_obj, uart_it_type_e it_type)
{
  uint16_t tmp_len;
  uint8_t  current_memory_id;
  uint16_t remain_data_counter;
  
  uint8_t  *pdata        = dma_obj->buff[0];
  uint16_t write_counter = fifo_free_count(dma_obj->data_fifo);
  
  get_dma_memory_msg(dma_obj->huart->hdmarx->Instance, &current_memory_id, &remain_data_counter);
  
  if (UART_IDLE_IT == it_type)
  {
    if (current_memory_id)
    {
      dma_obj->write_index = dma_obj->buff_size*2 - remain_data_counter;
    }
    else
    {
      dma_obj->write_index = dma_obj->buff_size - remain_data_counter;
    }
  }
  else if (UART_DMA_FULL_IT == it_type)
  {
    if (current_memory_id)
    {
      dma_obj->write_index = dma_obj->buff_size;
    }
    else
    {
      dma_obj->write_index = dma_obj->buff_size*2;
    }
  }
  
#if 0
  if (UART_IDLE_IT == it_type)
  {
    dma_obj->write_index = dma_obj->buff_size - remain_data_counter;
  }
  else if (UART_DMA_HALF_IT == it_type)
  {
    dma_obj->write_index = dma_obj->buff_size/2;
  }
  else if (UART_DMA_FULL_IT == it_type)
  {
    dma_obj->write_index = dma_obj->buff_size;
  }
#endif
  
  tmp_len = dma_obj->write_index - dma_obj->read_index;
  //independent thread, need not mutex
  write_counter = fifo_s_puts_no_mutex(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len);
  dma_obj->read_index = (dma_obj->read_index + write_counter) % (dma_obj->buff_size*2);
}


uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf)
{
  //memset(tx_buf, 0, 100);
  //static uint8_t seq;
  
  uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;
  frame_header_t *p_header = (frame_header_t*)tx_buf;
  
  p_header->sof          = sof;
  p_header->data_length  = len;
  p_header->seq          = 0;
  //p_header->seq          = seq++;
  
  memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);
  append_crc8_check_sum(tx_buf, HEADER_LEN);
  memcpy(&tx_buf[HEADER_LEN + CMD_LEN], p_data, len);
  append_crc16_check_sum(tx_buf, frame_length);
  
  return tx_buf;
}

void data_upload_handler(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf)
{
  uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;
  
  protocol_packet_pack(cmd_id, p_data, len, sof, tx_buf);
  
  if (sof == UP_REG_ID)
    write_uart_blocking(&COMPUTER_HUART, tx_buf, frame_length);
  else if (sof == DN_REG_ID)
    write_uart_blocking(&JUDGE_HUART, tx_buf, frame_length);
}

uint32_t send_packed_fifo_data(fifo_s_t *pfifo, uint8_t sof)
{
  uint8_t  tx_buf[PROTOCAL_FRAME_MAX_SIZE];
  uint32_t fifo_count = fifo_used_count(pfifo);
  
  if (fifo_count)
  {
    fifo_s_gets(pfifo, tx_buf, fifo_count);
    
    if (sof == UP_REG_ID)
      write_uart_blocking(&COMPUTER_HUART, tx_buf, fifo_count);
    //else if (sof == DN_REG_ID)
      //write_uart_blocking(&JUDGE_HUART, tx_buf, fifo_count);
    else
      return 0;
  }
  
  return fifo_count;
}

