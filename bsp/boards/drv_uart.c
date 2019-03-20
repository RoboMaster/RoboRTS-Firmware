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

#include "usart.h"
#include "dma.h"
#include "drv_uart.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
static uint8_t usart6_rx_buff[USART6_RX_BUFFER_SIZE];
static uint8_t usart6_tx_buff[USART6_TX_BUFFER_SIZE];
static uint8_t usart6_tx_fifo_buff[USART6_TX_FIFO_SIZE];

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
static uint8_t usart3_rx_buff[USART3_RX_BUFFER_SIZE];
static uint8_t usart3_tx_buff[USART3_TX_BUFFER_SIZE];
static uint8_t usart3_tx_fifo_buff[USART3_TX_FIFO_SIZE];

usart_manage_obj_t usart3_manage_obj =
    {
        .call_back_f = NULL};

usart_manage_obj_t usart6_manage_obj =
    {
        .call_back_f = NULL};

static void usart_rec_to_buff(usart_manage_obj_t *m_obj, interrput_type int_type);
static void usart_transmit_hook(usart_manage_obj_t *m_obj);

void usart_rx_callback_register(usart_manage_obj_t *m_obj, usart_call_back fun)
{
  m_obj->call_back_f = fun;
  return;
}

void usart3_rx_callback_register(usart_call_back fun)
{
  usart_rx_callback_register(&usart3_manage_obj, fun);
  return;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    usart_rec_to_buff(&usart3_manage_obj, INTERRUPT_TYPE_DMA_HALF);
  }
  else if (huart == &huart6)
  {
    usart_rec_to_buff(&usart6_manage_obj, INTERRUPT_TYPE_DMA_HALF);
  }

  return;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    usart_rec_to_buff(&usart3_manage_obj, INTERRUPT_TYPE_DMA_ALL);
  }
  else if (huart == &huart6)
  {
    usart_rec_to_buff(&usart6_manage_obj, INTERRUPT_TYPE_DMA_ALL);
  }

  return;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    usart_transmit_hook(&usart3_manage_obj);
  }
  else if (huart == &huart6)
  {
    usart_transmit_hook(&usart6_manage_obj);
  }

  return;
}

void usart3_idle_callback(void)
{
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    usart_rec_to_buff(&usart3_manage_obj, INTERRUPT_TYPE_UART);
  }
}

void usart6_idle_callback(void)
{
  if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart6);
    usart_rec_to_buff(&usart6_manage_obj, INTERRUPT_TYPE_UART);
  }
}

void usart3_manage_init(void)
{
  usart3_manage_obj.rx_buffer = usart3_rx_buff;
  usart3_manage_obj.rx_buffer_size = USART3_RX_BUFFER_SIZE;
  usart3_manage_obj.dma_h = &hdma_usart3_rx;
  usart3_manage_obj.uart_h = &huart3;
  usart3_manage_obj.tx_fifo_buffer = usart3_tx_fifo_buff;
  usart3_manage_obj.tx_fifo_size = USART3_TX_FIFO_SIZE;
  usart3_manage_obj.tx_buffer_size = USART3_TX_BUFFER_SIZE;
  usart3_manage_obj.tx_buffer = usart3_tx_buff;
  usart3_manage_obj.is_sending = 0;

  fifo_s_init(&(usart3_manage_obj.tx_fifo), usart3_tx_fifo_buff, USART3_TX_FIFO_SIZE);

  HAL_UART_Receive_DMA(&huart3, usart3_rx_buff, USART3_RX_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}

void usart6_manage_init(void)
{
  usart6_manage_obj.rx_buffer = usart6_rx_buff;
  usart6_manage_obj.rx_buffer_size = USART6_RX_BUFFER_SIZE;
  usart6_manage_obj.dma_h = &hdma_usart6_rx;
  usart6_manage_obj.uart_h = &huart6;
  usart6_manage_obj.tx_fifo_buffer = usart6_tx_fifo_buff;
  usart6_manage_obj.tx_fifo_size = USART6_TX_FIFO_SIZE;
  usart6_manage_obj.tx_buffer_size = USART6_TX_BUFFER_SIZE;
  usart6_manage_obj.tx_buffer = usart6_tx_buff;
  usart6_manage_obj.is_sending = 0;

  fifo_s_init(&(usart6_manage_obj.tx_fifo), usart6_tx_fifo_buff, USART6_TX_FIFO_SIZE);

  HAL_UART_Receive_DMA(&huart6, usart6_rx_buff, USART6_RX_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
}

int usart6_printf(char *fmt, ...)
{

  va_list arg;
  uint8_t buff[USART6_PRINTF_BUFF_SIZE];
  uint8_t printf_len;

  va_start(arg, fmt);
  printf_len = vsnprintf((char *)buff, USART6_PRINTF_BUFF_SIZE, fmt, arg);
  va_end(arg);

  if (printf_len > USART6_PRINTF_BUFF_SIZE)
  {
    printf_len = USART6_PRINTF_BUFF_SIZE;
  }

  usart_transmit(&usart6_manage_obj, buff, printf_len);

  return 0;
}

void usart3_transmit(uint8_t *buff, uint16_t len)
{
  usart_transmit(&usart3_manage_obj, buff, len);
}

void usart6_transmit(uint8_t *buff, uint16_t len)
{
  usart_transmit(&usart6_manage_obj, buff, len);
}

UART_Err usart_transmit(usart_manage_obj_t *m_obj, uint8_t *buf, uint16_t len)
{
  uint16_t to_send_len;
  uint16_t to_tx_fifo_len;

  if (m_obj->is_sending == 0)
  {
    if (len < m_obj->tx_buffer_size)
    {
      to_send_len = len;
      to_tx_fifo_len = 0;
    }
    else if (len < m_obj->tx_buffer_size + m_obj->tx_fifo_size)
    {
      to_send_len = m_obj->tx_buffer_size;
      to_tx_fifo_len = len - m_obj->tx_buffer_size;
    }
    else
    {
      to_send_len = m_obj->tx_buffer_size;
      to_tx_fifo_len = m_obj->tx_fifo_size;
    }
  }
  else
  {
    if (len < m_obj->tx_fifo_size)
    {
      to_send_len = 0;
      to_tx_fifo_len = len;
    }
    else
    {
      to_send_len = 0;
      to_tx_fifo_len = m_obj->tx_fifo_size;
    }
  }

  if (to_send_len > 0)
  {
    memcpy(m_obj->tx_buffer, buf, to_send_len);
    m_obj->is_sending = 1;
    HAL_UART_Transmit_DMA(m_obj->uart_h, m_obj->tx_buffer, to_send_len);
  }
  if (to_tx_fifo_len > 0)
  {
    fifo_s_puts(&(m_obj->tx_fifo), (char *)(buf) + to_send_len, to_tx_fifo_len);
  }

  return ERR_NORAML;
}

static void usart_transmit_hook(usart_manage_obj_t *m_obj)
{
  uint16_t fifo_data_num = 0;
  uint16_t send_num = 0;

  fifo_data_num = m_obj->tx_fifo.used_num;

  if (fifo_data_num != 0)
  {
    if (fifo_data_num < m_obj->tx_buffer_size)
    {
      send_num = fifo_data_num;
    }
    else
    {
      send_num = m_obj->tx_buffer_size;
    }
    fifo_s_gets(&(m_obj->tx_fifo), (char *)(m_obj->tx_buffer), send_num);
    m_obj->is_sending = 1;
    HAL_UART_Transmit_DMA(m_obj->uart_h, m_obj->tx_buffer, send_num);
  }
  else
  {
    m_obj->is_sending = 0;
  }
  return;
}

static void usart_rec_to_buff(usart_manage_obj_t *m_obj, interrput_type int_type)
{
  uint16_t read_end_ptr = 0;
  uint16_t read_length = 0;
  uint16_t read_success_length = 0;
  uint16_t read_start_ptr = m_obj->read_start_index;
  uint8_t *pdata = m_obj->rx_buffer;

  UNUSED(read_success_length);

  uint16_t buff_left = m_obj->dma_h->Instance->NDTR;

  if (int_type == INTERRUPT_TYPE_UART)
  {
    read_end_ptr = m_obj->rx_buffer_size - buff_left;
  }

  if (int_type == INTERRUPT_TYPE_DMA_HALF)
  {
    read_end_ptr = m_obj->rx_buffer_size / 2;
  }

  if (int_type == INTERRUPT_TYPE_DMA_ALL)
  {
    read_end_ptr = m_obj->rx_buffer_size;
  }

  read_length = read_end_ptr - m_obj->read_start_index;

  if (m_obj->call_back_f != NULL)
  {
    uint8_t *read_ptr = pdata + read_start_ptr;
    read_success_length = m_obj->call_back_f(read_ptr, read_length);
  }

  m_obj->read_start_index = (m_obj->read_start_index + read_length) % (m_obj->rx_buffer_size);

  return;
}
