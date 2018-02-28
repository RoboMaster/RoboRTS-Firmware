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
/** @file bsp_uart.c
 *  @version 1.0
 *  @date Apr 2017
 *
 *  @brief uart receive data from DBus/judge_system/manifold etc.
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "bsp_uart.h"
#include "remote_ctrl.h"
#include "detect_task.h"
#include "judgement_info.h"
#include "infantry_info.h"
#include "info_interactive.h"
#include "sys_config.h"
#include "usart.h"
#include "cmsis_os.h"
#include "communicate.h"

/* dma double buffer */
uint8_t judge_dma_rxbuff[2][UART_RX_DMA_SIZE];
uint8_t pc_dma_rxbuff[2][UART_RX_DMA_SIZE];

/**
  * @brief   clear idle it flag after uart receive a frame data
  * @param   uart IRQHandler id
  * @usage   call in uart_receive_handler() function
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
  /* clear idle it flag avoid idle interrupt all the time */
  __HAL_UART_CLEAR_IDLEFLAG(huart);
  
  /* handle received data in idle interrupt */
  if (huart == &DBUS_HUART)
  {
    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);
    
    /* handle dbus data dbus_buf from DMA */
    //uint32_t status = taskENTER_CRITICAL_FROM_ISR();
    if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
    {
      rc_callback_handler(&rc, dbus_buf);
      err_detector_hook(REMOTE_CTRL_OFFLINE);
    }
    //taskEXIT_CRITICAL_FROM_ISR(status);
    
    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);

  }
  else if ((huart == &JUDGE_HUART) || (huart == &COMPUTER_HUART))
  {
    uart_idle_interrupt_signal(huart);
    
  }
  else
  {
  }
}

/**
  * @brief   callback this function when uart interrupt 
  * @param   uart IRQHandler id
  * @usage   call in uart handler function USARTx_IRQHandler()
  */
void uart_receive_handler(UART_HandleTypeDef *huart)
{  
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && 
      __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
  {
    uart_rx_idle_callback(huart);
  }
}

/**
  * @brief   enable global uart it and do not use DMA transfer done it
  * @param   uart IRQHandler id, receive buff, buff size
  * @retval  set success or fail
  */
static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
  uint32_t tmp1 = 0;

  tmp1 = huart->RxState;
  if (tmp1 == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0))
    {
        return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(huart);

    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    
    huart->ErrorCode  = HAL_UART_ERROR_NONE;

    /* Enable the DMA Stream */
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,
                  (uint32_t)pData, Size);

    /* Enable the DMA transfer for the receiver request by setting the DMAR bit
    in the UART CR3 register */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}










/* Current memory buffer used is Memory 0 */
//if((hdma->Instance->CR & DMA_SxCR_CT) == RESET)
static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef* huart = ( UART_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  uart_dma_full_signal(huart);
}
/* Current memory buffer used is Memory 1 */
static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef* huart = ( UART_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  uart_dma_full_signal(huart);
}
static HAL_StatusTypeDef DMAEx_MultiBufferStart_IT(DMA_HandleTypeDef *hdma, \
                                                   uint32_t SrcAddress, \
                                                   uint32_t DstAddress, \
                                                   uint32_t SecondMemAddress, \
                                                   uint32_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  /* Memory-to-memory transfer not supported in double buffering mode */
  if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
  {
    hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
    return HAL_ERROR;
  }
  
  /* Set the UART DMA transfer complete callback */
  /* Current memory buffer used is Memory 1 callback */
  hdma->XferCpltCallback   = dma_m0_rxcplt_callback;
  /* Current memory buffer used is Memory 0 callback */
  hdma->XferM1CpltCallback = dma_m1_rxcplt_callback;
  
  /* Check callback functions */
  if ((NULL == hdma->XferCpltCallback) || (NULL == hdma->XferM1CpltCallback))
  {
    hdma->ErrorCode = HAL_DMA_ERROR_PARAM;
    return HAL_ERROR;
  }
  
  /* Process locked */
  __HAL_LOCK(hdma);
  
  if(HAL_DMA_STATE_READY == hdma->State)
  {
    /* Change DMA peripheral state */
    hdma->State = HAL_DMA_STATE_BUSY;
    /* Initialize the error code */
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;
    /* Enable the Double buffer mode */
    hdma->Instance->CR |= (uint32_t)DMA_SxCR_DBM;
    /* Configure DMA Stream destination address */
    hdma->Instance->M1AR = SecondMemAddress;
    
    /* Configure DMA Stream data length */
    hdma->Instance->NDTR = DataLength;
    /* Configure the source, destination address */
    if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
    {
      hdma->Instance->PAR = DstAddress;
      hdma->Instance->M0AR = SrcAddress;
    }
    else
    {
      hdma->Instance->PAR = SrcAddress;
      hdma->Instance->M0AR = DstAddress;
    }
    
    /* Clear TC flags */
    __HAL_DMA_CLEAR_FLAG (hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
    /* Enable TC interrupts*/
    hdma->Instance->CR  |= DMA_IT_TC;
    
    /* Enable the peripheral */
    __HAL_DMA_ENABLE(hdma);
    
    /* Change the DMA state */
    hdma->State = HAL_DMA_STATE_READY;
  }
  else
  {
    /* Return error status */
    status = HAL_BUSY;
  }
  
  /* Process unlocked */
  __HAL_UNLOCK(hdma);
  
  return status; 
}

/**
  * @brief   initialize uart device 
  */
void dbus_uart_init(void)
{
  //open uart idle it
  __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
  __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);

  UART_Receive_DMA_No_IT(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}
void judgement_uart_init(void)
{
  //open uart idle it
  __HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
  __HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
  
  // Enable the DMA transfer for the receiver request
  SET_BIT(JUDGE_HUART.Instance->CR3, USART_CR3_DMAR);
  
  DMAEx_MultiBufferStart_IT(JUDGE_HUART.hdmarx, \
                           (uint32_t)&JUDGE_HUART.Instance->DR, \
                           (uint32_t)judge_dma_rxbuff[0], \
                           (uint32_t)judge_dma_rxbuff[1], \
                           UART_RX_DMA_SIZE);
  
}
void computer_uart_init(void)
{
  //open uart idle it
  __HAL_UART_CLEAR_IDLEFLAG(&COMPUTER_HUART);
  __HAL_UART_ENABLE_IT(&COMPUTER_HUART, UART_IT_IDLE);
  
  // Enable the DMA transfer for the receiver request
  SET_BIT(COMPUTER_HUART.Instance->CR3, USART_CR3_DMAR);
  
  DMAEx_MultiBufferStart_IT(COMPUTER_HUART.hdmarx, \
                           (uint32_t)&COMPUTER_HUART.Instance->DR, \
                           (uint32_t)pc_dma_rxbuff[0], \
                           (uint32_t)pc_dma_rxbuff[1], \
                           UART_RX_DMA_SIZE);
}

/**
  * @brief  Rx Transfer completed callback
  * @param  huart: UART handle
  * @note   report end of DMA Rx transfer
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_read_completed_signal(huart);
}

/**
  * @brief  Tx Transfer completed callback
  * @param  huart: UART handle. 
  * @note   report end of DMA Tx transfer
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_write_completed_signal(huart);
}

/**
  * @brief  Returns the current memory target used by double buffer transfer.
  * @param  dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *          to 7 to select the DMA Stream.
  * @retval The memory target number: 0 for Memory0 or 1 for Memory1. 
  */
uint8_t dma_current_memory_target(DMA_Stream_TypeDef *dma_stream)
{
  uint8_t tmp = 0;

  /* Get the current memory target */
  if ((dma_stream->CR & DMA_SxCR_CT) != 0)
  {
    /* Current memory buffer used is Memory 1 */
    tmp = 1;
  }
  else
  {
    /* Current memory buffer used is Memory 0 */
    tmp = 0;
  }
  return tmp;
}
/**
  * @brief  Returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param  dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *          to 7 to select the DMA Stream.
  * @retval The number of remaining data units in the current DMAy Streamx transfer.
  */
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  /* Return the number of remaining data units for DMAy Streamx */
  return ((uint16_t)(dma_stream->NDTR));
}

