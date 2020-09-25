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

#include "usart.h"
#include "drv_dr16.h"

extern UART_HandleTypeDef huart3;

int UART_Receive_DMA_No_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size);

dr16_rx_callback_t dr16_rx_callback = NULL;
dr16_rx_callback_t dr16_forword_callback = NULL;

static uint8_t dr16_uart_rx_buff[DR16_RX_BUFFER_SIZE];

void dr16_uart_init(dr16_rx_callback_t rx_fn,
                    dr16_rx_callback_t forword_fn)
{
    UART_Receive_DMA_No_IT(&huart3, dr16_uart_rx_buff, DR16_RX_BUFFER_SIZE);

    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    dr16_rx_callback = rx_fn;
    dr16_forword_callback = forword_fn;
}

/**
  * @brief  uart idle interupt
  * @param
  * @retval error code
  */
uint32_t dr16_uart_rx_data_handle(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
        /* clear idle it flag avoid idle interrupt all the time */
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        /* clear DMA transfer complete flag */
        __HAL_DMA_DISABLE(huart->hdmarx);

        /* handle dbus data dbus_buf from DMA */
        if ((DR16_RX_BUFFER_SIZE - huart->hdmarx->Instance->NDTR) == DR16_DATA_LEN)
        {
            if (dr16_rx_callback != NULL)
            {
                dr16_rx_callback(dr16_uart_rx_buff, DR16_DATA_LEN);
            }

            if (dr16_forword_callback != NULL)
            {
                dr16_forword_callback(dr16_uart_rx_buff, DR16_DATA_LEN);
            }
        }

        /* restart dma transmission */
        __HAL_DMA_SET_COUNTER(huart->hdmarx, DR16_RX_BUFFER_SIZE);
        __HAL_DMA_ENABLE(huart->hdmarx);
    }
    return 0;
}

/**
  * @brief  dr16 uart dma configration
  * @param
  * @retval error code
  */
int UART_Receive_DMA_No_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{
    uint32_t tmp = 0;

    tmp = huart->RxState;
    if (tmp == HAL_UART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(huart);

        huart->pRxBuffPtr = pData;
        huart->RxXferSize = Size;

        huart->ErrorCode = HAL_UART_ERROR_NONE;

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
