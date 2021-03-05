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

/* standard can frame S/R driver*/

#include "can.h"
#include "drv_can.h"

struct can_manage_obj can1_manage;
struct can_manage_obj can2_manage;

static uint8_t can1_tx_fifo_buff[CAN1_TX_FIFO_SIZE];
static uint8_t can2_tx_fifo_buff[CAN2_TX_FIFO_SIZE];

/******************** Custom API ****************************/
/**
  * @brief  CAN1, CAN2 manage object init.
  * @param
  * @retval void
  */
void can_manage_init(void)
{
    can1_manage.is_sending = 0;
    can1_manage.hcan = &hcan1;

    can1_manage.can_rec_callback = NULL;
    can2_manage.can_rec_callback = NULL;

    /* asynchronism fifo init */
    fifo_init(&(can1_manage.tx_fifo),
              can1_tx_fifo_buff,
              sizeof(struct can_std_msg),
              CAN1_TX_FIFO_UNIT_NUM);

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR_WARNING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_BUSOFF);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR_PASSIVE);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_LAST_ERROR_CODE);

    can2_manage.is_sending = 0;
    can2_manage.hcan = &hcan2;

    fifo_init(&(can2_manage.tx_fifo),
              can2_tx_fifo_buff,
              sizeof(struct can_std_msg),
              CAN2_TX_FIFO_UNIT_NUM);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);

    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_ERROR);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_ERROR_WARNING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_BUSOFF);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_ERROR_PASSIVE);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_LAST_ERROR_CODE);

    return;
}

uint32_t can1_std_transmit(uint16_t std_id, uint8_t *data, uint16_t len)
{
    return can_msg_bytes_send(&hcan1, std_id, data, len);
}

uint32_t can2_std_transmit(uint16_t std_id, uint8_t *data, uint16_t len)
{
    return can_msg_bytes_send(&hcan2, std_id, data, len);
}

/******************** Usual API ****************************/
/**
  * @brief  register can callback function.
  * @param
  * @retval error code
  */
int32_t can_fifo0_rx_callback_register(can_manage_obj_t m_obj, can_stdmsg_rx_callback_t fun)
{
    m_obj->can_rec_callback = fun;
    return E_OK;
}

/**
  * @brief  can transmit function
  * @param
  * @retval send successful length
  */
uint32_t can_msg_bytes_send(CAN_HandleTypeDef *hcan,
                            uint16_t std_id, uint8_t *data, uint16_t len)
{
    uint8_t *send_ptr;
    uint16_t send_num;
    can_manage_obj_t m_obj;
    struct can_std_msg msg;

    send_ptr = data;
    msg.std_id = std_id;
    send_num = 0;

    if (hcan == &hcan1)
    {
        m_obj = &can1_manage;
    }
    else if (hcan == &hcan2)
    {
        m_obj = &can2_manage;
    }
    else
    {
        return 0;
    }

    while (send_num < len)
    {
        if (fifo_is_full(&(m_obj->tx_fifo)))
        {
            //can is error
            m_obj->is_sending = 0;
            break;
        }

        if (len - send_num >= 8)
        {
            msg.dlc = 8;
        }
        else
        {
            msg.dlc = len - send_num;
        }

        //memcpy(msg.data, data, msg.dlc);
        *((uint32_t *)(msg.data)) = *((uint32_t *)(send_ptr));
        *((uint32_t *)(msg.data + 4)) = *((uint32_t *)(send_ptr + 4));

        send_ptr += msg.dlc;
        send_num += msg.dlc;

        fifo_put(&(m_obj->tx_fifo), &msg);
    }

    if ((m_obj->is_sending) == 0 && (!(fifo_is_empty(&(m_obj->tx_fifo)))))
    {
        CAN_TxHeaderTypeDef header;
        uint32_t send_mail_box;

        header.StdId = std_id;
        header.IDE = CAN_ID_STD;
        header.RTR = CAN_RTR_DATA;

        while (HAL_CAN_GetTxMailboxesFreeLevel(m_obj->hcan) && (!(fifo_is_empty(&(m_obj->tx_fifo)))))
        {
            fifo_get(&(m_obj->tx_fifo), &msg);
            header.DLC = msg.dlc;
            HAL_CAN_AddTxMessage(m_obj->hcan, &header, msg.data, &send_mail_box);

            m_obj->is_sending = 1;
        }
    }

    return send_num;
}

/**
  * @brief  send complete callback
  * @param
  * @retval void
  */
static void can_tx_mailbox_complete_hanle(can_manage_obj_t m_obj)
{
    struct can_std_msg msg;
    CAN_TxHeaderTypeDef header;
    uint32_t send_mail_box;

    CRITICAL_SETCION_ENTER();

    if (!fifo_is_empty(&(m_obj->tx_fifo)))
    {
        while (!fifo_is_empty(&(m_obj->tx_fifo)))
        {
            if (HAL_CAN_GetTxMailboxesFreeLevel(m_obj->hcan))
            {

                fifo_get_noprotect(&(m_obj->tx_fifo), &msg);

                header.StdId = msg.std_id;
                header.DLC = msg.dlc;
                header.IDE = CAN_ID_STD;
                header.RTR = CAN_RTR_DATA;

                HAL_CAN_AddTxMessage(m_obj->hcan, &header, msg.data, &send_mail_box);
            }
            else
            {
                m_obj->is_sending = 0;
            }
        }
    }
    else
    {
        m_obj->is_sending = 0;
    }

    CRITICAL_SETCION_EXIT();

    return;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        can_tx_mailbox_complete_hanle(&can1_manage);
    }
    else if (hcan == &hcan2)
    {
        can_tx_mailbox_complete_hanle(&can2_manage);
    }
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        can_tx_mailbox_complete_hanle(&can1_manage);
    }
    else if (hcan == &hcan2)
    {
        can_tx_mailbox_complete_hanle(&can2_manage);
    }
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        can_tx_mailbox_complete_hanle(&can1_manage);
    }
    else if (hcan == &hcan2)
    {
        can_tx_mailbox_complete_hanle(&can2_manage);
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
        can_tx_mailbox_complete_hanle(&can1_manage);
    }
    else if (hcan == &hcan2)
    {
        can_tx_mailbox_complete_hanle(&can2_manage);
    }
    HAL_CAN_ResetError(hcan);
}

/**
  * @brief  can rx interupt
  * @param
  * @retval void
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &hcan1)
    {
        if (can1_manage.can_rec_callback != NULL)
        {
            (*(can1_manage.can_rec_callback))(&rx_header, rx_data);
        }
    }
    else if (hcan == &hcan2)
    {
        if (can2_manage.can_rec_callback != NULL)
        {
            (*(can2_manage.can_rec_callback))(&rx_header, rx_data);
        }
    }
}
