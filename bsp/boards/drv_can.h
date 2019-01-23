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

#ifndef __DRV_CAN_H__
#define __DRV_CAN_H__

#ifdef DRV_CAN_H_GLOBAL
#define DRV_CAN_H_EXTERN
#else
#define DRV_CAN_H_EXTERN extern
#endif

#include "fifo.h"

typedef int32_t (*can_stdmsg_rx_callback_t)(CAN_RxHeaderTypeDef *header, uint8_t *data);

#define CAN1_TX_FIFO_UNIT_NUM (64)
#define CAN1_TX_FIFO_SIZE (CAN1_TX_FIFO_UNIT_NUM * sizeof(struct can_std_msg))

#define CAN2_TX_FIFO_UNIT_NUM (64)
#define CAN2_TX_FIFO_SIZE (CAN2_TX_FIFO_UNIT_NUM * sizeof(struct can_std_msg))

#define MAX_CAN_REGISTER_NUM 5

typedef struct can_manage_obj *can_manage_obj_t;

struct can_manage_obj
{
  CAN_HandleTypeDef *hcan;
  fifo_t tx_fifo;
  uint8_t *tx_fifo_buffer;
  uint8_t is_sending;
  can_stdmsg_rx_callback_t can_rec_callback[MAX_CAN_REGISTER_NUM];
};

struct can_std_msg
{
  uint32_t std_id;
  uint8_t dlc;
  uint8_t data[8];
};

extern struct can_manage_obj can1_manage;
extern struct can_manage_obj can2_manage;

void can_manage_init(void);
int32_t can_fifo0_rx_callback_register(can_manage_obj_t m_obj, can_stdmsg_rx_callback_t fun);

uint32_t can_msg_bytes_send(CAN_HandleTypeDef *hcan,
                            uint8_t *data, uint16_t len, uint16_t std_id);

#endif // __DRV_CAN_H__
