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

#include "can.h"
#include "board.h"
#include "usbd_cdc_if.h"
#include "dbus.h"
#include "detect.h"
#include "communicate.h"
#include "timer_task.h"
#include "offline_check.h"
#include "init.h"
#include "infantry_cmd.h"
#include "referee_system.h"
#include "protocol.h"

static int32_t can2_send_data(uint32_t std_id, uint8_t *p_data, uint32_t len);
static void protocol_send_success_callback(void);
static int32_t usb_interface_send(uint8_t *p_data, uint32_t len);

extern osThreadId communicate_task_t;

int32_t uwb_rcv_callback(CAN_RxHeaderTypeDef *header, uint8_t *rx_data)
{
  static uint32_t uwb_time;
  static uint8_t uwb_seq;
  static struct uwb_data uwb_data;
  if (header->StdId == 0X259)
  {
    if ((get_time_ms() - uwb_time > 10))
      uwb_seq = 0;
    uwb_time = HAL_GetTick();
    memcpy((uint8_t *)&uwb_data + uwb_seq * 8, rx_data, 8);
    uwb_seq++;
    if (uwb_seq == 3)
    {
      protocol_send(MANIFOLD2_ADDRESS, CMD_PUSH_UWB_INFO, &uwb_data, sizeof(uwb_data));
      uwb_seq = 0;
    }
  }
  return header->DLC;
}

int32_t can2_rcv_callback(CAN_RxHeaderTypeDef *header, uint8_t *rx_data)
{
  protocol_can_rcv_data(PROTOCOL_CAN_PORT2, header->StdId, rx_data, header->DLC);
  osSignalSet(communicate_task_t, RECV_PROTOCOL_SIGNAL);
  return header->DLC;
}

static int32_t usb_rcv_callback(uint8_t *buf, uint32_t len)
{
  protocol_uart_rcv_data(PROTOCOL_USB_PORT, buf, len);
  osSignalSet(communicate_task_t, RECV_PROTOCOL_SIGNAL);
  return len;
}

int32_t dr16_rx_data_by_can(uint8_t *buff, uint16_t len)
{
  rc_device_t rc_dev;
  struct detect_device *rc_offline;
  rc_offline = get_offline_dev();
  rc_dev = (rc_device_t)device_find("can_rc");
  rc_device_date_update(rc_dev, buff);
  detect_device_update(rc_offline, RC_OFFLINE_EVENT);
  return 0;
}

int32_t gimbal_adjust(void)
{
  protocol_send(GIMBAL_ADDRESS, CMD_GIMBAL_ADJUST, 0, 0);
  return 0;
}

int32_t manifold2_heart_package(uint8_t *buff, uint16_t len)
{
  return 0;
}

int32_t report_firmware_version(uint8_t *buff, uint16_t len)
{
  return FIRMWARE_VERSION;
}

void communicate_task(void const *argument)
{
  uint8_t app;
  app = get_sys_cfg();

  if (app == CHASSIS_APP)
  {
    protocol_local_init(CHASSIS_ADDRESS);
    protocol_can_interface_register("gimbal_can2", 4096, 1, PROTOCOL_CAN_PORT2, GIMBAL_CAN_ID, CHASSIS_CAN_ID, can2_send_data);
    protocol_uart_interface_register("manifold2", 4096, 1, PROTOCOL_USB_PORT, usb_interface_send);
    protocol_set_route(GIMBAL_ADDRESS, "gimbal_can2");
    protocol_set_route(MANIFOLD2_ADDRESS, "manifold2");
  }
  else
  {
    protocol_local_init(GIMBAL_ADDRESS);
    protocol_can_interface_register("chassis_can2", 4096, 1, PROTOCOL_CAN_PORT2, CHASSIS_CAN_ID, GIMBAL_CAN_ID, can2_send_data);
    protocol_set_route(CHASSIS_ADDRESS, "chassis_can2");
    protocol_set_route(MANIFOLD2_ADDRESS, "chassis_can2");
    protocol_rcv_cmd_register(CMD_RC_DATA_FORWORD, dr16_rx_data_by_can);
  }

  protocol_rcv_cmd_register(CMD_MANIFOLD2_HEART, manifold2_heart_package);
  protocol_rcv_cmd_register(CMD_REPORT_VERSION, report_firmware_version);

  usb_vcp_rx_callback_register(usb_rcv_callback);
  soft_timer_register(usb_tx_flush, NULL, 1);
	protocol_send_list_add_callback_reg(protocol_send_success_callback);

  can_fifo0_rx_callback_register(&can2_manage, uwb_rcv_callback);
  can_fifo0_rx_callback_register(&can2_manage, can2_rcv_callback);

  while (1)
  {
    osEvent event;

    event = osSignalWait(SEND_PROTOCOL_SIGNAL | RECV_PROTOCOL_SIGNAL | REFEREE_SIGNAL, osWaitForever);

    if (event.status == osEventSignal)
    {
      if (event.value.signals & SEND_PROTOCOL_SIGNAL)
      {
        protocol_send_flush();
      }

      if (event.value.signals & RECV_PROTOCOL_SIGNAL)
      {
        protocol_unpack_flush();
      }

      if (event.value.signals & REFEREE_SIGNAL)
      {
        referee_unpack_fifo_data();
      }
    }
  }
}

static int32_t can2_send_data(uint32_t std_id, uint8_t *p_data, uint32_t len)
{
  return can_msg_bytes_send(&hcan2, p_data, len, std_id);
}

static int32_t usb_interface_send(uint8_t *p_data, uint32_t len)
{
  CDC_Transmit_FS(p_data, len);
  return 0;
}

static void protocol_send_success_callback(void)
{
  osSignalSet(communicate_task_t, SEND_PROTOCOL_SIGNAL);
}
