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

#include "main.h"
#include "can.h"
#include "board.h"
#include "init.h"
#include "motor.h"
#include "os_timer.h"
#include "infantry_cmd.h"
#include "protocol.h"
#include "event_mgr.h"
#include "event.h"
#include "referee_system.h"
#include "shell.h"

#include "infantry_cmd.h"
#include "app_manage.h"

#include "log.h"

static struct app_manage *board_app;

static publisher_t dbusPub;
static publisher_t uwbPub;

static int32_t motor_canstd_send(enum device_can can, struct can_msg msg);
static int32_t motor_can_output_1ms(void *argc);
static int32_t uwb_rcv_callback(CAN_RxHeaderTypeDef *header, uint8_t *rx_data);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_6)
    {
        if (board_app->user_input_callback)
        {
            board_app->user_input_callback();
        }
    }

    if (GPIO_Pin == GPIO_PIN_0)
    {
        if (board_app->user_key_callback)
        {
            board_app->user_key_callback();
        }
    }
}

void referee_cmd_handle(uint16_t cmd_id, uint8_t *pdata, uint16_t len)
{
    if (board_app->referee_cmd_callback)
    {
        board_app->referee_cmd_callback(cmd_id, pdata, len);
    }
}

/* usart3: receive dbus data */
int32_t dr16_rx_data_by_uart(uint8_t *buff, uint16_t len)
{
    EventMsgPost(&dbusPub, buff, DBUS_MSG_LEN);
    if (board_app->dbus_rx_complete)
    {
        board_app->dbus_rx_complete();
    }
    return 0;
}

int32_t dr16_rx_data_by_can(uint8_t *buff, uint16_t len)
{
    EventMsgPost(&dbusPub, buff, DBUS_MSG_LEN);
    if (board_app->dbus_rx_complete)
    {
        board_app->dbus_rx_complete();
    }
    return 0;
}

int32_t dr16_forward_by_can(uint8_t *buff, uint16_t len)
{
    protocol_send(PROTOCOL_BROADCAST_ADDR, CMD_RC_DATA_FORWORD, buff, len);
    return 0;
}

/**
  * @brief  usb vcp interupt
  * @param
  * @retval void
  */
int32_t usb_rcv_callback(uint8_t *buf, uint32_t len)
{
    protocol_uart_rcv_data(USB_COM, buf, len);
    return len;
}

/**
  * @brief  can1 interupt
  * @param
  * @retval void
  */
int32_t can1_msg_rec(CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    protocol_can_rcv_data(CAN1_PORT, header->StdId, data, header->DLC);
    if (board_app->can1_msg_callback)
    {
        board_app->can1_msg_callback(header->StdId, data, header->DLC);
    }
    return 0;
}

/**
  * @brief  can2 interupt
  * @param
  * @retval void
  */
int32_t can2_msg_rec(CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    motor_data_update(DEVICE_CAN2, header->StdId, data);
    uwb_rcv_callback(header, data);
    if (board_app->can2_msg_callback)
    {
        board_app->can2_msg_callback(header->StdId, data, header->DLC);
    }
    return 0;
}

int32_t uwb_rcv_callback(CAN_RxHeaderTypeDef *header, uint8_t *rx_data)
{
    static uint32_t uwb_time;
    static uint8_t uwb_seq;
    static struct uwb_data uwb_data;
    if (header->StdId == 0X259)
    {
        if ((get_time_ms() - uwb_time > 10))
        {
            uwb_seq = 0;
        }
        uwb_time = HAL_GetTick();
        memcpy((uint8_t *)&uwb_data + uwb_seq * 8, rx_data, 8);
        uwb_seq++;
        if (uwb_seq == 3)
        {
            /*TODO: uwb data handle */
            EventMsgPost(&uwbPub, &uwb_data, UWB_MSG_LEN);
            uwb_seq = 0;
        }
    }
    return header->DLC;
}

/**
  * @brief  usart1 interupt, debug shell
  * @param
  * @retval void
  */
uint32_t usart1_rx_callback(uint8_t *buff, uint16_t len)
{
    shell_interupt(buff, len);
    return 0;
}

/**
  * @brief  usart6 interupt, this uart is uesd to receive referee data.
  * @param
  * @retval void
  */
uint32_t usart6_rx_callback(uint8_t *buff, uint16_t len)
{
    referee_uart_rx_data_handle(buff, len);
    return 0;
}

static int status_led_period = 300;

/**
  * @brief  type c board init
  * @param
  * @retval void
  */
void board_config(void)
{
    /* system log */
    usart1_manage_init();
    log_printf("\r\n\r\n"
               "***********RoboMaster AI Robot**************\r\n");
    log_printf("* Copy right: All right reserved.\r\n");
    log_printf("* Release Time: %s.\r\n", __TIME__);
    log_printf("********************************************\r\n");

    board_app = get_current_app();

    soft_timer_init();
    usart6_manage_init();
    can_manage_init();
    pwm_device_init();
    bmi088_device_init();

    /* protocol object init */
    protocol_local_object_init();
    /* DBUS message */
    EventPostInit(&dbusPub, DBUS_MSG, DBUS_MSG_LEN);
    /* UWB message */
    EventPostInit(&uwbPub, UWB_MSG, UWB_MSG_LEN);

    soft_timer_register(usb_tx_flush, NULL, 1);
    soft_timer_register(beep_ctrl_times, NULL, 1);
    soft_timer_register(green_led_toggle, &status_led_period, 5);

    motor_can_send_register(motor_canstd_send);
    soft_timer_register(motor_can_output_1ms, NULL, 1);

    /* dbus callback init */
    dr16_uart_init(dr16_rx_data_by_uart, dr16_forward_by_can);
    /* dbus data cmd register */
    protocol_rcv_cmd_register(CMD_RC_DATA_FORWORD, dr16_rx_data_by_can);

    /* referee protocol init */
    referee_param_init(usart1_transmit, NULL, referee_cmd_handle);

    usart1_rx_callback_register(usart1_rx_callback);
    usart6_rx_callback_register(usart6_rx_callback);

    usb_vcp_rx_callback_register(usb_rcv_callback);
    can_fifo0_rx_callback_register(&can1_manage, can1_msg_rec);
    can_fifo0_rx_callback_register(&can2_manage, can2_msg_rec);

    /* protocol interface init */
    protocol_uart_interface_register("usb", 1024, 1, USB_COM, usb_interface_send);
}

int32_t motor_can_output_1ms(void *argc)
{
    motor_can_output(DEVICE_CAN_ALL);
    return 0;
}

int32_t motor_canstd_send(enum device_can can, struct can_msg msg)
{
    if (can == DEVICE_CAN1)
    {
        can1_std_transmit(msg.id, msg.data, 8);
    }
    else if (can == DEVICE_CAN2)
    {
        can2_std_transmit(msg.id, msg.data, 8);
    }
    return 0;
}

uint32_t get_time_us(void)
{
    return TIM9->CNT;
}

uint32_t get_time_ms(void)
{
    return HAL_GetTick();
}

float get_time_ms_us(void)
{
    return get_time_ms() + get_time_us() / 1000.0f;
}
