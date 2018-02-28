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
/** @file infantry_info.c
 *  @version 1.0
 *  @date Oct 2017
 *
 *  @brief the information from computer
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "infantry_info.h"
#include "protocol.h"
#include "info_interactive.h"
#include "judgement_info.h"
#include "communicate.h"
#include "comm_task.h"
#include "string.h"
#include "sys_config.h"
#include "cmsis_os.h"

/* data send */
send_pc_t    pc_send_mesg;
/* data receive */
receive_pc_t pc_recv_mesg;

/**
  * @brief    get computer control message
  */
extern TaskHandle_t judge_unpack_task_t;
void pc_data_handler(uint8_t *p_frame)
{
  frame_header_t *p_header = (frame_header_t*)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;

  taskENTER_CRITICAL();
  
  switch (cmd_id)
  {
    case CHASSIS_CTRL_ID:
      memcpy(&pc_recv_mesg.chassis_control_data, data_addr, data_length);
    break;

    case GIMBAL_CTRL_ID:
      memcpy(&pc_recv_mesg.gimbal_control_data, data_addr, data_length);
    break;

    case SHOOT_CTRL_ID:
      memcpy(&pc_recv_mesg.shoot_control_data, data_addr, data_length);
    break;
    
    case ERROR_LEVEL_ID:
      memcpy(&pc_recv_mesg.global_error_level, data_addr, data_length);
    break;
    
    case INFANTRY_STRUCT_ID:
      memcpy(&pc_recv_mesg.structure_data, data_addr, data_length);
    break;
          
    case CALI_GIMBAL_ID:
      memcpy(&pc_recv_mesg.cali_cmd_data, data_addr, data_length);
    break;
    
    case STU_CUSTOM_DATA_ID:
      memcpy(&pc_recv_mesg.show_in_client_data, data_addr, data_length);
    break;
    
    case ROBOT_TO_CLIENT_ID:
      memcpy(&pc_recv_mesg.pc_to_server_data, data_addr, data_length);
    break;
    

  }
  
  taskEXIT_CRITICAL();
  
  /* forward data */
  if (cmd_id == STU_CUSTOM_DATA_ID)
  {
    data_packet_pack(STU_CUSTOM_DATA_ID, (uint8_t *)&pc_recv_mesg.show_in_client_data,
                     sizeof(client_show_data_t), DN_REG_ID);
    
    osSignalSet(judge_unpack_task_t, JUDGE_UART_TX_SIGNAL);
  }
  else if (cmd_id == ROBOT_TO_CLIENT_ID)
  {
    data_packet_pack(ROBOT_TO_CLIENT_ID, (uint8_t *)&pc_recv_mesg.pc_to_server_data,
                     sizeof(user_to_server_t), DN_REG_ID);
    
    osSignalSet(judge_unpack_task_t, JUDGE_UART_TX_SIGNAL);
  }
}







