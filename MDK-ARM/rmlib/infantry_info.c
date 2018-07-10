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

#include "modeswitch_task.h"
#include "remote_ctrl.h"
#include "gimbal_task.h"
#include "chassis_task.h"
/* data send */
send_pc_t    pc_send_mesg;
/* data receive */
receive_pc_t pc_recv_mesg;

//for debug
int pc_seq            = 0;
int once_lost_num     = 0;
int lost_pack_percent = 0;

int pack_num_cnt   = 0;
int lost_num_sum_t = 0;
int pack_lost      = 0;


int gim_mode_js;
int pitch_angle_js;
int yaw_angle_js;
int dis_mm_js;

uint32_t pc_yaw_time;

int8_t recv_pc_glb  = 0;
int8_t glb_err_exit = 0;


int pc_state;

extern gimbal_t gim;
extern infantry_mode_e glb_ctrl_mode;
gimbal_mode_e last_gim_mode;
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

  //lost pack monitor
  pack_num_cnt++;
  
  if (pack_num_cnt <= 100)
  {
    once_lost_num = p_header->seq - pc_seq - 1;
    
    if (once_lost_num < 0)
    {
      once_lost_num += 256;
    }
    
    lost_num_sum_t += once_lost_num;
  }
  else
  {
    lost_pack_percent = lost_num_sum_t;
    lost_num_sum_t    = 0;
    pack_num_cnt      = 0;
  }
  
  
  if (once_lost_num != 0)
  {
    pack_lost = 1;
  }
  else
  {
    pack_lost = 0;
  }
  
  pc_seq = p_header->seq;
  //end lost pack monitor
  
  
  taskENTER_CRITICAL();
  
  switch (cmd_id)
  {
    case CHASSIS_CTRL_ID:
    {
      memcpy(&pc_recv_mesg.chassis_control_data, data_addr, data_length);
      
      if (glb_ctrl_mode == AUTO_CTRL_MODE)
      {
        switch (rc.sw1)
        {
          case RC_UP:
          case RC_MI:
          {
            chassis.ctrl_mode = (chassis_mode_e)pc_recv_mesg.chassis_control_data.ctrl_mode;
          }break;
          
          case RC_DN:
          {
            chassis.ctrl_mode = CHASSIS_STOP;
          }break;
          
        }
        
      }
      
    }
    break;

    case GIMBAL_CTRL_ID:
    {
      memcpy(&pc_recv_mesg.gimbal_control_data, data_addr, data_length);
      gim_mode_js    = pc_recv_mesg.gimbal_control_data.ctrl_mode;
      yaw_angle_js   = pc_recv_mesg.gimbal_control_data.yaw_ref*1000;
      pitch_angle_js = pc_recv_mesg.gimbal_control_data.pit_ref*1000;
      dis_mm_js      = pc_recv_mesg.gimbal_control_data.tgt_dist;
      pc_yaw_time    = HAL_GetTick();
    
      if (glb_ctrl_mode == AUTO_CTRL_MODE)
      {
        switch (rc.sw1)
        {
          case RC_UP:
          case RC_MI:
          {
            //patrol and relative mode
            gim.ctrl_mode = (gimbal_mode_e)pc_recv_mesg.gimbal_control_data.ctrl_mode;
            
            /* gimbal first enter patrol mode */
            /* patrol valid only in gimbal auto mode */
            if (last_gim_mode != GIMBAL_PATROL_MODE && gim.ctrl_mode == GIMBAL_PATROL_MODE)
              gim.pid.yaw_angle_ref = gim.sensor.yaw_relative_angle;
            
          }break;
          
          default:
          {
            gim.ctrl_mode = GIMBAL_RELAX;
          }break;
        }
        
        last_gim_mode = gim.ctrl_mode;
      }
    }
    break;

    case SHOOT_CTRL_ID:
      memcpy(&pc_recv_mesg.shoot_control_data, data_addr, data_length);
    break;
    
    case ERROR_LEVEL_ID:
    {
      memcpy(&pc_recv_mesg.global_error_level, data_addr, data_length);
      
      pc_state = pc_recv_mesg.global_error_level.err_level;
      
      if ((judge_recv_mesg.game_information.game_process == 1) 
       && (judge_recv_mesg.game_information.stage_remain_time <= 300)
       && (judge_recv_mesg.game_information.stage_remain_time >= 240))
      {
        recv_pc_glb = 1;
        
        if (pc_recv_mesg.global_error_level.err_level != GLOBAL_NORMAL)
          glb_err_exit = 1;
      }
      
    }
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







