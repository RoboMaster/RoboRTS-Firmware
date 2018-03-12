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
/** @file keyboard.c
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief keyboard message handle
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
 
#include "keyboard.h"
#include "bsp_uart.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "info_get_task.h"
#include "modeswitch_task.h"
#include "ramp.h"
#include "remote_ctrl.h"
#include "cmsis_os.h"
#include "sys_config.h"

/* mouse button long press time */
#define LONG_PRESS_TIME  1000  //ms
/* key acceleration time */
#define KEY_ACC_TIME     1500  //ms

kb_ctrl_t km;

ramp_t fb_ramp = RAMP_GEN_DAFAULT;
ramp_t lr_ramp = RAMP_GEN_DAFAULT;


void key_fsm(kb_state_e *sta, uint8_t key)
{
  switch (*sta)
  {
    case KEY_RELEASE:
    {
      if (key)
        *sta = KEY_WAIT_EFFECTIVE;
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_WAIT_EFFECTIVE:
    {
      if (key)
        *sta = KEY_PRESS_ONCE;
      else
        *sta = KEY_RELEASE;
    }break;
    
    
    case KEY_PRESS_ONCE:
    {
      if (key)
      {
        *sta = KEY_PRESS_DOWN;
        if (sta == &km.lk_sta)
          km.lk_cnt = 0;
        else
          km.rk_cnt = 0;
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_DOWN:
    {
      if (key)
      {
        if (sta == &km.lk_sta)
        {
          if (km.lk_cnt++ > LONG_PRESS_TIME/INFO_GET_PERIOD)
            *sta = KEY_PRESS_LONG;
        }
        else
        {
          if (km.rk_cnt++ > LONG_PRESS_TIME/INFO_GET_PERIOD)
            *sta = KEY_PRESS_LONG;
        }
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_LONG:
    {
      if (!key)
      {
        *sta = KEY_RELEASE;
      }
    }break;
    
    default:
    break;
      
  }
}

static void move_speed_ctrl(uint8_t fast, uint8_t slow)
{
  if (fast)
  {
    km.move = FAST_MODE;
    km.x_spd_limit = CHASSIS_KB_MAX_SPEED_X;
    km.y_spd_limit = CHASSIS_KB_MAX_SPEED_Y;
  }
  else if (slow)
  {
    km.move = SLOW_MODE;
    km.x_spd_limit = 0.7f * CHASSIS_KB_MAX_SPEED_X;
    km.y_spd_limit = 0.7f * CHASSIS_KB_MAX_SPEED_Y;
  }
  else
  {
    km.move = NORMAL_MODE;
    km.x_spd_limit = 0.85f * CHASSIS_KB_MAX_SPEED_X;
    km.y_spd_limit = 0.85f * CHASSIS_KB_MAX_SPEED_Y;
  }
}

static void move_direction_ctrl(uint8_t forward, uint8_t back,
                                uint8_t left,    uint8_t right)
{
  //add ramp
  if (forward)
  {
    km.vx = km.x_spd_limit * ramp_calc(&fb_ramp);
  }
  else if (back)
  {
    km.vx = -km.x_spd_limit * ramp_calc(&fb_ramp);
  }
  else
  {
    km.vx = 0;
    ramp_init(&fb_ramp, KEY_ACC_TIME/INFO_GET_PERIOD);
  }

  if (left)
  {
    km.vy = km.y_spd_limit * ramp_calc(&lr_ramp);
  }
  else if (right)
  {
    km.vy = -km.y_spd_limit * ramp_calc(&lr_ramp);
  }
  else
  {
    km.vy = 0;
    ramp_init(&lr_ramp, KEY_ACC_TIME/INFO_GET_PERIOD);
  }
  
  if (forward || back || left || right)
    km.twist_ctrl = 0;
}

static void chassis_operation_func(uint8_t twist_chassis)
{
  if (twist_chassis)
    km.twist_ctrl = 1;
}


static void kb_fric_ctrl(uint8_t open_fric,  uint8_t close_fric)
{
  if (open_fric)
    shot.fric_wheel_run = 1;
  
  if (close_fric)
    shot.fric_wheel_run = 0;
}

static void kb_shoot_cmd(uint8_t single_fir, uint8_t cont_fir)
{
  if (single_fir)
  {
    shot.shoot_cmd   = 1;
    shot.c_shoot_cmd = 0;
  }
  
  if (cont_fir)
  {
    shot.shoot_cmd   = 0;
    shot.c_shoot_cmd = 1;
  }
  else
    shot.c_shoot_cmd = 0;

}
static void gimbal_operation_func(int16_t pit_ref_spd, int16_t yaw_ref_spd,
                                  uint8_t shoot_buff,  uint8_t track_armor)
{
  km.pit_v = -pit_ref_spd * 0.01f;
  km.yaw_v = -yaw_ref_spd * 0.01f;
  
  
  
  if (shoot_buff)
    km.buff_ctrl = 1;
  
  if (track_armor)
    km.track_ctrl = 1;
  else
    km.track_ctrl = 0;

}

static void exit_buff_hook(uint8_t forward, uint8_t back,
                           uint8_t left,    uint8_t right)
{
  if (forward || back || left || right)
    km.buff_ctrl = 0;
}

void keyboard_global_hook(void)
{
  if (km.kb_enable)
  {
    key_fsm(&km.lk_sta, rc.mouse.l);
    key_fsm(&km.rk_sta, rc.mouse.r);
  }
}


void keyboard_chassis_hook(void)
{
  if (km.kb_enable)
  {
    move_speed_ctrl(FAST_SPD, SLOW_SPD);
    
    move_direction_ctrl(FORWARD, BACK, LEFT, RIGHT);
    
    chassis_operation_func(TWIST_CTRL);
  }
  else
  {
    km.vx = 0;
    km.vy = 0;
    km.twist_ctrl = 0;
  }
}

void keyboard_gimbal_hook(void)
{
  if (km.kb_enable)
  {
    gimbal_operation_func(rc.mouse.y, rc.mouse.x, BUFF_CTRL, TRACK_CTRL);
    
    exit_buff_hook(FORWARD, BACK, LEFT, RIGHT);
  }
  else
  {
    km.pit_v = 0;
    km.yaw_v = 0;
    km.buff_ctrl = 0;
    km.track_ctrl = 0;
  }
}

void keyboard_shoot_hook(void)
{
  //friction wheel control
  kb_fric_ctrl(KB_OPEN_FRIC_WHEEL, KB_CLOSE_FIRC_WHEEL);
  //single or continuous trigger bullet control
  kb_shoot_cmd(KB_SINGLE_SHOOT, KB_CONTINUE_SHOOT);
}
