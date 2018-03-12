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
/** @file shoot_task.c
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief shoot bullet task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "shoot_task.h"
#include "gimbal_task.h"
#include "detect_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "remote_ctrl.h"
#include "bsp_io.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_io.h"
#include "keyboard.h"
#include "pid.h"
#include "sys_config.h"
#include "cmsis_os.h"
#include "string.h"

/* stack usage monitor */
UBaseType_t shoot_stack_surplus;

/* shot task global parameter */
shoot_t   shot;
trigger_t trig;

uint32_t shoot_time_last;
int shoot_time_ms;
void shot_task(void const *argu)
{
  osEvent event;
  
  while (1)
  {
    event = osSignalWait(SHOT_TASK_EXE_SIGNAL, osWaitForever);
    
    if (event.status == osEventSignal)
    {
      if (event.value.signals & SHOT_TASK_EXE_SIGNAL)
      {
        shoot_time_ms = HAL_GetTick() - shoot_time_last;
        shoot_time_last = HAL_GetTick();
        
        fric_wheel_ctrl();
        
        if (!shot.fric_wheel_run)
        {
          shot.shoot_cmd   = 0;
          shot.c_shoot_cmd = 0;
        }
        
#ifdef OLD_TRIGGER  
        /*
        if (shot.fric_wheel_run)
        {
          if (glb_sw.last_sw1 == RC_DN)
            trig.pos_ref = moto_trigger.total_ecd;
          if (shot.shoot_cmd)
          {
            trig.pos_ref = moto_trigger.total_ecd;
            trig.pos_ref += 130922 * trig.dir;
            shot.shoot_cmd = 0;
          }
        
          pid_calc(&pid_trigger, moto_trigger.total_ecd / 100, trig.pos_ref / 100);
          
          if (shot.c_shoot_cmd)
            trig.spd_ref = -4000;
          else
            trig.spd_ref = pid_trigger.out;
          
          block_bullet_handle();
          pid_calc(&pid_trigger_speed, moto_trigger.speed_rpm, trig.spd_ref);
        }
        else
        {
          pid_trigger_speed.out = 0;
        }
        */
#else
        
        trig.key = get_trigger_key_state();
        
        if (shot.fric_wheel_run)
        {
          shoot_bullet_handle();
        }
        else
        {
          pid_trigger_speed.out = 0;
        }
        
        trig.key_last = trig.key;
#endif
      }
    }
    
    shoot_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}



void block_bullet_handle(void)
{
  uint32_t stall_count = 0;
  uint32_t stall_inv_count = 0;
  uint8_t  stall_f = 0;
  
  if (pid_trigger_speed.out <= -4000)
  {
    if (stall_f == 0)
      stall_count ++;
  }
  else
    stall_count = 0;
  
  if (stall_count >= 50)         //0.25s
  {
    stall_f = 1;
    stall_count = 0;
  }
  
  if (stall_f == 1)
  {
    stall_inv_count++;
    
    if (stall_inv_count >= 100)  //0.5s
    {
      stall_f = 0;
      stall_inv_count = 0;
    }
    else
      trig.spd_ref = 2000;
  }
}

static void fric_wheel_ctrl(void)
{
  if (shot.fric_wheel_run)
  {
    turn_on_friction_wheel(shot.fric_wheel_spd);
    turn_on_laser();
  }
  else
  {
    turn_off_friction_wheel();
    turn_off_laser();
  }
}

int debug_tri_speed = 1500;
int shot_cmd;
static void shoot_bullet_handle(void)
{
  shot_cmd = shot.shoot_cmd;
  if (shot.shoot_cmd)
  {
    if (trig.one_sta == TRIG_INIT)
    {
      if (trig.key == 0)
      {
        trig.one_sta = TRIG_PRESS_DOWN;
        trig.one_time = HAL_GetTick();
      }
    }
    else if (trig.one_sta == TRIG_PRESS_DOWN)
    {
      if (HAL_GetTick() - trig.one_time >= 2000) //before the rising
      {
        trig.one_sta = TRIG_ONE_DONE;
      }

      if ((trig.key_last == 0) && (trig.key))    //Rising edge trigger button bounce
      {
        trig.one_sta = TRIG_BOUNCE_UP;
        trig.one_time = HAL_GetTick();
      }
    }
    else if (trig.one_sta == TRIG_BOUNCE_UP)
    {
      if (HAL_GetTick() - trig.one_time >= 2000)
      {
        trig.one_sta = TRIG_ONE_DONE;
      }
      
      if ((trig.key_last) && (trig.key == 0))    //Falling edge trigger button be press
      {
        trig.one_sta = TRIG_ONE_DONE;
      }
    }
    else
    {
    }
    
    if (trig.one_sta == TRIG_ONE_DONE)
    {
      trig.spd_ref = 0;
      trig.one_sta = TRIG_INIT;
      
      shot.shoot_cmd = 0;
      shot.shot_bullets++;
    }
    else
      trig.spd_ref = debug_tri_speed;//trig.feed_bullet_spd;
    
  }
  else if (shot.c_shoot_cmd)
  {
    trig.one_sta = TRIG_INIT;
    trig.spd_ref = trig.c_shot_spd;
    
    if ((trig.key_last == 0) && (trig.key == 1))
      shot.shot_bullets++;
    
    //block_bullet_handle();
  }
  else
  {
    if (trig.key)       //not trigger
      trig.spd_ref = debug_tri_speed;//trig.feed_bullet_spd;
    else
      trig.spd_ref = 0;
  }
  
  pid_calc(&pid_trigger_speed, moto_trigger.speed_rpm, trig.spd_ref);
}

void shot_param_init(void)
{
  memset(&shot, 0, sizeof(shoot_t));
  
  shot.ctrl_mode      = SHOT_DISABLE;
  shot.fric_wheel_spd = DEFAULT_FRIC_WHEEL_SPEED;
  //shot.remain_bullets = 0;
  
  memset(&trig, 0, sizeof(trigger_t));
  
  trig.dir             = 1;
  trig.feed_bullet_spd = 2000;
  trig.c_shot_spd      = 4000;
  trig.one_sta         = TRIG_INIT;
  
}

