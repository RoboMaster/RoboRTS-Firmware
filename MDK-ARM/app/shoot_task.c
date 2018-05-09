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

/* setting */
#define RELOAD_TIMEOUT 2500
#define ABS(num) ((num)>0?(num):-(num))

/* stack usage monitor */
UBaseType_t shoot_stack_surplus;

/* shot task global parameter */
shoot_t   shot;
bullet_supply_t bupply;

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
        fric_wheel_ctrl();
        if(rc.sw1 == RC_DN || rc.mouse.l == 1){
          shot.shoot_cmd = 1;
        } else {
          shot.shoot_cmd = 0;
        }
        gun_17_handle();
      }
    }
    
    shoot_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}

static void gun_17_handle(void){
  static uint8_t last_cmd = 0;
  
  bupply.bbkey_state = get_bbkey_state();
        
  if(shot.shoot_state == WAITING_CMD){
    if(shot.shoot_cmd == 1 && last_cmd == 0){
      /* ignore shoot command when friction wheel is disable */
      if (shot.fric_wheel_run){
        bupply.spd_ref = bupply.feed_bullet_spd;
        shot.shooted_count = 0;
        shot.timestamp = HAL_GetTick();
        shot.shoot_state = SHOOTING;
      }
    } else if(shot.shoot_cmd == 2 && last_cmd == 0){
      bupply.spd_ref = bupply.feed_bullet_spd;
      shot.timestamp = HAL_GetTick();
      shot.shoot_state = RELOADING;
    }
  } else if(shot.shoot_state == SHOOTING){
    if(shoot_bullet_handle()){
      ;
    } else {
      //no need to update timestamp 
      //because shoot_bullet_handle() has made it
      bupply.spd_ref = bupply.feed_bullet_spd;
      shot.shoot_state = RELOADING;
    }
  } else if(shot.shoot_state == RELOADING){
    if(bupply.bbkey_state == BBKEY_ON){
      //reload complete
      bupply.spd_ref = 0;
      shot.shoot_state = WAITING_CMD;
    } else if(HAL_GetTick()-shot.timestamp > RELOAD_TIMEOUT){
      //timeout
      bupply.spd_ref = 0;
      shot.shoot_state = WAITING_CMD;
    }
  } else if(shot.shoot_state == STUCK_HANDLING){
    if(stuck_handle()){
      ;
    } else {
      bupply.spd_ref = bupply.feed_bullet_spd;
      shot.timestamp = HAL_GetTick();
      shot.shoot_state = RELOADING;
    }
  }
  if(stuck_detect()){
    bupply.spd_ref = -bupply.spd_ref;
    shot.timestamp = HAL_GetTick();
    shot.shoot_state = STUCK_HANDLING;
  }
  pid_calc(&pid_trigger_speed, moto_trigger.speed_rpm, bupply.spd_ref);
  bupply.bbkey_state_last = bupply.bbkey_state;
  last_cmd = shot.shoot_cmd;
}

void switch_shoot_mode(shoot_mode_e mode){
  shot.shoot_mode = mode;
  switch(shot.shoot_mode){
    case SEMI_ONE:
      shot.shoot_spd = 15;
      shot.shoot_num = 1;
      shot.fric_wheel_spd = 1600;
      break;
    case SEMI_THREE:
      shot.shoot_spd = 15;
      shot.shoot_num = 3;
      shot.fric_wheel_spd = 1400;
      break;
    case AUTO:
      shot.shoot_spd = 15;
      shot.shoot_num = 0;
      shot.fric_wheel_spd = 1200;
      break;
  }
}

static uint8_t stuck_detect(void){
  static uint32_t last_uptime;
  if(ABS(moto_trigger.speed_rpm) < ABS(bupply.spd_ref)/3){
    if(HAL_GetTick()-last_uptime > 500){
      //stuck confirm
      return 1;
    }
  } else {
    last_uptime = HAL_GetTick();
  }
  return 0;
}

static uint8_t stuck_handle(void)
{
  if(HAL_GetTick()-shot.timestamp > 100000/ABS(bupply.spd_ref)){
    return 0;
  } else {
    return 1;
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

static uint8_t shoot_bullet_handle(void)
{
  static uint8_t holding = 0;
  uint32_t time_now = HAL_GetTick();
  if(time_now-shot.timestamp > RELOAD_TIMEOUT){
    holding = 0;  //reset state
    return 0;
  } else if((shot.shooted_count >= shot.shoot_num) && (shot.shoot_mode != AUTO || shot.shoot_cmd == 0)){
    holding = 0;  //reset state
    return 0;
  } else {
    if(holding){
      if(time_now-shot.timestamp >= 1000/shot.shoot_spd){
        holding = 0;  //shoot
        bupply.spd_ref = bupply.feed_bullet_spd;
      }
    } else {
      if(bupply.bbkey_state_last == BBKEY_ON && bupply.bbkey_state == BBKEY_OFF){
        shot.shooted_count++;
        shot.timestamp = HAL_GetTick();
      } else if(bupply.bbkey_state_last == BBKEY_OFF && bupply.bbkey_state == BBKEY_ON){
        holding = 1;  //waiting next shoot
        bupply.spd_ref = 0;
      }
    }
    return 1;
  }
}

void fric_esc_cali(void){
  uint8_t cali_done = 0;
  turn_on_friction_wheel(2000);
  while(!cali_done){
    if(get_key_state() == 0){
      turn_on_friction_wheel(1000);
      cali_done = 0;
    }
  }
}

void shot_param_init(void)
{
  memset(&shot, 0, sizeof(shoot_t));
  
  shot.ctrl_mode      = SHOT_DISABLE;
  shot.shoot_state    = WAITING_CMD;
  switch_shoot_mode(SEMI_ONE);
  //shot.remain_bullets = 0;
  
  memset(&bupply, 0, sizeof(bullet_supply_t));
  
  bupply.feed_bullet_spd = TRI_MOTO_POSITIVE_DIR*TRIGGER_MOTOR_SPEED; //2000; //changed by H.F.
}

