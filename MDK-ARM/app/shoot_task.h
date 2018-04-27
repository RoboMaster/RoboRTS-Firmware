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
/** @file shoot_task.h
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief shoot bullet task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
 
#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

#include "stm32f4xx_hal.h"

/* shoot task control period time (ms) */
#define SHOT_TASK_PERIOD 5

typedef enum
{
  SHOT_DISABLE       = 0,
  REMOTE_CTRL_SHOT   = 1,
  KEYBOARD_CTRL_SHOT = 2,
  SEMIAUTO_CTRL_SHOT = 3,
  AUTO_CTRL_SHOT     = 4,
} shoot_ctrl_mode_e;

typedef enum{
  SEMI_ONE    = 1,
  SEMI_THREE  = 2,
  AUTO        = 3,
} shoot_mode_e;

typedef enum{
  WAITING_CMD     = 0,
  SHOOTING        = 1,
  RELOADING       = 2,
  STUCK_HANDLING  = 3,
} shoot_state_e;

typedef enum
{
  TRIG_INIT       = 0,
  TRIG_PRESS_DOWN = 1,
  TRIG_BOUNCE_UP  = 2,
  TRIG_ONE_DONE   = 3,
} trig_state_e;



typedef __packed struct
{
  /* shoot task relevant param */
  //public
  shoot_ctrl_mode_e ctrl_mode;
  shoot_mode_e shoot_mode;
  uint8_t      shoot_cmd;	//1 for shot, 2 for reload
  uint8_t      fric_wheel_run; //run or not
  uint16_t     shot_bullets;
  uint16_t     remain_bullets;
  //private
  shoot_state_e shoot_state;
  uint8_t      shoot_spd; //shoot speed(frequence)
  uint8_t      shoot_num; //number of bullet to be shooted each cmd
	uint8_t      shooted_count; //number of bullet have shooted each cmd
  uint16_t     fric_wheel_spd;
  uint32_t     timestamp; //store last key action time
} shoot_t;

typedef __packed struct
{
  /* bullet supply motor param */
  int32_t   spd_ref;
  int32_t   pos_ref;
  int32_t   feed_bullet_spd;
  
  uint8_t bbkey_state; //state of bullet block key
  uint8_t bbkey_state_last;
} bullet_supply_t;

#define BBKEY_ON 1  //bullet is on the bullet block key
#define BBKEY_OFF 0

void shot_param_init(void);
void shot_task(void const *argu);
void switch_shoot_mode(shoot_mode_e mode);

static uint8_t shoot_bullet_handle(void);
static void fric_wheel_ctrl(void);
static uint8_t stuck_detect(void);
static uint8_t stuck_handle(void);

extern shoot_t   shot;

#endif
