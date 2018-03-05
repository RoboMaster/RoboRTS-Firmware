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
} shoot_mode_e;

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
  shoot_mode_e ctrl_mode;
  uint8_t      shoot_cmd;
  uint32_t     c_shoot_time;   //continuous
  uint8_t      c_shoot_cmd;
  uint8_t      fric_wheel_run; //run or not
  uint16_t     fric_wheel_spd;
  uint16_t     shoot_bullets;
  uint16_t     remain_bullets;
} shoot_t;

typedef __packed struct
{
  /* trigger motor param */
  int32_t   spd_ref;
  int32_t   pos_ref;
  int8_t    dir;
  uint8_t   key;
  uint8_t   key_last;
  uint32_t  one_time;
  int32_t   feed_bullet_spd;
  int32_t   c_shoot_spd;
  
  trig_state_e one_sta;
} trigger_t;

typedef enum
{
  SHOOT_CMD,
  FRIC_CTRL,
} shoot_type_e;

void shoot_param_init(void);
void shoot_task(void const *argu);

static void shoot_bullet_handler(void);
static void fric_wheel_ctrl(void);

extern shoot_t   shoot;
extern trigger_t trig;

#endif
