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
/** @file keyboard.h
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief keyboard message handle
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
 
#ifndef __KEY_BOARD_H__
#define __KEY_BOARD_H__

#include "stm32f4xx_hal.h"
#include "shoot_task.h"

/* control key definition */
//      direction  key
#define FORWARD    (rc.kb.bit.W)
#define BACK       (rc.kb.bit.S)
#define LEFT       (rc.kb.bit.A)
#define RIGHT      (rc.kb.bit.D)
//      speed      key
#define FAST_SPD   (rc.kb.bit.SHIFT)
#define SLOW_SPD   (rc.kb.bit.CTRL)
//      function   key or mouse operate
#define TWIST_CTRL (rc.kb.bit.E)
#define BUFF_CTRL  (rc.kb.bit.F)
#define TRACK_CTRL (km.rk_sta == KEY_PRESS_LONG)
//      shoot relevant       key or mouse operation
#define KB_SINGLE_SHOOT     (km.lk_sta == KEY_PRESS_ONCE)
#define KB_CONTINUE_SHOOT   (km.lk_sta == KEY_PRESS_LONG)
#define KB_OPEN_FRIC_WHEEL  (rc.kb.bit.Q)
#define KB_CLOSE_FIRC_WHEEL (rc.kb.bit.Q && rc.kb.bit.SHIFT)

/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/
//#define W 			0x0001		//bit 0
//#define S 			0x0002
//#define A 			0x0004
//#define D 			0x0008
//#define SHIFT 	0x0010
//#define CTRL 		0x0020
//#define Q 			0x0040
//#define E				0x0080
//#define R 			0x0100
//#define F 			0x0200
//#define G 			0x0400
//#define Z 			0x0800
//#define X 			0x1000
//#define C 			0x2000
//#define V 			0x4000		//bit 15
//#define B				0x8000
/******************************************************/

typedef enum 
{
  NORMAL_MODE = 0,
  FAST_MODE,
  SLOW_MODE,
} kb_move_e;

typedef enum
{
  KEY_RELEASE = 0,
  KEY_WAIT_EFFECTIVE,
  KEY_PRESS_ONCE,
  KEY_PRESS_DOWN,
  KEY_PRESS_LONG,
} kb_state_e;

typedef struct
{
  float vx;
  float vy;
  float vw;
  
  float pit_v;
  float yaw_v;
  
  uint8_t twist_ctrl;
  uint8_t buff_ctrl;
  uint8_t track_ctrl;
  
  uint8_t kb_enable;
  
  uint16_t lk_cnt;
  uint16_t rk_cnt;
  
  kb_state_e lk_sta;
  kb_state_e rk_sta;
  
  kb_move_e move;
  
  uint16_t x_spd_limit;
  uint16_t y_spd_limit;

} kb_ctrl_t;

extern kb_ctrl_t km;

void keyboard_global_hook(void);
void keyboard_chassis_hook(void);
void keyboard_gimbal_hook(void);
void keyboard_shoot_hook(void);

#endif
