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
/** @file remote_ctrl.c
 *  @version 1.0
 *  @date June 2017
 *
 *  @brief remote control message handle
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
 
#include "remote_ctrl.h"
#include "sys_config.h"
#include "stdlib.h"
#include "string.h"

sw_record_t glb_sw;
rc_info_t   rc;
rc_ctrl_t   rm;

uint8_t  dbus_buf[DBUS_BUFLEN];
 
void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{
  rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch3 -= 1024;
  rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc->ch4 -= 1024;
  
  /* prevent remote control zero deviation */
  if(rc->ch1 <= 5 && rc->ch1 >= -5)
    rc->ch1 = 0;
  if(rc->ch2 <= 5 && rc->ch2 >= -5)
    rc->ch2 = 0;
  if(rc->ch3 <= 5 && rc->ch3 >= -5)
    rc->ch3 = 0;
  if(rc->ch4 <= 5 && rc->ch4 >= -5)
    rc->ch4 = 0;
  
  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;
  
  if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660))
  {
    memset(rc, 0, sizeof(rc_info_t));
    return ;
  }

  rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
  rc->mouse.y = buff[8] | (buff[9] << 8);
  rc->mouse.z = buff[10] | (buff[11] << 8);

  rc->mouse.l = buff[12];
  rc->mouse.r = buff[13];

  rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
}


static void chassis_operation_func(int16_t forward_back, int16_t left_right, int16_t rotate)
{
  rm.vx =  forward_back / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X;
  rm.vy = -left_right / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y;
  rm.vw = -rotate / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_R;
}

void remote_ctrl_chassis_hook(void)
{
  chassis_operation_func(rc.ch2, rc.ch1, rc.ch3);
}

static void gimbal_operation_func(int16_t pit_ctrl, int16_t yaw_ctrl)
{
  /* gimbal coordinate system is right hand coordinate system */
  rm.pit_v = -pit_ctrl * 0.002f;
  rm.yaw_v = -yaw_ctrl * 0.002f;
}

void remote_ctrl_gimbal_hook(void)
{
  gimbal_operation_func(rc.ch4, rc.ch3);
}


static void rc_fric_ctrl(uint8_t ctrl_fric)
{
  if (ctrl_fric)
  {
    shoot.fric_wheel_run = !shoot.fric_wheel_run;
  }
}
static void rc_shoot_cmd(uint8_t single_fir, uint8_t cont_fir)
{
  if (single_fir)
  {
    shoot.c_shoot_time = HAL_GetTick();
    shoot.shoot_cmd   = 1;
    shoot.c_shoot_cmd = 0;
  }
  
  if (cont_fir && (HAL_GetTick() - shoot.c_shoot_time >= 2000))
  {
    shoot.shoot_cmd   = 0;
    shoot.c_shoot_cmd = 1;
  }
  else
    shoot.c_shoot_cmd = 0;
}


void remote_ctrl_shoot_hook(void)
{
  //friction wheel control
  rc_fric_ctrl(RC_CTRL_FRIC_WHEEL);
  //single or continuous trigger bullet control
  rc_shoot_cmd(RC_SINGLE_SHOOT, RC_CONTINUE_SHOOT);
}
