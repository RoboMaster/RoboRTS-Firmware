/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
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

#include "sys.h"
#include "shoot.h"
#include "dbus.h"
#include "shoot_task.h"

int32_t shoot_firction_toggle(shoot_t pshoot);

void shoot_task(void const *argument)
{
  uint32_t period = osKernelSysTick();
  rc_device_t prc_dev = NULL;

  shoot_t pshoot = NULL;
  pshoot = shoot_find("shoot");
  prc_dev = rc_device_find("can_rc");

  if (prc_dev == NULL)
  {
  }

  uint32_t shoot_time;

  while (1)
  {
    if (rc_device_get_state(prc_dev, RC_S1_MID2UP) == RM_OK)
    {
      shoot_firction_toggle(pshoot);
    }

    if (rc_device_get_state(prc_dev, RC_S1_MID2DOWN) == RM_OK)
    {
      shoot_set_cmd(pshoot, SHOOT_ONCE_CMD, 1);
      shoot_time = get_time_ms();
    }

    if (rc_device_get_state(prc_dev, RC_S2_DOWN) != RM_OK)
    {
      if (rc_device_get_state(prc_dev, RC_S1_DOWN) == RM_OK)
      {
        if (get_time_ms() - shoot_time > 2500)
        {
          shoot_set_cmd(pshoot, SHOOT_CONTINUOUS_CMD, 0);
        }
      }

      if (rc_device_get_state(prc_dev, RC_S1_MID) == RM_OK)
      {
        shoot_set_cmd(pshoot, SHOOT_STOP_CMD, 0);
      }
    }

    shoot_execute(pshoot);
    osDelayUntil(&period, 5);
  }
}

int32_t shoot_firction_toggle(shoot_t pshoot)
{
  static uint8_t toggle = 0;
  if (toggle)
  {
    shoot_set_fric_speed(pshoot, 1000, 1000);
  }
  else
  {
    shoot_set_fric_speed(pshoot, 1250, 1250);
  }
  toggle = ~toggle;
  return 0;
}
